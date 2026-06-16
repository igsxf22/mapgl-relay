#!/usr/bin/env python3
"""
DroneKit telemetry relay with gimbal servo support.

Sends the 24-float packet to the 3D viewer via:
- Always: REST POST /webrtc/telemetry (reliable fallback)
- Optionally: WebRTC DataChannel "telemetry" (lower latency)

Gimbal pitch is read from SERVO_OUTPUT_RAW and converted to an angle.
"""

import os
import asyncio
import json
import math
import time

import requests
from aiortc import RTCConfiguration, RTCIceServer, RTCPeerConnection
from dronekit import connect, Vehicle
from pymavlink import mavutil


# Persistent HTTP session
HTTP_SESSION = requests.Session()
HTTP_SESSION.headers.update({"Connection": "keep-alive"})


# -----------------------------------------------------------------------------
# Servo / Gimbal helpers
# -----------------------------------------------------------------------------

def set_servo_pwm(vehicle: Vehicle, channel: int, pwm: int) -> None:
    """Send a MAV_CMD_DO_SET_SERVO command."""
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel, pwm,
        0, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)


def _servo_listener(vehicle, name, message):
    """Update vehicle.channels_out from SERVO_OUTPUT_RAW."""
    for i in range(1, 17):
        key = f'servo{i}_raw'
        if hasattr(message, key):
            vehicle.channels_out[i] = getattr(message, key)


def enable_servo_output(vehicle: Vehicle) -> None:
    """Enable reading of raw servo outputs."""
    vehicle.channels_out = {i: 1500 for i in range(1, 17)}
    vehicle.add_message_listener('SERVO_OUTPUT_RAW', _servo_listener)


def pwm_to_gimbal_pitch(pwm: int) -> float:
    """
    Convert servo PWM (1000-2000) to gimbal pitch in degrees.
    1000 PWM → -89.9° (tilted back)
    2000 PWM → -0.1°  (almost level, avoids gimbal lock)
    """
    pwm = max(1000, min(2000, pwm))
    # Linear map: 1000→-89.9, 2000→-0.1
    return -89.9 + (pwm - 1000) * (89.8 / 1000)


# -----------------------------------------------------------------------------
# Telemetry packet
# -----------------------------------------------------------------------------

def build_telemetry_packet(vehicle: Vehicle) -> dict:
    """Return the canonical 24-float packet with gimbal data."""
    platform = {
        "lat": float(vehicle.location.global_frame.lat or 0.0),
        "lon": float(vehicle.location.global_frame.lon or 0.0),
        "alt": float(vehicle.location.global_frame.alt or 0.0),
        "roll": math.degrees(vehicle.attitude.roll),
        "pitch": math.degrees(vehicle.attitude.pitch),
        "yaw": math.degrees(vehicle.attitude.yaw),
    }

    # Gimbal pitch from servo channel 5 (adjust channel as needed)
    servo_pwm = vehicle.channels_out.get(5, 1500)
    gimbal_pitch = pwm_to_gimbal_pitch(servo_pwm)

    mount1 = {"roll": 0.0, "pitch": gimbal_pitch, "yaw": 0.0}
    mount2 = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}

    camera = {"fov": 60.0, "mode": 0.0}

    aux = {}
    if vehicle.groundspeed is not None:
        aux["groundspeed"] = float(vehicle.groundspeed)
    if vehicle.airspeed is not None:
        aux["airspeed"] = float(vehicle.airspeed)
    if vehicle.battery is not None and vehicle.battery.voltage is not None:
        aux["battery_voltage"] = float(vehicle.battery.voltage)
    if getattr(vehicle, "heading", None) is not None:
        aux["heading"] = float(vehicle.heading)

    while len(aux) < 10:
        aux[f"aux_{len(aux)}"] = 0.0

    packet = {
        "platform": platform,
        "mount1": mount1,
        "mount2": mount2,
        "camera": camera,
        "aux": aux,
    }

    total = len(platform) + len(mount1) + len(mount2) + len(camera) + len(aux)
    assert total == 24, f"Expected 24 floats, got {total}"
    return packet


def post_telemetry(base_url: str, packet: dict) -> None:
    """Fast REST fallback."""
    url = f"{base_url.rstrip('/')}/webrtc/telemetry"
    try:
        HTTP_SESSION.post(url, json=packet, timeout=2)
    except Exception:
        pass


# -----------------------------------------------------------------------------
# WebRTC + Main loop
# -----------------------------------------------------------------------------

async def run_relay(vehicle: Vehicle, base_url: str, hz: float) -> None:
    """Main telemetry streaming loop."""
    interval = 1.0 / max(hz, 1.0)
    use_webrtc = os.getenv("USE_WEBRTC", "1") != "0"

    data_channel = None
    dc_open = False

    if use_webrtc:
        config = RTCConfiguration(
            iceServers=[RTCIceServer(urls=["stun:stun.l.google.com:19302"])]
        )
        pc = RTCPeerConnection(config)
        data_channel = pc.createDataChannel("telemetry")

        @data_channel.on("open")
        def on_open():
            nonlocal dc_open
            dc_open = True
            print("[webrtc] DataChannel open")

        @data_channel.on("close")
        def on_close():
            nonlocal dc_open
            dc_open = False

        print("[webrtc] Creating offer...")
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)

        try:
            resp = HTTP_SESSION.post(
                f"{base_url}/webrtc/offer",
                json={"sdp": pc.localDescription.sdp, "type": pc.localDescription.type},
                timeout=5,
            ).json()
            sid = resp.get("session_id")
            if sid:
                print(f"\n[webrtc] Session ready: {sid}")
                print(f'    Run: window.connectTelemetry("{sid}")\n')
        except Exception as exc:
            print(f"[webrtc] Offer failed: {exc}")

    print(f"[relay] Streaming at {hz} Hz")

    start_time = time.time()
    packet_count = 0

    try:
        while True:
            packet = build_telemetry_packet(vehicle)
            packet_count += 1

            post_telemetry(base_url, packet)

            if dc_open and data_channel and data_channel.readyState == "open":
                try:
                    data_channel.send(json.dumps(packet))
                except Exception:
                    pass

            now = time.time()
            if now - start_time > 2.0:
                method = "REST+DC" if dc_open else "REST"
                alt = packet["platform"]["alt"]
                actual = packet_count / 2.0
                print(f"[{method}] {hz}Hz  alt={alt:.1f}  gimbal={packet['mount1']['pitch']:.1f}°")
                packet_count = 0
                start_time = now

            await asyncio.sleep(interval)

    finally:
        if "pc" in locals():
            await pc.close()


async def main() -> None:
    connect_str = os.getenv("DRONEKIT_CONNECTION", "tcp:127.0.0.1:5763")
    hz = float(os.getenv("TELEMETRY_HZ", "24"))
    base_url = os.getenv("RELAY_BASE", "http://localhost:9001")

    print(f"[relay] Connecting to {connect_str}...")
    vehicle = connect(connect_str, wait_ready=True, rate=30)
    print("[relay] Vehicle connected")

    # Enable servo output tracking (for gimbal)
    enable_servo_output(vehicle)

    try:
        await run_relay(vehicle, base_url, max(hz, 1))
    finally:
        vehicle.close()
        print("[relay] Disconnected")


if __name__ == "__main__":
    asyncio.run(main())