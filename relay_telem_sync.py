#!/usr/bin/env python3
"""
Synchronous (threaded) version of relay_telem.py

DroneKit telemetry relay with gimbal servo support.
Uses threading instead of asyncio for simpler integration.
"""

import os
import json
import math
import time
import threading
from typing import Optional

import requests
from aiortc import RTCConfiguration, RTCIceServer, RTCPeerConnection
from dronekit import connect, Vehicle
from pymavlink import mavutil


# Persistent HTTP session
HTTP_SESSION = requests.Session()
HTTP_SESSION.headers.update({"Connection": "keep-alive"})


# -----------------------------------------------------------------------------
# Servo / Gimbal helpers (same as relay_telem.py)
# -----------------------------------------------------------------------------

def set_servo_pwm(vehicle: Vehicle, channel: int, pwm: int) -> None:
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel, pwm,
        0, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)


def _servo_listener(vehicle, name, message):
    for i in range(1, 17):
        key = f'servo{i}_raw'
        if hasattr(message, key):
            vehicle.channels_out[i] = getattr(message, key)


def enable_servo_output(vehicle: Vehicle) -> None:
    vehicle.channels_out = {i: 1500 for i in range(1, 17)}
    vehicle.add_message_listener('SERVO_OUTPUT_RAW', _servo_listener)


def pwm_to_gimbal_pitch(pwm: int) -> float:
    pwm = max(1000, min(2000, pwm))
    return -89.9 + (pwm - 1000) * (89.8 / 1000)


# -----------------------------------------------------------------------------
# Telemetry packet builder
# -----------------------------------------------------------------------------

def build_telemetry_packet(vehicle: Vehicle) -> dict:
    platform = {
        "lat": float(vehicle.location.global_frame.lat or 0.0),
        "lon": float(vehicle.location.global_frame.lon or 0.0),
        "alt": float(vehicle.location.global_frame.alt or 0.0),
        "roll": math.degrees(vehicle.attitude.roll),
        "pitch": math.degrees(vehicle.attitude.pitch),
        "yaw": math.degrees(vehicle.attitude.yaw),
    }

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
    url = f"{base_url.rstrip('/')}/webrtc/telemetry"
    try:
        HTTP_SESSION.post(url, json=packet, timeout=2)
    except Exception:
        pass


# -----------------------------------------------------------------------------
# WebRTC handling (runs in its own thread with asyncio)
# -----------------------------------------------------------------------------

def _run_webrtc_session(base_url: str, result: dict):
    """Run WebRTC offer creation in a background thread."""
    import asyncio

    async def webrtc_task():
        config = RTCConfiguration(
            iceServers=[RTCIceServer(urls=["stun:stun.l.google.com:19302"])]
        )
        pc = RTCPeerConnection(config)
        data_channel = pc.createDataChannel("telemetry")

        dc_open = {"open": False}

        @data_channel.on("open")
        def on_open():
            dc_open["open"] = True
            print("[webrtc] DataChannel open")

        @data_channel.on("close")
        def on_close():
            dc_open["open"] = False

        print("[webrtc] Creating offer...")
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)

        try:
            resp = HTTP_SESSION.post(
                f"{base_url}/webrtc/offer",
                json={"sdp": pc.localDescription.sdp, "type": pc.localDescription.type},
                timeout=5,
            ).json()
            session_id = resp.get("session_id")
            if session_id:
                                print(f"\n[webrtc] Session ready: {session_id}")
                result["session_id"] = session_id
        except Exception as exc:
            print(f"[webrtc] Offer failed: {exc}")

        result["pc"] = pc
        result["data_channel"] = data_channel
        result["dc_open"] = dc_open

        # Keep the connection alive
        while True:
            await asyncio.sleep(1)

    asyncio.run(webrtc_task())


# -----------------------------------------------------------------------------
# Main synchronous relay loop
# -----------------------------------------------------------------------------

def run_relay_sync(vehicle: Vehicle, base_url: str, hz: float) -> None:
    interval = 1.0 / max(hz, 1.0)
    use_webrtc = os.getenv("USE_WEBRTC", "1") != "0"

    webrtc_result = {}
    webrtc_thread: Optional[threading.Thread] = None

    if use_webrtc:
        webrtc_thread = threading.Thread(
            target=_run_webrtc_session,
            args=(base_url, webrtc_result),
            daemon=True
        )
        webrtc_thread.start()
        # Give WebRTC a moment to initialize
        time.sleep(0.5)

    print(f"[relay] Streaming at {hz} Hz (sync/threaded mode)")

    start_time = time.time()
    packet_count = 0

    try:
        while True:
            packet = build_telemetry_packet(vehicle)
            packet_count += 1

            # Always send REST
            post_telemetry(base_url, packet)

            # Send via WebRTC if ready
            dc = webrtc_result.get("data_channel")
            dc_open = webrtc_result.get("dc_open", {}).get("open", False)
            if dc_open and dc and dc.readyState == "open":
                try:
                    dc.send(json.dumps(packet))
                except Exception:
                    pass

            # Status output
            now = time.time()
            if now - start_time > 2.0:
                method = "REST+DC" if dc_open else "REST"
                alt = packet["platform"]["alt"]
                gimbal = packet["mount1"]["pitch"]
                actual_hz = packet_count / 2.0
                print(f"[{method}] target={hz}Hz actual≈{actual_hz:.1f}Hz alt={alt:.1f} gimbal={gimbal:.1f}°")
                packet_count = 0
                start_time = now

            time.sleep(interval)

    except KeyboardInterrupt:
        print("\n[relay] Shutting down...")
    finally:
        if "pc" in webrtc_result:
            # Best effort close
            try:
                import asyncio
                asyncio.get_event_loop().run_until_complete(webrtc_result["pc"].close())
            except Exception:
                pass


def main() -> None:
    connect_str = os.getenv("DRONEKIT_CONNECTION", "tcp:127.0.0.1:5763")
    hz = float(os.getenv("TELEMETRY_HZ", "24"))
    base_url = os.getenv("RELAY_BASE", "http://localhost:9001")

    print(f"[relay] Connecting to vehicle at {connect_str}...")
    vehicle = connect(connect_str, wait_ready=True, rate=30)
    print("[relay] Vehicle connected")

    enable_servo_output(vehicle)

    try:
        run_relay_sync(vehicle, base_url, max(hz, 1))
    finally:
        vehicle.close()
        print("[relay] Disconnected")


if __name__ == "__main__":
    main()