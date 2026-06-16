#!/usr/bin/env python3
"""
DroneKit telemetry relay.

Sends the 24-float packet to the 3D viewer via:
- Always: REST POST /webrtc/telemetry (reliable fallback)
- Optionally: WebRTC DataChannel "telemetry" (lower latency)

The external script is the offerer. Browser must run:
    window.connectTelemetry("session-id")
"""

import os
import asyncio
import json
import math
import time

import requests
from aiortc import RTCConfiguration, RTCIceServer, RTCPeerConnection
from dronekit import connect, Vehicle


# Persistent HTTP session for speed
HTTP_SESSION = requests.Session()
HTTP_SESSION.headers.update({"Connection": "keep-alive"})


def build_telemetry_packet(vehicle: Vehicle) -> dict:
    """Return the canonical 24-float packet shape."""
    platform = {
        "lat": float(vehicle.location.global_frame.lat or 0.0),
        "lon": float(vehicle.location.global_frame.lon or 0.0),
        "alt": float(vehicle.location.global_frame.alt or 0.0),
        "roll": math.degrees(vehicle.attitude.roll),
        "pitch": math.degrees(vehicle.attitude.pitch),
        "yaw": math.degrees(vehicle.attitude.yaw),
    }

    mount = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}

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
        "mount1": mount,
        "mount2": mount,
        "camera": camera,
        "aux": aux,
    }

    total_floats = (
        len(platform) + len(mount) + len(mount) + len(camera) + len(aux)
    )
    assert total_floats == 24, f"Expected 24 floats, got {total_floats}"
    return packet


def post_telemetry(base_url: str, packet: dict) -> None:
    """Fast REST fallback send."""
    url = f"{base_url.rstrip('/')}/webrtc/telemetry"
    try:
        HTTP_SESSION.post(url, json=packet, timeout=2)
    except Exception:
        # Silent on hot path
        pass


async def run_relay(vehicle: Vehicle, base_url: str, hz: float) -> None:
    """Main telemetry loop."""
    interval = 1.0 / max(hz, 1.0)
    use_webrtc = os.getenv("USE_WEBRTC", "1") != "0"

    data_channel = None
    dc_open = False
    session_id = None

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
                json={
                    "sdp": pc.localDescription.sdp,
                    "type": pc.localDescription.type,
                },
                timeout=5,
            ).json()
            session_id = resp.get("session_id")
            if session_id:
                print(f"\n[webrtc] Session ready: {session_id}")
                print(f'    Run: window.connectTelemetry("{session_id}")\n')
        except Exception as exc:
            print(f"[webrtc] Offer failed: {exc}")

    print(f"[relay] Streaming at {hz} Hz via REST (DC when ready)")

    start_time = time.time()
    packet_count = 0

    try:
        while True:
            packet = build_telemetry_packet(vehicle)
            packet_count += 1

            # Always send REST (reliable)
            post_telemetry(base_url, packet)

            # Send on DataChannel if available
            if dc_open and data_channel and data_channel.readyState == "open":
                try:
                    data_channel.send(json.dumps(packet))
                except Exception:
                    pass

            # Light status every 2 seconds
            now = time.time()
            if now - start_time > 2.0:
                method = "REST+DC" if dc_open else "REST"
                alt = packet["platform"]["alt"]
                actual_hz = packet_count / 2.0
                print(f"[{method}] target={hz}Hz  actual≈{actual_hz:.1f}Hz  alt={alt:.1f}")
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

    print(f"[relay] Connecting to vehicle at {connect_str}...")
    vehicle = connect(connect_str, wait_ready=True, rate=30)
    print("[relay] Vehicle connected")

    try:
        await run_relay(vehicle, base_url, max(hz, 1))
    finally:
        vehicle.close()
        print("[relay] Disconnected")


if __name__ == "__main__":
    asyncio.run(main())