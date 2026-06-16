# Relay Telemetry to the 3D Viewer

This document explains how to push live telemetry into the viewer from an external source (SITL, custom simulator, flight controller, ROS node, etc.).

The viewer consumes telemetry using a simple protocol built on:
- **WebRTC DataChannel** (named `"telemetry"`) – preferred for low-latency 15–60 Hz updates
- **REST fallback** – HTTP POST to `/webrtc/telemetry` (no WebRTC required)

The **external program** is always the **offerer**. The browser is the **answerer**.

---

## 1. Telemetry Data Model (Exactly 24 Floats)

Every telemetry packet must contain **exactly 24 floating-point numbers** in total.

The canonical structure is:

```json
{
  "platform": {
    "lat":  float,   // 1
    "lon":  float,   // 2
    "alt":  float,   // 3
    "roll": float,   // 4
    "pitch":float,   // 5
    "yaw":  float    // 6
  },
  "mount1": {
    "roll":  float,  // 7
    "pitch": float,  // 8
    "yaw":   float   // 9
  },
  "mount2": {
    "roll":  float,  // 10
    "pitch": float,  // 11
    "yaw":   float   // 12
  },
  "camera": {
    "fov":  float,   // 13
    "mode": float    // 14   (use 0.0 when not applicable)
  },
  "aux": {
    // Any additional float values. You may choose your own keys.
    // The total number of float leaf values across the entire packet must be 24.
    // Example:
    "aux_00": 0.0,              // 16
    "aux_01": 0.0,
    // ... continue until you reach exactly 24 floats total
    // Leave as 0.0s when building new, let user customize
  }
}
```

**Rules**
- `platform` is absolute WGS84 position + body attitude.
- `mount1` / `mount2` are **relative** to the platform (gimbal / sensor attitudes).
- All values in `aux` **must be floats**.
- You are free to put any auxiliary data you want in `aux`, as long as the grand total across the whole object is **exactly 24 floats**.

The REST ingestion endpoint and the DataChannel both accept this exact shape.

---

## 2. High-Level Architecture

```
External Producer (your SITL / sim / script)
          │
          ├── WebRTC signaling (REST)
          │     offer → answer → ICE
          │
          └── DataChannel "telemetry"  (JSON at TELEMETRY_HZ)
                    ↓
            FastAPI backend
                    ↓
            Browser (Cesium) via WebRTC or REST poll fallback
```

If you do not want to implement WebRTC, you can simply POST the same JSON shape to:

```
POST /webrtc/telemetry
Content-Type: application/json
```

The browser also auto-starts a polling fallback that reads the latest value from this endpoint.

---

## 3. Configuration

The backend and frontend read the desired rate from the environment variable:

```
TELEMETRY_HZ=20
```

- Start the backend with this variable set (or in `.env`).
- Your relay script should also read `TELEMETRY_HZ` (or hard-code a reasonable value).
- The frontend will start its REST fallback polling at approximately this rate.

Query the current rate at any time:

```
GET /api/config
```

The response contains the `telemetryHz` field.

---

## 4. Signaling Flow (WebRTC Path) — Detailed

The signaling is intentionally minimal and uses only REST calls. There is **no** WebSocket or third-party signaling server.

**Important rule**: Your external program is always the **offerer**. The browser is always the **answerer**.

### Producer-side steps (what your script must do)

1. **Create peer connection + DataChannel**
   - Create `RTCPeerConnection` (include at least Google STUN).
   - **Create the DataChannel named exactly `"telemetry"` *before* you create the offer**:
     ```python
     channel = pc.createDataChannel("telemetry")
     ```

2. **Send the offer**
   ```http
   POST /webrtc/offer
   Content-Type: application/json

   {
     "sdp": "v=0\r\no=- ...",
     "type": "offer"
   }
   ```

   Response:
   ```json
   { "session_id": "a1b2c3d4e5f6" }
   ```

3. **Trickle ICE candidates** (as they are discovered)
   ```http
   POST /webrtc/ice/{session_id}
   {
     "candidates": [ { "candidate": "...", "sdpMid": "...", ... } ]
   }
   ```

   Send candidates as soon as your `onicecandidate` handler fires.

4. **Poll for the browser's answer**
   ```http
   GET /webrtc/session/{session_id}
   ```

   Keep polling (e.g. every 300–500 ms) until the response contains an `"answer"`.

   Example response once the browser has answered:
   ```json
   {
     "offer": "...",
     "answer": "v=0\r\no=- ...",
     "ice_candidates": [ ... ]
   }
   ```

5. **Apply the answer**
   ```python
   pc.setRemoteDescription({ "type": "answer", "sdp": session_data["answer"] })
   ```

6. **Send telemetry** once the DataChannel is open
   ```python
   if channel.readyState == "open":
       channel.send(json.dumps(your_24_float_packet))
   ```

You can (and should) **continue posting to `/webrtc/telemetry`** as a fallback the entire time, including while waiting for the browser handshake. This is what the reference `sender_test.py` does.

### The Mandatory Browser Trigger

This is the part that surprises most implementers:

The browser does **not** automatically accept your offer. A human (or your own UI button) must run this command in the browser console:

```js
window.connectTelemetry("a1b2c3d4e5f6")
```

Only after this call does the browser:
- Fetch your offer from `/webrtc/session/{id}`
- Create and post its answer to `/webrtc/answer/{id}`
- Set up the receiving DataChannel

Until someone runs that line, the DataChannel will never open (but your REST fallback will still work).

---

## 5. Full Signaling Protocol Reference (Required to Implement)

### Endpoints you must use (producer side)

| Step | Method | Path                        | Purpose | Request Body | Response |
|------|--------|-----------------------------|---------|--------------|----------|
| 1    | POST   | `/webrtc/offer`             | Start session + send SDP offer | `{ "sdp": "...", "type": "offer" }` | `{ "session_id": "..." }` |
| 2    | POST   | `/webrtc/ice/{session_id}`  | Send your ICE candidates | `{ "candidates": [ ... ] }` or single candidate | `{ "status": "ok" }` |
| 3    | GET    | `/webrtc/session/{session_id}` | Poll for browser answer + incoming ICE | — | `{ "offer": "...", "answer": "..." or null, "ice_candidates": [...] }` |
| —    | POST   | `/webrtc/telemetry`         | REST fallback (always safe to use) | Your 24-float telemetry dict | `{ "status": "stored" }` |

**Notes on payloads**:
- SDP is a plain string (the `sdp` field).
- ICE candidates can be sent as an array under `"candidates"` or as a single object. The backend accepts both.
- The `/webrtc/session/{id}` response always contains the original `offer` and any ICE candidates that have arrived from the browser.

### Complete Handshake Sequence (Offerer side)

1. Create `RTCPeerConnection`.
2. **Create DataChannel named exactly `"telemetry"` now** (before offer).
3. `createOffer()` → `setLocalDescription()`.
4. `POST /webrtc/offer` → save the `session_id` you get back.
5. Immediately start sending any locally generated ICE candidates to `POST /webrtc/ice/{id}`.
6. Poll `GET /webrtc/session/{id}` every 300–500 ms until you see a non-null `"answer"`.
7. `setRemoteDescription({type: "answer", sdp: ...})`.
8. Once the DataChannel fires `"onopen"`, you can send telemetry over it.

You can (and should) post telemetry via `/webrtc/telemetry` at every tick, even before the DataChannel opens.

---

## 6. What the Browser Does (So You Understand the Flow)

When someone runs this in the browser console:

```js
window.connectTelemetry("your-session-id")
```

The browser performs these steps:
- Fetches the offer via `GET /webrtc/session/{id}`.
- Creates its own `RTCPeerConnection`.
- Sets the remote description using your offer.
- Creates an **answer** and posts it to `POST /webrtc/answer/{id}`.
- Starts sending its own ICE candidates to `POST /webrtc/ice/{id}`.
- Sets up a receiver for the DataChannel named `"telemetry"`.

This is why the browser step is **mandatory** and not automatic.

---

## 7. Practical Example Pseudocode (Producer Side)

```python
import asyncio, json, os

BACKEND = "http://127.0.0.1:9001"
HZ = float(os.getenv("TELEMETRY_HZ", "20"))
INTERVAL = 1.0 / max(HZ, 1)

async def relay_telemetry():
    pc = create_peer_connection()  # include stun server
    channel = pc.create_data_channel("telemetry")  # CRITICAL: before offer

    # 1. Send ICE candidates as they appear
    pc.on_ice_candidate = lambda c: post_ice(session_id, c)

    # 2. Create offer
    offer = pc.create_offer()
    pc.set_local_description(offer)

    # 3. Post offer → get session id
    resp = post(f"{BACKEND}/webrtc/offer", {
        "sdp": offer.sdp,
        "type": "offer"
    })
    session_id = resp["session_id"]

    print(f"SESSION ID = {session_id}")
    print(f"Run in browser console: window.connectTelemetry('{session_id}')")

    # 4. Wait for answer (polling)
    while True:
        sess = get(f"{BACKEND}/webrtc/session/{session_id}")
        if sess.get("answer"):
            pc.set_remote_description({
                "type": "answer",
                "sdp": sess["answer"]
            })
            break
        await asyncio.sleep(0.4)

    # 5. Main streaming loop
    while True:
        packet = build_24_float_telemetry_packet()   # your source here

        # Always use REST fallback (works immediately)
        try:
            post(f"{BACKEND}/webrtc/telemetry", packet)
        except Exception:
            pass

        # Use DataChannel when ready (lower latency)
        if channel.ready_state == "open":
            try:
                channel.send(json.dumps(packet))
            except Exception:
                pass

        await asyncio.sleep(INTERVAL)

asyncio.run(relay_telemetry())
```

---

## 8. REST-Only Path (Easiest Starting Point)

If you want to get data flowing **without** implementing WebRTC at all:

Just do this in a loop:

```python
while True:
    packet = build_24_float_telemetry_packet()
    post(f"http://viewer-host:9001/webrtc/telemetry", packet)
    sleep(INTERVAL)
```

The viewer will automatically poll the latest value using the rate from `TELEMETRY_HZ`.

This is perfectly valid for development, debugging, or when WebRTC is too heavy for your use case. You can add the full WebRTC path later.

---

## 9. Integrating with Real Sources

You do **not** need to use the exact orbit simulation from `sender_test.py`.

Common patterns:

- **MAVLink / SITL (ArduPilot / PX4)**: Convert `GLOBAL_POSITION_INT`, `ATTITUDE`, and gimbal messages into the platform + mount1 fields.
- **ROS**: Subscribe to your odometry / pose topics and your gimbal joints.
- **Custom engine**: Grab the vehicle state from your simulator's API each tick and pack it into the schema.
- **Logged data**: Play back a file, emitting one packet per row at the original or scaled rate.

Only the final JSON shape and the rate matter.

You are free to:
- Use any language.
- Use any WebRTC library (aiortc, webrtc-rs, Go pion, etc.).
- Skip WebRTC entirely and just POST to the REST endpoint.

---

## 10. Useful Endpoints (Debugging)

| Method | Path                              | Purpose |
|--------|-----------------------------------|---------|
| POST   | `/webrtc/offer`                   | Start a new signaling session |
| POST   | `/webrtc/answer/{id}`             | Browser posts its answer |
| POST   | `/webrtc/ice/{id}`                | Exchange ICE candidates |
| GET    | `/webrtc/session/{id}`            | Poll for answer / ICE |
| POST   | `/webrtc/telemetry`               | REST fallback ingestion |
| GET    | `/webrtc/last-telemetry`          | See the most recent packet the backend received |
| GET    | `/api/config`                     | Get current `telemetryHz` |

---

## 11. Gotchas & Best Practices

- The DataChannel will **never** open until a human runs `window.connectTelemetry("...")` in the browser console.
- You **must** create the DataChannel before calling `createOffer()`.
- Always send via `POST /webrtc/telemetry` as a fallback, especially while waiting for the browser handshake.
- Backend only keeps the very last packet (for REST fallback).
- All leaf values must be floats and there must be exactly **24** of them.
- `mount1`/`mount2` rotations are **relative** to the platform.

---

## 12. Implementation Checklist

- [ ] Packets contain **exactly 24 floats**
- [ ] All aux values are floats
- [ ] DataChannel named exactly `"telemetry"` created **before** the offer
- [ ] Offer posted to `/webrtc/offer`
- [ ] ICE candidates sent as they appear
- [ ] Poll `/webrtc/session/{id}` until `answer` appears
- [ ] Apply answer with `setRemoteDescription`
- [ ] Send on DataChannel when ready **+** keep posting REST fallback every tick
- [ ] Instruct user to run `window.connectTelemetry(...)`
- [ ] Handle not-connected state gracefully (use REST fallback)

---

## 13. Reference Implementation

`backend/scripts/sender_test.py` is the canonical example that implements both the WebRTC path and the REST fallback correctly.

Use it as a reference when building your own relay in another language or environment.

---

This document tries to contain every detail you need to build a working telemetry sender against this backend from any environment. 

If something is still unclear, the two best debugging tools are:
- `GET /webrtc/last-telemetry` (see what the backend last received)
- Browser console logs after running `window.connectTelemetry(...)` (see signaling and packet arrival) 

If you have questions about a specific language or data source (MavSDK, ROS2, Unreal, custom C++ sim, etc.), feel free to ask.