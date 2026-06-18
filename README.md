# dkrelay

DroneKit telemetry relay for streaming vehicle data to a web-based 3D viewer.

## Features

- Streams canonical **24-float telemetry packets**
- Supports both **REST fallback** and **WebRTC DataChannel** for low-latency delivery
- Gimbal pitch support via servo output monitoring (channel 5 by default)
- Two relay implementations:
  - `relay_telem.py` — async version
  - `relay_telem_sync.py` — synchronous/threaded version (simpler integration)

## Quick Start

```bash
# Copy environment settings
cp .env.example .env

# Run the synchronous relay (recommended)
python relay_telem_sync.py

# Or run the async version
python relay_telem.py
```

### Environment Variables

| Variable                | Default                    | Description |
|-------------------------|----------------------------|-----------|
| `DRONEKIT_CONNECTION`   | `tcp:127.0.0.1:5763`       | Connection string for the vehicle |
| `RELAY_BASE`            | `http://localhost:9001`    | Base URL of the telemetry receiver |
| `TELEMETRY_HZ`          | `24`                       | Telemetry rate in Hz |
| `USE_WEBRTC`            | `1`                        | Set to `0` to disable WebRTC |

## How It Works

The relay connects to a vehicle via DroneKit, reads position, attitude, and servo output, then forwards a standardized 24-float telemetry packet to the viewer backend.

- **REST path** always works immediately.
- **WebRTC DataChannel** can be enabled for lower latency (requires a browser console step to complete signaling).

See [RELAY_README.md](RELAY_README.md) for the full telemetry schema, signaling protocol, and integration details.

## Requirements

```bash
pip install -r requirements.txt
# or with uv
uv sync
```

## License

MIT