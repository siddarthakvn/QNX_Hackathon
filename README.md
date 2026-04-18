# GestSense

**Emergency alerts triggered by gestures, dispatched by QNX.**

Built for **Q-eHACK 2026** — BlackBerry QNX Educational Hackathon
Team Ignite · B V Raju Institute of Technology

---

## What it does

Stand in front of a camera. Make one of four emergency gestures. Hold it for 3 seconds.

A real-time system running on QNX Neutrino instantly:
- Recognizes the gesture (Ambulance, Police, Fire, or Distress)
- Reads live environmental data from sensors
- Fires a buzzer and LED pattern on the physical hardware
- Sends a detailed alert — with GPS and sensor context — to a dispatch dashboard
- Routes the alert to the right department, and cascades to multiple departments for Fire and Distress

No phone. No panic button. No voice call.

---

## The four gestures

| Gesture | Pose | Dispatches To |
| --- | --- | --- |
| **Ambulance** | Cross arms over chest | Medical |
| **Police** | Hands up, spread wide | Police |
| **Fire** | T-pose (arms out) | Fire + Medical + Police |
| **Distress** | Hands behind head | Rescue + Medical + Police |

Each gesture must be held for 3 seconds to prevent false alarms.

---

## How the pieces fit together

```
┌─────────────┐     UDP      ┌────────────────┐     UDP     ┌────────────┐
│  Windows PC │ ───────────▶ │   QNX Pi 4     │ ──────────▶ │  Dashboard │
│             │   :5005      │                │   :5006     │ (browser)  │
│  Camera     │              │  3 daemons     │             │            │
│  YOLOv8     │ ◀─────────── │  Real-time     │ ◀────────── │  React UI  │
│  MediaPipe  │   CANCEL     │  dispatch      │   :5007     │  + ACK     │
└─────────────┘              └────────────────┘             └────────────┘
     "eyes"                     "brain + body"                  "screen"
```

**Three computers, three jobs.** The PC has the GPU for AI. The Pi runs QNX for real-time dispatch. The browser is for operators.

---

## The QNX side — three daemons

Each daemon runs as a separate process. If one crashes, the others keep working.

### 1. `sensor_daemon`
Reads the BME688 temperature and gas sensor over I²C. Reads GPS coordinates from the NEO-M8N over UART. Publishes the latest snapshot via a QNX named channel.

### 2. `alert_daemon`
Listens for gesture alerts on UDP port 5005. For each alert:
- Queries `sensor_daemon` for the current snapshot
- Tells `io_daemon` to start the matching buzzer pattern
- Applies cascade routing rules (Fire → also dispatch Medical + Police)
- Forwards everything to the dashboard
- Measures end-to-end latency with `ClockCycles()`
- Runs inside a QNX Adaptive Partition with a guaranteed 40% CPU budget

### 3. `io_daemon`
Drives the physical LED and buzzer patterns on the Raspberry Pi GPIO. Each gesture has its own distinct pattern. Receives a `PULSE_ACK` to stop the pattern instantly when an alert is cancelled.

---

## Why QNX?

Emergency dispatch is safety-critical. It needs:

- **Deterministic timing** — the buzzer must fire within a known time bound, every time
- **Fault isolation** — a crash in one component cannot take down the system
- **Priority inheritance** — to prevent the classic Mars Pathfinder bug
- **Certification** — QNX Neutrino is ISO 26262 / IEC 61508 certifiable

Linux cannot give us hard guarantees on any of these. QNX was designed for exactly this class of problem.

---

## Priorities (SCHED_FIFO unless noted)

| Priority | Owner | Why |
| --- | --- | --- |
| 25 | UDP receiver | Network ingress must never queue up |
| 23 | ACK receiver | Stop the alert fast |
| 22 | Buzzer/LED pattern | Timing-sensitive output |
| 21 | IO message loop | Receives IPC from alert_daemon |
| 15 | Sensor threads (RR) | Environmental data, 3s polling is fine |
| 5 | Metrics (OTHER) | Informational only, can be starved |

---

## Measured latency

| Path | Latency |
| --- | --- |
| UDP packet → sensor snapshot | ~ 40 µs |
| Sensor snapshot → io dispatch | ~ 30 µs |
| End-to-end dispatch | < 200 µs |
| Cancel → buzzer off | < 1 ms |

Measured using QNX's `ClockCycles()` CPU cycle counter.

---

## Cancel flow

A false alarm must stop instantly. Two paths, same kernel action:

- **Person cancels** — closed fist held for 2 seconds
- **Operator cancels** — click ACKNOWLEDGE on the dashboard

Both paths converge on `MsgSendPulse(io_daemon, PULSE_ACK)`. Buzzer stops. Green LED turns on. Dashboard clears.

---

## Hardware

- Raspberry Pi 4 running QNX Neutrino 8.0
- Bosch BME688 on I²C (temperature, humidity, gas resistance)
- u-blox NEO-M8N GPS on UART `/dev/ser1`
- Red LED on GPIO 17, Green LED on GPIO 27, Buzzer on GPIO 19

---

## Repo layout

```
gestsense-qnx/
├── qnx/
│   ├── sensor_daemon/src/       sensor_daemon.c, gestsense.h
│   ├── io_daemon/src/           io_daemon.c, gestsense.h
│   └── alert_daemon/src/        alert_daemon.c, gestsense.h
│
├── ai/
│   ├── app.py                   YOLOv8 + MediaPipe gesture recognition
│   └── requirements.txt
│
├── dashboard/
│   ├── server.py                WebSocket bridge
│   └── dashboard.html           React UI
│
└── docs/
    ├── architecture.png
    ├── flowchart.png
    └── screenshots/
```

---

## How to run

### 1. QNX daemons on the Pi

Import each of the three projects into QNX Momentics. Build each one. Then run in this order:

```
./sensor_daemon &
./io_daemon &
./alert_daemon &
```

### 2. AI client on the PC

```
cd ai
pip install -r requirements.txt
python app.py
```

Edit the `QNX_HOST` value at the top of `app.py` to point at your Pi's IP address.

### 3. Dashboard

```
cd dashboard
pip install websockets
python server.py
```

Open `http://localhost:8080` in a browser.

### 4. Try it

Stand in front of the camera. Hold the T-pose for 3 seconds. Watch the buzzer fire, the dashboard light up with three alerts (Fire + Medical + Police), and check the QNX logs.

To cancel — hold a closed fist for 2 seconds, or click ACKNOWLEDGE on the dashboard.

---

## UDP protocol

| Port | Direction | What |
| --- | --- | --- |
| 5005 | AI → QNX | Gesture alerts and cancels |
| 5006 | QNX → Dashboard | Alerts forwarded to operators |
| 5007 | Dashboard → QNX | Operator acknowledgements |

All payloads are flat JSON with fields like `type`, `gesture`, `person`, `alert`, `timestamp`, and sensor context.

---

## What the code actually uses

- **POSIX pthreads** for threading (mutex, condition variable, priority scheduling)
- **QNX name service** (`name_attach`, `name_open`) for daemon discovery
- **QNX synchronous messages** (`MsgSend`, `MsgReceive`, `MsgReply`) for inter-daemon calls
- **QNX pulses** (`MsgSendPulse`, `PULSE_ACK`) for asynchronous cancel
- **QNX Adaptive Partitioning** (`SchedCtl`) for guaranteed CPU budget
- **POSIX sockets** for UDP networking
- **QNX `devctl`** for I²C access
- **QNX `mmap_device_memory`** for direct GPIO register control

---

## Team

**Team Ignite**
- KVN Sai Siddartha — 23211A04B5
- K Vishnu Varshith — 23211A0498

B V Raju Institute of Technology, Hyderabad

---

## License

MIT
