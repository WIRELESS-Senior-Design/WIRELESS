# W.I.R.E.L.E.S.S.

**Wireless Integrated Remote Embedded Low-Energy Smart System**

W.I.R.E.L.E.S.S. is a custom low-frequency wireless home automation platform developed for Senior Design.  
The system implements a layered communication architecture combining embedded firmware, digital signal processing, and modular wireless nodes.

---

## Overview

The platform consists of:

### 🔹 Central Hub
- Manages device pairing and network control  
- Handles transmit/receive processing  
- Provides user interaction interface  
- Runs real-time task scheduling  

### 🔹 Wireless Nodes
- Low-power endpoint devices  
- Actuator control (lighting, locking mechanisms, automation devices)  
- Event-driven firmware  
- Custom protocol decoding  

### 🔹 Custom Communication Protocol
- Compact fixed-length frame structure  
- Device ID + Command segmentation  
- Manchester encoding for reliable edge-based decoding  
- Deterministic timing capture  

---

## Architecture Philosophy

- Layered communication model (Physical → Data Link → Application)
- Deterministic edge-based signal detection
- Modular firmware design
- Hardware abstraction for scalability
- Version-controlled engineering workflow

---

## Repository Structure

```text
firmware/
├── hub/
│   ├── core/            # RF, protocol handling
│   ├── ui/              # Display and input handling
│   └── shared/          # Hub-level utilities
│
├── node/                # Wireless node firmware
├── shared/              # Shared protocol definitions
│
hardware/
├── pcb/
└── fpga/
│
software/
├── website/
└── tools/
│
docs/
├── architecture/
├── protocol/
└── meetings/
```
