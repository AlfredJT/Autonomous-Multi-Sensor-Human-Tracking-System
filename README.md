# STRIX — Autonomous Multi-Sensor Human Tracking Turret

<!-- ![STRIX Banner](docs/images/banner.jpg) -->

STRIX is an autonomous tracking turret that detects, verifies, and follows live human targets using a sensor fusion pipeline of computer vision, thermal imaging, and mmWave radar. It runs on a dual-MCU architecture that separates high-level sensor fusion and decision-making (ESP32-S3) from low-level motor execution (Arduino), with the two communicating over a custom serial protocol.

A HuskyLens vision module acts as the primary tracker using onboard human-recognition neural networks. An MLX90640 thermal camera verifies that a tracked target is a live human by reading its heat signature, and automatically takes over tracking if the HuskyLens loses its target. An HLK-LD2450 mmWave radar provides independent secondary confirmation to cut down false positives. The system is fully self-contained — dual-power design (12V LiPo or wall power) — with a live WebSocket dashboard for real-time monitoring of sensor health, tracking state, thermal imaging, and radar.

## Table of Contents
- [System Architecture](#system-architecture)
- [Sensor Fusion & Tracking](#sensor-fusion--tracking)
- [Power System](#power-system)
- [Cooling System](#cooling-system)
- [Bill of Materials](#bill-of-materials)
- [Engineering Challenges](#engineering-challenges)
- [Media / Demos](#media--demos)

---

## System Architecture

<!-- ![System Block Diagram](docs/images/system-diagram.jpg) -->

### ESP32-S3 — Sensor Fusion & Brain
The ESP32-S3 is the central processing unit. It owns every sensor interface, runs all tracking/fusion logic, hosts the live dashboard, and issues final angle commands to the Arduino.

It manages four sensor peripherals concurrently, each on its own bus:
| Sensor | Interface | Notes |
|---|---|---|
| HuskyLens | I2C Bus 0 | Onboard human-recognition model |
| MLX90640 | I2C Bus 1 @ 1MHz | 16Hz checkerboard readout, 16-bit ADC |
| LD2450 Radar | UART1 @ 256000 baud | 1024-byte RX buffer |
| DHT22 | — | Ambient chassis temp |

Each sensor is independently health-tracked by polling time-since-last-packet, streamed live to the dashboard as health bars.

**Main loop:** non-blocking timing structure, sensor polling every 50ms (~20Hz fusion cycle). Actual cycle duration is measured and streamed to the dashboard as live loop Hz / cycle ms. DHT22 reads and WebSocket client cleanup run on independent timers.

**Sensor processing per cycle:**
- **HuskyLens** — requests a packet, filters for the human ID, converts the reported pixel center into an offset from frame center, computes horizontal/vertical angles using calculated focal length.
- **MLX90640** — pulls a frame, scans for flagged pixels, computes a weighted centroid, derives its angular offset, and streams the full raw frame to the dashboard for a live heatmap.
- **LD2450** — reads up to 3 targets per cycle, converts raw X/Y byte data into corrected coordinates, derives distance and angle.

**Arduino handoff:** after each HuskyLens/MLX cycle, the ESP32-S3 sends a framed UART message over UART2 to the Arduino. Format: a letter header for sensor source (`H`/`M`/`L`), a 1/0 detection flag, then computed X/Y angles. The Arduino only ever receives final angle data — it has no direct interaction with any sensor.

**Dashboard:** ESPAsyncWebServer + AsyncWebSocket instance serving the dashboard HTML from PROGMEM, pushing live data over `/ws` using a small set of typed message prefixes, one per sensor.

### Arduino — Motion Controller
The Arduino holds no sensor-fusion logic. It executes movement from target data relayed by the ESP32-S3, and independently manages homing and search behavior when no target is present.

**Communication:** receives data over a SoftwareSerial link. Incoming lines are parsed by prefix, each updating its own sensor's target-found flag and angle data. Each sensor source has an independent 5-second timeout — if no update arrives in that window, its target-found flag resets to false.

**Motion control:** two NEMA17 steppers driven via AccelStepper in STEP/DIR mode:
- **Stepper 1** — Pan axis
- **Stepper 2** — Tilt axis

Target angles are converted to motor steps using axis-specific calibrated constants derived from gear ratios and TMC2209 specs — **7.0 steps/degree** (pan), **6.0317 steps/degree** (tilt). Movement is issued as relative steps, only when the incoming angle has changed from the last commanded angle, avoiding redundant `move()` calls.

**Homing & search:** on startup, Stepper 1 (pan) runs continuously until a limit button triggers, marking horizontal origin. Stepper 2 (tilt) performs a fixed 90° CCW relative move to mark vertical origin. Homing runs once per boot; the Arduino ignores all ESP32-S3 communication until it completes. Once homed, if no target is confirmed from any source, the turret runs a preset scan — cycling Stepper 1 through 8 fixed equidistant positions, pausing 2 seconds at each, while continuously checking for new angle data so it can break out the instant a target is acquired.

---

## Sensor Fusion & Tracking

<!-- ![Sensor Fusion Diagram](docs/images/sensor-fusion.jpg) -->

### HuskyLens — Primary Tracker
Onboard neural networks for object, facial, and target recognition/tracking, built on a 320x240 OV2640 camera. Processes frames internally and outputs 10–16 byte packets with target coordinates and ID. It's the primary tracking membrane, carrying priority over every other sensor. Reported coordinates plus a computed focal length are used to derive rotation angles to center the target. Its output is cross-referenced against the thermal camera as a confirmation gate, verifying the tracked target is a live human. Because it depends on consistent lighting, it's vulnerable to failure in darkness — in which case tracking falls back to the thermal system.

<!-- ![HuskyLens Setup](docs/images/huskylens.jpg) -->

### MLX90640 — Thermal Fallback Tracker
A 768-pixel IR array outputting 16-bit readings per pixel. Uses a checkerboard readout system updating alternating pixels at 16Hz to balance smooth visuals against noise/latency. The MCU flags any pixel within the algorithm's threshold of **32.87°C–40.43°C**. Average human body temp is 36.1–37.2°C; calibration testing found a margin of error from ambient conditions and target distance of roughly ±3.23°C, which set the final threshold range. Flagged pixels are used to compute an intensity-weighted center of mass; its mean X/Y coordinates plus computed focal length determine relative target angles. This system only activates if the HuskyLens fails to send new data for 3+ seconds, so the two never interfere.

<!-- ![Thermal Heatmap](docs/images/thermal-heatmap.jpg) -->

### LD2450 — Radar Confirmation Gate
An FMCW (Frequency Modulated Continuous Wave) mmWave sensor. Emits high-frequency radio waves and analyzes frequency shifts and phase delays of returning signals, natively running Range-FFT and Angle-of-Arrival processing to derive spatial coordinates for up to 3 targets. In the tracking algorithm it's used solely as a confirmation gate for the thermal system — since radar is unaffected by lighting, it's well suited to verify thermal readings. Its angular data proved unreliable during testing, so the algorithm doesn't use its spatial data at all; instead it's used purely for target verification and distance reporting.

<!-- ![Radar Dashboard View](docs/images/radar-view.jpg) -->

---

## Power System

<!-- ![Power Distribution](docs/images/power-system.jpg) -->

The turret supports two power sources:
- **Primary:** 12V LiPo battery into a custom distribution board that routes power to each subsystem.
- **Secondary:** wall outlet power via a salvaged AC-DC converter — a Samsung BN44-00989A PSU board, with output leads separated into power/ground and soldered to the custom distribution board. Steps 120V AC down to a stable 14.1V DC line.

From the 14.1V rail, two buck converters handle fine-tuning:
- **5V buck** — powers the ESP32-S3, HuskyLens, MLX90640, LD2450, DHT22, and the VREF sampling lines on both TMC2209 drivers
- **9V buck** — powers the custom driver board for the auxiliary cooling fans

The Arduino Uno R3, both buck converters, and both TMC2209 VMOT inputs connect directly to the 14.1V rail (the 5V rail is used only for TMC2209 logic sampling, not motor power). Estimated average consumption: **25–45 Wh**.

---

## Cooling System

<!-- ![Cooling System](docs/images/cooling-system.jpg) -->

Built around a custom dual-motor driver board controlling two KFK-180 motors (one intake, one exhaust). The driver uses an N-channel MOSFET (11N65M5, salvaged from an old printer) driven by a 2N2222A NPN transistor acting as a gate driver — level-shifting the MCU's 5V logic to switch a 9V gate supply. This enables PWM control over the higher-voltage motor supply while keeping full compatibility with low-voltage MCU logic. A DHT22 inside the chassis reports temperature to the ESP32-S3, which runs a dedicated algorithm for dynamic fan speed based on internal chassis temperature.

---

## Bill of Materials

| Component | Qty | Cost |
|---|---|---|
| Buck Converters | 2 | $1.60 each |
| ESP32-S3 | 1 | $10 |
| Arduino Uno R3 | 1 | $5 |
| TMC2209 Driver | 2 | $7.57 each |
| NEMA17 Stepper | 2 | $15 each |
| KFK-180 Motor | 2 | Salvaged |
| DHT22 | 1 | $6 |
| Cooling Fan Driver | 1 | Salvaged |
| MLX90640 | 1 | $66 |
| LD2450 | 1 | $14 |
| HuskyLens 1 | 1 | $35 |
| 12V LiPo Battery | 1 | $15 |
| AC-DC Converter | 1 | Salvaged |
| Slip Ring | 1 | $22 |
| Wires (~100) | — | $0.20 each |
| **Total** | | **$241.34** |

---

## Engineering Challenges

<!-- ![Build Process](docs/images/build-process.jpg) -->

**Motor torque limitations** — The original design used 28BYJ-48 steppers, which proved far too weak for the system's scale and load. Solved by redesigning the turret around NEMA17 steppers and TMC2209 drivers.

**Slip ring integration** — Mechanically integrating the slip ring was a serious challenge. Solved with a separate axle between the motor and the slip ring, using the motor axle to turn a gear connected to the main axle.

**I2C bus conflicts** — HuskyLens and MLX90640 originally shared a single I2C bus. The bus speed required for stable MLX90640 reads interfered with HuskyLens communication, and a HuskyLens failure could take the entire bus down. Fixed by splitting them onto fully separate I2C buses, isolating each sensor's failure domain.

**MOSFET triggering** — The salvaged high-switching MOSFET on the cooling driver board wouldn't trigger reliably at low gate voltages, but excessive gate voltage introduced audible motor noise. Resolved by tuning the buck converter output to the minimum voltage that reliably switched the MOSFET while keeping noise low.

**Manufacturing tolerances** — FDM printing on the Bambu Lab P1S introduced small but consequential dimensional deviations on mating parts. Resolved through iteration — dialing in print settings and designing in tolerances to reliably account for the printer's inconsistency.

---

## Media / Demos

<!-- Drop build photos, dashboard screenshots, and demo clips/gifs here -->
