# Autonomous Multi-Sensor Human Tracking System
<img width="1086" height="1448" alt="Turret" src="https://github.com/user-attachments/assets/2d85b5f4-60f8-4771-b67e-f65d6f5900d0" />

AMHTS detects, verifies, and tracks live human targets using a sensor fusion network of AI edge vision, thermal imaging, and mmWave radar. It runs on a dual-MCU architecture that separates sensor fusion management and decision-making (ESP32-S3) from low-level motor execution (Arduino UNO R3), with the two communicating over UART protocol.

A HuskyLens edge vision module acts as the primary tracking sensor using onboard human-recognition neural networks. An MLX90640 thermal camera verifies that a tracked target is a live human by reading its heat signature, and automatically takes over tracking if the HuskyLens loses its target. An HLK-LD2450 mmWave radar serves as secondary verification for the MLX90640 when it takes over tracking. The system is fully self-contained and portable with a dedicated LiPo battery.

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
<img width="709" height="675" alt="image" src="https://github.com/user-attachments/assets/bb350649-c60e-41c1-b882-080c40900e72" />


### ESP32-S3 — Sensor Fusion & Brain
The ESP32-S3 is the central processing unit. It handles every sensor interface, runs all tracking/fusion logic, and issues final angle commands to the Arduino.

It manages four sensor peripherals concurrently, each on its own bus:
| Sensor | Interface | Notes |
|---|---|---|
| HuskyLens | I2C Bus 0 | Onboard human-recognition model |
| MLX90640 | I2C Bus 1 @ 1MHz | 16Hz checkerboard readout, 16-bit ADC |
| LD2450 Radar | UART1 @ 256000 baud | 1024-byte RX buffer |
| DHT22 | GPIO Pin | Ambient chassis temp |

**Main loop:** non-blocking timing structure, sensor polling every 50ms (~20Hz fusion cycle). DHT22 readings are separate because it's updating speed is significantly slower than the rest of the sensors.

**Sensor processing per cycle:**
- **HuskyLens** — requests a packet, filters for the set human ID of 1, converts the reported pixel center into an offset from frame center, computes horizontal/vertical angles using calculated focal length.
- **MLX90640** — pulls a frame, scans for flagged pixels, computes a weighted centroid, derives its angular offset, and streams the full raw frame to the dashboard for a live heatmap.
- **LD2450** — reads up to 3 targets per cycle, converts raw X/Y byte data into corrected coordinates, derives distance and angle.

**Arduino handoff:** after each HuskyLens/MLX cycle, the ESP32-S3 sends a framed UART message over UART2 to the Arduino. Format: a letter header for sensor source (`H`/`M`/`L`), a 1 or 0 to denote whther or not a target was detected, then computed X/Y angles. The Arduino only ever receives final angle data — it has no direct interaction with any sensor.

### Arduino — Motion Controller
The Arduino holds no sensor-fusion logic. It executes movement from target data relayed by the ESP32-S3, and independently manages homing and search behavior when no target is present.

**Communication:** receives data over a SoftwareSerial link. Incoming lines are parsed by prefix, each updating its own sensor's target found flag and angle data. Each sensor source has an independent 5-second timeout. If no update arrives in that window, its target flag resets to false.

**Motion control:** two NEMA17 steppers driven via AccelStepper in STEP/DIR mode:
- **Stepper 1** — Pan axis
- **Stepper 2** — Tilt axis

Target angles are converted to motor steps using calibrated constants derived from gear ratios and TMC2209 specs — **7.0 steps/degree** (pan), **6.0317 steps/degree** (tilt). Movement is issued as relative steps, only when the incoming angle has changed from the last commanded angle, avoiding redundant `move()` calls.

**Homing & search:** on startup, Stepper 1 (pan) runs continuously until the limit button triggers, marking horizontal origin. Stepper 2 (tilt) performs a fixed 90° CCW relative move to mark vertical origin. Homing runs once per boot and the Arduino ignores all ESP32-S3 communication until it completes. Once homed, if no target is confirmed from any sensor packet, the turret runs a preset rotation — cycling Stepper 1 through 8 fixed equidistant positions across 360 degrees, pausing 2 seconds at each, while continuously checking for new angle data so it can break out the instant a target is acquired.

---

## Sensor Fusion & Tracking
<img width="709" height="731" alt="image" src="https://github.com/user-attachments/assets/6bb40b10-edf6-4d0e-a314-7b8153e5e798" />

*This was testing for the HuskyLens ability to report coordinates to the MCU, the MLX90640 was turned off.

### HuskyLens — Primary Tracker
<img width="638" height="430" alt="image" src="https://github.com/user-attachments/assets/a2a10236-64a9-41b3-bb09-b8845c493baa" />


Onboard neural networks for object, facial, and target recognition/tracking, built on a 320x240 OV2640 camera. Processes frames internally and outputs 10–16 byte packets with target coordinates and ID. It's the primary tracking membrane, carrying priority over every other sensor. Reported coordinates and it's computed focal length are used to derive rotation angles to center the target. Its output is cross-referenced against the thermal camera as a confirmation gate, verifying the tracked target is a live human. Because it depends on consistent lighting, it's vulnerable to failure in darkness — in which case tracking falls back to the thermal system.


### MLX90640 — Thermal Fallback Tracker
<img width="464" height="328" alt="image" src="https://github.com/user-attachments/assets/92b8d510-ffaa-49ff-9dbd-4323d4543954" />

A 768-pixel IR array outputting 16-bit readings per pixel. Uses a checkerboard readout system updating alternating pixels at 16Hz to balance smooth visuals with latency. The MCU flags any pixel within the algorithm's threshold of **32.87°C–40.43°C**. Average human body temp is 36.1–37.2°C but calibration testing found a margin of error from ambient conditions and target distance of roughly ±3.23°C, which set the final threshold range. Flagged pixels are used to compute an intensity-weighted center of mass and its mean X/Y coordinates plus computed focal length determine relative target angles. This system only activates if the HuskyLens fails to send new data for 3+ seconds, so the two never interfere.


### LD2450 — Radar Confirmation Gate
<img width="465" height="270" alt="image" src="https://github.com/user-attachments/assets/fa1a4976-3ccc-47f3-a6fe-bc2118784cb1" />

An FMCW (Frequency Modulated Continuous Wave) mmWave sensor. Emits high-frequency radio waves and analyzes frequency shifts and phase delays of returning signals, natively running Range-FFT and Angle-of-Arrival processing to derive spatial coordinates for up to 3 targets. In the tracking algorithm it's used solely as a confirmation gate for the thermal system — since radar is unaffected by lighting, it's well suited to verify thermal readings. Its angular data proved unreliable during testing, so the algorithm doesn't use its spatial data at all; instead it's used purely for target verification and distance reporting.

---

## Power System

<img width="1073" height="934" alt="image" src="https://github.com/user-attachments/assets/08208a03-4227-4588-9186-f42233e4915e" />
<img width="483" height="329" alt="image" src="https://github.com/user-attachments/assets/a97f5ae7-d138-41c2-8b76-942f6f83f6ca" />

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
