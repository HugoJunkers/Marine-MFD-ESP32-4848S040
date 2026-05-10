<h1 align="center">⛵ Marine MFD – NMEA2000 Smart Display</h1>
<p align="center">
  <strong>Professional boat instrument panel – built from a modified <code>ESP32-4848S040</code> touch display</strong><br/>
  Full NMEA2000 integration, LVGL graphics, Raymarine EV‑1 autopilot control
</p>

<p align="center">
  <img src="https://img.shields.io/badge/version-0.5.2-blue.svg" alt="Version"/>
  <img src="https://img.shields.io/badge/LVGL-8.3-green.svg" alt="LVGL"/>
  <img src="https://img.shields.io/badge/NMEA2000-✔-brightgreen.svg" alt="NMEA2000"/>
  <img src="https://img.shields.io/badge/Hardware-ESP32--4848S040-orange.svg" alt="Hardware"/>
  <img src="https://img.shields.io/github/issues/HugoJunkers/Marine-MFD-ESP32-4848S040" alt="GitHub issues"/>
  <img src="https://img.shields.io/github/discussions/HugoJunkers/Marine-MFD-ESP32-4848S040" alt="GitHub discussions"/>
</p>

## ⚠️ Important Safety Notice & Disclaimer

> **This project is experimental!**  
> The software controls safety‑critical systems (autopilot) and provides navigation data. A software bug, an ESP32 crash, or CAN/Wi‑Fi issues could cause the boat to steer uncontrollably or display incorrect depth readings.
>
> * **Never** use this as your primary navigation system.
> * **Always** keep the autopilot’s physical control panel within reach to force standby mode in an emergency.
> * Use of this project is entirely **at your own risk**. The author accepts no liability for damage to vessels, people, or property.

---

## 📌 Project Overview

This project turns a **low‑cost 4‑inch touch display** (ESP32‑4848S040, ~25€ on AliExpress) into a fully functional **Marine Multi‑Function Display** for your boat. The device connects directly to the NMEA2000 network (CAN bus) and replaces multiple analog instruments with a single, modern touch interface.

The original consumer display (intended for home automation) is heavily modified: the 220 V power supply and relay section are removed, a rugged 12 V→5 V DC‑DC converter is added, and a CAN transceiver is wired to the ESP32’s GPIOs. After flashing the provided firmware, you get a professional NMEA2000 gateway with 11 instrument screens, autopilot control, waypoint navigation, and data age colour coding.

---

## ✨ Key Features

| Category             | Details                                                                                   |
|----------------------|-------------------------------------------------------------------------------------------|
| **Instrument pages** | Depth, SOG, STW, Water temp, Heading (analog compass + digital), Wind (AWS/AWA & TWS/TWA) |
| **Wind screen**      | Switchable apparent/true mode, colour‑coded needle + text, damping for smooth animation  |
| **Autopilot**        | Full Raymarine EV‑1 support: STBY, AUTO, WIND, TRACK, ±1°/±10°, Tack Port/Starboard      |
| **Navigation data**  | Waypoint name, DTW (nm), TTG (minutes) – displayed on TRACK page                         |
| **Setup page**       | LED brightness slider, AP simulation (test without real network), firmware version       |
| **Data quality**     | Fresh data = teal, old = muted yellow, stale/invalid = grey                               |
| **Touch navigation** | Horizontal swipe = next/previous page, vertical swipe up = Home, down = Autopilot        |
| **Watchdog**         | Monitors LVGL task and NMEA stack; auto‑reboot on freeze                                  |
| **NMEA2000 Rx/Tx**   | Sends key commands; listens to all relevant PGNs (129025, 130306, 127250, 65379, …)      |
| **Power supply**     | Single cable solution – 5‑wire NMEA2000 provides both data and 12V power                 |

---

## 🔧 Hardware Details (ESP32-4848S040)

- **Display Model**: ESP32-4848S040 (also known as *JCZN Guition ESP32-4848S040*)
- **Processor**: ESP32-S3-WROOM-1 (ESP32-S3 with 16 MB Flash and 8 MB PSRAM)
- **Display Driver**: ST7701 (480×480 pixel, RGB interface)
- **Touch Controller**: GT911 (capacitive, I²C, address 0x5D)
- **Backlight**: PWM via GPIO38

---

## 🔌 NMEA2000 Connection (CAN Bus)

| Component                  | Description                                                                                   |
|----------------------------|-----------------------------------------------------------------------------------------------|
| **Connector Type**         | 5‑pin Micro‑C connector (M12 × 1.0, A‑coded) – **male panel mount connector** with screw thread |
| **Standard**               | Device and cable connector according to DeviceNet / CANopen specification                     |
| **Example Part**           | *NMEA2000 male connector / Micro‑C* (e.g., SVB article number 61045) 
								or way cheaper – search for: "M12 5 pin male panel mount connector NMEA 2000" |

---

## ⚠️ Known Software Limitations (Please Read Carefully!)

> **To avoid disappointment:** The following limitations have not been fixed yet. If you cannot accept them, please do not build this project.

| Issue                                      | Description                                                                                   |
|--------------------------------------------|-----------------------------------------------------------------------------------------------|
| **Horizontal display flicker**             | Occasional horizontal flickering, especially when widgets are overlapped.                     |
| **Incomplete screen refresh (ghosting)**   | Sometimes remnants of previous content remain visible – leftovers of grids or buttons.       |

**Root Cause:**  
The ESP32‑4848S040 uses an RGB display without its own graphics memory (GRAM). The ESP32 must continuously stream image data. When LVGL writes to a smaller buffer, it gets copied to the full‑frame buffer – if this copy process is not perfectly controlled, the described artefacts appear.

**Solution Approach (not included in this project):**  
A complete fix would require deep modifications to the ESP32’s RGB driver to manually manage the copy process and only display the full buffer after rendering is complete. This is complex and has not been implemented here.

> **Bottom Line:** If occasional visual glitches bother you or you have no patience for such artefacts, this project is not for you.

---


## 📱 Screen Gallery

<p align="center">
  <img src="images/Marine%20Touch%20MFD%20ESP32%2000001.jpeg" width="600"/>
</p>

<p align="center">
  <img src="images/Marine%20Touch%20MFD%20ESP32%2000002.jpeg" width="600"/>
</p>

<p align="center">
  <img src="images/Marine%20Touch%20MFD%20ESP32%2000003.jpeg" width="600"/>
</p>

<p align="center">
  <img src="images/Marine%20Touch%20MFD%20ESP32%2000004.jpeg" width="600"/>
</p>

<p align="center">
  <img src="images/Marine%20Touch%20MFD%20ESP32%2000005.jpeg" width="600"/>
</p>

<p align="center">
  <img src="images/Marine%20Touch%20MFD%20ESP32%2000006.jpeg" width="600"/>
</p>

<p align="center">
  <img src="images/Marine%20Touch%20MFD%20ESP32%2000007.jpeg" width="600"/>
</p>

<p align="center">
  <img src="images/Marine%20Touch%20MFD%20ESP32%2000008.jpeg" width="600"/>
</p>

<p align="center">
  <img src="images/Marine%20Touch%20MFD%20ESP32%2000009.jpeg" width="600"/>
</p>

<p align="center">
  <img src="images/Marine%20Touch%20MFD%20ESP32%2000010.jpeg" width="600"/>
</p>

<p align="center">
  <img src="images/Marine%20Touch%20MFD%20ESP32%2000011.jpeg" width="600"/>
</p>


## 🛠️ Hardware Modifications – ESP32‑4848S040

The base unit is an **ESP32‑4848S040** (480×480, capacitive touch, originally with a 220 V → 5 V power supply and relays). We transform it into a marine‑ready NMEA2000 device.

### 🔧 Required Modification Steps

1.  **Remove all unnecessary components**
    - Relay(s)
    - 220 V AC to 5 V DC converter board
    - Keep only the 6‑pin connector that links to the mainboard.
    
1a. **Use a knife or a small milling cutter to cut some of the original traces in order to reuse them for soldering the diode and a couple of wires – checkout the modification pictures**

2.  **Install a 12 V → 5 V DC‑DC converter** (3 A capable)
    - Powers the ESP32, display backlight, and CAN transceiver.

3.  **Add a male NMEA2000 connector** (Micro‑C style) to the enclosure back.

4.  **Route four wires** from the NMEA2000 plug to the mainboard:
    - **+12 V** (red) → DC‑DC converter input
    - **GND** (black) → DC‑DC converter input & ESP32 GND
    - **CAN‑L** (blue) → transceiver CANL
    - **CAN‑H** (yellow) → transceiver CANH

5.  **Cut the TX or RX trace** on the mainboard to obtain a 3.3 V rail for the CAN transceiver.
    - Repurpose the former RX pin as a **3.3 V output** from the ESP32’s onboard regulator.

6.  **Install a WCMCU230 (or SN65HVD230) CAN transceiver module**
    - Connect **3.3 V** to the repurposed RX pin (now a power output).
    - Connect **CAN_TX** and **CAN_RX** to available GPIOs on the 6‑pin connector.
    - Verified configuration: **GPIO1 (L3) = CAN_TX**, **GPIO2 (L2) = CAN_RX**
    - (Alternative if those are not free: solder directly to GPIO36 – but this requires soldering on the ESP32 chip itself - change configuration in software.)

7.  **Power supply options**
    - 12 V input via the CAN bus (using a Schottky diode parallel to the bus) **or** via two screw terminals added to the enclosure (with reverse polarity protection).
    - The DC‑DC converter outputs 5 V – feed it to the **5 V pin** of the 6‑pin board connector.

8.  **Optional: Mini speaker** (requires re‑routing audio GPIOs – not needed in this project)

---

## 📦 Bill of Materials (Modified Display)

| Part                               | Description / Source                                      | Qty |
|------------------------------------|-----------------------------------------------------------|-----|
| **ESP32‑4848S040**                 | 4" 480×480 touch display (AliExpress, ~25€)              | 1   |
| **DC‑DC converter**                | 12 V → 5 V / 3 A (e.g., LM2596‑based, best enclosed)     | 1   |
| **NMEA2000 male connector**        | 12 mm waterproof panel mount (Micro‑C compatible)        | 1   |
| **CAN transceiver**                | WCMCU230 or SN65HVD230 module (3.3 V)                    | 1   |
| **Schottky diode**                 | 1N5819 or similar (for 12 V OR‑ing)                      | 1   |
| **Screw terminals** (optional)     | 2‑pin, 5.08 mm pitch (for direct 12 V input)             | 1   |
| **4‑pin cable + connectors**       | For internal connections                                 | 1   |
| **Heat shrink tubing & wires**     | AWG24, colours: red/black/blue/yellow if needed          | –   |

---

## 🖼️ Hardware Modification Gallery

> ⚠️ **Safety first**  
> - Disconnect the device from any power source before opening.  
> - The original board contains high‑voltage traces (220 V) – handle with care and remove them completely.  
> - Use a **2 A slow‑blow fuse** on the 12 V input line.

<p align="center">
  <img src="images/Hardware%20Modifications%2000001.jpeg" width="600"/>
</p>

<p align="center">
  <img src="images/Hardware%20Modifications%2000002.jpeg" width="600"/>
</p>

<p align="center">
  <img src="images/Hardware%20Modifications%2000003.jpeg" width="600"/>
</p>

<p align="center">
  <img src="images/Hardware%20Modifications%2000004.jpeg" width="600"/>
</p>

<p align="center">
  <img src="images/Hardware%20Modifications%2000005.jpeg" width="600"/>
</p>

<p align="center">
  <img src="images/Hardware%20Modifications%2000006.jpeg" width="600"/>
</p>

<p align="center">
  <img src="images/Hardware%20Modifications%2000007.jpeg" width="600"/>
</p>

<p align="center">
  <img src="images/Hardware%20Modifications%2000008.jpeg" width="600"/>
</p>



### 1️⃣ Open the case and remove original PSU & relays
Unscrew the back cover. Desolder the 220 V AC/DC converter board and the relay(s). Keep only the 6‑pin connector that goes to the mainboard.


### 2️⃣ Install the DC‑DC converter
Fix the 12 V→5 V converter inside the case (e.g., with double‑sided tape or a 3D‑printed bracket). Connect its input to the incoming 12 V wires and output to the **5 V pin** of the 6‑pin board connector.


### 3️⃣ Cut the TX/RX trace and repurpose for 3.3 V
Locate the trace that originally fed the relay board. Cut it and connect a wire from the ESP32’s **3.3 V** regulator output to the now‑isolated pin (e.g., former RX). This pin will supply power to the CAN transceiver.


### 4️⃣ Connect CAN transceiver (WCMCU230)
Solder the transceiver module according to the verified pinout:

| Module pin | ESP32‑4848S040 connection         |
|------------|------------------------------------|
| **3.3 V**  | Repurposed RX pin (from step 3)    |
| **GND**    | Board GND (pin 4 on 6‑pin header)  |
| **CAN_TX** | GPIO1 (L3 on the 6‑pin connector)  |
| **CAN_RX** | GPIO2 (L2 on the 6‑pin connector)  |
| **CANH**   | Yellow wire → NMEA2000 connector H |
| **CANL**   | Blue wire → NMEA2000 connector L   |

Secure the module with hot glue.


### 5️⃣ Mount NMEA2000 connector & final wiring
Drill a 12 mm hole into the case back. Install the male NMEA2000 plug. Connect the four wires as per the table above. Optionally add screw terminals for a direct 12 V power input (with Schottky diode).


### 6️⃣ Flash firmware and test
Connect the USB‑to‑UART programmer (the board has a built‑in USB‑C port) and upload the `main.cpp` code using Arduino IDE or PlatformIO. After boot, the display should show the home screen. Connect the device to an active NMEA2000 network – the coloured dot at the top‑right turns green when data is received.



---

## 🗂️ Repository Structure
