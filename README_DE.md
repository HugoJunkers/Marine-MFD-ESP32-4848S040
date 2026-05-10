# ⛵ OpenMarine ESP32 MFD & Autopilot Controller

![Titelbild des Displays](images/dashboard.jpg)

Ein Open-Source Multi-Function Display (MFD) für Segelyachten. Das Projekt basiert auf einem ESP32-S3 und liest über einen CAN-Transceiver Live-Daten aus dem **NMEA2000-Netzwerk**. Es bietet zudem eine direkte Steuerung für Raymarine Evolution Autopiloten.

---

## ⚠️ Wichtiger Sicherheitshinweis & Haftungsausschluss
> **Dieses Projekt ist experimentell!** 
> Die Software steuert sicherheitskritische Systeme (Autopilot) und liefert Navigationsdaten. Ein Softwarefehler, ein Absturz des ESP32 oder WLAN/CAN-Probleme können dazu führen, dass das Boot unkontrolliert manövriert oder falsche Tiefen anzeigt. 
> * **Niemals** als primäres Navigationssystem nutzen!
> * **Immer** das physische Steuerpanel des Autopiloten griffbereit haben, um im Notfall den Standby-Modus zu erzwingen.
> * Nutzung erfolgt vollständig auf **eigene Gefahr**. Der Autor übernimmt keinerlei Haftung für Schäden an Schiff, Personen oder Material.

---

## ✨ Funktionen

* **Wind- & Navigationsdaten:** Flüssige Anzeige von TWA/TWS (True Wind), AWA/AWS, SOG, STW und Tiefe.
* **Autopilot (Raymarine EV-1):**
  * Wechsel zwischen STBY, AUTO, WIND und TRCK.
  * Kursänderungen (-10°, -1°, +1°, +10°) und Tack-Befehle direkt über das Touch-Display.
* **Touchbedienung:** Slide horizontally/vertically oder Direktwahl über den home-screen

## 📸 Screenshots

| Wind Dashboard | Autopilot Steuerung |
| :---: | :---: |
| ![Wind](images/dashboard.jpg) | ![Autopilot](images/autopilot.jpg) |

## 🛠️ Hardware-Setup
* **Mikrocontroller:** ESP32-S3 (mit PSRAM)
* **Display:** 480x480 IPS Touch (z.B. ST7701S / GT911)
* **Netzwerk:** MCP2515 CAN-Modul (mit 120 Ohm Abschlusswiderstand)

## 💻 Installation (PlatformIO)
1. Repository klonen.
2. Das Projekt in VSCode / PlatformIO öffnen.
3. In der `platformio.ini` sicherstellen, dass die NMEA2000-Bibliotheken und LVGL referenziert sind.
4. Flashen und das Display genießen!

## 📜 Lizenz
Dieses Projekt ist unter der **MIT Lizenz** veröffentlicht. Details siehe `LICENSE` Datei.