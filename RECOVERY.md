# LightTracker RX Board Recovery Guide

## STATUS: RESOLVED (re-verified 2026-06-13)
Both boards are working. SWD recovery was NOT needed — the RX board's bootloader
turned out to be intact (it re-enumerated as Arduino M0, PID 0x804e).

- **TX board** (USB serial `B5290C2C…1E3D`): TX firmware verified live —
  transmits 923.2 MHz / SF8 / BW125 / 16 dBm every 60 s.
- **RX board** (USB serial `FCD0A6F3…0F3D`): was found silent (no serial output
  for 80 s while TX was active — wrong/halted firmware) and reflashed via USB on
  2026-06-13 with `build/rx/lora-asset-tracker-rx.ino.hex`
  (84,192 bytes written + verified by avrdude).
- **End-to-end link verified (2026-06-13)**: RX prints full
  "Incoming Packet Telemetry" (trackerID 1234, GPS fix lat 18.78/long 98.95,
  RSSI -41.00 dBm, SNR 12.25 dB, 923.20 MHz / SF8 / BW125) for the packet TX
  sent during a 95 s dual-port capture.

### Diagnosis tips (learned the hard way)
- USB PID tells the mode: **0x004d = bootloader, 0x804e = sketch running**.
  Enumeration at 0x804e does NOT prove the sketch is healthy — both sketches
  halt forever pre-setup if pin A1 reads LOW at boot, and the SAMD core brings
  up USB before `setup()`.
- TX firmware only prints once per 60 s cycle (DEVMODE is commented out), so
  serial captures must span **≥80 s** before concluding a TX board is dead.
  RX firmware prints on every received packet, so RX silence while TX is
  active IS diagnostic.
- Ports renumber after resets/replugs — re-verify port↔board mapping via
  `system_profiler SPUSBDataType` serial numbers immediately before any upload.

### Toolchain fix that made USB upload work on Apple Silicon
The bundled avrdude at
`~/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino9/bin/avrdude`
was an i386 binary (cannot run on Apple Silicon, even under Rosetta). It was
replaced with the x86_64 build from the `6.3.0-arduino17` tarball (runs via
Rosetta 2; same 6.3 CLI and avrdude.conf format):
- sha256 of installed binary: `ed141269562838a3d4d6d52b6ed76ad3c29e7023fd7258734f021dc9cea047b0`
- original preserved as `avrdude.i386.bak` in the same directory

Note: `arduino:samd:mzero_bl` (Arduino M0, arduino.org bootloader) genuinely
uploads via **avrdude / stk500v2** — not bossac. Never use Homebrew avrdude
(8.x) with this board; that is what caused the original corruption scare.
Upload command:
```bash
arduino-cli upload -b arduino:samd:mzero_bl -p /dev/cu.usbmodemXXXX \
  --input-file build/rx/lora-asset-tracker-rx.ino.hex -v
```

---

The SWD procedure below is retained as a **fallback only**, in case a board
ever stops enumerating on USB entirely.

## What Happened (original incident)
The SAMD21 flash was corrupted by an incompatible avrdude version (8.1 from Homebrew vs the expected 6.3.0-arduino9). The bootloader region was overwritten, so the board no longer enumerates on USB.

## What You Need
An SWD programmer. Cheapest option: **Raspberry Pi Pico** ($5) flashed as CMSIS-DAP.

Other options:
- J-Link EDU Mini (~$20)
- Segger J-Link (~$60)
- Any CMSIS-DAP compatible debugger

## SWD Connections on LightTracker Board
Connect 3 wires from the SWD programmer to the LightTracker's SWD pads:
- **SWDIO** → SWD Data
- **SWCLK** → SWD Clock  
- **GND** → Ground

(The SWD pads are small test points on the LightTracker-B board. Refer to the SAMD21 datasheet: PA30 = SWCLK, PA31 = SWDIO)

## Recovery Steps

### 1. Install OpenOCD
```bash
brew install openocd
```

### 2. Erase the chip (clears corrupted flash)
```bash
openocd -f interface/cmsis-dap.cfg -f target/at91samdXX.cfg \
  -c "init; targets; reset halt; at91samd chip-erase; shutdown"
```

### 3. Flash the bootloader
The bootloader hex file is at:
`~/Library/Arduino15/packages/arduino/hardware/samd/1.8.14/bootloaders/mzero/Bootloader_D21_M0_150515.hex`

```bash
openocd -f interface/cmsis-dap.cfg -f target/at91samdXX.cfg \
  -c "init; targets; reset halt; program ~/Library/Arduino15/packages/arduino/hardware/samd/1.8.14/bootloaders/mzero/Bootloader_D21_M0_150515.hex verify; reset; shutdown"
```

### 4. Flash the RX firmware via USB (board should enumerate again)
After the bootloader is restored, the board will appear on USB again. Then use Arduino IDE 2.x (which has native Apple Silicon support) to upload the RX firmware normally.

## Using Raspberry Pi Pico as CMSIS-DAP
1. Download picoprobe/debugprobe UF2 from: https://github.com/raspberrypi/debugprobe/releases
2. Hold BOOTSEL on Pico, plug in USB, drag the UF2 file to the RPI-RP2 drive
3. Wire: Pico GP2 → SWDIO, Pico GP3 → SWCLK, Pico GND → GND
4. Run the OpenOCD commands above

## Files Already Compiled
- RX firmware: `./build/rx/lora-asset-tracker-rx.ino.hex` (923.2 MHz for Thailand)
- TX firmware: `./build/tx/lora-asset-tracker-tx.ino.hex` (923.2 MHz for Thailand, airborne mode)
