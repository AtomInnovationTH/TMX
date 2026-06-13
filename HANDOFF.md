# HANDOFF — LightTracker LoRa Chat project

**Date:** 2026-06-12 20:30 +07 · **Repo:** https://github.com/AtomInnovationTH/TMX
**Live app:** https://atominnovationth.github.io/TMX/ (GitHub Pages, `main` → `/docs`)

## TL;DR — everything works

Two LightTracker-B boards (Arduino M0 / SAMD21 + SX1262, VID 0x2A03) are flashed
with chat firmware **v1.1** and verified end-to-end: browser ⇄ Web Serial ⇄ board
⇄ 923.2 MHz LoRa ⇄ board ⇄ browser. Chat, delivery ACKs, GPS position exchange,
distance/bearing display, and the radar panel all pass live hardware tests.

## Current hardware state

| Board | USB serial | Last seen at | Running |
|---|---|---|---|
| 1 | `B5290C2C…1E3D` | `/dev/cu.usbmodem1101` | lora-chat v1.1 |
| 2 | `FCD0A6F3…0F3D` | `/dev/cu.usbmodem2101` | lora-chat v1.1 |

Both on USB, both healthy. Bench link quality: RSSI ≈ -40 dBm, SNR ≈ 13 dB.
Board 1 gets indoor GPS fixes (11 sats near the window); fixes flicker indoors —
normal, handled gracefully everywhere.

## What's in the repo

```
lora-chat/lora-chat.ino    chat firmware (both boards run the same sketch)
docs/index.html            entire web app, single file, zero deps (Pages root)
build/chat/*.hex           prebuilt chat firmware (committed on purpose)
build/tx/, build/rx/       original tracker firmware .hex — restore path
lora-asset-tracker-*/      tracker sketches, retuned to 923.2 MHz (AS923 TH)
RECOVERY.md                avrdude fix + SWD recovery (fallback only; resolved)
libraries/                 vendored Arduino libs (RadioLib 4.5.0 — OLD API!)
.kilo/plans/               session plans (gitignored)
```

## Critical toolchain knowledge (the expensive lessons)

1. **`arduino:samd:mzero_bl` uploads via avrdude/stk500v2 — NOT bossac.**
   The arduino.org M0 bootloader speaks STK500v2. This is correct and verified
   (`boards.txt:633`, `platform.txt:155`).
2. **NEVER use Homebrew avrdude (8.x)** — it corrupted a board once (see
   RECOVERY.md history). It is currently uninstalled; keep it that way.
3. The bundled avrdude at
   `~/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino9/bin/avrdude`
   was replaced with an **x86_64 build** (stock i386 can't run on Apple Silicon).
   Backup: `avrdude.i386.bak` alongside. sha256 `ed141269…047b0`.
4. USB flashing **cannot brick** these boards: bootloader region (first 8 KB) is
   protected; worst case = double-tap reset → bootloader (PID 0x004d) → reflash.
5. RadioLib is **4.5.0 (old API)**: `ERR_NONE`, `CHANNEL_FREE`, not the modern
   `RADIOLIB_ERR_*` names. Don't "fix" code to the new API without upgrading.

## Standard commands (run from repo root)

```bash
# build
arduino-cli compile -b arduino:samd:mzero_bl --libraries ./libraries \
  --output-dir build/chat lora-chat

# flash (per board, explicit port — check with: ls /dev/cu.usbmodem*)
arduino-cli upload -b arduino:samd:mzero_bl -p /dev/cu.usbmodemXXXX \
  --input-file build/chat/lora-chat.ino.hex

# restore original tracker firmware (TX or RX)
arduino-cli upload -b arduino:samd:mzero_bl -p /dev/cu.usbmodemXXXX \
  --input-file build/tx/lora-asset-tracker-tx.ino.hex
```

## Verification harness

`/var/folders/.../T/kilo/chat_test.py` (temp dir — may be gone; trivial to
recreate from git history of this handoff era). It opens both CDC ports raw
(termios, **echo off — important**, echoed JSON creates feedback loops), drives
the NDJSON protocol, and asserts 13 checks: ready/gps events, join+pos,
bidirectional rx with pos trailer, sent/ack. Last run: **13/13 PASS**.

Serial gotcha: boards only print on events; an empty capture window is NOT
proof of a dead sketch. PID 0x804e = sketch running, 0x004d = bootloader.

## Protocol summary (full spec in lora-chat.ino header comment)

- **Serial (115200, NDJSON):** in: `ping/name/join/tx` · out:
  `ready/rx/join/sent/ack/gps/err`
- **Air:** `[0xC4][ver=01][type|0x80=pos][senderId:2][msgId:2][nameLen][name][payload][pos trailer:12]`
  - type: 0=chat 1=join 2=ack(lean, no trailer); trailer = lat/lon (i32 ×1e7),
    alt i16 m, sats u8 (0 = no fix), batt u8 (V×20)
  - CAD listen-before-talk + random backoff; ACK + 1 retransmit; dedupe ring
- Radio: 923.2 MHz / SF8 / BW125 / CR4-5 / sync 0x12 / 16 dBm — **must match
  the tracker sketches** if you change anything.

## Web app notes (docs/index.html)

- Web Serial = Chrome/Edge/Opera **desktop only**; Pages is HTTPS so it works;
  `file://` also works for local dev.
- Auto-reconnects to a previously granted port on load. Two windows on one
  laptop = valid test rig (second window must pick the other board manually —
  Chrome locks a port to one tab).
- Radar panel (📡 button): north-up SVG range circle, auto-scaling rings,
  peers fade after 2 min, dropped after 15 min. Positions update only when
  peer messages arrive (message-driven, by design).
- Security posture: no secrets anywhere, strict CSP meta, no external requests,
  user content rendered via `textContent` only.

## Known limitations / honest caveats

- syncWord is not encryption; payloads are plaintext on the air.
- Half-duplex; >2 participants share the channel (CAD mitigates, no TDMA).
- ≤180 chars/message; no chunking.
- Peer positions are message-driven — no continuous tracking.
- Git author is `J <j@Js-MacBook-Pro.local>` (user was told; cosmetic).

## Next-shift candidates (discussed, not started)

1. **Periodic position beacon** (~30–60 s, firmware) → live radar tracking.
   Cheap: reuse TYPE_JOIN or add TYPE_BEACON=3; remember airtime budget.
2. SF7 option for 2× speed at shorter range (one constant in firmware).
3. Encryption (e.g., XChaCha20 with pre-shared key in localStorage + firmware).
4. In-browser firmware flashing (stk500v2 over Web Serial) — big lift, would
   make the app fully self-contained for newbies.
5. Message chunking for >180 chars.

## State of the working tree

At handoff: clean except this file (HANDOFF.md, untracked). Last pushed commit:
`c83bed2` "Add radar panel…". `.kilo/` and build intermediates are gitignored.
Remotes: `origin` = AtomInnovationTH/TMX, `upstream` = lightaprs/LightTracker-1.0.
Push auth: macOS Keychain (no tokens in files — keep it that way).
