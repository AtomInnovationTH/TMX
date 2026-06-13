# HANDOFF — LightTracker LoRa Chat project

**Date:** 2026-06-13 18:35 +07 · **Repo:** https://github.com/AtomInnovationTH/TMX
**Live app:** https://atominnovationth.github.io/TMX/ (GitHub Pages, `main` → `/docs`)

## TL;DR — everything works

Two LightTracker-B boards (Arduino M0 / SAMD21 + SX1262, VID 0x2A03) run chat
firmware **v1.3** and are verified end-to-end: browser ⇄ Web Serial ⇄ board ⇄
923.2 MHz LoRa ⇄ board ⇄ browser. Chat, delivery ACKs, GPS position exchange,
distance/bearing, the radar panel, **periodic position beacons (live tracking)**
and the **desktop telemetry panel** all work.

Since the prior handoff (firmware v1.1): firmware advanced to **v1.3** (TYPE_BEACON
every ~60 s + a much richer `gps` telemetry event), and the web app gained a
two-column desktop layout, a telemetry panel, and a round of UI hardening
(commit `7172950`). The web pass at commit `629b405` reworked the desktop layout
(50/50 split, message-pane Reset button) and made the telemetry panel calmer.
The **latest web pass** (commits `97cb7b5`→`8a4a071`, firmware untouched) reskinned
the message pane: a messenger-style **single left-rail chat** (all messages flow
top-to-bottom, own messages indented + green, time/receipt *inside* the bubble),
colored initial avatars + sender grouping + day pills, the header moved **into**
the left column (app retitled **LoRaChat v1.4**) so the radar/telemetry column
runs full height, and a local-only **"This browser"** client-profile section at
the bottom of the telemetry panel. Earlier web passes deliberately *removed* a
few over-engineered extras — see "Web app notes".

## Current hardware state

| Board | USB serial | Last seen at | Running |
|---|---|---|---|
| 1 | `B5290C2C…1E3D` | `/dev/cu.usbmodem1101` | lora-chat v1.3 |
| 2 | `FCD0A6F3…0F3D` | `/dev/cu.usbmodem2101` | lora-chat v1.3 |

Both on USB, both healthy. Bench link quality: RSSI ≈ -40 dBm, SNR ≈ 13 dB.
Board 1 gets indoor GPS fixes (11 sats near the window); fixes flicker indoors —
normal, and now handled gracefully (the web app holds the last known position
and shows a "GPS lost · age" staleness indicator instead of blanking).
Sanity check a board's firmware: connect in the app — the `ready` event reports
`fw`; if a board shows no beacons / no `gps` telemetry it's still on ≤v1.2,
reflash from `build/chat/`.

## What's in the repo

```
lora-chat/lora-chat.ino    chat firmware v1.3 (both boards run the same sketch)
docs/index.html            entire web app, single file, zero deps (Pages root)
build/chat/*.hex           prebuilt chat firmware v1.3 (committed on purpose)
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
the NDJSON protocol, and asserts the v1.1 baseline: ready/gps events, join+pos,
bidirectional rx with pos trailer, sent/ack. Last full run on v1.1: **13/13 PASS**.

For v1.3 the harness should be extended to also assert the new surface: the rich
`gps` fields (`fixType/hacc/pdop/speed/course/utc/up/noise/tx/rx`), a `beacon`
event arriving within ~60–90 s of join, and its decoded `ext`
(`speed/course/hacc/fixType`). Not yet wired into the committed harness.

Serial gotcha: boards only print on events; an empty capture window is NOT
proof of a dead sketch. PID 0x804e = sketch running, 0x004d = bootloader.

## Protocol summary (full spec in lora-chat.ino header comment)

- **Serial (115200, NDJSON):** in: `ping/name/join/tx` · out:
  `ready/rx/join/beacon/sent/ack/gps/err`
  - `ready` carries `fw` (now `"1.3"`). `gps` (rich, ~2 s): `fix/fixType/sats/
    lat/lon/alt/speed/course/hacc/pdop/utc/batt/up/noise/tx/rx`. `beacon`: a
    peer's silent periodic position broadcast (feeds radar/roster/telemetry),
    with `ext:{speed,course,hacc,fixType}` when the sender is fw 1.3+.
- **Air:** `[0xC4][ver=01][type|0x80=pos][senderId:2][msgId:2][nameLen][name][payload][pos trailer:12]`
  - type: 0=chat 1=join 2=ack(lean, no trailer) **3=beacon**; trailer = lat/lon
    (i32 ×1e7), alt i16 m, sats u8 (0 = no fix), batt u8 (V×20)
  - beacon payload (fw 1.3+, 4 B): `[speed:u8 km/h][course:u8 deg/2][hacc:u8 m]
    [fixType:u8]` — additive; fw ≤1.1 ignores type 3 entirely
  - CAD listen-before-talk + random backoff; ACK + 1 retransmit; dedupe ring
  - beacons start after the browser joins, only with a fix, jittered to desync
- Radio: 923.2 MHz / SF8 / BW125 / CR4-5 / sync 0x12 / 16 dBm — **must match
  the tracker sketches** if you change anything.

## Web app notes (docs/index.html)

- Web Serial = Chrome/Edge/Opera **desktop only**; Pages is HTTPS so it works;
  `file://` also works for local dev.
- Auto-reconnects to a previously granted port on load. Two windows on one
  laptop = valid test rig (second window must pick the other board manually —
  Chrome locks a port to one tab).
- **Layout:** two-column desktop grid, **50/50 split**. The app **header now lives
  inside the left column** (not a full-width top bar), so it matches the chat-pane
  width and the right column (📡 toggles it) runs **full height** = a taller
  radar/telemetry "tracker" pane. Title is **LoRaChat v1.4 · 923.2 MHz**; the
  Reset button is folded into that header (the old slim "Messages" bar is gone).
  The connect/hero screen has no header (the connect card is self-contained); the
  header appears once you're in the chat view and stays after a disconnect.
  Below 900 px it falls back to a single column (chat on top, tracker below).
  New messages always scroll the log to the latest.
- **Message pane (messenger style, single left rail):** every message reads
  **top-to-bottom down one left edge** — no left/right zig-zag. Incoming = colored
  **initial avatar** (hashed hue) + sender name (shown once per turn) + white
  bubble; **own** = no avatar, **indented 72 px** + **LINE-green** bubble. Time +
  delivery receipt float **inside** the bubble's bottom-right (WhatsApp-style),
  not in an outboard column. Receipts are **words** — `Sending → Sent → Read`,
  or `Not sent · retry` (click to resend; 12 s `ACK_FAIL_MS` timer). Consecutive
  same-sender messages **group** (avatar/name once, tight spacing, tail only on
  the first bubble); **day-divider pills** mark calendar days; a tinted chat
  canvas (`--chat-bg`) makes bubbles float. Grouping state lives in
  `lastSender/lastGroupTs/lastDay`; `clearChat`/`addSystem` reset it. History
  (`lorachat.history`, 200 max) re-renders with the same grouping on reload.
- **Reset button** (in the message-pane header): clears the chat log + saved
  history (`lorachat.history`) only, behind a `confirm()`. Connection, peers,
  radar and telemetry are intentionally left untouched.
- **Radar panel:** SVG range circle (N at top, so the caption is just
  "Outer ring N m"), auto-scaling rings, peers fade after 2 min, dropped after
  15 min. **Beacon-driven** (~60 s) plus message-driven, so tracking updates
  without anyone chatting. Peer dots get a course arrow when moving; the center
  dot dims when our own fix is stale. (`#radarSvg` max-width bumped to 440 px now
  the column is taller.)
- **Telemetry panel:** "My board" + focused peer (click a roster chip / radar
  dot / sender name) + a **"This browser"** section (see below). **Stable,
  low-distraction by design:** the rows are built once and only their values
  update in place (no reflow, no rows appearing or disappearing — missing data
  shows `—`). The friendly headline keeps a constant size/colour; overall state
  is shown by a small fixed **status dot** (`.tdot` green/amber/red), *not* by
  recolouring or resizing the headline or value rows. Bad states are plain-text
  markers (`· low`, `· busy`, `· weak`). Derived-but-cheap: `SNR →
  strong/ok/weak`, battery `↗/↘` trend arrow.
- **"This browser" client profile (LOCAL ONLY — never transmitted):** a section
  at the bottom of the telemetry panel that fingerprints the *local* browser
  once per session, plus a few live bits. Static: browser+version & OS+version
  (UA-Client-Hints `getHighEntropyValues`, UA-string fallback for Safari/FF),
  device arch/model, CPU cores + `deviceMemory`, **unmasked WebGL GPU**, display
  (res/dpr/depth), languages, time zone + GMT offset, Web API support
  (Serial/BT/USB/HID/Geo/WebGPU), cookies/DNT, storage estimate, media-device
  counts, battery. Live (refreshed on the 10 s tick + resize/online/visibility):
  presence dot (`active`/`idle Ns`/`tab hidden`/`offline` from input + onLine +
  `document.hidden`), window size, input type, theme, clock, `navigator.connection`.
  Everything is feature-detected and guarded; reuses the `thRow/thHead` skeleton.
  Decision on record: smaz text compression and broadcasting any of this over the
  air were **discussed and declined** (air is plaintext; not worth the airtime).
- **GPS dropout handling (the better pattern):** the last good fix is retained
  through dropouts (`myPos` + `myPosTs`); the header pill, radar note and
  telemetry show a `last known · age` staleness marker instead of blanking.
- **Per-message footer** is intentionally lean now that the panel owns
  continuous state: the in-bubble time + `Sending/Sent/Read` receipt, plus a maps
  link for *where the peer was at send time*. Signal/battery numbers live in the
  panel, not under every bubble. Header has a small inline-SVG 4-bar signal meter.
- **Deliberately removed (kept it simple / robust):** RSSI sparkline, the
  GPS-derived "closing/opening" range trend, and battery-runtime extrapolation
  — all noisy or gimmicky for a browser chat app (see commit `7172950`). Also
  gone (commit `629b405`): the telemetry headline's colour/size changes and
  per-value red/amber recolouring, and the "New messages ↓" scroll badge. And
  in the messenger rework (`97cb7b5`→): right-aligned own bubbles, the outboard
  side-meta column, and the per-minute timestamp-hiding — the left/right jump and
  side columns were the readability complaint. Don't re-add without a real need.
- Security posture: no secrets anywhere, strict CSP meta, no external requests,
  user content rendered via `textContent` only.

## Known limitations / honest caveats

- syncWord is not encryption; payloads are plaintext on the air.
- Half-duplex; >2 participants share the channel (CAD mitigates, no TDMA).
- ≤180 chars/message; no chunking.
- Peer positions update on messages + ~60 s beacons (not truly continuous).
- The "This browser" telemetry section is **local display only** — it is never
  serialized or sent over LoRa (no protocol/firmware change was made for it).
- Git author is `J <j@Js-MacBook-Pro.local>` (user was told; cosmetic).

## Next-shift candidates (discussed, not started)

1. Extend `chat_test.py` to assert the v1.3 surface (rich `gps` fields + a
   `beacon` with decoded `ext`), then re-run the two-window rig.
2. SF7 option for 2× speed at shorter range (one constant in firmware).
3. Encryption (e.g., XChaCha20 with pre-shared key in localStorage + firmware).
4. In-browser firmware flashing (stk500v2 over Web Serial) — big lift, would
   make the app fully self-contained for newbies.
5. Message chunking for >180 chars.

Done since prior handoff: periodic position beacon (live radar tracking) +
desktop telemetry panel — was next-shift #1, now shipped in fw 1.3 / web.

## State of the working tree

This handoff pass was **web-only**; firmware is unchanged since `130d702` (still
v1.3, boards not reflashed). The message-pane reskin + browser-profile work
landed in five commits on `main`:

```
97cb7b5  Make chat pane feel like LINE
380501f  Refine chat pane layout, taller tracker column
e8467bc  Reduce left-right jump with in-bubble meta
cb1ffdc  Left-align all messages into a single vertical flow
8a4a071  Add "This browser" client profile to telemetry; indent own messages more
```

Last pushed commit at handoff: **`8a4a071`** (plus this HANDOFF.md update).
Working tree otherwise clean. `.kilo/` and build intermediates are gitignored.
Remotes: `origin` = AtomInnovationTH/TMX, `upstream` = lightaprs/LightTracker-1.0.
Push auth: macOS Keychain (no tokens in files — keep it that way).
