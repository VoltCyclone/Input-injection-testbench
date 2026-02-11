# Mouse Bridge Device API Reference

> Consolidated protocol documentation for KMBox B+, Ferrum One, and MAKCU
> mouse bridge devices. Compiled from official sources and device testing.

---

## Device Overview

| | KMBox B+ | Ferrum One | MAKCU |
|---|---|---|---|
| **Default baud** | 2,000,000 | 2,000,000 | 115,200 |
| **Baud range** | Fixed | Fixed | 115,200 – 4,000,000 |
| **Protocol** | ASCII | ASCII (KMBox-compatible) | ASCII + Binary V2 |
| **Max poll rate** | 1 kHz | 8 kHz | Varies (bInterval) |
| **Pass-through** | `km.transform()` | `km.transform()` | `km.bypass()` |
| **Bézier curves** | No | No | Yes |
| **Absolute move** | No | No | Yes (`km.moveto`) |
| **Button locking** | No | No | Yes |
| **Axis streaming** | No | No | Yes |
| **Raw HID frame** | No | No | Yes (`km.mo`) |

**Ferrum One** is a documented drop-in replacement for the KMBox B+ Pro.
It accepts all standard KMBox commands identically. The Ferrum "Software API"
(extended binary protocol) requires the Windows companion app and is not
available over direct serial.

---

## 1. ASCII Protocol (All Devices)

### Transport

- **Serial**: 8N1, no flow control
- **TX** (host → device): `km.command(args)\r\n`
- **RX** (device → host): `km.<echo>\r\n>>>`
- The `km.` prefix is optional on MAKCU (`.move(1,1)` works)
- Setters echo their input as ACK (suppressible via `km.echo(0)` on MAKCU)
- The `>>>` prompt indicates command completion

### 1.1 Mouse Movement

#### `km.move(dx, dy)`

Relative mouse move.

| Param | Type | Range | Description |
|-------|------|-------|-------------|
| `dx` | int | -127 to 127 | Horizontal delta (pixels) |
| `dy` | int | -127 to 127 | Vertical delta (pixels) |

```
TX: km.move(10,-3)\r\n
RX: km.move(10,-3)\r\n>>>\r\n
```

**Devices**: KMBox ✓ Ferrum ✓ MAKCU ✓

#### `km.move(dx, dy, segments)`

Segmented move — firmware interpolates the delta across N USB HID frames.

| Param | Type | Range | Description |
|-------|------|-------|-------------|
| `dx` | int | -127 to 127 | Total horizontal delta |
| `dy` | int | -127 to 127 | Total vertical delta |
| `segments` | int | 1–512 | Number of interpolation steps |

```
TX: km.move(100,50,8)\r\n
```

**Devices**: KMBox ✓ Ferrum ✓ MAKCU ✓ (MAKCU supports up to 512 segments)

#### `km.move(dx, dy, segments, cx1, cy1, cx2, cy2)`

Cubic Bézier curve move. Control points define the curve shape; the firmware
interpolates through the Bézier path over N segments.

| Param | Type | Description |
|-------|------|-------------|
| `cx1, cy1` | int | First control point |
| `cx2, cy2` | int | Second control point |

```
TX: km.move(100,50,8,40,25,80,10)\r\n
```

**Devices**: MAKCU ✓ (KMBox/Ferrum: unknown)

#### `km.moveto(x, y [, segments [, cx1, cy1, cx2, cy2]])`

Absolute move. Device internally computes the required delta from current
position. Requires `km.screen(w, h)` to be configured first.

```
TX: km.moveto(640,360)\r\n
TX: km.moveto(100,50,8,40,25,80,10)\r\n
```

**Devices**: MAKCU only

### 1.2 Mouse Buttons

#### `km.left(state)` / `km.right(state)` / `km.middle(state)` / `km.side1(state)` / `km.side2(state)`

Individual button control.

| State | Description |
|-------|-------------|
| 0 | Release (sends USB frame) |
| 1 | Press down |
| 2 | Silent release — sets internal state to 0 without sending a USB frame |

**GET** (no args): Returns lock state: 0=none, 1=raw locked, 2=injected locked, 3=both.

```
TX: km.left(1)\r\n        # press left button
TX: km.left()\r\n          # query lock state
RX: km.left(0)\r\n>>>\r\n
```

**Devices**: KMBox ✓ Ferrum ✓ MAKCU ✓

#### `km.click(button [, count [, delay_ms]])`

Schedule mouse click(s) with optional count and inter-click delay.

| Param | Type | Description |
|-------|------|-------------|
| `button` | int | Button number (1=left, 2=right, 3=middle, 4=side1, 5=side2) |
| `count` | int | Number of clicks (optional) |
| `delay_ms` | int | Inter-click delay in ms (optional; default: random 35–75ms) |

```
TX: km.click(1,3)\r\n     # triple left-click with random timing
```

**Devices**: KMBox ✓ Ferrum ✓ MAKCU ✓

#### `km.turbo(button [, delay_ms])`

Enable rapid-fire mode for a held button. Delay is auto-rounded to the
mouse endpoint's bInterval for USB synchronization.

| Param | Description |
|-------|-------------|
| `turbo()` | Query active turbo settings |
| `turbo(btn)` | Enable turbo with random 35–75ms delay |
| `turbo(btn, ms)` | Enable turbo with specific delay |
| `turbo(btn, 0)` | Disable turbo for specific button |
| `turbo(0)` | Disable all turbo |

**Devices**: MAKCU only

### 1.3 Scroll & Axes

#### `km.wheel(delta)`

Scroll wheel.

| Param | Type | Description |
|-------|------|-------------|
| `delta` | int | Scroll steps. **MAKCU**: clamped to ±1 per call |

**Devices**: KMBox ✓ Ferrum ✓ MAKCU ✓ (clamped)

#### `km.pan(steps)` / `km.tilt(steps)`

Horizontal pan and tilt scroll axes.

**Devices**: MAKCU only

#### `km.mo(buttons, x, y, wheel, pan, tilt)`

Send a complete raw HID mouse frame in one command.

| Param | Type | Description |
|-------|------|-------------|
| `buttons` | u8 | Button bitmask (bit0=left, bit1=right, bit2=middle, ...) |
| `x, y` | int | Movement deltas (one-shot) |
| `wheel, pan, tilt` | int | Scroll values (one-shot) |

`mo(0)` clears all state. Button mask is **stateful** (persists); x/y/wheel/pan/tilt are one-shots.

```
TX: km.mo(1,10,5,0,0,0)\r\n   # left button down + move (10,5)
```

**Devices**: MAKCU only

### 1.4 Advanced Mouse (MAKCU Only)

#### `km.silent(x, y)`

Move to absolute position then perform a silent left click.

#### `km.getpos()`

Returns current pointer position.

```
RX: km.getpos(123,456)\r\n>>>\r\n
```

#### `km.screen(w, h)`

Set virtual screen dimensions (required for `moveto` and `getpos`).

```
TX: km.screen(1920,1080)\r\n
```

#### `km.lock_<target>(state)`

Lock a button or axis. Target is part of the command name.

| Buttons | Axes (full) | Axes (directional) |
|---------|-------------|-------------------|
| `ml` (left) | `mx` | `mx+`, `mx-` |
| `mm` (middle) | `my` | `my+`, `my-` |
| `mr` (right) | `mw` | `mw+`, `mw-` |
| `ms1` (side1) | | |
| `ms2` (side2) | | |

State: `1`=lock, `0`=unlock. GET returns current state.

```
TX: km.lock_mx(1)\r\n      # lock all X-axis movement
TX: km.lock_ml(1)\r\n      # lock left button
TX: km.lock_mw+(1)\r\n     # lock positive scroll only
```

#### `km.catch_<target>(mode)`

Enable catch on a locked **button** (not axes). Requires the corresponding
`km.lock_<target>` to be set first.

| Mode | Description |
|------|-------------|
| 0 | Auto catch |
| 1 | Manual catch |

#### `km.remap_button(src, dst)`

Remap physical mouse buttons. Auto-clears conflicting mappings.
Both directions can be mapped simultaneously (swap).

| Value | Button |
|-------|--------|
| 0 | Reset all |
| 1 | Left |
| 2 | Right |
| 3 | Middle |
| 4 | Side1 |
| 5 | Side2 |

```
TX: km.remap_button(1,2)\r\n    # swap left→right
TX: km.remap_button(0)\r\n      # reset all
```

#### `km.remap_axis(inv_x, inv_y, swap)`

Remap physical axes. Each flag is 0 or 1.

```
TX: km.remap_axis(0,1,0)\r\n    # invert Y only
TX: km.remap_axis(0)\r\n        # reset all
```

#### `km.invert_x(state)` / `km.invert_y(state)` / `km.swap_xy(state)`

Individual axis inversion and swap controls. Physical input only.

### 1.5 Pass-through Control

#### `km.transform(sx, sy, enable)` — KMBox / Ferrum

Controls physical mouse input pass-through on KMBox and Ferrum.

| Configuration | Effect |
|---------------|--------|
| `km.transform(0, 0, 1)` | Block physical mouse |
| `km.transform(256, 256, 0)` | Restore full pass-through |

**Devices**: KMBox ✓ Ferrum ✓

#### `km.bypass(mode)` — MAKCU

Controls USB endpoint pass-through on MAKCU.

| Mode | Effect |
|------|--------|
| 0 | Off (restore USB write, disable telemetry) |
| 1 | Mouse bypass (enables `km.mouse(1,1)`, disables USB write) |
| 2 | Keyboard bypass (enables `km.keyboard(1,1)`, disables USB write) |

**Devices**: MAKCU only

### 1.6 Keyboard Commands

#### `km.down(key)` / `km.up(key)`

Key down / key up. Key can be a HID code (0–255) or a quoted string name.

```
TX: km.down('shift')\r\n
TX: km.up("ctrl")\r\n
TX: km.down(4)\r\n            # HID code 4 = 'a'
```

#### `km.press(key [, hold_ms [, rand_ms]])`

Press and release a key.

| Param | Default | Description |
|-------|---------|-------------|
| `hold_ms` | random 35–75ms | Hold duration |
| `rand_ms` | 0 | Randomization added to hold_ms |

Duration is auto-rounded to the keyboard's bInterval for USB synchronization.

```
TX: km.press('a')\r\n          # random 35-75ms hold
TX: km.press('d', 50)\r\n      # exactly 50ms hold
TX: km.press('d', 50, 10)\r\n  # 50ms + random 0-10ms
```

**Devices**: KMBox ✓ Ferrum ✓ MAKCU ✓

#### `km.string("text")`

Type an ASCII string with automatic Shift handling.

| Constraint | Value |
|------------|-------|
| Max length | 256 characters |
| Hold time | Random 35–75ms per character |
| Inter-char delay | 10ms |

```
TX: km.string("Hello World!")\r\n
```

**Devices**: MAKCU only

#### `km.init()`

Release all pressed keys and clear keyboard state.

#### `km.isdown(key)`

Query key state. Returns `0`=up, `1`=down.

#### `km.disable(key1, key2, ...)` / `km.disable(key, mode)`

Block keys from passing through to the host.

```
TX: km.disable('a','c','f')\r\n     # disable multiple keys
TX: km.disable('a', 0)\r\n          # re-enable 'a'
TX: km.disable()\r\n                 # list disabled keys
```

#### `km.mask(key, mode)` / `km.remap(source, target)`

Key masking and remapping. `remap(src, 0)` clears.

### 1.7 Key Reference

Single-character letters are **case-sensitive** (`'a'` types lowercase, `'A'` types uppercase with auto-Shift).
Multi-character names are **case-insensitive** (`'f1'`, `'F1'`, `'ctrl'`, `'CTRL'` all work).

| Category | Keys |
|----------|------|
| **Letters** | `'a'`–`'z'` (HID 4–29) |
| **Numbers** | `'0'`–`'9'` (HID 30–39) |
| **Control** | `'enter'`/`'return'`, `'escape'`/`'esc'`, `'backspace'`/`'back'`, `'tab'`, `'space'` |
| **Symbols** | `'minus'`, `'equals'`, `'leftbracket'`, `'rightbracket'`, `'backslash'`, `'semicolon'`, `'quote'`, `'grave'`, `'comma'`, `'period'`, `'slash'`, `'capslock'` |
| **Function** | `'f1'`–`'f12'` (HID 58–69) |
| **Navigation** | `'insert'`, `'home'`, `'pageup'`, `'delete'`, `'end'`, `'pagedown'`, `'right'`, `'left'`, `'down'`, `'up'` |
| **Modifiers** | `'ctrl'`/`'leftctrl'`, `'shift'`/`'leftshift'`, `'alt'`/`'leftalt'`, `'win'`/`'gui'`/`'cmd'`, and right variants (`'rctrl'`, `'rshift'`, `'ralt'`, `'rgui'`) |
| **Numpad** | `'kp0'`–`'kp9'`, `'kpdivide'`, `'kpmultiply'`, `'kpminus'`, `'kpplus'`, `'kpenter'`, `'kpperiod'` |

Generic names (`'ctrl'`, `'shift'`, `'alt'`, `'gui'`) default to the **left** variant.

### 1.8 Streaming (MAKCU Only)

#### `km.mouse(mode, period_ms)`

Stream mouse data as 8-byte binary frames.

| Mode | Description |
|------|-------------|
| 1 | Raw (physical input before remapping) |
| 2 | Constructed frame (after remapping/masking) |
| 0 | Disable streaming |

Period: 1–1000ms. Streaming output: `km.mouse<8 bytes>\r\n>>>`

#### `km.buttons(mode, period_ms)`

Stream button state as 1-byte bitmask.

Output: `km.buttons<mask_u8>\r\n>>>`

#### `km.axis(mode, period_ms)`

Stream axis data. Output format: `raw(x,y,w)` or `mut(x,y,w)`.

#### `km.keyboard(mode, period_ms)`

Stream keyboard data. Output format: `keyboard(raw,shift,'h')` or
`keyboard(constructed,ctrl,shift,'a')`.

### 1.9 System Commands

| Command | Description | Devices |
|---------|-------------|---------|
| `km.info()` | System info (MAC, temp, RAM, FW, CPU, uptime) | All |
| `km.version()` | Firmware version string | All |
| `km.device()` | Primary device: `(keyboard)`, `(mouse)`, `(none)` | MAKCU |
| `km.reboot()` | Reboot (responds then reboots) | All |
| `km.help()` | List available commands | All |

### 1.10 Configuration Commands

#### `km.baud(rate)`

Change UART baud rate. Applies immediately; host must reopen serial at new speed.

| Param | Range |
|-------|-------|
| rate | 115,200 – 4,000,000 |
| 0 | Reset to default (115,200) |

**Devices**: MAKCU (KMBox/Ferrum: fixed baud)

#### `km.echo(0|1)`

Suppress (`0`) or enable (`1`) setter echo ACKs.

**Devices**: MAKCU only

#### `km.release(timer_ms)`

Auto-release monitoring. When timer expires, releases any active locks,
buttons, or keys that remain held. Persistent across reboots.

| Param | Range |
|-------|-------|
| timer_ms | 500–300,000 (5 minutes) |
| 0 | Disable |

**Devices**: MAKCU only

#### `km.serial("text")`

Get/set attached device serial number. Change is persistent across firmware updates.
MAKCU does not allow changing serial numbers for devices that don't have one.

**Devices**: MAKCU only

#### `km.hs(0|1)`

USB high-speed compatibility mode (persistent).

**Devices**: MAKCU only

#### `km.led(target, mode [, times, delay_ms])`

LED control.

| Target | Description |
|--------|-------------|
| 1 | Device LED |
| 2 | Host LED (via UART) |

| Mode | Description |
|------|-------------|
| 0 | Off |
| 1 | On |
| times, delay_ms | Flash N times at delay_ms interval |

**Devices**: MAKCU only

#### `km.log(level)`

Log verbosity 0–5. Persists for 3 power cycles, then auto-disables.

**Devices**: MAKCU only

#### `km.fault()`

Returns stored parse fault info (MAC, endpoint, interface, reason, raw HID
descriptor bytes). Useful for debugging devices that fail to enumerate.

**Devices**: MAKCU only

---

## 2. MAKCU V2 Binary Protocol

Lower-latency binary framing available on MAKCU alongside the ASCII protocol.
Mirrors the ASCII command set but with binary encoding for reduced parsing overhead.

### Transport

Both protocols coexist on the same serial port. The parser distinguishes
frames by the `0x50` start byte.

### Frame Format

**TX (host → device)**:

```
[0x50] [CMD] [LEN_LO] [LEN_HI] [PAYLOAD...]
```

| Field | Size | Description |
|-------|------|-------------|
| `0x50` | 1 byte | Frame start marker (ASCII 'P') |
| CMD | 1 byte | Command byte |
| LEN | 2 bytes | Payload length (little-endian u16) |
| PAYLOAD | N bytes | Command-specific data |

**RX — Setters**:

```
[0x50] [CMD] [LEN_LO] [LEN_HI] [status]
```

| Status | Meaning |
|--------|---------|
| `0x00` | OK (success) |
| `0x01` | ERR (error) |

**RX — Getters**:

```
[0x50] [CMD] [LEN_LO] [LEN_HI] [data...]
```

All multi-byte values are **little-endian**.

### Command Bytes

#### Mouse Buttons

| CMD | ASCII Equivalent | Payload (SET) | Payload (GET) |
|-----|-----------------|---------------|---------------|
| `0x08` | `km.left(state)` | `[state:u8]` | Returns lock state: u8 |
| `0x0A` | `km.middle(state)` | `[state:u8]` | Returns lock state: u8 |
| `0x11` | `km.right(state)` | `[state:u8]` | Returns lock state: u8 |
| `0x12` | `km.side1(state)` | `[state:u8]` | Returns lock state: u8 |
| `0x13` | `km.side2(state)` | `[state:u8]` | Returns lock state: u8 |

State values: `0`=release, `1`=down, `2`=silent_release

**Example — Press left button**:
```
TX: 50 08 01 00 01
     │  │  │  │  └── state=1 (down)
     │  │  │  └───── LEN_HI=0
     │  │  └──────── LEN_LO=1 (1 byte payload)
     │  └─────────── CMD=0x08 (left)
     └────────────── start
RX: 50 08 01 00 00
                 └── status=0x00 (OK)
```

#### System

| CMD | ASCII Equivalent | Payload (SET) |
|-----|-----------------|---------------|
| `0xB1` | `km.baud(rate)` | `[rate:u32 LE]` |

**Example — Query baud rate**:
```
TX: 50 B1 00 00           (no payload = GET)
RX: 50 B1 04 00 00 C2 01 00
                   └──────── 0x0001C200 = 115200 (little-endian)
```

#### Legacy Baud Rate Frame

A separate frame format also accepted for baud rate changes:

```
[0xDE] [0xAD] [LEN_LO] [LEN_HI] [0xA5] [baud_rate:u32 LE]
```

**Example — Set 115200 baud**:
```
DE AD 05 00 A5 00 C2 01 00
│  │  │  │  │  └──────────── baud = 115200 (LE32)
│  │  │  │  └─────────────── cmd = 0xA5
│  │  └──┴────────────────── length = 5
└──┴──────────────────────── sync = 0xDEAD
```

### Notes

- Binary mouse move/mo command bytes are **not documented** in the public
  MAKCU API as of v3.9. Use the ASCII `km.move()` command for mouse movement.
- The binary and ASCII parsers coexist — MAKCU responds to ASCII commands
  with ASCII responses even when the binary protocol is active.
- Streaming responses (`km.mouse`, `km.buttons`) use their own binary
  subformat within the ASCII response frame.

---

## 3. Ferrum Software API (Windows Only)

The Ferrum One has an extended "Software API" that supports KMBox Net-style
commands over UDP and DHZBox-style commands over UDP, in addition to the
KM serial commands. This API requires the Ferrum companion app running on
Windows and is **not available over direct serial on macOS/Linux**.

For direct serial communication (the use case of this tool), Ferrum
uses the standard ASCII protocol documented in Section 1.

**Documentation**: https://ferrumllc.github.io/software_api.html

---

## 4. Wire Examples

### Relative Mouse Move — All Devices (ASCII)

```
Host → Device:  km.move(10,-3)\r\n
Device → Host:  km.move(10,-3)\r\n>>>\r\n
```

Total TX bytes: 17

### Button Press — MAKCU Binary V2

```
Host → Device:  50 08 01 00 01       (left down)
Device → Host:  50 08 01 00 00       (OK)

Host → Device:  50 08 01 00 00       (left release)
Device → Host:  50 08 01 00 00       (OK)
```

Total TX bytes per action: 5

### Bézier Curve Move — MAKCU ASCII

```
Host → Device:  km.move(100,50,8,40,25,80,10)\r\n
Device → Host:  km.move(100,50,8,40,25,80,10)\r\n>>>\r\n
```

The firmware generates 8 intermediate HID frames along the cubic Bézier
curve defined by start=(0,0), P1=(40,25), P2=(80,10), end=(100,50).

---

## 5. Sources

| Source | URL |
|--------|-----|
| MAKCU API v3.9 | https://www.makcu.com/en/api |
| Ferrum Developer Docs | https://ferrumllc.github.io/ |
| Ferrum Software API | https://ferrumllc.github.io/software_api.html |
| Ferrum Product Page | https://xferrum.ai/products/ferrum-one |
