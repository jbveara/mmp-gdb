# MMP-GDB — Host/Firmware Split

## Architecture

```
Central laptop
    │
    ├── SSH → RPi-A1 → nRF52840-DK (USB/J-Link UART)
    ├── SSH → RPi-A2 → nRF52840-DK
    └── SSH → RPi-A3 → nRF52840-DK

Each RPi runs:  python3 anchor.py --role 1 --port /dev/ttyACM0
```

## Files

| File | Runs on | Purpose |
|------|---------|---------|
| `main.c` | nRF52840 | Shell commands, hardware init, role dispatch |
| `ring_exchange.h/.c` | nRF52840 | UWB TX/RX protocol, timestamp capture, TS line output |
| `node_config.h` | nRF52840 | Timing constants, frame layout |
| `prj.conf` | nRF52840 | Zephyr build config |
| `anchor.py` | Raspberry Pi / PC | Serial bridge, distance math, stdout output |

`ring_distances.h` and `ring_distances.c` are no longer needed — remove from CMake.

---

## Why Zephyr Shell?

The Zephyr shell subsystem (`CONFIG_SHELL=y`) provides:
- Full UART RX handling with buffering — no custom ISR or ring buffer code needed
- Human-readable commands, debuggable with any terminal (PuTTY, minicom, screen)
- Shell log backend safely interleaves log messages with output lines
- Single well-tested Kconfig symbol — no choice/dependency issues

---

## Build — single firmware binary for all three boards

Remove any `-DROLE_ANCHOR_A1/A2/A3` from `CMakeLists.txt`.

```cmake
target_sources(app PRIVATE
    src/main.c
    src/ring_exchange.c
    # ring_distances.c — deleted
)
```

```bash
west build --build-dir build --pristine --board nrf52840dk/nrf52840 \
    -- -DSHIELD=qorvo_dws3000 \
       -DBOARD_ROOT=C:/SPL/zephyr-dw3000-examples/dw3000-decadriver/zephyr
```

---

## Running

### On Windows (testing)
```bash
python anchor.py --role 1 --port COM23 --baud 115200
```

### On Raspberry Pi (deployment)
```bash
python3 anchor.py --role 1 --port /dev/ttyACM0
```

Start A2 and A3 before A1. Responders sit in their listen loop until
A1 begins transmitting.

---

## Startup sequence

```
# Board boots — shell starts automatically
*** Booting nRF Connect SDK v3.3.1 ***
uart:~$ INF,MMP-GDB_ready — type: role <1|2|3>  then: start

# anchor.py sends "role 1\n"
uart:~$ role 1
INF,role_set_A1 (initiator)

# anchor.py sends "start\n"
uart:~$ start
INF,starting
RDY,1,192016385,16385
INF,A1_starting_period_500ms
TS,1,0,12345678,...
TS,1,1,12345699,...
```

---

## Output format

**stdout** (one line per round, pipe-friendly):
```
DIST,<role>,<seq>,<d_P1P2_m>,<d_P1P3_m>,<d_P2P3_m>
DIST,1,42,1.2340,3.4560,2.1230
```

**stderr** (firmware passthrough, prefixed `FW>`):
```
FW> INF,MMP-GDB_ready — type: role <1|2|3>  then: start
FW> RDY,1,192016385,16385
FW> INF,A1_starting_period_500ms
```

Distances of `-1.0` indicate a ranging error for that pair in that round.

---

## Serial protocol

### Host → firmware (shell commands)
```
role <1|2|3>    Set anchor role
start           Initialise DW3000 and begin
help            List all available commands (Zephyr built-in)
```

### Firmware → host (structured lines, interspersed with shell prompt)
```
RDY,<role>,<DELTA_TICKS>,<TX_ANT_DLY>    Startup confirmation
TS,<role>,<seq>,<ts0..5>,<ci0..5>        One per completed round
ERR,<seq>,<reason>                        Error
INF,<message>                             Informational
uart:~$                                   Shell prompt (ignored by anchor.py)
```

---

## Manual testing with a terminal

You can drive the board directly from PuTTY or any serial terminal:

```
uart:~$ role 2
INF,role_set_A2 (responder)
uart:~$ start
INF,starting
RDY,2,192016385,16385
INF,A2_listening
```

Tab completion and `help` work as on any Zephyr shell.

---

## Central aggregation (optional)

```bash
# On central laptop — collect all three streams
ssh pi@rpi-a1 python3 anchor.py --role 1 | tee a1.csv &
ssh pi@rpi-a2 python3 anchor.py --role 2 | tee a2.csv &
ssh pi@rpi-a3 python3 anchor.py --role 3 | tee a3.csv &
```

---

## Next step — USB CDC ACM

For RPi deployment, switching to `CONFIG_USB_CDC_ACM=y` eliminates
J-Link driver dependencies on the RPi. The board appears as
`/dev/ttyACM0` with no J-Link tooling needed.
