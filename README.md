# MMP-GDB Ring Exchange

Phase 2 of the SPL development roadmap. Three nRF52840DK + DWM3000 boards
perform a bidirectional ring message exchange, collecting six timestamps
(T1–T6) per round for offline processing.

## Project layout

```
mmp-gdb/
├── CMakeLists.txt
├── prj.conf
└── src/
    ├── main.c           — thin entry point, role-dispatches to ring_exchange
    ├── ring_exchange.c  — all UWB message chaining logic
    ├── ring_exchange.h  — public API + timestamp struct
    └── node_config.h    — addresses, timing constants, frame format
```

Platform sources (`port.c`, `config_options.c`) and the decadriver are
referenced directly from `../zephyr-dw3000-examples/` — no duplication.

## Build & flash

Open three VS Code NRF Connect extension terminals (or PowerShell with
`west` on PATH). Run one build+flash per board:

### Board 1 — A1 (initiator)
```powershell
west build --pristine `
  --build-dir C:/SPL/mmp-gdb/build_A1 `
  C:/SPL/mmp-gdb `
  --board nrf52840dk/nrf52840 `
  -- -DROLE=ANCHOR_A1 `
     -DSHIELD=qorvo_dws3000 `
     -DBOARD_ROOT=C:/SPL/zephyr-dw3000-examples/dw3000-decadriver/zephyr

west flash --build-dir C:/SPL/mmp-gdb/build_A1
```

### Board 2 — A2 (responder, leg 1)
```powershell
west build --pristine `
  --build-dir C:/SPL/mmp-gdb/build_A2 `
  C:/SPL/mmp-gdb `
  --board nrf52840dk/nrf52840 `
  -- -DROLE=ANCHOR_A2 `
     -DSHIELD=qorvo_dws3000 `
     -DBOARD_ROOT=C:/SPL/zephyr-dw3000-examples/dw3000-decadriver/zephyr

west flash --build-dir C:/SPL/mmp-gdb/build_A2
```

### Board 3 — A3 (responder, leg 2 + ring close)
```powershell
west build --pristine `
  --build-dir C:/SPL/mmp-gdb/build_A3 `
  C:/SPL/mmp-gdb `
  -- -DROLE=ANCHOR_A3 `
     -DSHIELD=qorvo_dws3000 `
     -DBOARD_ROOT=C:/SPL/zephyr-dw3000-examples/dw3000-decadriver/zephyr

west flash --build-dir C:/SPL/mmp-gdb/build_A3
```

## Expected RTT output

**A1 (every 500 ms):**
```
RING seq=0 T1=123456789 T2=123456900 T3=123520000 T4=123520120 T5=123580000 T6=123580150
RING seq=1 T1=...
```

**A2 (each ring):**
```
A2: fwd sent (seq=0 T2=123456900 T3=123520000)
```

**A3 (each ring):**
```
A3: close sent (seq=0 T4=123520120 T5=123580000)
```

## Timestamp format

All values are raw DWT timer units (~15.65 ps per unit, 40-bit counter).
To convert to nanoseconds: `T_ns = T_dwt * 0.015650040064103`

## Next steps

1. Verify ring closes reliably (no timeouts) at 0.5 s period
2. Reduce `ROUND_PERIOD_MS` and `RESP_TX_DELAY_UUS` to characterise minimum stable period
3. Add reverse ring (A1 → A3 → A2 → A1) for second measurement set
4. Add HMAC-SHA256 commitment layer (Phase 3)
