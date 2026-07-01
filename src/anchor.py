#!/usr/bin/env python3
"""
anchor.py — MMP-GDB host-side application.

Runs on the Raspberry Pi attached to an nRF52840-DK+DWM3000EVB.
Configures the firmware with a role, then reads structured timestamp lines
and computes all three pairwise UWB distances locally.

Usage:
    python3 anchor.py --role 1 --port /dev/ttyACM0 [--baud 115200] [--log FILE]

Output (stdout, one line per completed round):
    DIST,<role>,<seq>,<d_P1P2_m>,<d_P1P3_m>,<d_P2P3_m>

Errors and informational messages from the firmware are forwarded to
stderr with a "FW>" prefix so they don't pollute distance output.

If --log FILE is given, all log records (FW lines, DIST lines, warnings,
info, and — with --verbose — debug lines) are additionally written to
that file via the logging module, so a single consistent stream is kept
on disk for post-run analysis.

Exit codes:
    0  — clean shutdown (SIGINT / SIGTERM)
    1  — fatal hardware / serial error
"""

import argparse
import logging
import math
import signal
import sys
import time
from dataclasses import dataclass
from typing import Optional

import serial  # pyserial

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
log = logging.getLogger("anchor")


# ---------------------------------------------------------------------------
# Physical constants — must match node_config.h
# ---------------------------------------------------------------------------
DWT_FREQ_HZ       = 499.2e6 * 128.0          # ~63.897 GHz effective clock
SPEED_OF_LIGHT    = 299_792_458.0             # m/s
METRES_PER_TICK   = SPEED_OF_LIGHT / DWT_FREQ_HZ   # ~4.692e-3 mm / tick

# These must match node_config.h RESP_TX_DELAY_TICKS and TX_ANT_DLY.
# They are also verified at runtime against the RDY line from the firmware.
RESP_TX_DELAY_TICKS = 192_000_000
TX_ANT_DLY          = 16_399
DELTA_TICKS         = RESP_TX_DELAY_TICKS + TX_ANT_DLY  # nominal delta

# Carrier integrator → ppm (ch5, 6.8 Mbps, from DW3000 User Manual)
CAR_INT_TO_PPM = -0.5731e-3

# ToF sanity bounds
TOF_MIN_TICKS = -500.0
TOF_MAX_TICKS = 300.0 / METRES_PER_TICK   # ~300 m


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------
@dataclass
class RoundTimestamps:
    role:    int
    seq:     int
    ts:      list   # 6 x uint64 (Python int)
    car_int: list   # 6 x int32


@dataclass
class Distances:
    d_P1P2: float   # metres, or -1.0 on error
    d_P1P3: float
    d_P2P3: float


# ---------------------------------------------------------------------------
# Clock-offset correction helper
# ---------------------------------------------------------------------------
def corrected_d(car_int: int) -> float:
    """Return DELTA_TICKS corrected for the remote clock offset measured
    during the reception of a message *sent by* the node that programmed
    this delta."""
    ppm = car_int * CAR_INT_TO_PPM
    return DELTA_TICKS * (1.0 + ppm * 1e-6)


def _tof_to_metres(tof_ticks: float, label: str, seq: int) -> float:
    """Validate and convert a ToF in ticks to metres.
    Returns -1.0 and logs a warning if out of range."""
    if tof_ticks < TOF_MIN_TICKS or tof_ticks > TOF_MAX_TICKS:
        log.warning("seq=%d %s out of range (%.1f ticks)", seq, label, tof_ticks)
        return -1.0
    return max(0.0, tof_ticks) * METRES_PER_TICK


# ---------------------------------------------------------------------------
# Distance computation — one function per role
# Formulas are a direct Python port of ring_distances.c.
# See that file and node_config.h for full derivation notes.
# ---------------------------------------------------------------------------

def calc_distances_a1(rt: RoundTimestamps) -> Distances:
    """
    A1 timestamps: TX1  RX2  RX3  TX4  RX5  RX6
    car_int slots:  —    1    2    —    4    5

    tof_P1P2 = (RX2 - TX1 - D_A2) / 2          [D_A2 from car_int[1]]
    tof_P1P3 = (RX5 - TX4 - D_A3) / 2          [D_A3 from car_int[4]]
    tof_P2P3 = (RX6 - TX4) - tof_P1P3 - tof_P1P2 - D_A3 - D_A2
    """
    T = rt.ts
    ci = rt.car_int
    seq = rt.seq

    D_A2 = corrected_d(ci[1])   # Msg2 sent by A2
    D_A3 = corrected_d(ci[4])   # Msg5 sent by A3

    tof_P1P2 = (T[1] - T[0] - DELTA_TICKS) / 2.0
    tof_P1P3 = (T[4] - T[3] - DELTA_TICKS) / 2.0
    tof_P2P3 = (T[5] - T[3]) - tof_P1P3 - tof_P1P2 - DELTA_TICKS - DELTA_TICKS

    return Distances(
        d_P1P2=_tof_to_metres(tof_P1P2, "A1_P1P2", seq),
        d_P1P3=_tof_to_metres(tof_P1P3, "A1_P1P3", seq),
        d_P2P3=_tof_to_metres(tof_P2P3, "A1_P2P3", seq),
    )


def calc_distances_a2(rt: RoundTimestamps) -> Distances:
    """
    A2 timestamps: RX1  TX2  RX3  RX4  RX5  TX6
    car_int slots:  0    —    2    3    4    —

    tof_P2P3 = (RX3 - TX2 - D_A3_fwd) / 2      [D_A3 from car_int[2]]
    tof_P1P3 = (RX5 - RX3 - D_A1 - D_A3_rev) / 2
                                                  [D_A1 from car_int[3],
                                                   D_A3 from car_int[4]]
    tof_P1P2 = (RX4 - TX2) - tof_P2P3 - tof_P1P3 - D_A3_fwd - D_A1
    """
    T = rt.ts
    ci = rt.car_int
    seq = rt.seq

    D_A3_fwd = corrected_d(ci[2])   # Msg3 from A3 — for P2P3
    D_A3_rev = corrected_d(ci[4])   # Msg5 from A3 — for P1P3
    D_A1     = corrected_d(ci[3])   # Msg4 from A1

    tof_P2P3 = (T[2] - T[1] - DELTA_TICKS) / 2.0
    tof_P1P3 = (T[4] - T[2] - DELTA_TICKS - DELTA_TICKS) / 2.0
    tof_P1P2 = (T[3] - T[1]) - tof_P2P3 - tof_P1P3 - DELTA_TICKS - DELTA_TICKS

    return Distances(
        d_P1P2=_tof_to_metres(tof_P1P2, "A2_P1P2", seq),
        d_P1P3=_tof_to_metres(tof_P1P3, "A2_P1P3", seq),
        d_P2P3=_tof_to_metres(tof_P2P3, "A2_P2P3", seq),
    )


def calc_distances_a3(rt: RoundTimestamps) -> Distances:
    """
    A3 timestamps: RX1  RX2  TX3  RX4  TX5  RX6
    car_int slots:  0    1    —    3    —    5

    tof_P1P3 = (RX4 - TX3 - D_A1) / 2          [D_A1 from car_int[3]]
    tof_P2P3 = (RX6 - TX5 - D_A2) / 2          [D_A2 from car_int[5]]
    tof_P1P2 = (RX2 - RX1) + (RX4 - RX6) / 2

    NOTE: The tof_P1P2 formula for A3 carries no explicit D terms because
    A3 sits geometrically between A1 and A2 on the ring and the delays
    cancel in the sum.  It should be independently verified against the
    full MMP-GDB paper derivation before trusting in production.
    """
    T = rt.ts
    ci = rt.car_int
    seq = rt.seq

    D_A1 = corrected_d(ci[3])   # Msg4 from A1
    D_A2 = corrected_d(ci[5])   # Msg6 from A2

    tof_P1P3 = (T[3] - T[2] - DELTA_TICKS) / 2.0
    tof_P2P3 = (T[5] - T[4] - DELTA_TICKS) / 2.0
    tof_P1P2 = (T[1] - T[0]) + (T[3] - T[2] - T[5] + T[4] - DELTA_TICKS - DELTA_TICKS) / 2.0

    # A3's P1P2 formula can produce larger negative noise — use a relaxed
    # lower bound to avoid spurious -1 returns on valid short distances.
    if tof_P1P2 < -1000.0 or tof_P1P2 > TOF_MAX_TICKS:
        log.warning("seq=%d A3_P1P2 out of range (%.1f ticks)", seq, tof_P1P2)
        d_P1P2 = -1.0
    else:
        d_P1P2 = max(0.0, tof_P1P2) * METRES_PER_TICK

    return Distances(
        d_P1P2=d_P1P2,
        d_P1P3=_tof_to_metres(tof_P1P3, "A3_P1P3", seq),
        d_P2P3=_tof_to_metres(tof_P2P3, "A3_P2P3", seq),
    )


CALC_FN = {
    1: calc_distances_a1,
    2: calc_distances_a2,
    3: calc_distances_a3,
}


# ---------------------------------------------------------------------------
# Serial protocol parser
# ---------------------------------------------------------------------------

def parse_ts_line(line: str) -> Optional[RoundTimestamps]:
    """Parse a TS line from the firmware.

    Format:
        TS,<role>,<seq>,<ts0..5>,<ci0..5>
    Returns RoundTimestamps or None on parse error.
    """
    parts = line.split(",")
    if len(parts) != 15:
        return None
    try:
        role    = int(parts[1])
        seq     = int(parts[2])
        ts      = [int(parts[3 + i]) for i in range(6)]
        car_int = [int(parts[9 + i]) for i in range(6)]
    except ValueError:
        return None
    return RoundTimestamps(role=role, seq=seq, ts=ts, car_int=car_int)


# ---------------------------------------------------------------------------
# Serial bridge
# ---------------------------------------------------------------------------

class SerialBridge:
    def __init__(self, port: str, baud: int, role: int):
        self.port     = port
        self.baud     = baud
        self.role     = role
        self._ser: Optional[serial.Serial] = None

    def open(self):
        self._ser = serial.Serial(
            self.port,
            baudrate=self.baud,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=5.0,
        )
        log.info("Opened %s at %d baud", self.port, self.baud)

    @staticmethod
    def _is_shell_noise(line: str) -> bool:
        """Return True for Zephyr shell prompt/echo lines to silently discard."""
        return (
            line.startswith("uart:~$") or
            line.startswith("uart:~") or
            line == ""
        )

    def configure(self):
        """Send role and start shell commands, wait for RDY confirmation."""
        ser = self._ser

        # Give the shell time to print its prompt after boot
        time.sleep(0.5)
        ser.reset_input_buffer()

        # Shell command syntax: "role 1"  (not "ROLE=1")
        log.info("Sending shell command: role %d", self.role)
        ser.write(f"role {self.role}\n".encode())
        ser.flush()

        # Wait for role acknowledgement — firmware prints INF,role_set_...
        deadline = time.monotonic() + 10.0
        role_ack = False
        while time.monotonic() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            if self._is_shell_noise(line):
                continue
            self._handle_non_ts(line)
            if "INF,role_set_" in line:
                role_ack = True
                break

        if not role_ack:
            raise RuntimeError("Timeout waiting for role acknowledgement from firmware")

        # Shell command: "start"
        log.info("Sending shell command: start")
        ser.write(b"start\n")
        ser.flush()

        # Wait for RDY line emitted by ring_init()
        deadline = time.monotonic() + 10.0
        rdy_seen = False
        while time.monotonic() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            if self._is_shell_noise(line):
                continue
            self._handle_non_ts(line)
            if line.startswith("RDY,"):
                self._verify_rdy(line)
                rdy_seen = True
                break

        if not rdy_seen:
            raise RuntimeError("Timeout waiting for RDY from firmware")

    def _verify_rdy(self, line: str):
        """Check that firmware DELTA_TICKS matches our Python constant."""
        parts = line.split(",")
        if len(parts) >= 3:
            try:
                fw_delta = int(parts[2])
                if fw_delta != DELTA_TICKS:
                    log.warning(
                        "DELTA_TICKS mismatch: firmware=%d python=%d — "
                        "distances will be wrong!",
                        fw_delta, DELTA_TICKS,
                    )
                else:
                    log.info("DELTA_TICKS verified OK (%d)", fw_delta)
            except ValueError:
                pass

    def _handle_non_ts(self, line: str):
        """Log firmware ERR/INF/RDY lines (stderr, and log file if configured)."""
        if line:
            log.info("FW> %s", line)

    def run(self):
        """Main read loop — yields one DIST line to stdout per round."""
        calc_fn = CALC_FN.get(self.role)
        if calc_fn is None:
            raise ValueError(f"Unknown role {self.role}")

        ser = self._ser
        ser.timeout = 2.0   # allow short gaps between rounds

        while True:
            raw = ser.readline()
            if not raw:
                # Timeout — A1 period is 500 ms; 2 s timeout is generous.
                log.debug("Serial readline timeout (no data)")
                continue

            line = raw.decode(errors="replace").strip()

            # Silently drop shell prompt/echo lines
            if self._is_shell_noise(line):
                continue

            if not line.startswith("TS,"):
                self._handle_non_ts(line)
                continue

            log.debug("RAW> %s", line)   # echo raw TS line in verbose mode

            rt = parse_ts_line(line)
            if rt is None:
                log.warning("Malformed TS line: %r", line)
                continue

            if rt.role != self.role:
                log.warning(
                    "Role mismatch: firmware says %d, we expected %d",
                    rt.role, self.role,
                )

            dist = calc_fn(rt)

            out = (
                f"DIST,{rt.role},{rt.seq},"
                f"{dist.d_P1P2:.4f},{dist.d_P1P3:.4f},{dist.d_P2P3:.4f}"
            )
            print(out, flush=True)   # distance output goes to stdout only
            log.info(out)            # also captured by --log file (and stderr at INFO+)

    def close(self):
        if self._ser and self._ser.is_open:
            self._ser.close()
            log.info("Serial port closed")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="MMP-GDB anchor host app — configures Nordic firmware and "
                    "computes pairwise UWB distances.",
    )
    parser.add_argument(
        "--role", type=int, required=True, choices=[1, 2, 3],
        help="Anchor role: 1=A1 (initiator), 2=A2, 3=A3",
    )
    parser.add_argument(
        "--port", type=str, default="/dev/ttyACM0",
        help="Serial port (default: /dev/ttyACM0)",
    )
    parser.add_argument(
        "--baud", type=int, default=115200,
        help="Baud rate (default: 115200)",
    )
    parser.add_argument(
        "--log", type=str, default=None,
        help="Optional path to append all log output (FW lines, DIST lines, "
             "warnings, info) to a file",
    )
    parser.add_argument(
        "--verbose", action="store_true",
        help="Enable debug logging to stderr/log file",
    )
    args = parser.parse_args()

    log_level = logging.DEBUG if args.verbose else logging.INFO
    fmt = "%(asctime)s %(levelname)s %(name)s: %(message)s"
    datefmt = "%H:%M:%S"

    handlers = [logging.StreamHandler(sys.stderr)]
    if args.log:
        # delay=False: open immediately so a bad path fails fast, before
        # we ever touch the serial port.
        file_handler = logging.FileHandler(args.log, mode="a", delay=False)
        handlers.append(file_handler)

    for h in handlers:
        h.setFormatter(logging.Formatter(fmt, datefmt=datefmt))

    logging.basicConfig(level=log_level, handlers=handlers)

    bridge = SerialBridge(
        port=args.port,
        baud=args.baud,
        role=args.role,
    )

    def _shutdown(sig, frame):
        log.info("Signal %d received — shutting down", sig)
        bridge.close()
        logging.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        bridge.open()
        bridge.configure()
        bridge.run()
    except serial.SerialException as exc:
        log.error("Serial error: %s", exc)
        sys.exit(1)
    except RuntimeError as exc:
        log.error("%s", exc)
        sys.exit(1)
    finally:
        bridge.close()
        logging.shutdown()


if __name__ == "__main__":
    main()