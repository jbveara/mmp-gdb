/*
 * ring_distances.c — MMP-GDB distance computation
 *
 * Each node calls ring_calc_distances() after a complete round.
 * Every node observes all 6 broadcasts, so each has its own set of
 * 6 locally-measured timestamps and applies the column formula for its role.
 *
 * Timestamp array layout:
 *   ts[0]=Msg1  ts[1]=Msg2  ts[2]=Msg3  ts[3]=Msg4  ts[4]=Msg5  ts[5]=Msg6
 *
 *   A1: TX1  RX2  RX3  TX4  RX5  RX6
 *   A2: RX1  TX2  RX3  RX4  RX5  TX6
 *   A3: RX1  RX2  TX3  RX4  TX5  RX6
 *
 * Delta handling
 * --------------
 * Every responder programs its TX delay using RESP_TX_DELAY_TICKS directly
 * in raw 40-bit tick space (see send_delayed in ring_exchange.c).  The
 * hardware therefore stamps TX ≈ RX_trigger + RESP_TX_DELAY_TICKS, with the
 * only error being the ≤256-tick sub-byte rounding of the TX register
 * (~1.2 mm worst case).  Using the same constant here means the formula
 * delta always matches what was actually stamped — no per-board calibration,
 * no crystal-frequency correction needed.
 *
 * DWT tick period ≈ 1 / (499.2 MHz × 128) ≈ 15.65 ps
 * Speed of light:  299,792,458 m/s
 */

#include "ring_distances.h"
#include "node_config.h"

#include <zephyr/sys/printk.h>

/* ---------------------------------------------------------------------------
 * Physical constants
 * -------------------------------------------------------------------------*/
#define DWT_FREQ_HZ         (499.2e6 * 128.0)
#define SPEED_OF_LIGHT_MPS   299792458.0
#define METRES_PER_TICK     (SPEED_OF_LIGHT_MPS / DWT_FREQ_HZ)

/* Single shared delta — RESP_TX_DELAY_TICKS + TX_ANT_DLY matches what
 * predicted_tx_ts actually stamps (see node_config.h for derivation). */
#define D  ((double)DELTA_TICKS)

/* Guard: reject ToF outside [0, 300 m] */
#define TOF_MAX_TICKS  (300.0 / METRES_PER_TICK)

/* ---------------------------------------------------------------------------
 * ring_calc_distances
 * -------------------------------------------------------------------------*/
int ring_calc_distances(const round_timestamps_t *rt, ring_distances_t *dist)
{
    if (!rt || !rt->valid || !dist) {
        return -1;
    }

    const double T1 = (double)rt->ts[0];
    const double T2 = (double)rt->ts[1];
    const double T3 = (double)rt->ts[2];
    const double T4 = (double)rt->ts[3];
    const double T5 = (double)rt->ts[4];
    const double T6 = (double)rt->ts[5];

    int ret = 0;

#if defined(ROLE_ANCHOR_A1)
    /*
     * A1 timestamps: TX1  RX2  RX3  TX4  RX5  RX6
     *
     * tof_P1P2: A2 replied to Msg1 with delay D
     *   RX2 - TX1 = 2·tof_P1P2 + D
     *   tof_P1P2 = (RX2 - TX1 - D) / 2
     *
     * tof_P1P3: A3 replied to Msg4 with delay D
     *   RX5 - TX4 = 2·tof_P1P3 + D
     *   tof_P1P3 = (RX5 - TX4 - D) / 2
     *
     * tof_P2P3: A2 replied to Msg5 (from A3) with delay D.
     *   RX6 = TX4 + tof_P1P3 + D + tof_P2P3 + D + tof_P1P2
     *   tof_P2P3 = (RX6 - TX4) - tof_P1P3 - tof_P1P2 - 2D
     */
    double tof_P1P2 = (T2 - T1 - D) / 2.0;
    double tof_P1P3 = (T5 - T4 - D) / 2.0;
    double tof_P2P3 = (T6 - T4) - tof_P1P3 - tof_P1P2 - 2.0*D;

    if (tof_P1P2 < -500.0 || tof_P1P2 > TOF_MAX_TICKS) {
        printk("A1: tof_P1P2 out of range (%.1f ticks)\n", tof_P1P2);
        dist->d_P1P2 = -1.0; ret = -1;
    } else {
        dist->d_P1P2 = (tof_P1P2 < 0.0 ? 0.0 : tof_P1P2) * METRES_PER_TICK;
    }

    if (tof_P1P3 < -500.0 || tof_P1P3 > TOF_MAX_TICKS) {
        printk("A1: tof_P1P3 out of range (%.1f ticks)\n", tof_P1P3);
        dist->d_P1P3 = -1.0; ret = -1;
    } else {
        dist->d_P1P3 = (tof_P1P3 < 0.0 ? 0.0 : tof_P1P3) * METRES_PER_TICK;
    }

    if (tof_P2P3 < -500.0 || tof_P2P3 > TOF_MAX_TICKS) {
        printk("A1: tof_P2P3 out of range (%.1f ticks)\n", tof_P2P3);
        dist->d_P2P3 = -1.0; ret = -1;
    } else {
        dist->d_P2P3 = (tof_P2P3 < 0.0 ? 0.0 : tof_P2P3) * METRES_PER_TICK;
    }

    printk("A1 seq=%u  d_P1P2=%.3f m  d_P1P3=%.3f m  d_P2P3=%.3f m\n",
           rt->seq, dist->d_P1P2, dist->d_P1P3, dist->d_P2P3);

#elif defined(ROLE_ANCHOR_A2)
    /*
     * A2 timestamps: RX1  TX2  RX3  RX4  RX5  TX6
     *
     * tof_P2P3: A3 replied to Msg2 with delay D
     *   RX3 - TX2 = 2·tof_P2P3 + D
     *   tof_P2P3 = (RX3 - TX2 - D) / 2
     *
     * tof_P1P3: tof(A3→A2) cancels in RX5-RX3 (both sent by A3),
     *   leaving TX5_A3 - TX3_A3 = 2·tof_P1P3 + 2D
     *   tof_P1P3 = (RX5 - RX3 - 2D) / 2
     *
     * tof_P1P2: A1 sent Msg4 after receiving Msg3 with delay D.
     *   RX4 = TX2 + tof_P2P3 + D + tof_P1P3 + D + tof_P1P2
     *   tof_P1P2 = (RX4 - TX2) - tof_P2P3 - tof_P1P3 - 2D
     */
    double tof_P2P3 = (T3 - T2 - D) / 2.0;
    double tof_P1P3 = (T5 - T3 - 2.0*D) / 2.0;
    double tof_P1P2 = (T4 - T2) - tof_P2P3 - tof_P1P3 - 2.0*D;

    if (tof_P2P3 < -500.0 || tof_P2P3 > TOF_MAX_TICKS) {
        printk("A2: tof_P2P3 out of range (%.1f ticks)\n", tof_P2P3);
        dist->d_P2P3 = -1.0; ret = -1;
    } else {
        dist->d_P2P3 = (tof_P2P3 < 0.0 ? 0.0 : tof_P2P3) * METRES_PER_TICK;
    }

    if (tof_P1P3 < -500.0 || tof_P1P3 > TOF_MAX_TICKS) {
        printk("A2: tof_P1P3 out of range (%.1f ticks)\n", tof_P1P3);
        dist->d_P1P3 = -1.0; ret = -1;
    } else {
        dist->d_P1P3 = (tof_P1P3 < 0.0 ? 0.0 : tof_P1P3) * METRES_PER_TICK;
    }

    if (tof_P1P2 < -500.0 || tof_P1P2 > TOF_MAX_TICKS) {
        printk("A2: tof_P1P2 out of range (%.1f ticks)\n", tof_P1P2);
        dist->d_P1P2 = -1.0; ret = -1;
    } else {
        dist->d_P1P2 = (tof_P1P2 < 0.0 ? 0.0 : tof_P1P2) * METRES_PER_TICK;
    }

    printk("A2 seq=%u  d_P1P2=%.3f m  d_P1P3=%.3f m  d_P2P3=%.3f m\n",
           rt->seq, dist->d_P1P2, dist->d_P1P3, dist->d_P2P3);

#elif defined(ROLE_ANCHOR_A3)
    /*
     * A3 timestamps: RX1  RX2  TX3  RX4  TX5  RX6
     *
     * tof_P1P3: A1 replied to Msg3 with delay D
     *   RX4 - TX3 = 2·tof_P1P3 + D
     *   tof_P1P3 = (RX4 - TX3 - D) / 2
     *
     * tof_P2P3: A2 replied to Msg5 with delay D
     *   RX6 - TX5 = 2·tof_P2P3 + D
     *   tof_P2P3 = (RX6 - TX5 - D) / 2
     *
     * tof_P1P2: no D terms — pure timestamp geometry, D cancels.
     *   tof_P1P2 = (RX2 - RX1) + (RX4 - RX6) / 2
     *   Crystal offset does not bias this term. Small negatives
     *   (<1000 ticks, ~5 mm) are sub-register jitter, clamped to zero.
     */
    double tof_P1P3 = (T4 - T3 - D) / 2.0;
    double tof_P2P3 = (T6 - T5 - D) / 2.0;
    double tof_P1P2 = (T2 - T1) + (T4 - T6) / 2.0;

    if (tof_P1P3 < -500.0 || tof_P1P3 > TOF_MAX_TICKS) {
        printk("A3: tof_P1P3 out of range (%.1f ticks)\n", tof_P1P3);
        dist->d_P1P3 = -1.0; ret = -1;
    } else {
        dist->d_P1P3 = (tof_P1P3 < 0.0 ? 0.0 : tof_P1P3) * METRES_PER_TICK;
    }

    if (tof_P2P3 < -500.0 || tof_P2P3 > TOF_MAX_TICKS) {
        printk("A3: tof_P2P3 out of range (%.1f ticks)\n", tof_P2P3);
        dist->d_P2P3 = -1.0; ret = -1;
    } else {
        dist->d_P2P3 = (tof_P2P3 < 0.0 ? 0.0 : tof_P2P3) * METRES_PER_TICK;
    }

    if (tof_P1P2 < -1000.0 || tof_P1P2 > TOF_MAX_TICKS) {
        printk("A3: tof_P1P2 out of range (%.1f ticks)\n", tof_P1P2);
        dist->d_P1P2 = -1.0; ret = -1;
    } else {
        dist->d_P1P2 = (tof_P1P2 < 0.0 ? 0.0 : tof_P1P2) * METRES_PER_TICK;
    }

    printk("A3 seq=%u  d_P1P2=%.3f m  d_P1P3=%.3f m  d_P2P3=%.3f m\n",
           rt->seq, dist->d_P1P2, dist->d_P1P3, dist->d_P2P3);

#endif

    return ret;
}