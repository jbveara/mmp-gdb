#ifndef RING_DISTANCES_H
#define RING_DISTANCES_H

#include "ring_exchange.h"   /* round_timestamps_t */

/* ---------------------------------------------------------------------------
 * Distance results for one complete MMP-GDB round.
 *
 * All three pairwise distances are computed by every node independently
 * using only its own locally-observed timestamps and the known δ delay.
 * A value of -1.0 indicates a ranging error (negative or out-of-range ToF).
 * -------------------------------------------------------------------------*/
typedef struct {
    double d_P1P2;   /* A1 <-> A2, metres */
    double d_P1P3;   /* A1 <-> A3, metres */
    double d_P2P3;   /* A2 <-> A3, metres */
} ring_distances_t;

/* Compute distances from this node's locally-observed timestamps.
 * Applies the MMP-GDB column formula for the calling node's role
 * (selected at build time via ROLE_ANCHOR_A1/A2/A3).
 *
 * rt   — completed round_timestamps_t (rt->valid must be 1)
 * dist — output struct, always fully populated (–1.0 on error fields)
 *
 * Returns 0 if all three distances are valid, –1 if any ToF was bad. */
int ring_calc_distances(const round_timestamps_t *rt, ring_distances_t *dist);

#endif /* RING_DISTANCES_H */