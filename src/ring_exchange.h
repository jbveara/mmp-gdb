#ifndef RING_EXCHANGE_H
#define RING_EXCHANGE_H

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Runtime role identifiers (set via ring_set_role() before ring_init())
 * -------------------------------------------------------------------------*/
typedef enum {
    ROLE_NONE = 0,
    ROLE_A1   = 1,   /* initiator */
    ROLE_A2   = 2,   /* responder */
    ROLE_A3   = 3,   /* responder */
} anchor_role_t;

/* ---------------------------------------------------------------------------
 * Per-node timestamp log for one complete MMP-GDB round (6 messages).
 *
 * ts[N] is a TX timestamp when this node sent message N+1, or an RX
 * timestamp when it received it.  car_int[N] holds the DW3000 carrier
 * integrator reading at each RX event; 0 for TX slots.
 *
 * Role      msg:  1     2     3     4     5     6
 * A1              TX1   RX2   RX3   TX4   RX5   RX6
 * A2              RX1   TX2   RX3   RX4   RX5   TX6
 * A3              RX1   RX2   TX3   RX4   TX5   RX6
 * -------------------------------------------------------------------------*/
typedef struct {
    uint8_t  valid;      /* 1 when all 6 messages completed successfully */
    uint8_t  seq;        /* round sequence number */
    uint64_t ts[6];      /* ts[0]=msg1 .. ts[5]=msg6 */
    int32_t  car_int[6]; /* carrier integrator at each RX; 0 for TX slots */
} round_timestamps_t;

/* ---------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------*/

/* Set role before calling ring_init(). */
void ring_set_role(anchor_role_t role);

/* Initialise DW3000 for the configured role. Call after dwt_initialise(). */
int ring_init(void);

/* A1 only: run one complete round.  Fills *rt on success.
 * Returns 0 on success, -1 on error/timeout. */
__attribute__((noinline)) int ring_run_round(round_timestamps_t *rt);

/* A2/A3 only: blocking responder loop.  Never returns. */
void ring_responder_loop(void);

/* Emit one TS line to UART for the host. */
void ring_print_timestamps(const round_timestamps_t *rt);

#endif /* RING_EXCHANGE_H */
