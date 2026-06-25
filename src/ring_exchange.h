#ifndef RING_EXCHANGE_H
#define RING_EXCHANGE_H

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Per-node timestamp log for one complete MMP-GDB round (6 messages).
 *
 * Each field is either a TX timestamp (when this node sent the message)
 * or an RX timestamp (when this node received the message).
 * Unused fields (0) indicate the node neither sent nor received that message
 * in its primary role — but in practice all nodes receive all messages.
 *
 * Naming: tx_msgN = TX time of message N, rx_msgN = RX time of message N.
 * All timestamps in raw DWT units (~15.65 ps per unit, 40-bit counter).
 *
 * A1: tx_msg1, rx_msg2, rx_msg3, tx_msg4, rx_msg5, rx_msg6
 * A2: rx_msg1, tx_msg2, rx_msg3, rx_msg4, rx_msg5, tx_msg6
 * A3: rx_msg1, rx_msg2, tx_msg3, rx_msg4, tx_msg5, rx_msg6
 * -------------------------------------------------------------------------*/
typedef struct {
    uint8_t  valid;    /* 1 when all 6 messages received/sent successfully */
    uint8_t  seq;      /* round sequence number */
    uint64_t ts[6];    /* ts[0]=msg1 .. ts[5]=msg6, TX or RX per role */
} __attribute__((packed)) round_timestamps_t;

/* ---------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------*/

/* Initialise DW3000 for this node's role. Call once after dwt_initialise(). */
int ring_init(void);

/* A1 only: run one complete round (all 6 messages). Fills *rt on success.
 * Returns 0 on success, -1 on error/timeout. */
__attribute__((noinline)) int ring_run_round(round_timestamps_t *rt);

/* A2/A3 only: blocking responder loop. Never returns. */
void ring_responder_loop(void);

/* Print timestamps to RTT. */
void ring_print_timestamps(const round_timestamps_t *rt);

#endif /* RING_EXCHANGE_H */
