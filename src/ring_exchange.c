/*
 * ring_exchange.c
 * MMP-GDB 6-message bidirectional ring exchange — runtime role selection.
 *
 * Role is set at runtime via ring_set_role() (called from main.c after the
 * host sends "ROLE=<1|2|3>\n" over UART).  A single firmware binary is
 * flashed to all three boards.
 *
 * Distance computation has been removed entirely from the firmware.
 * After each round the node emits one "TS,..." line over UART; the host
 * Python application performs all distance math.
 *
 * Message sequence:
 *   Msg1: A1 → all   (forward initiation)
 *   Msg2: A2 → all   (forward hop 1)
 *   Msg3: A3 → all   (forward hop 2)
 *   Msg4: A1 → all   (reverse initiation)
 *   Msg5: A3 → all   (reverse hop 1)
 *   Msg6: A2 → all   (reverse hop 2 / round close)
 */

#include "ring_exchange.h"
#include "node_config.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include <stddef.h>

#include "deca_device_api.h"
#include "deca_interface.h"
#include "shared_defines.h"
#include "shared_functions.h"
#include "port.h"
#include "config_options.h"

/* ---------------------------------------------------------------------------
 * Runtime role state
 * -------------------------------------------------------------------------*/
static anchor_role_t g_role = ROLE_NONE;

void ring_set_role(anchor_role_t role)
{
    g_role = role;
}

/* ---------------------------------------------------------------------------
 * DW3000 config
 * -------------------------------------------------------------------------*/
static dwt_config_t uwb_config = {
    .chan            = UWB_CHANNEL,
    .txPreambLength  = DWT_PLEN_128,
    .rxPAC           = DWT_PAC8,
    .txCode          = UWB_PREAM_CODE,
    .rxCode          = UWB_PREAM_CODE,
    .sfdType         = DWT_SFD_DW_8,
    .dataRate        = DWT_BR_6M8,
    .phrMode         = DWT_PHRMODE_STD,
    .phrRate         = DWT_PHRRATE_STD,
    .sfdTO           = UWB_SFD_TIMEOUT,
    .stsMode         = DWT_STS_MODE_OFF,
    .stsLength       = DWT_STS_LEN_64,
    .pdoaMode        = DWT_PDOA_M0,
};

/* ---------------------------------------------------------------------------
 * Frame buffers
 * -------------------------------------------------------------------------*/
static uint8_t tx_buf[FRAME_LEN_MAX_APP];
static uint8_t rx_buf[FRAME_LEN_MAX_APP];

static uint8_t seq_num = 0;

/* ---------------------------------------------------------------------------
 * Timestamp packing helpers (5-byte LE, DWT 40-bit counter)
 * -------------------------------------------------------------------------*/
static void pack_ts(uint8_t *buf, uint64_t ts)
{
    buf[0] = (uint8_t)(ts & 0xFF);
    buf[1] = (uint8_t)((ts >>  8) & 0xFF);
    buf[2] = (uint8_t)((ts >> 16) & 0xFF);
    buf[3] = (uint8_t)((ts >> 24) & 0xFF);
    buf[4] = (uint8_t)((ts >> 32) & 0xFF);
}

static uint64_t unpack_ts(const uint8_t *buf)
{
    return  (uint64_t)buf[0]        |
           ((uint64_t)buf[1] <<  8) |
           ((uint64_t)buf[2] << 16) |
           ((uint64_t)buf[3] << 24) |
           ((uint64_t)buf[4] << 32);
}

/* ---------------------------------------------------------------------------
 * Frame builder — broadcast destination
 * -------------------------------------------------------------------------*/
static uint8_t build_frame(uint8_t fc, uint16_t src,
                            const uint8_t *payload, uint8_t plen)
{
    memset(tx_buf, 0, sizeof(tx_buf));
    tx_buf[FRAME_FC_IDX]       = fc;
    tx_buf[FRAME_SEQ_IDX]      = seq_num;
    tx_buf[FRAME_PAN_IDX]      = (uint8_t)(ADDR_PAN & 0xFF);
    tx_buf[FRAME_PAN_IDX + 1]  = (uint8_t)(ADDR_PAN >> 8);
    tx_buf[FRAME_DEST_IDX]     = (uint8_t)(ADDR_BROADCAST & 0xFF);
    tx_buf[FRAME_DEST_IDX + 1] = (uint8_t)(ADDR_BROADCAST >> 8);
    tx_buf[FRAME_SRC_IDX]      = (uint8_t)(src & 0xFF);
    tx_buf[FRAME_SRC_IDX + 1]  = (uint8_t)(src >> 8);
    if (payload && plen) {
        memcpy(&tx_buf[FRAME_PAYLOAD_IDX], payload, plen);
    }
    return FRAME_PAYLOAD_IDX + plen + 2;
}

/* ---------------------------------------------------------------------------
 * TX helpers
 * -------------------------------------------------------------------------*/
static int send_immediate(uint8_t fc, uint16_t src,
                          const uint8_t *payload, uint8_t plen)
{
    uint8_t flen = build_frame(fc, src, payload, plen);
    dwt_writetxdata(flen, tx_buf, 0);
    dwt_writetxfctrl(flen, 0, 1);
    if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
        printk("ERR,0,send_immediate_FC%02X\n", fc);
        return -1;
    }
    waitforsysstatus(NULL, NULL, DWT_INT_TXFRS_BIT_MASK, 0);
    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
    return 0;
}

static int send_delayed(uint8_t fc, uint16_t src,
                        const uint8_t *payload, uint8_t plen,
                        uint64_t rx_ts, uint32_t *tx_time_out)
{
    uint32_t ref_time  = (uint32_t)(rx_ts >> 8);
    uint32_t dx_offset = (uint32_t)(RESP_TX_DELAY_TICKS >> 8);
    dwt_setreferencetrxtime(ref_time);
    dwt_setdelayedtrxtime(dx_offset);

    uint32_t tx_time = ref_time + dx_offset;
    if (tx_time_out) *tx_time_out = tx_time;

    uint8_t flen = build_frame(fc, src, payload, plen);
    dwt_writetxdata(flen, tx_buf, 0);
    dwt_writetxfctrl(flen, 0, 1);

    if (dwt_starttx(DWT_START_TX_DLY_REF) != DWT_SUCCESS) {
        printk("ERR,0,send_delayed_FC%02X\n", fc);
        /* Do NOT soft-reset here — caller will continue/return and the
         * responder loop will re-sync on the next Msg1 naturally. */
        return -1;
    }
    waitforsysstatus(NULL, NULL, DWT_INT_TXFRS_BIT_MASK, 0);
    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
    return 0;
}

/* ---------------------------------------------------------------------------
 * RX helper
 * Returns FC byte on success, -1 on timeout/error.
 * -------------------------------------------------------------------------*/
static int wait_rx(uint64_t *rx_ts_out, int32_t *car_int_out)
{
    uint32_t status_lo = 0;
    dwt_setrxtimeout(RX_TIMEOUT_UUS);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    waitforsysstatus(&status_lo, NULL,
                     DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR,
                     0);

    if (!(status_lo & DWT_INT_RXFCG_BIT_MASK)) {
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        return -1;
    }

    uint16_t flen = dwt_getframelength();
    dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

    if (flen < FRAME_PAYLOAD_IDX + 2 || flen > FRAME_LEN_MAX_APP) {
        return -1;
    }
    dwt_readrxdata(rx_buf, flen - 2, 0);

    /* Basic PAN ID sanity check — reject frames from other networks */
    uint16_t pan = (uint16_t)rx_buf[FRAME_PAN_IDX] |
                   ((uint16_t)rx_buf[FRAME_PAN_IDX + 1] << 8);
    if (pan != ADDR_PAN) {
        return -1;
    }

    if (rx_ts_out) {
        uint64_t _ts = get_rx_timestamp_u64();
        memcpy(rx_ts_out, &_ts, sizeof(_ts));
    }
    if (car_int_out) {
        *car_int_out = dwt_readcarrierintegrator();
    }
    return rx_buf[FRAME_FC_IDX];
}

/* Compute predicted TX timestamp from the value written to the DX_TIME
 * register (bits[39:8]).  The hardware adds TX_ANT_DLY to the stamp. */
static uint64_t predicted_tx_ts(uint32_t tx_time)
{
    return (((uint64_t)(tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
}

/* ---------------------------------------------------------------------------
 * ring_init
 * -------------------------------------------------------------------------*/
int ring_init(void)
{
    if (g_role == ROLE_NONE) {
        printk("ERR,0,ring_init_no_role\n");
        return -1;
    }

    if (dwt_configure(&uwb_config) != DWT_SUCCESS) {
        printk("ERR,0,dwt_configure_failed\n");
        return -1;
    }

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_configureframefilter(DWT_FF_DISABLE, 0);

    /* Announce readiness to host.
     * Format: RDY,<role>,<DELTA_TICKS>,<TX_ANT_DLY>
     * The host uses DELTA_TICKS to verify its local constant matches. */
    printk("RDY,%u,%llu,%u\n",
           (unsigned)g_role,
           (unsigned long long)DELTA_TICKS,
           (unsigned)TX_ANT_DLY);

    return 0;
}

/* ---------------------------------------------------------------------------
 * ring_print_timestamps
 *
 * Emits one line consumed by the host Python app:
 *
 *   TS,<role>,<seq>,<ts0>,<ts1>,<ts2>,<ts3>,<ts4>,<ts5>,
 *      <ci0>,<ci1>,<ci2>,<ci3>,<ci4>,<ci5>
 *
 * All timestamps are unsigned 64-bit decimal.
 * car_int values are signed 32-bit decimal.
 * -------------------------------------------------------------------------*/
void ring_print_timestamps(const round_timestamps_t *rt)
{
    if (!rt->valid) {
        printk("ERR,%u,invalid_round\n", rt->seq);
        return;
    }
    printk("TS,%u,%u,"
           "%llu,%llu,%llu,%llu,%llu,%llu,"
           "%d,%d,%d,%d,%d,%d\n",
           (unsigned)g_role,
           (unsigned)rt->seq,
           (unsigned long long)rt->ts[0],
           (unsigned long long)rt->ts[1],
           (unsigned long long)rt->ts[2],
           (unsigned long long)rt->ts[3],
           (unsigned long long)rt->ts[4],
           (unsigned long long)rt->ts[5],
           rt->car_int[0],
           rt->car_int[1],
           rt->car_int[2],
           rt->car_int[3],
           rt->car_int[4],
           rt->car_int[5]);
}

/* ---------------------------------------------------------------------------
 * A1 — INITIATOR
 * Sends Msg1, listens for Msg2+Msg3, sends Msg4, listens for Msg5+Msg6.
 * -------------------------------------------------------------------------*/
__attribute__((noinline)) int ring_run_round(round_timestamps_t *rt)
{
    if (g_role != ROLE_A1) {
        return -1;
    }

    memset(rt, 0, sizeof(*rt));
    rt->seq = seq_num;

    /* Msg1: A1 → all */
    if (send_immediate(FC_MSG1, ADDR_A1, NULL, 0) != 0) return -1;
    rt->ts[0] = get_tx_timestamp_u64();   /* TX1 — car_int[0] unused (TX) */

    /* Msg2: listen for A2 */
    int fc = wait_rx(&rt->ts[1], &rt->car_int[1]);
    if (fc != FC_MSG2) {
        printk("ERR,%u,expected_MSG2_got_%02X\n", seq_num, (unsigned)fc);
        return -1;
    }

    /* Msg3: listen for A3 */
    fc = wait_rx(&rt->ts[2], &rt->car_int[2]);
    if (fc != FC_MSG3) {
        printk("ERR,%u,expected_MSG3_got_%02X\n", seq_num, (unsigned)fc);
        return -1;
    }

    /* Msg4: A1 → all (delayed from RX3) */
    uint32_t tx4_time;
    if (send_delayed(FC_MSG4, ADDR_A1, NULL, 0, rt->ts[2], &tx4_time) != 0) return -1;
    rt->ts[3] = predicted_tx_ts(tx4_time); /* TX4 — car_int[3] unused (TX) */

    /* Msg5: listen for A3 */
    fc = wait_rx(&rt->ts[4], &rt->car_int[4]);
    if (fc != FC_MSG5) {
        printk("ERR,%u,expected_MSG5_got_%02X\n", seq_num, (unsigned)fc);
        return -1;
    }

    /* Msg6: listen for A2 */
    fc = wait_rx(&rt->ts[5], &rt->car_int[5]);
    if (fc != FC_MSG6) {
        printk("ERR,%u,expected_MSG6_got_%02X\n", seq_num, (unsigned)fc);
        return -1;
    }

    rt->valid = 1;
    seq_num++;
    return 0;
}

/* ---------------------------------------------------------------------------
 * A2 — RESPONDER
 * Listens for Msg1, sends Msg2, listens for Msg3+Msg4+Msg5, sends Msg6.
 * -------------------------------------------------------------------------*/
static void run_a2(void)
{
    round_timestamps_t rt;

    while (true) {
        memset(&rt, 0, sizeof(rt));

        /* Msg1: wait for A1 */
        int fc = wait_rx(&rt.ts[0], &rt.car_int[0]);
        if (fc != FC_MSG1) continue;
        rt.seq = rx_buf[FRAME_SEQ_IDX];

        /* Msg2: A2 → all (delayed from RX1) */
        uint32_t tx2_time;
        if (send_delayed(FC_MSG2, ADDR_A2, NULL, 0, rt.ts[0], &tx2_time) != 0) continue;
        rt.ts[1] = predicted_tx_ts(tx2_time); /* TX2 — car_int[1] unused (TX) */

        /* Msg3: listen for A3 */
        fc = wait_rx(&rt.ts[2], &rt.car_int[2]);
        if (fc != FC_MSG3) {
            printk("ERR,%u,A2_expected_MSG3_got_%02X\n", rt.seq, (unsigned)fc);
            continue;
        }

        /* Msg4: listen for A1 */
        fc = wait_rx(&rt.ts[3], &rt.car_int[3]);
        if (fc != FC_MSG4) {
            printk("ERR,%u,A2_expected_MSG4_got_%02X\n", rt.seq, (unsigned)fc);
            continue;
        }

        /* Msg5: listen for A3 */
        fc = wait_rx(&rt.ts[4], &rt.car_int[4]);
        if (fc != FC_MSG5) {
            printk("ERR,%u,A2_expected_MSG5_got_%02X\n", rt.seq, (unsigned)fc);
            continue;
        }

        /* Msg6: A2 → all (delayed from RX5) */
        uint32_t tx6_time;
        if (send_delayed(FC_MSG6, ADDR_A2, NULL, 0, rt.ts[4], &tx6_time) != 0) continue;
        rt.ts[5] = predicted_tx_ts(tx6_time); /* TX6 — car_int[5] unused (TX) */

        rt.valid = 1;
        ring_print_timestamps(&rt);
    }
}

/* ---------------------------------------------------------------------------
 * A3 — RESPONDER
 * Listens for Msg1+Msg2, sends Msg3, listens for Msg4, sends Msg5, listens Msg6.
 * -------------------------------------------------------------------------*/
static void run_a3(void)
{
    round_timestamps_t rt;

    while (true) {
        memset(&rt, 0, sizeof(rt));

        /* Msg1: wait for A1 */
        int fc = wait_rx(&rt.ts[0], &rt.car_int[0]);
        if (fc != FC_MSG1) continue;
        rt.seq = rx_buf[FRAME_SEQ_IDX];

        /* Msg2: listen for A2 */
        fc = wait_rx(&rt.ts[1], &rt.car_int[1]);
        if (fc != FC_MSG2) {
            printk("ERR,%u,A3_expected_MSG2_got_%02X\n", rt.seq, (unsigned)fc);
            continue;
        }

        /* Msg3: A3 → all (delayed from RX2) */
        uint32_t tx3_time;
        if (send_delayed(FC_MSG3, ADDR_A3, NULL, 0, rt.ts[1], &tx3_time) != 0) continue;
        rt.ts[2] = predicted_tx_ts(tx3_time); /* TX3 — car_int[2] unused (TX) */

        /* Msg4: listen for A1 */
        fc = wait_rx(&rt.ts[3], &rt.car_int[3]);
        if (fc != FC_MSG4) {
            printk("ERR,%u,A3_expected_MSG4_got_%02X\n", rt.seq, (unsigned)fc);
            continue;
        }

        /* Msg5: A3 → all (delayed from RX4) */
        uint32_t tx5_time;
        if (send_delayed(FC_MSG5, ADDR_A3, NULL, 0, rt.ts[3], &tx5_time) != 0) continue;
        rt.ts[4] = predicted_tx_ts(tx5_time); /* TX5 — car_int[4] unused (TX) */

        /* Msg6: listen for A2 */
        fc = wait_rx(&rt.ts[5], &rt.car_int[5]);
        if (fc != FC_MSG6) {
            printk("ERR,%u,A3_expected_MSG6_got_%02X\n", rt.seq, (unsigned)fc);
            continue;
        }

        rt.valid = 1;
        ring_print_timestamps(&rt);
    }
}

/* ---------------------------------------------------------------------------
 * ring_responder_loop — dispatches to A2 or A3 based on runtime role.
 * Never returns.
 * -------------------------------------------------------------------------*/
void ring_responder_loop(void)
{
    if (g_role == ROLE_A2) {
        printk("INF,A2_listening\n");
        run_a2();
    } else if (g_role == ROLE_A3) {
        printk("INF,A3_listening\n");
        run_a3();
    } else {
        printk("ERR,0,responder_loop_called_on_non_responder\n");
    }
}
