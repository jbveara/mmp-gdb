/*
 * ring_exchange.c
 * MMP-GDB 6-message bidirectional ring exchange.
 *
 * Message sequence:
 *   Msg1: A1 → all   (forward initiation)
 *   Msg2: A2 → all   (forward hop 1)
 *   Msg3: A3 → all   (forward hop 2)
 *   Msg4: A1 → all   (reverse initiation)
 *   Msg5: A3 → all   (reverse hop 1)
 *   Msg6: A2 → all   (reverse hop 2 / round close)
 *
 * All messages are broadcasts — every node receives every message.
 * Each node logs TX or RX timestamp for each message per its role.
 *
 * Phase 2: plain UWB, no crypto. Crypto commitment added in Phase 3.
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
 * Frame buffers (TX and RX share separate buffers)
 * -------------------------------------------------------------------------*/
static uint8_t tx_buf[FRAME_LEN_MAX_APP];
static uint8_t rx_buf[FRAME_LEN_MAX_APP];

static uint8_t seq_num = 0;

/* ---------------------------------------------------------------------------
 * Timestamp packing helpers (5 bytes LE, DWT 40-bit)
 * -------------------------------------------------------------------------*/
static void pack_ts(uint8_t *buf, uint64_t ts)
{
    buf[0] = (uint8_t)(ts & 0xFF);
    buf[1] = (uint8_t)((ts >> 8)  & 0xFF);
    buf[2] = (uint8_t)((ts >> 16) & 0xFF);
    buf[3] = (uint8_t)((ts >> 24) & 0xFF);
    buf[4] = (uint8_t)((ts >> 32) & 0xFF);
}

static uint64_t unpack_ts(const uint8_t *buf)
{
    return  (uint64_t)buf[0]        |
           ((uint64_t)buf[1] << 8)  |
           ((uint64_t)buf[2] << 16) |
           ((uint64_t)buf[3] << 24) |
           ((uint64_t)buf[4] << 32);
}

/* ---------------------------------------------------------------------------
 * Frame builder — broadcast destination
 * Returns total frame length including 2-byte HW CRC placeholder.
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
 * send_immediate: TX now, no RX after
 * send_delayed:   TX at scheduled time
 * Both return 0 on success, -1 on failure.
 * -------------------------------------------------------------------------*/
static int send_immediate(uint8_t fc, uint16_t src,
                          const uint8_t *payload, uint8_t plen)
{
    uint8_t flen = build_frame(fc, src, payload, plen);
    dwt_writetxdata(flen, tx_buf, 0);
    dwt_writetxfctrl(flen, 0, 1);
    if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
        printk("send_immediate FC=0x%02X failed\n", fc);
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
    uint32_t tx_time = (uint32_t)((rx_ts + (RESP_TX_DELAY_UUS * UUS_TO_DWT_TIME)) >> 8);
    dwt_setdelayedtrxtime(tx_time);
    if (tx_time_out) *tx_time_out = tx_time;

    uint8_t flen = build_frame(fc, src, payload, plen);
    dwt_writetxdata(flen, tx_buf, 0);
    dwt_writetxfctrl(flen, 0, 1);

    if (dwt_starttx(DWT_START_TX_DELAYED) != DWT_SUCCESS) {
        printk("send_delayed FC=0x%02X failed\n", fc);
        dwt_softreset(0);
        k_msleep(10);
        return -1;
    }
    waitforsysstatus(NULL, NULL, DWT_INT_TXFRS_BIT_MASK, 0);
    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
    return 0;
}

/* ---------------------------------------------------------------------------
 * RX helper — wait for next frame, return FC or -1 on timeout/error.
 * Fills rx_buf and records RX timestamp via get_rx_timestamp_u64().
 * -------------------------------------------------------------------------*/
static int wait_rx(uint64_t *rx_ts_out)
{
    uint32_t status_lo = 0;
    dwt_setrxtimeout(RX_TIMEOUT_UUS);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    waitforsysstatus(&status_lo, NULL,
                     DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR, 0);

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

    if (rx_ts_out) {
        uint64_t _ts = get_rx_timestamp_u64(); memcpy(rx_ts_out, &_ts, sizeof(_ts));
    }
    return rx_buf[FRAME_FC_IDX];
}

/* Compute predicted TX timestamp from delayed TX time */
static uint64_t predicted_tx_ts(uint32_t tx_time)
{
    return (((uint64_t)tx_time) << 8) + TX_ANT_DLY;
}

/* ---------------------------------------------------------------------------
 * ring_init
 * -------------------------------------------------------------------------*/
int ring_init(void)
{
    if (dwt_configure(&uwb_config) != DWT_SUCCESS) {
        printk("ERR: dwt_configure failed\n");
        return -1;
    }

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Frame filtering disabled — all nodes receive all broadcasts */
    dwt_configureframefilter(DWT_FF_DISABLE, 0);

    printk("MMP-GDB init OK — role %s addr 0x%04X\n", MY_ROLE_STR, MY_ADDR);
    return 0;
}

/* ===========================================================================
 * A1 — INITIATOR
 * Sends Msg1, listens for Msg2+Msg3, sends Msg4, listens for Msg5+Msg6.
 * =========================================================================*/
#if defined(ROLE_ANCHOR_A1)

__attribute__((noinline)) int ring_run_round(round_timestamps_t *rt)
{
    memset(rt, 0, sizeof(*rt));
    rt->seq = seq_num;

    /* --- Msg1: A1 → all -------------------------------------------------- */
    if (send_immediate(FC_MSG1, ADDR_A1, NULL, 0) != 0) return -1;
    rt->ts[0] = get_tx_timestamp_u64();   /* TX1 */

    /* --- Msg2: listen for A2 --------------------------------------------- */
    int fc = wait_rx(&rt->ts[1]);         /* RX2 */
    if (fc != FC_MSG2) {
        printk("A1: expected MSG2 got 0x%02X\n", fc);
        return -1;
    }

    /* --- Msg3: listen for A3 --------------------------------------------- */
    fc = wait_rx(&rt->ts[2]);             /* RX3 */
    if (fc != FC_MSG3) {
        printk("A1: expected MSG3 got 0x%02X\n", fc);
        return -1;
    }

    /* --- Msg4: A1 → all (delayed from RX3) -------------------------------- */
    uint32_t tx4_time;
    if (send_delayed(FC_MSG4, ADDR_A1, NULL, 0, rt->ts[2], &tx4_time) != 0) return -1;
    rt->ts[3] = predicted_tx_ts(tx4_time); /* TX4 */

    /* --- Msg5: listen for A3 --------------------------------------------- */
    fc = wait_rx(&rt->ts[4]);             /* RX5 */
    if (fc != FC_MSG5) {
        printk("A1: expected MSG5 got 0x%02X\n", fc);
        return -1;
    }

    /* --- Msg6: listen for A2 --------------------------------------------- */
    fc = wait_rx(&rt->ts[5]);             /* RX6 */
    if (fc != FC_MSG6) {
        printk("A1: expected MSG6 got 0x%02X\n", fc);
        return -1;
    }

    rt->valid = 1;
    seq_num++;
    return 0;
}

void ring_print_timestamps(const round_timestamps_t *rt)
{
    if (!rt->valid) {
        printk("A1 RING[%u] INVALID\n", rt->seq);
        return;
    }
    printk("A1 seq=%u TX1=%llu RX2=%llu RX3=%llu TX4=%llu RX5=%llu RX6=%llu\n",
           rt->seq,
           (unsigned long long)rt->ts[0],
           (unsigned long long)rt->ts[1],
           (unsigned long long)rt->ts[2],
           (unsigned long long)rt->ts[3],
           (unsigned long long)rt->ts[4],
           (unsigned long long)rt->ts[5]);
}

void ring_responder_loop(void) { }

/* ===========================================================================
 * A2 — RESPONDER
 * Listens for Msg1, sends Msg2, listens for Msg3+Msg4+Msg5, sends Msg6.
 * =========================================================================*/
#elif defined(ROLE_ANCHOR_A2)

__attribute__((noinline)) int ring_run_round(round_timestamps_t *rt)
{
    (void)rt; return -1;
}

void ring_print_timestamps(const round_timestamps_t *rt)
{
    if (!rt->valid) {
        printk("A2 RING[%u] INVALID\n", rt->seq);
        return;
    }
    printk("A2 seq=%u RX1=%llu TX2=%llu RX3=%llu RX4=%llu RX5=%llu TX6=%llu\n",
           rt->seq,
           (unsigned long long)rt->ts[0],
           (unsigned long long)rt->ts[1],
           (unsigned long long)rt->ts[2],
           (unsigned long long)rt->ts[3],
           (unsigned long long)rt->ts[4],
           (unsigned long long)rt->ts[5]);
}

void ring_responder_loop(void)
{
    round_timestamps_t rt;
    printk("A2: listening...\n");

    while (true) {
        memset(&rt, 0, sizeof(rt));

        /* --- Msg1: wait for A1 ------------------------------------------- */
        int fc = wait_rx(&rt.ts[0]);      /* RX1 */
        if (fc != FC_MSG1) {
            continue;
        }
        rt.seq = rx_buf[FRAME_SEQ_IDX];

        /* --- Msg2: A2 → all (delayed from RX1) --------------------------- */
        uint32_t tx2_time;
        if (send_delayed(FC_MSG2, ADDR_A2, NULL, 0, rt.ts[0], &tx2_time) != 0) continue;
        rt.ts[1] = predicted_tx_ts(tx2_time); /* TX2 */

        /* --- Msg3: listen for A3 ----------------------------------------- */
        fc = wait_rx(&rt.ts[2]);           /* RX3 */
        if (fc != FC_MSG3) {
            printk("A2: expected MSG3 got 0x%02X\n", fc);
            continue;
        }

        /* --- Msg4: listen for A1 ----------------------------------------- */
        fc = wait_rx(&rt.ts[3]);           /* RX4 */
        if (fc != FC_MSG4) {
            printk("A2: expected MSG4 got 0x%02X\n", fc);
            continue;
        }

        /* --- Msg5: listen for A3 ----------------------------------------- */
        fc = wait_rx(&rt.ts[4]);           /* RX5 */
        if (fc != FC_MSG5) {
            printk("A2: expected MSG5 got 0x%02X\n", fc);
            continue;
        }

        /* --- Msg6: A2 → all (delayed from RX5) --------------------------- */
        uint32_t tx6_time;
        if (send_delayed(FC_MSG6, ADDR_A2, NULL, 0, rt.ts[4], &tx6_time) != 0) continue;
        rt.ts[5] = predicted_tx_ts(tx6_time); /* TX6 */

        rt.valid = 1;
        ring_print_timestamps(&rt);
    }
}

/* ===========================================================================
 * A3 — RESPONDER
 * Listens for Msg1+Msg2, sends Msg3, listens for Msg4, sends Msg5, listens Msg6.
 * =========================================================================*/
#elif defined(ROLE_ANCHOR_A3)

__attribute__((noinline)) int ring_run_round(round_timestamps_t *rt)
{
    (void)rt; return -1;
}

void ring_print_timestamps(const round_timestamps_t *rt)
{
    if (!rt->valid) {
        printk("A3 RING[%u] INVALID\n", rt->seq);
        return;
    }
    printk("A3 seq=%u RX1=%llu RX2=%llu TX3=%llu RX4=%llu TX5=%llu RX6=%llu\n",
           rt->seq,
           (unsigned long long)rt->ts[0],
           (unsigned long long)rt->ts[1],
           (unsigned long long)rt->ts[2],
           (unsigned long long)rt->ts[3],
           (unsigned long long)rt->ts[4],
           (unsigned long long)rt->ts[5]);
}

void ring_responder_loop(void)
{
    round_timestamps_t rt;
    printk("A3: listening...\n");

    while (true) {
        memset(&rt, 0, sizeof(rt));

        /* --- Msg1: wait for A1 ------------------------------------------- */
        int fc = wait_rx(&rt.ts[0]);      /* RX1 */
        if (fc != FC_MSG1) {
            continue;
        }
        rt.seq = rx_buf[FRAME_SEQ_IDX];

        /* --- Msg2: listen for A2 ----------------------------------------- */
        fc = wait_rx(&rt.ts[1]);           /* RX2 */
        if (fc != FC_MSG2) {
            printk("A3: expected MSG2 got 0x%02X\n", fc);
            continue;
        }

        /* --- Msg3: A3 → all (delayed from RX2) --------------------------- */
        uint32_t tx3_time;
        if (send_delayed(FC_MSG3, ADDR_A3, NULL, 0, rt.ts[1], &tx3_time) != 0) continue;
        rt.ts[2] = predicted_tx_ts(tx3_time); /* TX3 */

        /* --- Msg4: listen for A1 ----------------------------------------- */
        fc = wait_rx(&rt.ts[3]);           /* RX4 */
        if (fc != FC_MSG4) {
            printk("A3: expected MSG4 got 0x%02X\n", fc);
            continue;
        }

        /* --- Msg5: A3 → all (delayed from RX4) --------------------------- */
        uint32_t tx5_time;
        if (send_delayed(FC_MSG5, ADDR_A3, NULL, 0, rt.ts[3], &tx5_time) != 0) continue;
        rt.ts[4] = predicted_tx_ts(tx5_time); /* TX5 */

        /* --- Msg6: listen for A2 ----------------------------------------- */
        fc = wait_rx(&rt.ts[5]);           /* RX6 */
        if (fc != FC_MSG6) {
            printk("A3: expected MSG6 got 0x%02X\n", fc);
            continue;
        }

        rt.valid = 1;
        ring_print_timestamps(&rt);
    }
}

#endif /* role selection */
