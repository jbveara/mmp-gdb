#ifndef NODE_CONFIG_H
#define NODE_CONFIG_H

/* ---------------------------------------------------------------------------
 * Role is selected at RUNTIME via UART command "ROLE=<1|2|3>\n".
 * There are no compile-time ROLE_ANCHOR_A1/A2/A3 defines in this build.
 * The single firmware binary is flashed to all three boards.
 * -------------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------
 * UWB short addresses
 * -------------------------------------------------------------------------*/
#define ADDR_A1  0x0001U
#define ADDR_A2  0x0002U
#define ADDR_A3  0x0003U
#define ADDR_PAN 0xDECAU

/* ---------------------------------------------------------------------------
 * UWB PHY config — channel 5, 6.8 Mbps
 * -------------------------------------------------------------------------*/
#define UWB_CHANNEL      5
#define UWB_PREAM_CODE   9
#define UWB_SFD_TIMEOUT  (128 + 1 + 8)

/* ---------------------------------------------------------------------------
 * Antenna delays
 * -------------------------------------------------------------------------*/
#define TX_ANT_DLY  16399U
#define RX_ANT_DLY  16399U

/* ---------------------------------------------------------------------------
 * Reply delay — programmed by responder hardware; also used by host Python
 * to reconstruct the nominal delta.  Must match on both sides.
 *
 * RESP_TX_DELAY_TICKS : raw ticks loaded into the DX_TIME register offset
 * DELTA_TICKS         : the actual stamped delta = RESP_TX_DELAY_TICKS + TX_ANT_DLY
 *                       (see node_config.h in the original codebase for derivation)
 *
 * These values are also printed in the RDY line so the host can verify them.
 * -------------------------------------------------------------------------*/
#define RESP_TX_DELAY_TICKS  192000000ULL
#define DELTA_TICKS          (RESP_TX_DELAY_TICKS + TX_ANT_DLY)

/* A1 RX timeout — covers two hops of delay plus margin */
#define RX_TIMEOUT_UUS  20000U

/* Interval between complete rounds (A1 side) */
#define ROUND_PERIOD_MS  500U

/* ---------------------------------------------------------------------------
 * Frame layout
 * -------------------------------------------------------------------------*/
#define FRAME_FC_IDX       0
#define FRAME_SEQ_IDX      1
#define FRAME_PAN_IDX      2
#define FRAME_DEST_IDX     4
#define FRAME_SRC_IDX      6
#define FRAME_PAYLOAD_IDX  8

#define ADDR_BROADCAST  0xFFFFU

/* Function codes */
#define FC_MSG1  0x31U
#define FC_MSG2  0x32U
#define FC_MSG3  0x33U
#define FC_MSG4  0x34U
#define FC_MSG5  0x35U
#define FC_MSG6  0x36U

#define FRAME_MAX_PAYLOAD  25U
#define FRAME_LEN_MAX_APP  (FRAME_PAYLOAD_IDX + FRAME_MAX_PAYLOAD + 2U)

#endif /* NODE_CONFIG_H */
