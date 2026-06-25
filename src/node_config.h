#ifndef NODE_CONFIG_H
#define NODE_CONFIG_H

/* ---------------------------------------------------------------------------
 * Role guards — exactly one must be defined via CMake
 * -------------------------------------------------------------------------*/
#if !defined(ROLE_ANCHOR_A1) && !defined(ROLE_ANCHOR_A2) && !defined(ROLE_ANCHOR_A3)
#error "No role defined. Set add_definitions(-DROLE_ANCHOR_A1/A2/A3) in CMakeLists.txt"
#endif

/* ---------------------------------------------------------------------------
 * UWB short addresses
 * -------------------------------------------------------------------------*/
#define ADDR_A1  0x0001U
#define ADDR_A2  0x0002U
#define ADDR_A3  0x0003U
#define ADDR_PAN 0xDECAU

#if defined(ROLE_ANCHOR_A1)
  #define MY_ADDR      ADDR_A1
  #define MY_ROLE_STR  "A1 (initiator)"
#elif defined(ROLE_ANCHOR_A2)
  #define MY_ADDR      ADDR_A2
  #define MY_ROLE_STR  "A2 (responder)"
#elif defined(ROLE_ANCHOR_A3)
  #define MY_ADDR      ADDR_A3
  #define MY_ROLE_STR  "A3 (responder)"
#endif

/* ---------------------------------------------------------------------------
 * UWB PHY config — channel 5, matches DS-TWR examples
 * -------------------------------------------------------------------------*/
#define UWB_CHANNEL      5
#define UWB_PREAM_CODE   9
#define UWB_SFD_TIMEOUT  (128 + 1 + 8)

/* ---------------------------------------------------------------------------
 * Timing
 * -------------------------------------------------------------------------*/
#define TX_ANT_DLY       16385U
#define RX_ANT_DLY       16385U

/* Processing delay δ_i at each responder (same for all nodes in Phase 2).
 * Must be long enough for SPI + processing overhead after RX.
 * Tighten after characterising actual δ_i. */
#define RESP_TX_DELAY_UUS  3000U

/* A1 listens for each expected message — generous timeout covers
 * two hops of RESP_TX_DELAY_UUS plus air time plus margin. */
#define RX_TIMEOUT_UUS     20000U

/* Interval between complete rounds (A1 side) */
#define ROUND_PERIOD_MS    500U

/* ---------------------------------------------------------------------------
 * Frame layout
 *
 *  [0]     function code  (FC_MSG1..FC_MSG6)
 *  [1]     sequence number
 *  [2-3]   PAN ID (LE)
 *  [4-5]   destination address (LE) — broadcast = 0xFFFF
 *  [6-7]   source address (LE)
 *  [8..]   payload (timestamps, see ring_exchange.c)
 *  last 2  HW CRC (appended by DW3000)
 * -------------------------------------------------------------------------*/
#define FRAME_FC_IDX       0
#define FRAME_SEQ_IDX      1
#define FRAME_PAN_IDX      2
#define FRAME_DEST_IDX     4
#define FRAME_SRC_IDX      6
#define FRAME_PAYLOAD_IDX  8

#define ADDR_BROADCAST     0xFFFFU

/* Function codes — one per protocol message */
#define FC_MSG1  0x31U   /* A1 → all: round initiation (forward) */
#define FC_MSG2  0x32U   /* A2 → all: forward hop 1 */
#define FC_MSG3  0x33U   /* A3 → all: forward hop 2 */
#define FC_MSG4  0x34U   /* A1 → all: reverse initiation */
#define FC_MSG5  0x35U   /* A3 → all: reverse hop 1 */
#define FC_MSG6  0x36U   /* A2 → all: reverse hop 2 / round close */

/* Maximum payload bytes (25 bytes for Msg6 + 8 header + 2 CRC = 35) */
#define FRAME_MAX_PAYLOAD  25U
#define FRAME_LEN_MAX_APP  (FRAME_PAYLOAD_IDX + FRAME_MAX_PAYLOAD + 2U)

#endif /* NODE_CONFIG_H */
