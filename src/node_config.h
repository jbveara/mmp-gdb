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

/* Reply delay for all responder nodes, in raw 40-bit DWT timestamp ticks.
 *
 * This single constant is used in two places and must be identical in both:
 *   1. ring_exchange.c send_delayed() — programs the hardware TX schedule
 *   2. ring_distances.c              — subtracted in every ToF formula
 *
 * The formula delta must match what the hardware actually stamps.
 * predicted_tx_ts computes:
 *   TX_ts = ((rx_ts + RESP_TX_DELAY_TICKS) & ~0x1FF) + TX_ANT_DLY
 *
 * So the stamped delta (TX_ts - RX_ts) = RESP_TX_DELAY_TICKS + TX_ANT_DLY
 * plus up to ±256 ticks of 512-tick register quantisation (~±1.2 mm).
 *
 * RX_ANT_DLY does not appear here — it cancels symmetrically across the
 * full two-way flight in the ToF formulas.
 *
 * Therefore the correct formula constant is RESP_TX_DELAY_TICKS + TX_ANT_DLY,
 * which is what DELTA_TICKS resolves to in ring_distances.c.
 *
 * Do NOT use UUS_TO_DWT_TIME — that constant uses a different tick scaling
 * (65536 per µs vs the true hardware rate of 63897.6 ticks/µs).
 */
#define RESP_TX_DELAY_TICKS  192000000ULL
#define DELTA_TICKS          (RESP_TX_DELAY_TICKS + TX_ANT_DLY)

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