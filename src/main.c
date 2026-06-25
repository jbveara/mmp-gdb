/*
 * main.c — MMP-GDB ring exchange
 * Role selected at build time via add_definitions(-DROLE_ANCHOR_A1/A2/A3)
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <dw3000_hw.h>
#include "deca_probe_interface.h"
#include "deca_device_api.h"
#include "deca_interface.h"
#include "port.h"
#include "node_config.h"
#include "ring_exchange.h"
#include "ring_distances.h"

int main(void)
{
    printk("\n*** MMP-GDB ring exchange ***\n");
    printk("Role: %s\n", MY_ROLE_STR);

    dw3000_hw_init();
    dw3000_hw_reset();
    Sleep(2);

    dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        printk("FATAL: dwt_initialise failed\n");
        return -1;
    }
    printk("DW3000 init OK\n");

    if (ring_init() != 0) {
        printk("FATAL: ring_init failed\n");
        return -1;
    }

#if defined(ROLE_ANCHOR_A1)
    round_timestamps_t rt;
    ring_distances_t dist;
    printk("A1: starting — period %u ms\n", ROUND_PERIOD_MS);
    while (1) {
        if (ring_run_round(&rt) == 0) {
            ring_print_timestamps(&rt);
            ring_calc_distances(&rt, &dist);
        } else {
            printk("A1: round failed\n");
        }
        k_msleep(ROUND_PERIOD_MS);
    }
#else
    ring_responder_loop();
#endif

    return 0;
}
