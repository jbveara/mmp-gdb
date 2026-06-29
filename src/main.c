/*
 * main.c — MMP-GDB ring exchange, Zephyr Shell interface.
 *
 * The Zephyr shell runs on the UART console and handles all RX input.
 * Two shell commands are registered:
 *
 *   role <1|2|3>   — set role at runtime (1=A1, 2=A2, 3=A3)
 *   start          — initialise DW3000 and begin operation
 *
 * Output lines (TS,... ERR,... INF,...) are written with printk() and
 * appear interleaved with the shell prompt — anchor.py filters them by
 * prefix, ignoring "uart:~$" prompt lines.
 *
 * Requires in prj.conf:
 *   CONFIG_SHELL=y
 *   CONFIG_SHELL_BACKEND_SERIAL=y   (default when CONFIG_SHELL=y on UART)
 *
 * Shell log backend (CONFIG_LOG_BACKEND_SHELL) replaces the UART log
 * backend so log noise doesn't corrupt TS lines on a separate channel.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/shell/shell.h>
#include <string.h>
#include <stdlib.h>

#include <dw3000_hw.h>
#include "deca_probe_interface.h"
#include "deca_device_api.h"
#include "deca_interface.h"
#include "port.h"
#include "node_config.h"
#include "ring_exchange.h"

/* ---------------------------------------------------------------------------
 * Shared state between shell commands and the main thread.
 * Protected by a semaphore — shell commands run in the shell thread.
 * -------------------------------------------------------------------------*/
static anchor_role_t g_role   = ROLE_NONE;
static bool          g_start  = false;
static bool          g_hw_ok  = false;

/* Semaphore: main thread blocks on it; "start" command gives it. */
K_SEM_DEFINE(start_sem, 0, 1);

/* ---------------------------------------------------------------------------
 * Shell command: role <1|2|3>
 * -------------------------------------------------------------------------*/
static int cmd_role(const struct shell *sh, size_t argc, char **argv)
{
    if (argc != 2) {
        shell_error(sh, "Usage: role <1|2|3>");
        return -EINVAL;
    }

    int r = atoi(argv[1]);
    if (r < 1 || r > 3) {
        shell_error(sh, "Invalid role %d — use 1 (A1), 2 (A2), or 3 (A3)", r);
        return -EINVAL;
    }

    g_role = (anchor_role_t)r;
    const char *names[] = { "", "A1 (initiator)", "A2 (responder)", "A3 (responder)" };
    shell_print(sh, "INF,role_set_%s", names[r]);
    return 0;
}

/* ---------------------------------------------------------------------------
 * Shell command: start
 * -------------------------------------------------------------------------*/
static int cmd_start(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    if (g_role == ROLE_NONE) {
        shell_error(sh, "Set role first: role <1|2|3>");
        return -EINVAL;
    }

    if (g_start) {
        shell_error(sh, "Already started");
        return -EALREADY;
    }

    g_start = true;
    shell_print(sh, "INF,starting");
    k_sem_give(&start_sem);
    return 0;
}

/* Register commands — available as soon as the shell starts */
SHELL_CMD_ARG_REGISTER(role,  NULL, "Set anchor role: role <1|2|3>", cmd_role,  2, 0);
SHELL_CMD_REGISTER    (start, NULL, "Start UWB operation",            cmd_start);

/* ---------------------------------------------------------------------------
 * main — waits for shell commands, then runs UWB protocol
 * -------------------------------------------------------------------------*/
int main(void)
{
    printk("INF,MMP-GDB_ready — type: role <1|2|3>  then: start\n");

    /* Block until the shell "start" command gives the semaphore */
    k_sem_take(&start_sem, K_FOREVER);

    ring_set_role(g_role);

    /* ---- Hardware init --------------------------------------------------- */
    dw3000_hw_init();
    dw3000_hw_reset();
    Sleep(2);

    dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        printk("ERR,0,dwt_initialise_failed\n");
        return -1;
    }

    if (ring_init() != 0) {
        printk("ERR,0,ring_init_failed\n");
        return -1;
    }

    g_hw_ok = true;

    /* ---- Run ------------------------------------------------------------- */
    if (g_role == ROLE_A1) {
        round_timestamps_t rt;
        printk("INF,A1_starting_period_%ums\n", ROUND_PERIOD_MS);
        while (1) {
            if (ring_run_round(&rt) == 0) {
                ring_print_timestamps(&rt);
            } else {
                printk("ERR,%u,round_failed\n", rt.seq);
            }
            k_msleep(ROUND_PERIOD_MS);
        }
    } else {
        ring_responder_loop(); /* never returns */
    }

    return 0;
}
