/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/runtime.h"
#include "pico/runtime_init.h"

#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/ticks.h"
#include "hardware/timer.h"
#include "hardware/vreg.h"
#include "hardware/xosc.h"
#include "hardware/resets.h"


#define STARTUP_DELAY ((((XOSC_HZ / KHZ) + 128) / 256) * PICO_XOSC_STARTUP_DELAY_MULTIPLIER)

// From runtime_init_clocks() - src/rp2_common/pico_runtime_init/runtime_init_clocks.c
// This has been reduced down to just CLK_SYS and related code - everything should be
//     inlined so it staysin the .flashtext section
void boot2_init_clocks(void) {
    clocks_hw->resus.ctrl = 0;

    // Adapted from xosc_init() - src/rp2_common/hardware_xosc/xosc.c
    xosc_hw->ctrl = XOSC_CTRL_FREQ_RANGE_VALUE_1_15MHZ;
    xosc_hw->startup = STARTUP_DELAY;
    hw_set_bits(&xosc_hw->ctrl, XOSC_CTRL_ENABLE_VALUE_ENABLE << XOSC_CTRL_ENABLE_LSB);
    while(!(xosc_hw->status & XOSC_STATUS_STABLE_BITS)) tight_loop_contents();

    hw_clear_bits(&clocks_hw->clk[clk_sys].ctrl, CLOCKS_CLK_SYS_CTRL_SRC_BITS);
    while (clocks_hw->clk[clk_sys].selected != 0x1) tight_loop_contents();
    // end xosc_init()




    // Adapted from pll_init() - src/rp2_common/hardware_pll/pll.c
    PLL pll = pll_sys;
    uint32_t refdiv = PLL_SYS_REFDIV;
    uint32_t vco_freq = PLL_SYS_VCO_FREQ_HZ;
    uint32_t post_div1 = PLL_SYS_POSTDIV1;
    uint32_t post_div2 = 1;//PLL_SYS_POSTDIV2; // change this to adjust copy speed
    uint32_t ref_freq = XOSC_HZ / refdiv;
    uint32_t fbdiv = vco_freq / ref_freq;

    uint32_t pdiv = (post_div1 << PLL_PRIM_POSTDIV1_LSB) |
                    (post_div2 << PLL_PRIM_POSTDIV2_LSB);

    if (!((pll->cs & PLL_CS_LOCK_BITS) &&
        (refdiv == (pll->cs & PLL_CS_REFDIV_BITS)) &&
        (fbdiv  == (pll->fbdiv_int & PLL_FBDIV_INT_BITS)) &&
        (pdiv   == (pll->prim & (PLL_PRIM_POSTDIV1_BITS | PLL_PRIM_POSTDIV2_BITS))))) {

            reset_unreset_block_num_wait_blocking(RESET_PLL_SYS);

            pll->cs = refdiv;
            pll->fbdiv_int = fbdiv;

            uint32_t power = PLL_PWR_PD_BITS | PLL_PWR_VCOPD_BITS;

            hw_clear_bits(&pll->pwr, power);
            while (!(pll->cs & PLL_CS_LOCK_BITS)) tight_loop_contents();
            pll->prim = pdiv;
            hw_clear_bits(&pll->pwr, PLL_PWR_POSTDIVPD_BITS);
    }
   // end pll_init()





#if SYS_CLK_VREG_VOLTAGE_AUTO_ADJUST && defined(SYS_CLK_VREG_VOLTAGE_MIN)
        if (vreg_get_voltage() < SYS_CLK_VREG_VOLTAGE_MIN) {
            vreg_set_voltage(SYS_CLK_VREG_VOLTAGE_MIN);
            busy_wait_at_least_cycles((uint32_t)((SYS_CLK_VREG_VOLTAGE_AUTO_ADJUST_DELAY_US * (uint64_t)XOSC_HZ) / 1000000));
        }
#endif



    // Adapted from clock_configure_internal() - src/rp2_common/hardware_clocks/clocks.c
    clock_handle_t clock = clk_sys;
    uint32_t src = CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX;
    uint32_t auxsrc = CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS;
    uint32_t div = (1u << CLOCKS_CLK_GPOUT0_DIV_INT_LSB);
    clock_hw_t *clock_hw = &clocks_hw->clk[clock];


    if (div > clock_hw->div) clock_hw->div = div;

    if ((clock == clk_sys || clock == clk_ref) && src == CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX) {
        hw_clear_bits(&clock_hw->ctrl, CLOCKS_CLK_REF_CTRL_SRC_BITS);
        while (!(clock_hw->selected & 1u))
            tight_loop_contents();
    }
    else {
        hw_clear_bits(&clock_hw->ctrl, CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS);
    }

    hw_write_masked(&clock_hw->ctrl,
        (auxsrc << CLOCKS_CLK_SYS_CTRL_AUXSRC_LSB),
        CLOCKS_CLK_SYS_CTRL_AUXSRC_BITS
    );

    if ((clock == clk_sys || clock == clk_ref)) {
        hw_write_masked(&clock_hw->ctrl,
            src << CLOCKS_CLK_REF_CTRL_SRC_LSB,
            CLOCKS_CLK_REF_CTRL_SRC_BITS
        );
        while (!(clock_hw->selected & (1u << src)))
            tight_loop_contents();
    }

    hw_set_bits(&clock_hw->ctrl, CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS);

    clock_hw->div = div;
    // end clock_configure_internal()

    //busy_wait_at_least_cycles(200000);
}

/*! \brief  Handle a hard_assert condition failure
*  \ingroup pico_runtime
*
* This weak function provides the default implementation (call \ref panic with "Hard assert") for if a \ref hard_assert
* condition fail in non debug builds. You can provide your own strong implementation to replace the default behavior
*
* \sa hard_assert
*/

void __weak hard_assertion_failure(void) {
    panic("Hard assert");
}

static void runtime_run_initializers_from(uintptr_t *from) {

    // Start and end points of the constructor list,
    // defined by the linker script.
    extern uintptr_t __preinit_array_end;

    // Call each function in the list, based on the mask
    // We have to take the address of the symbols, as __preinit_array_start *is*
    // the first function value, not the address of it.
    for (uintptr_t *p = from; p < &__preinit_array_end; p++) {
        uintptr_t val = *p;
        ((void (*)(void))val)();
    }
}

void runtime_run_initializers(void) {
    extern uintptr_t __preinit_array_start;
    runtime_run_initializers_from(&__preinit_array_start);
}

// We keep the per-core initializers in the standard __preinit_array so a standard C library
// initialization will force the core 0 initialization, however we also want to be able to find
// them after the fact so that we can run them on core 1. Per core initializers have sections
// __preinit_array.ZZZZZ.nnnnn i.e. the ZZZZZ sorts below all the standard __preinit_array.nnnnn
// values, and then we sort within the ZZZZZ.
//
// We create a dummy initializer in __preinit_array.YYYYY (between the standard initializers
// and the per core initializers), so we find the first per core initializer. Whilst we could
// have done this via an entry in the linker script, we want to preserve backwards compatibility
// with RP2040 custom linker scripts.
static void first_per_core_initializer(void) {}
PICO_RUNTIME_INIT_FUNC(first_per_core_initializer, "YYYYY");

void runtime_run_per_core_initializers(void) {
    runtime_run_initializers_from(&__pre_init_first_per_core_initializer);
}