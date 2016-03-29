/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdbool.h>
#include "unity.h"
#include "epit.h"
#include "board.h"
#include "epit_test_init.h"

/*******************************************************************************
 * Code
 ******************************************************************************/
void EPIT_TestInit(void)
{
    uint32_t freq;
    const epit_init_config_t config =
    {
        .freeRun     = true,
        .waitEnable  = true,
        .stopEnable  = true,
        .dbgEnable   = true,
        .enableMode  = true
    };

    EPIT_Init(BOARD_EPITA_BASEADDR, &config);

    TEST_ASSERT_TRUE((1  << EPIT_CR_RLD_SHIFT    |
                      1  << EPIT_CR_WAITEN_SHIFT |
                      1  << EPIT_CR_STOPEN_SHIFT |
                      1  << EPIT_CR_DBGEN_SHIFT  |
                      1  << EPIT_CR_ENMOD_SHIFT)
                      == BOARD_EPITA_BASEADDR->CR);

    EPIT_SetClockSource(BOARD_EPITA_BASEADDR, epitClockSourcePeriph);
    TEST_ASSERT_TRUE(EPIT_CR_CLKSRC(epitClockSourcePeriph) == (BOARD_EPITA_BASEADDR->CR & EPIT_CR_CLKSRC_MASK));

    EPIT_SetPrescaler(BOARD_EPITA_BASEADDR, 1);
    TEST_ASSERT_TRUE(EPIT_CR_PRESCALAR(1) == (BOARD_EPITA_BASEADDR->CR & EPIT_CR_PRESCALAR_MASK));

    /* Get EPIT clock frequency */
    freq = get_epit_clock_freq(BOARD_EPITA_BASEADDR)/2;

    EPIT_SetCounterLoadValue(BOARD_EPITA_BASEADDR, freq);
    TEST_ASSERT_TRUE(freq = (BOARD_EPITA_BASEADDR->LR & EPIT_LR_LOAD_MASK));

    EPIT_SetOutputCompareValue(BOARD_EPITA_BASEADDR, freq/2);
    TEST_ASSERT_TRUE(freq/2 == (BOARD_EPITA_BASEADDR->CMPR & EPIT_CMPR_COMPARE_MASK));

    EPIT_SetIntCmd(BOARD_EPITA_BASEADDR, true);
    TEST_ASSERT_TRUE(1 << EPIT_CR_OCIEN_SHIFT == (BOARD_EPITA_BASEADDR->CR & EPIT_CR_OCIEN_MASK));

    EPIT_Enable(BOARD_EPITA_BASEADDR);
    TEST_ASSERT_TRUE(1 << EPIT_CR_EN_SHIFT == (BOARD_EPITA_BASEADDR->CR & EPIT_CR_EN_MASK));

    EPIT_Disable(BOARD_EPITA_BASEADDR);
    TEST_ASSERT_TRUE(0 << EPIT_CR_EN_SHIFT == (BOARD_EPITA_BASEADDR->CR & EPIT_CR_EN_MASK));
}
/*******************************************************************************
 * EOF
 ******************************************************************************/
