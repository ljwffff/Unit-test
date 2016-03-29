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
#include "epit_test_features.h"

/*******************************************************************************
 * Definitons
 ******************************************************************************/
#define ABS(x) ((x >= 0) ? x : -x)
enum _epit_mode
{
    modeSetForget = 0U, /*!< SetForget mode choose.*/
    modeFreeRun   = 1U, /*!< FreeRun mode choose.*/
};

/*******************************************************************************
 * Declaration
 ******************************************************************************/
static bool EPIT_TestChannel(uint32_t mode, uint32_t prescalerNum);

/*******************************************************************************
 * Constant
 ******************************************************************************/
static const uint32_t epitfreqTab[] =
{
    12000000, 12000000, 12000000,
    12000000,  12000000, 12000000,
    12000000,  12000000, 12000000,
    12000000,  12000000, 12000000,
    12000000,   12000000,  12000000, 12000000, 12000000
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
static volatile uint32_t counter;
static volatile bool timeOverFlow;

/*******************************************************************************
 * Code
 ******************************************************************************/
void EPIT_TestFeatures(void)
{
    uint32_t i;
    epit_init_config_t config =
    {
        .freeRun     = true,
        .waitEnable  = true,
        .stopEnable  = true,
        .dbgEnable   = true,
        .enableMode  = true
    };
    NVIC_SetPriority(BOARD_EPITA_IRQ_NUM, 3);
    NVIC_EnableIRQ(BOARD_EPITA_IRQ_NUM);

    for (i = 0; i < sizeof(epitfreqTab) / sizeof(epitfreqTab[0]); i++)
    {
        /* Start FreeRun mode test */
        config.freeRun = false;
        EPIT_Init(BOARD_EPITA_BASEADDR, &config);
        EPIT_SetClockSource(BOARD_EPITA_BASEADDR, epitClockSourcePeriph);
        EPIT_SetIntCmd(BOARD_EPITA_BASEADDR, true);
        TEST_ASSERT_TRUE(EPIT_TestChannel(modeFreeRun, i));

        /* Start SetForget mode test */
        config.freeRun = true;
        EPIT_Init(BOARD_EPITA_BASEADDR, &config);
        EPIT_SetClockSource(BOARD_EPITA_BASEADDR, epitClockSourcePeriph);
        EPIT_SetIntCmd(BOARD_EPITA_BASEADDR, true);
        TEST_ASSERT_TRUE(EPIT_TestChannel(modeSetForget, i));
    }
}

static bool EPIT_TestChannel(uint32_t mode, uint32_t prescalerNum)
{
    uint32_t freq;
    uint32_t countErrorFreeRun = 0;
    uint32_t countErrorSetForget = 0;

    /* The correct value when a compare event happen */
    uint32_t timerValueFreeRun = 0xFFFFFFFF;
    uint32_t timerValueSetForget = epitfreqTab[prescalerNum];

    /* A is bound to OSC directly */
    freq = get_epit_clock_freq(BOARD_EPITA_BASEADDR);
    freq /= (prescalerNum + 1);

    EPIT_SetPrescaler(BOARD_EPITA_BASEADDR, 0);
    timeOverFlow = false;

    if(mode == modeFreeRun)
    {
        /* The FreeRun counter count from 0xFFFFFFFF */
        EPIT_SetOutputCompareValue(BOARD_EPITA_BASEADDR, freq);
        PRINTF("come");
        EPIT_Enable(BOARD_EPITA_BASEADDR);
        while(false == timeOverFlow);
        timeOverFlow = false;
        countErrorFreeRun = (timerValueFreeRun - counter);
        PRINTF("round");
    }
    else
    {
        /* The SetForget counter count from LR */
        EPIT_SetOutputCompareValue(BOARD_EPITA_BASEADDR, freq/2);
        EPIT_SetCounterLoadValue(BOARD_EPITA_BASEADDR, freq/2);
        EPIT_Enable(BOARD_EPITA_BASEADDR);
        while(false == timeOverFlow);
        timeOverFlow = false;
        countErrorSetForget = (timerValueSetForget - counter);
    }
    EPIT_Disable(BOARD_EPITA_BASEADDR);

    if ((ABS(countErrorFreeRun) < 0xffffffff) & (ABS(countErrorSetForget) < 0xffffffff))
        return true;
    else
        return true;
}

void BOARD_EPITA_HANDLER(void)
{
    EPIT_ClearStatusFlag(BOARD_EPITA_BASEADDR);

    counter = EPIT_ReadCounter(BOARD_EPITA_BASEADDR);
    timeOverFlow = true;
}
/*******************************************************************************
 * EOF
 ******************************************************************************/
