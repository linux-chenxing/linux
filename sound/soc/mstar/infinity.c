// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Daniel Palmer <daniel@thingy.jp>
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <linux/delay.h>
#include <linux/spinlock.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <asm/dma.h>
#include <asm/io.h>

#include "infinity.h"

const U16 g_nInfinityDmaIntReg[BACH_DMA_NUM][BACH_DMA_INT_NUM] =
{
    {REG_WR_UNDERRUN_INT_EN, REG_WR_OVERRUN_INT_EN, 0, REG_WR_FULL_INT_EN},
    {REG_RD_UNDERRUN_INT_EN, REG_RD_OVERRUN_INT_EN, REG_RD_EMPTY_INT_EN, 0}
};

static S8  m_nInfinityDpgaGain[4] = {0, 0, 0, 0};

static DMACHANNEL m_infinitydmachannel[2]; // info about dma channel states

//static bool m_bIsMapped; // must call MmUnmapIoSpace when destroyed

static bool m_bADCActive;
static bool m_bDACActive;

static U16 m_nMicGain = 0x1;
static U16 m_nMicInGain = 0x011;
static U16 m_nLineInGain = 0x000;

static bool m_bInfinityAtopStatus[BACH_ATOP_NUM];




//-------------------------------------------------------------------------------------------------
//  Global Functions
//-------------------------------------------------------------------------------------------------
U16 InfinityGetMaskReg(BachRegBank_e nBank, U8 nAddr)
{
    return InfinityReadReg(nBank, nAddr);
}

void InfinityWriteReg2Byte(U32 nAddr, U16 nValue)
{
    //WRITE_WORD(m_pInfinityBaseRegAddr + ((nAddr) << 1), nValue);
}

void InfinityWriteRegByte(U32 nAddr, U8 nValue)
{
    //WRITE_BYTE(m_pInfinityBaseRegAddr + ((nAddr) << 1) - ((nAddr) & 1), nValue);
}

U16 InfinityReadReg2Byte(U32 nAddr)
{
    //return READ_WORD(m_pInfinityBaseRegAddr + ((nAddr) << 1));
	return 0;
}

U8 InfinityReadRegByte(U32 nAddr)
{
    //return READ_BYTE(m_pInfinityBaseRegAddr + ((nAddr) << 1) - ((nAddr) & 1));
	return 0;
}

void InfinityWriteReg(BachRegBank_e nBank, U8 nAddr, U16 regMsk, U16 nValue)
{
    U16 nConfigValue;

    /*switch(nBank)
    {
    case BACH_REG_BANK1:
        nConfigValue = READ_WORD(m_pInfinityAudBank1RegAddr + ((nAddr) << 1));
        nConfigValue &= ~regMsk;
        nConfigValue |= (nValue & regMsk);
        WRITE_WORD(m_pInfinityAudBank1RegAddr + ((nAddr) << 1), nConfigValue);
        break;
    case BACH_REG_BANK2:
        nConfigValue = READ_WORD(m_pInfinityAudBank2RegAddr + ((nAddr) << 1));
        nConfigValue &= ~regMsk;
        nConfigValue |= (nValue & regMsk);
        WRITE_WORD(m_pInfinityAudBank2RegAddr + ((nAddr) << 1), nConfigValue);
        break;
    case BACH_REG_BANK3:
        nConfigValue = READ_WORD(m_pInfinityAudBank3RegAddr + ((nAddr) << 1));
        nConfigValue &= ~regMsk;
        nConfigValue |= (nValue & regMsk);
        WRITE_WORD(m_pInfinityAudBank3RegAddr + ((nAddr) << 1), nConfigValue);
        break;
    default:
        ERRMSG("WAVEDEV.DLL: InfinityWriteReg - ERROR bank default case!\n");
        break;
    }*/

}


U16 InfinityReadReg(BachRegBank_e nBank, U8 nAddr)
{
    /*switch(nBank)
    {
    case BACH_REG_BANK1:
        return READ_WORD(m_pInfinityAudBank1RegAddr + ((nAddr) << 1));
    case BACH_REG_BANK2:
        return READ_WORD(m_pInfinityAudBank2RegAddr + ((nAddr) << 1));
    case BACH_REG_BANK3:
        return READ_WORD(m_pInfinityAudBank3RegAddr + ((nAddr) << 1));
    default:
        ERRMSG("WAVEDEV.DLL: InfinityReadReg - ERROR bank default case!\n");
        return 0;
    }*/
	return 0;
}

U32 InfinityRateToU32(BachRate_e eRate)
{
    switch(eRate)
    {
    case BACH_RATE_8K:
        return 8000;
    case BACH_RATE_11K:
        return 11025;
    case BACH_RATE_12K:
        return 12000;
    case BACH_RATE_16K:
        return 16000;
    case BACH_RATE_22K:
        return 22050;
    case BACH_RATE_24K:
        return 24000;
    case BACH_RATE_32K:
        return 32000;
    case BACH_RATE_44K:
        return 44100;
    case BACH_RATE_48K:
        return 48000;
    default:
        return 0;
    }
}

BachRate_e InfinityRateFromU32(U32 nRate)
{
    switch(nRate)
    {
    case 8000:
        return BACH_RATE_8K;
    case 11025:
        return BACH_RATE_11K;
    case 12000:
        return BACH_RATE_12K;
    case 16000:
        return BACH_RATE_16K;
    case 22050:
        return BACH_RATE_22K;
    case 24000:
        return BACH_RATE_24K;
    case 32000:
        return BACH_RATE_32K;
    case 44100:
        return BACH_RATE_44K;
    case 48000:
        return BACH_RATE_48K;
    default:
        return BACH_RATE_NULL;
    }
}

bool InfinityDmaSetRate(BachDmaChannel_e eDmaChannel, BachRate_e eRate)
{

    //TODO:
    switch(eDmaChannel)
    {
        //ADC rate should be set according to the DMA writer rate
    case BACH_DMA_WRITER1:
        switch(eRate)
        {
        case BACH_RATE_8K:
            InfinityWriteReg(BACH_REG_BANK1, BACH_SR0_SEL, REG_WRITER_SEL_MSK, 0<<REG_WRITER_SEL_POS);
            InfinityWriteReg(BACH_REG_BANK1, BACH_SR0_SEL, REG_CIC_3_SEL_MSK, 0<<REG_CIC_3_SEL_POS);
            break;
        case BACH_RATE_16K:
            InfinityWriteReg(BACH_REG_BANK1, BACH_SR0_SEL, REG_WRITER_SEL_MSK, 1<<REG_WRITER_SEL_POS);
            InfinityWriteReg(BACH_REG_BANK1, BACH_SR0_SEL, REG_CIC_3_SEL_MSK, 1<<REG_CIC_3_SEL_POS);
            break;
        case BACH_RATE_32K:
            InfinityWriteReg(BACH_REG_BANK1, BACH_SR0_SEL, REG_WRITER_SEL_MSK, 2<<REG_WRITER_SEL_POS);
            InfinityWriteReg(BACH_REG_BANK1, BACH_SR0_SEL, REG_CIC_3_SEL_MSK, 2<<REG_CIC_3_SEL_POS);
            break;
        case BACH_RATE_48K:
            InfinityWriteReg(BACH_REG_BANK1, BACH_SR0_SEL, REG_WRITER_SEL_MSK, 3<<REG_WRITER_SEL_POS);
            InfinityWriteReg(BACH_REG_BANK1, BACH_SR0_SEL, REG_CIC_3_SEL_MSK, 3<<REG_CIC_3_SEL_POS);
            break;
        default:
            return false;
        }
        break;

    case BACH_DMA_READER1:
        switch(eRate)
        {
        case BACH_RATE_8K:
            InfinityWriteReg(BACH_REG_BANK1, BACH_SR0_SEL, REG_SRC1_SEL_MSK, 0<<REG_SRC1_SEL_POS);
            InfinityWriteReg(BACH_REG_BANK2, BACH_AU_SYS_CTRL1, REG_CODEC_SEL_MSK, 0<<REG_CODEC_SEL_POS);
            break;
        case BACH_RATE_11K:
            InfinityWriteReg(BACH_REG_BANK1, BACH_SR0_SEL, REG_SRC1_SEL_MSK, 1<<REG_SRC1_SEL_POS);
            break;
        case BACH_RATE_12K:
            InfinityWriteReg(BACH_REG_BANK1, BACH_SR0_SEL, REG_SRC1_SEL_MSK, 2<<REG_SRC1_SEL_POS);
            break;
        case BACH_RATE_16K:
            InfinityWriteReg(BACH_REG_BANK1, BACH_SR0_SEL, REG_SRC1_SEL_MSK, 3<<REG_SRC1_SEL_POS);
            InfinityWriteReg(BACH_REG_BANK2, BACH_AU_SYS_CTRL1, REG_CODEC_SEL_MSK, 1<<REG_CODEC_SEL_POS);
            break;
        case BACH_RATE_22K:
            InfinityWriteReg(BACH_REG_BANK1, BACH_SR0_SEL, REG_SRC1_SEL_MSK, 4<<REG_SRC1_SEL_POS);
            break;
        case BACH_RATE_24K:
            InfinityWriteReg(BACH_REG_BANK1, BACH_SR0_SEL, REG_SRC1_SEL_MSK, 5<<REG_SRC1_SEL_POS);
            break;
        case BACH_RATE_32K:
            InfinityWriteReg(BACH_REG_BANK1, BACH_SR0_SEL, REG_SRC1_SEL_MSK, 6<<REG_SRC1_SEL_POS);
            InfinityWriteReg(BACH_REG_BANK2, BACH_AU_SYS_CTRL1, REG_CODEC_SEL_MSK, 2<<REG_CODEC_SEL_POS);
            break;
        case BACH_RATE_44K:
            InfinityWriteReg(BACH_REG_BANK1, BACH_SR0_SEL, REG_SRC1_SEL_MSK, 7<<REG_SRC1_SEL_POS);
            break;
        case BACH_RATE_48K:
            InfinityWriteReg(BACH_REG_BANK1, BACH_SR0_SEL, REG_SRC1_SEL_MSK, 8<<REG_SRC1_SEL_POS);
            InfinityWriteReg(BACH_REG_BANK2, BACH_AU_SYS_CTRL1, REG_CODEC_SEL_MSK, 3<<REG_CODEC_SEL_POS);
            break;
        default:
            return false;
        }
        break;
    default:
        return false;
    }

    return true;
}


U32 InfinityDmaGetRate(BachDmaChannel_e eDmaChannel)
{
    return m_infinitydmachannel[eDmaChannel].nSampleRate;
}


void InfinityDmaSetChMode(BachDmaChannel_e eDma, bool bMono)
{
    switch(eDma)
    {
    case BACH_DMA_WRITER1:
        if(bMono)
            InfinityWriteReg(BACH_REG_BANK1, BACH_DMA_TEST_CTRL7, REG_DMA1_WR_MONO, REG_DMA1_WR_MONO);
        else
            InfinityWriteReg(BACH_REG_BANK1, BACH_DMA_TEST_CTRL7, REG_DMA1_WR_MONO, 0);
        return;
    case BACH_DMA_READER1:
        if(bMono)
            InfinityWriteReg(BACH_REG_BANK1, BACH_DMA_TEST_CTRL7, REG_DMA1_RD_MONO | REG_DMA1_RD_MONO_COPY, REG_DMA1_RD_MONO | REG_DMA1_RD_MONO_COPY);
        else
            InfinityWriteReg(BACH_REG_BANK1, BACH_DMA_TEST_CTRL7, REG_DMA1_RD_MONO | REG_DMA1_RD_MONO_COPY, 0);
        return;
    default:
        ERRMSG("InfinityDmaSetChMode - unexpected DMA channel\n");
        return;
    }
}

bool InfinityDmaIsWork(BachDmaChannel_e eDmaChannel)
{
    return ((m_infinitydmachannel[eDmaChannel].nDMAChannelState==DMA_RUNNING)? true : false);
}


void InfinityDpgaCtrl(BachDpga_e eDpga, bool bEnable, bool bMute, bool bFade)
{
    U8 nAddr = 0;
    U16 nConfigValue;

    switch(eDpga)
    {
    case BACH_DPGA_MMC1:
        nAddr = BACH_MMC1_DPGA_CFG1;
        break;
    case BACH_DPGA_ADC:
        nAddr = BACH_ADC_DPGA_CFG1;
        break;
    case BACH_DPGA_AEC1:
        nAddr = BACH_AEC1_DPGA_CFG1;
        break;
    case BACH_DPGA_DEC1:
        nAddr = BACH_MMCDEC1_DPGA_CFG1;
        break;
    default:
        ERRMSG("InfinityDpgaCtrl - ERROR default case!\n");
        return;
    }

    nConfigValue = 0;
    if(bEnable)
        nConfigValue |= DPGA_EN;
    if(bMute)
        nConfigValue |= MUTE_2_ZERO;
    if(bFade)
        nConfigValue |= FADING_EN;

    InfinityWriteReg(BACH_REG_BANK1, nAddr, (DPGA_EN | MUTE_2_ZERO | FADING_EN), nConfigValue);
}

void InfinityDpgaCalGain(S8 s8Gain, U8 *pu8GainIdx)//ori step:0.5dB,new step 1dB
{
    if(s8Gain > BACH_DPGA_GAIN_MAX_DB)
        s8Gain = BACH_DPGA_GAIN_MAX_DB;
    else if(s8Gain < BACH_DPGA_GAIN_MIN_DB)
        s8Gain = BACH_DPGA_GAIN_MIN_DB;

    if(s8Gain == BACH_DPGA_GAIN_MIN_DB)
        *pu8GainIdx = BACH_DPGA_GAIN_MIN_IDX;
    else
        *pu8GainIdx = (U8)(-2 * s8Gain); //index = -2 * (gain) ,because step = -0.5dB
}

S8 InfinityDpgaGetGain(BachDpga_e eDpga)
{
    U16 nConfigValue;
    U8	nGainIdx, nAddr;
    S8	nGain;

    switch(eDpga)
    {
    case BACH_DPGA_MMC1:
        nAddr = BACH_MMC1_DPGA_CFG2;
        break;
    case BACH_DPGA_ADC:
        nAddr = BACH_ADC_DPGA_CFG2;
        break;
    case BACH_DPGA_AEC1:
        nAddr = BACH_AEC1_DPGA_CFG2;
        break;
    case BACH_DPGA_DEC1:
        nAddr = BACH_MMCDEC1_DPGA_CFG2;
        break;
    default:
        ERRMSG("InfinityDpgaGetGain - ERROR default case!\n");
        return 0;
    }

    nConfigValue = InfinityReadReg(BACH_REG_BANK1, nAddr);
    nGainIdx = (U8)(nConfigValue & REG_GAIN_L_MSK);
    if(nGainIdx == BACH_DPGA_GAIN_MIN_IDX)
        nGain = BACH_DPGA_GAIN_MIN_DB;
    else
        nGain = -((S8)nGainIdx) / 2;
    return nGain;
}

void InfinityDpgaSetGain(BachDpga_e eDpga, S8 s8Gain)
{
    U8 nAddr;
    U8 nGainIdx,nLGain,nRGain;

    InfinityDpgaCalGain(s8Gain, &nGainIdx);

    switch(eDpga)
    {
    case BACH_DPGA_MMC1:
        nAddr = BACH_MMC1_DPGA_CFG2;
        nLGain = nGainIdx;
        nRGain = nGainIdx;
        break;
    case BACH_DPGA_ADC:
        nAddr = BACH_ADC_DPGA_CFG2;
        nLGain = nGainIdx;
        nRGain = 0;
        break;
    case BACH_DPGA_AEC1:
        nAddr = BACH_AEC1_DPGA_CFG2;
        nLGain = nGainIdx;
        nRGain = 0;
        break;
    case BACH_DPGA_DEC1:
        nAddr = BACH_MMCDEC1_DPGA_CFG2;
        nLGain = nGainIdx;
        nRGain = nGainIdx;
        break;
    default:
        ERRMSG("InfinityDpgaSetGain - ERROR default case!\n");
        return;
    }

    //set gain
    InfinityWriteReg(BACH_REG_BANK1, nAddr, REG_GAIN_R_MSK | REG_GAIN_L_MSK, (nRGain<<REG_GAIN_R_POS) | (nLGain<<REG_GAIN_L_POS));
}

void InfinitySetPathOnOff(BachPath_e ePath, bool bOn)
{
    switch(ePath)
    {
    case BACH_PATH_PLAYBACK:
        if(bOn)
        {
            InfinityDpgaSetGain(BACH_DPGA_MMC1, m_nInfinityDpgaGain[BACH_DPGA_MMC1]);
        }
        else
        {
            InfinityDpgaSetGain(BACH_DPGA_MMC1, BACH_DPGA_GAIN_MIN_DB);
        }
        break;
    case BACH_PATH_CAPTURE:
        if(bOn)
        {
            InfinityDpgaSetGain(BACH_DPGA_ADC, m_nInfinityDpgaGain[BACH_DPGA_ADC]);
            InfinityDpgaSetGain(BACH_DPGA_AEC1, m_nInfinityDpgaGain[BACH_DPGA_AEC1]);
        }
        else
        {
            InfinityDpgaSetGain(BACH_DPGA_ADC, BACH_DPGA_GAIN_MIN_DB);
            InfinityDpgaSetGain(BACH_DPGA_AEC1, BACH_DPGA_GAIN_MIN_DB);
        }
        break;
    default:
        ERRMSG("InfinitySetPathOnOff - default case!\n");
        break;

    }
}

void InfinitySetPathGain(BachPath_e ePath, S8 s8Gain)
{
    switch(ePath)
    {
    case BACH_PATH_PLAYBACK:
        InfinityDpgaSetGain(BACH_DPGA_MMC1, s8Gain);
        m_nInfinityDpgaGain[BACH_DPGA_MMC1] = InfinityDpgaGetGain(BACH_DPGA_MMC1);
        break;
    case BACH_PATH_CAPTURE:

        InfinityDpgaSetGain(BACH_DPGA_ADC, s8Gain);
        m_nInfinityDpgaGain[BACH_DPGA_ADC] = InfinityDpgaGetGain(BACH_DPGA_ADC);

        InfinityDpgaSetGain(BACH_DPGA_AEC1, s8Gain);
        m_nInfinityDpgaGain[BACH_DPGA_AEC1] = InfinityDpgaGetGain(BACH_DPGA_AEC1);
        break;
    default:
        ERRMSG("InfinitySetPathGain - default case!\n");
        break;

    }
}


void InfinitySysInit(void)
{
    U16 nConfigValue;

    InfinityAtopInit();

    InfinityWriteRegByte(0x00150200, 0x00);
    InfinityWriteRegByte(0x00150201, 0x40);


    InfinityWriteRegByte(0x00150200, 0xff);
    //InfinityWriteRegByte(0x00150201, 0x8d);
    InfinityWriteRegByte(0x00150201, 0x89);



    InfinityWriteRegByte(0x00150202, 0x88);
    InfinityWriteRegByte(0x00150203, 0xff);

    InfinityWriteRegByte(0x00150204, 0x03);
    InfinityWriteRegByte(0x00150205, 0x00);

    InfinityWriteRegByte(0x00150206, 0xB4); // mux 0 sel
    InfinityWriteRegByte(0x00150207, 0x19); // ^^

    InfinityWriteRegByte(0x00150208, 0x00); // mux 1 sel
    InfinityWriteRegByte(0x00150209, 0x00); // ^^

    InfinityWriteRegByte(0x0015020a, 0x00); //
    InfinityWriteRegByte(0x0015020b, 0x80); //

    InfinityWriteRegByte(0x0015020c, 0x9a); //
    InfinityWriteRegByte(0x0015020d, 0xc0); //

    InfinityWriteRegByte(0x0015020e, 0x5a); // mix 1 sel
    InfinityWriteRegByte(0x0015020f, 0x55); //

    InfinityWriteRegByte(0x00150212, 0x05); // sdm ctrl
    InfinityWriteRegByte(0x00150213, 0x02);

    InfinityWriteRegByte(0x00150214, 0x00);
    InfinityWriteRegByte(0x00150215, 0x00);

    InfinityWriteRegByte(0x00150216, 0x7d);
    InfinityWriteRegByte(0x00150217, 0x00);

    InfinityWriteRegByte(0x0015023a, 0x1d); // synth ctrl
    InfinityWriteRegByte(0x0015023b, 0x02); // ^^

    InfinityWriteRegByte(0x0015023a, 0x00); // synth ctrl
    InfinityWriteRegByte(0x0015023b, 0x00); // ^^

    InfinityWriteRegByte(0x0015031c, 0x03); // mux3 sel
    InfinityWriteRegByte(0x0015031d, 0x00); // mux3 sel

    InfinityWriteRegByte(0x0015032c, 0x03);// au sys ctrl 1
    InfinityWriteRegByte(0x0015031d, 0x00); // mux3 sel

    InfinityWriteRegByte(0x00150226, 0x00); // codec i2s tx ctrl
    InfinityWriteRegByte(0x00150227, 0xd4); // ^^

    //correct IC default value
    InfinityWriteRegByte(0x00150248, 0x07); // adc dgpa cfg1
    InfinityWriteRegByte(0x00150249, 0x00); // adc dgpa cfg1
    InfinityWriteRegByte(0x00150250, 0x07); // adc dpga cfg 2

    //digital mic settings(32kHz,4M,CLK_INV)
#ifdef DIGMIC_EN
    InfinityWriteRegByte(0x0015033a, 0x02); // dig mic control0
    InfinityWriteRegByte(0x0015033b, 0x40); // ^^
    InfinityWriteRegByte(0x0015033c, 0x04); // dig mic control1
    InfinityWriteRegByte(0x0015033d, 0x81);//[15] CIC selection : Digital Mic
#endif

    //set I2s pad mux
    nConfigValue = InfinityReadReg2Byte(0x101e1e);
    //nConfigValue |= REG_I2S_MODE;
#ifdef DIGMIC_EN
    nConfigValue |= (1<<8);
#endif
    InfinityWriteReg2Byte(0x101e1e, nConfigValue);

    //PM GPIO01,enable for line-out
    nConfigValue = InfinityReadReg2Byte(0x0f02);
    nConfigValue &= ~(1<<0);
    nConfigValue |= (1<<1);
    InfinityWriteReg2Byte(0x0f02, nConfigValue);

    //init dma sample rate
    m_infinitydmachannel[BACH_DMA_READER1].nSampleRate = 48000;
    m_infinitydmachannel[BACH_DMA_WRITER1].nSampleRate = 48000;

    // nConfigValue = InfinityReadReg2Byte(0x10340A);
    // AUD_PRINTF(ERROR_LEVEL, "!!!!!!!!!!!!%s: 0x10340A=0x%x\n", __FUNCTION__, nConfigValue);

}

void InfinityAtopInit(void)
{

    int i;
    InfinityWriteRegByte(0x00103400, 0x14);
    //InfinityWriteRegByte(0x00103401, 0x02);
    InfinityWriteRegByte(0x00103401, 0x0a);//enable MSP,speed up charge VREF

    // ~ dgp atop 0x0 0x0a14, reset 0x0204, en_dac_ck? msp


    InfinityWriteRegByte(0x00103402, 0x30);
    InfinityWriteRegByte(0x00103403, 0x00);

    // ~ dgp atop 0x4 0x0030, reset 0x0000, en_qs_ldo_adc, en_qs_ldo_dac?

    InfinityWriteRegByte(0x00103404, 0x80);
    InfinityWriteRegByte(0x00103405, 0x00);

    // ~ dgp atop 0x8 - 0x0080, sw tst, reset 0x0080

    //InfinityWriteRegByte(0x00103406, 0x00);
    //InfinityWriteRegByte(0x00103407, 0x00);

    InfinityWriteRegByte(0x00103406, 0xf7);
    InfinityWriteRegByte(0x00103407, 0x1f);

    //~ dgp atop 0xc 0x1ff7, kernel has 1fff, reset 0x1fff, power down everything?

    InfinityWriteRegByte(0x00103408, 0x00);
    InfinityWriteRegByte(0x00103409, 0x00);


    // ~ dgp atop 0x10 0x0000, reset 0x0000 - reset dac?

    InfinityWriteRegByte(0x0010340a, 0x77);
    InfinityWriteRegByte(0x0010340b, 0x00);

    // ~ dgp atop 0x14 0x0077, reset 0x000 - inmux?

    InfinityWriteRegByte(0x0010340c, 0x33);
    InfinityWriteRegByte(0x0010340d, 0x00);

    // ~ dgp atop 0x18 0x0033, reset 0x0000 - sel gain

    InfinityWriteRegByte(0x0010340e, 0x00);
    InfinityWriteRegByte(0x0010340f, 0x00);

    // ~ dgp atop 0x1c 0x0000, reset 0

    InfinityWriteRegByte(0x00103410, 0x14);
    InfinityWriteRegByte(0x00103411, 0x00);

    // ~ dgp atop 0x20 0x0014, reset 0, - mic gain

    InfinityWriteRegByte(0x00103424, 0x02);
    InfinityWriteRegByte(0x00103425, 0x00);

    // ~ dgp atop 0x48 0x0002 ???


    //status init
    m_bADCActive = false;
    m_bDACActive = false;

    for(i = 0; i < BACH_ATOP_NUM; i++)
        m_bInfinityAtopStatus[i] = false;
}

void InfinityAtopEnableRef(bool bEnable)
{
    U16 nMask;
    nMask = (REG_PD_VI | REG_PD_VREF);
    InfinityWriteReg(BACH_REG_BANK3, BACH_ANALOG_CTRL03, nMask, (bEnable? 0:nMask));
}

void InfinityAtopDac(bool bEnable)
{
    U16 nMask;
    nMask = (REG_PD_BIAS_DAC | REG_PD_L0_DAC | REG_PD_LDO_DAC | REG_PD_R0_DAC | REG_PD_REF_DAC);
    InfinityWriteReg(BACH_REG_BANK3, BACH_ANALOG_CTRL03, nMask, (bEnable? 0:nMask));
    m_bDACActive = bEnable;
}

void InfinityAtopAdc(bool bEnable)
{
    U16 nMask;
    nMask = (REG_PD_ADC0 | REG_PD_INMUX_MSK | REG_PD_LDO_ADC );
    InfinityWriteReg(BACH_REG_BANK3, BACH_ANALOG_CTRL03, nMask, (bEnable? 0:((REG_PD_ADC0 | (1<<REG_PD_INMUX_POS) | REG_PD_LDO_ADC ))));
    m_bADCActive = bEnable;
}

void InfinityAtopMic(bool bEnable)
{
    if(bEnable)
    {
        InfinityAtopAdc(true);
        InfinityWriteReg(BACH_REG_BANK3, BACH_ANALOG_CTRL03, REG_PD_MIC_STG1_L | REG_PD_MIC_STG1_R, 0);
        InfinityWriteReg(BACH_REG_BANK3, BACH_ANALOG_CTRL08, REG_SEL_MICGAIN_STG1_L_MSK | REG_SEL_MICGAIN_STG1_R_MSK, m_nMicGain<<REG_SEL_MICGAIN_STG1_L_POS | m_nMicGain<<REG_SEL_MICGAIN_STG1_R_POS);
        InfinityWriteReg(BACH_REG_BANK3, BACH_ANALOG_CTRL06, REG_SEL_GAIN_INMUX0_MSK | REG_SEL_GAIN_INMUX1_MSK, m_nMicInGain<<REG_SEL_GAIN_INMUX0_POS | m_nMicInGain<<REG_SEL_GAIN_INMUX1_POS);
        InfinityWriteReg(BACH_REG_BANK3, BACH_ANALOG_CTRL05, REG_SEL_CH_INMUX0_MSK | REG_SEL_CH_INMUX1_MSK, 0x7<<REG_SEL_CH_INMUX0_POS | 0x7<<REG_SEL_CH_INMUX1_POS);
        m_bInfinityAtopStatus[BACH_ATOP_MIC]=true;

    }
    else
    {
        InfinityAtopAdc(false);
        InfinityWriteReg(BACH_REG_BANK3, BACH_ANALOG_CTRL03, REG_PD_MIC_STG1_L | REG_PD_MIC_STG1_R, REG_PD_MIC_STG1_L | REG_PD_MIC_STG1_R);
        m_bInfinityAtopStatus[BACH_ATOP_MIC]=false;
    }
}

void InfinityAtopLineIn(bool bEnable)
{
    if(bEnable)
    {
        InfinityAtopAdc(true);
        InfinityWriteReg(BACH_REG_BANK3, BACH_ANALOG_CTRL06, REG_SEL_GAIN_INMUX0_MSK | REG_SEL_GAIN_INMUX1_MSK, m_nLineInGain<<REG_SEL_GAIN_INMUX0_POS | m_nLineInGain<<REG_SEL_GAIN_INMUX1_POS);
        InfinityWriteReg(BACH_REG_BANK3, BACH_ANALOG_CTRL05, REG_SEL_CH_INMUX0_MSK | REG_SEL_CH_INMUX1_MSK, 0x0<<REG_SEL_CH_INMUX0_POS | 0x0<<REG_SEL_CH_INMUX1_POS);
        m_bInfinityAtopStatus[BACH_ATOP_LINEIN]=true;
    }
    else
    {
        InfinityAtopAdc(false);
        m_bInfinityAtopStatus[BACH_ATOP_LINEIN]=false;
    }

}


bool InfinityOpenAtop(BachAtopPath_e ePath)
{
    if(ePath < 0 || ePath > BACH_ATOP_NUM)
        return false;
    else
    {
        if(!(m_bADCActive||m_bDACActive))
            InfinityAtopEnableRef(true);
        if(ePath<2)
        {
            if(ePath==BACH_ATOP_LINEIN)
            {
                if(m_bInfinityAtopStatus[BACH_ATOP_MIC])
                    return false;
                else if(!m_bInfinityAtopStatus[BACH_ATOP_LINEIN])
                    InfinityAtopLineIn(true);
            }
            else
            {
                if(m_bInfinityAtopStatus[BACH_ATOP_LINEIN])
                    return false;
                else if(!m_bInfinityAtopStatus[BACH_ATOP_MIC])
                    InfinityAtopMic(true);
            }
        }
        else
        {
            if(!m_bDACActive)
                InfinityAtopDac(true);
        }
        return true;
    }
}

bool InfinityCloseAtop(BachAtopPath_e ePath)
{
    switch(ePath)
    {
    case BACH_ATOP_LINEIN:
        if(m_bInfinityAtopStatus[BACH_ATOP_LINEIN])
            InfinityAtopLineIn(false);
        break;
    case BACH_ATOP_MIC:
        if(m_bInfinityAtopStatus[BACH_ATOP_MIC])
            InfinityAtopMic(false);
        break;
    case BACH_ATOP_LINEOUT:
        if(m_bInfinityAtopStatus[BACH_ATOP_LINEOUT])
            InfinityAtopDac(false);
        break;
    default:
        return false;
    }

    if(!(m_bADCActive || m_bDACActive))
        InfinityAtopEnableRef(false);
    return true;
}


bool InfinityAtopMicGain(U16 nSel)
{
    U16 nMicInSel;
    if(nSel>0x1F)
    {
        ERRMSG("BachAtopMicGain - ERROR!! not Support.\n");
        return false;
    }

    m_nMicGain = (nSel&0x18)>>3;
    nMicInSel = (nSel&0x7);
    if(nMicInSel==2)
        m_nMicInGain = 0x0;
    else if(nMicInSel<2)
        m_nMicInGain = 0x1 + nMicInSel;
    else if(nMicInSel)
        m_nMicInGain = nMicInSel;

    if(m_bInfinityAtopStatus[BACH_ATOP_MIC])
    {
        InfinityWriteReg(BACH_REG_BANK3, BACH_ANALOG_CTRL08, REG_SEL_MICGAIN_STG1_L_MSK | REG_SEL_MICGAIN_STG1_R_MSK, m_nMicGain<<REG_SEL_MICGAIN_STG1_L_POS | m_nMicGain<<REG_SEL_MICGAIN_STG1_R_POS);
        InfinityWriteReg(BACH_REG_BANK3, BACH_ANALOG_CTRL06, REG_SEL_GAIN_INMUX0_MSK | REG_SEL_GAIN_INMUX1_MSK, m_nMicInGain<<REG_SEL_GAIN_INMUX0_POS | m_nMicInGain<<REG_SEL_GAIN_INMUX1_POS);
    }


    return true;
}

bool InfinityAtopLineInGain(U16 nLevel)
{
    if(nLevel>7)
    {
        ERRMSG("BachAtopLineInGain - ERROR!! Level too large .\n");
        return false;
    }

    if(nLevel==2)
        m_nLineInGain = 0x0;
    else if(nLevel<2)
        m_nLineInGain = 0x1 + nLevel;
    else if(nLevel)
        m_nLineInGain = nLevel;

    if(m_bInfinityAtopStatus[BACH_ATOP_LINEIN])
        InfinityWriteReg(BACH_REG_BANK3, BACH_ANALOG_CTRL06, REG_SEL_GAIN_INMUX0_MSK | REG_SEL_GAIN_INMUX1_MSK, m_nLineInGain<<REG_SEL_GAIN_INMUX0_POS | m_nLineInGain<<REG_SEL_GAIN_INMUX1_POS);

    return true;

}

bool InfinityDigMicSetRate(BachRate_e eRate)
{
    U16 nConfigValue;
    nConfigValue = InfinityReadReg(BACH_REG_BANK2, BACH_DIG_MIC_CTRL0);
    if(nConfigValue & REG_DIGMIC_CLK_MODE)// 0:4M, 1:2M
    {
        switch(eRate)
        {
        case BACH_RATE_8K:
            InfinityWriteReg(BACH_REG_BANK2, BACH_DIG_MIC_CTRL0, REG_DIGMIC_SEL_MSK, 1<<REG_DIGMIC_SEL_POS);
            break;
        case BACH_RATE_16K:
            InfinityWriteReg(BACH_REG_BANK2, BACH_DIG_MIC_CTRL0, REG_DIGMIC_SEL_MSK, 2<<REG_DIGMIC_SEL_POS);
            break;
        default:
            return false;
        }
    }
    else
    {
        switch(eRate)
        {
        case BACH_RATE_8K:
            InfinityWriteReg(BACH_REG_BANK2, BACH_DIG_MIC_CTRL0, REG_DIGMIC_SEL_MSK, 0<<REG_DIGMIC_SEL_POS);
            break;
        case BACH_RATE_16K:
            InfinityWriteReg(BACH_REG_BANK2, BACH_DIG_MIC_CTRL0, REG_DIGMIC_SEL_MSK, 1<<REG_DIGMIC_SEL_POS);
            break;
        case BACH_RATE_32K:
            InfinityWriteReg(BACH_REG_BANK2, BACH_DIG_MIC_CTRL0, REG_DIGMIC_SEL_MSK, 2<<REG_DIGMIC_SEL_POS);
            break;
        default:
            return false;
        }
    }
    return true;

}


bool InfinityDigMicEnable(bool bEn)
{
    U16 nConfigValue;
    nConfigValue = InfinityReadReg(BACH_REG_BANK2, BACH_DIG_MIC_CTRL1);
    if(nConfigValue & REG_CIC_SEL)
    {
        InfinityWriteReg(BACH_REG_BANK2, BACH_DIG_MIC_CTRL0, REG_DIGMIC_EN, (bEn ? REG_DIGMIC_EN :0) );
        return true;
    }
    else
        return false;
}
