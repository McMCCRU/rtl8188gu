/******************************************************************************
 *
 * Copyright(c) 2007 - 2017 Realtek Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 *****************************************************************************/

#define _USB_HALINIT_C_

#include <rtl8710b_hal.h>
#ifdef CONFIG_WOWLAN
	#include "hal_com_h2c.h"
#endif

static void _dbg_dump_macreg(PADAPTER padapter)
{
	u32 offset = 0;
	u32 val32 = 0;
	u32 index = 0;

	for (index = 0; index < 64; index++) {
		offset = index * 4;
		val32 = rtw_read32(padapter, offset);
		RTW_INFO("offset : 0x%02x ,val:0x%08x\n", offset, val32);
	}
}

static void _ConfigChipOutEP_8710(PADAPTER padapter,u8 NumOutPipe)
{
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);

	pHalData->OutEpQueueSel = 0;
	pHalData->OutEpNumber = 0;

	switch (NumOutPipe) {
	case 4:
		pHalData->OutEpQueueSel = TX_SELE_HQ | TX_SELE_LQ | TX_SELE_NQ;
		pHalData->OutEpNumber = 4;
		break;
	case 3:
		pHalData->OutEpQueueSel = TX_SELE_HQ | TX_SELE_LQ | TX_SELE_NQ;
		pHalData->OutEpNumber = 3;
		break;
	case 2:
		pHalData->OutEpQueueSel = TX_SELE_HQ | TX_SELE_NQ;
		pHalData->OutEpNumber = 2;
		break;
	case 1:
		pHalData->OutEpQueueSel = TX_SELE_HQ;
		pHalData->OutEpNumber = 1;
		break;
	default:
		break;
	}
}

static BOOLEAN HalUsbSetQueuePipeMapping8710BUsb(PADAPTER padapter, u8 NumInPipe, u8 NumOutPipe)
{
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);
	BOOLEAN result = _FALSE;

	_ConfigChipOutEP_8710(padapter, NumOutPipe);

	result = Hal_MappingOutPipe(padapter, NumOutPipe);

	RTW_INFO("USB NumInPipe(%u), NumOutPipe(%u/%u)\n"
		 , NumInPipe
		 , pHalData->OutEpNumber
		 , NumOutPipe);

	return result;
}

#ifdef CONFIG_GPIO_WAKEUP
/*
 * we set it high under init and fw will
 * give us Low Pulse when host wake up
 */
void HostWakeUpGpioClear(PADAPTER padapter)
{
	u32 value32;

	value32 = rtw_read32(padapter, REG_GPIO_PIN_CTRL_2);

	/* set GPIO 12 1 */
	value32 |= BIT(12); /*4+8 */
	/* GPIO 12 out put */
	value32 |= BIT(20); /*4+16 */

	rtw_write32(padapter, REG_GPIO_PIN_CTRL_2, value32);
} /* HostWakeUpGpioClear */

void HalSetOutPutGPIO(PADAPTER padapter, u8 index, u8 OutPutValue)
{
	if (index <= 7) {
		/* config GPIO mode */
		rtw_write8(padapter, REG_GPIO_PIN_CTRL + 3, rtw_read8(padapter, REG_GPIO_PIN_CTRL + 3) & ~BIT(index));

		/* config GPIO Sel */
		/* 0: input */
		/* 1: output */
		rtw_write8(padapter, REG_GPIO_PIN_CTRL + 2, rtw_read8(padapter, REG_GPIO_PIN_CTRL + 2) | BIT(index));

		/* set output value */
		if (OutPutValue)
			rtw_write8(padapter, REG_GPIO_PIN_CTRL + 1, rtw_read8(padapter, REG_GPIO_PIN_CTRL + 1) | BIT(index));
		else
			rtw_write8(padapter, REG_GPIO_PIN_CTRL + 1, rtw_read8(padapter, REG_GPIO_PIN_CTRL + 1) & ~BIT(index));
	} else {
		/* 88C Series: */
		/* index: 11~8 transform to 3~0 */
		/* 8723 Series: */
		/* index: 12~8 transform to 4~0 */
		index -= 8;

		/* config GPIO mode */
		rtw_write8(padapter, REG_GPIO_PIN_CTRL_2 + 3, rtw_read8(padapter, REG_GPIO_PIN_CTRL_2 + 3) & ~BIT(index));

		/* config GPIO Sel */
		/* 0: input */
		/* 1: output */
		rtw_write8(padapter, REG_GPIO_PIN_CTRL_2 + 2, rtw_read8(padapter, REG_GPIO_PIN_CTRL_2 + 2) | BIT(index));

		/* set output value */
		if (OutPutValue)
			rtw_write8(padapter, REG_GPIO_PIN_CTRL_2 + 1, rtw_read8(padapter, REG_GPIO_PIN_CTRL_2 + 1) | BIT(index));
		else
			rtw_write8(padapter, REG_GPIO_PIN_CTRL_2 + 1, rtw_read8(padapter, REG_GPIO_PIN_CTRL_2 + 1) & ~BIT(index));
	}
}
#endif

void rtl8710bu_interface_configure(PADAPTER padapter)
{
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);
	struct dvobj_priv *pdvobjpriv = adapter_to_dvobj(padapter);

	if (IS_HIGH_SPEED_USB(padapter)) {
		/* HIGH SPEED */
		pHalData->UsbBulkOutSize = USB_HIGH_SPEED_BULK_SIZE; /* 512 bytes */
	} else {
		/* FULL SPEED */
		pHalData->UsbBulkOutSize = USB_FULL_SPEED_BULK_SIZE; /* 64 bytes */
	}

	pHalData->interfaceIndex = pdvobjpriv->InterfaceNumber;

#ifdef CONFIG_USB_TX_AGGREGATION
	pHalData->UsbTxAggMode = 1;
	pHalData->UsbTxAggDescNum = 0x6; /* only 4 bits */
#endif

#ifdef CONFIG_USB_RX_AGGREGATION
	pHalData->rxagg_mode = RX_AGG_USB;
	pHalData->rxagg_usb_size = 0x5; /* unit: 4KB, for USB mode */
	pHalData->rxagg_usb_timeout = 0x20; /* unit: 32us, for USB mode */
	pHalData->rxagg_dma_size = 0xF; /* uint: 1KB, for DMA mode */
	pHalData->rxagg_dma_timeout = 0x20; /* unit: 32us, for DMA mode */
#endif

	HalUsbSetQueuePipeMapping8710BUsb(padapter,
			  pdvobjpriv->RtNumInPipes, pdvobjpriv->RtNumOutPipes);
}

static u32 _InitPowerOn_8710bu(PADAPTER padapter)
{
	u32 status = _SUCCESS;
	u32 value32 = 0;
	u16 value16 = 0;
	u8 value8 = 0;

	//add power sequence for new bitfile by sherry 20160616
	value8 = rtw_read8(padapter, REG_SYS_ISO_CTRL_8710B);
	value8 &= ~(BIT5);
	rtw_write8(padapter,REG_SYS_ISO_CTRL_8710B, value8); //0x8000[5] = 0
	
	value8 = rtw_read8(padapter, REG_SYS_FUNC_EN_8710B);
	value8 |= BIT0;
	rtw_write8(padapter,REG_SYS_FUNC_EN_8710B, value8); //0x8004[0] = 1

	value8 = rtw_read8(padapter, 0x20);
	value8 |= BIT0;
	rtw_write8(padapter, 0x20, value8); //0x8020[0] = 1
	
	rtw_write8(padapter, REG_AFE_CTRL_8710B, 0); //0x8050[7:0] = 0
	
	value8 = rtw_read8(padapter, REG_WL_STATUS_8710B);
	value8 |= BIT1;
	rtw_write8(padapter,REG_WL_STATUS_8710B, value8); //0x80f0[1] = 1

	if(!HalPwrSeqCmdParsing(padapter, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK, rtl8710B_card_enable_flow))
		return _FALSE;

	// Enable MAC DMA/WMAC/SCHEDULE/SEC block
	// Set CR bit10 to enable 32k calibration. Suggested by SD1 Gimmy. Added by tynli. 2011.08.31.
	rtw_write16(padapter, REG_CR_8710B, 0x00);  //suggseted by zhouzhou, by page, 20111230
	value16 = rtw_read16(padapter, REG_CR_8710B);
	value16 |= (HCI_TXDMA_EN | HCI_RXDMA_EN | TXDMA_EN | RXDMA_EN
				| PROTOCOL_EN | SCHEDULE_EN | ENSEC | CALTMR_EN);
	rtw_write16(padapter, REG_CR_8710B, value16);

	// Enable Hw sequence number.
	value8 = rtw_read8(padapter, REG_HWSEQ_CTRL_8710B);
	value8 |= 0x7f;
	rtw_write8(padapter, REG_HWSEQ_CTRL_8710B, value8);
	rtw_udelay_os(2);

	return status;
}

#if 0
/*
 * -------------------------------------------------------------------------
 * LLT R/W/Init function
 * -------------------------------------------------------------------------
 */
static u8 _LLTWrite(
	IN PADAPTER padapter,
	IN u32 address,
	IN u32 data
)
{
	u8 status = _SUCCESS;
	s8 count = POLLING_LLT_THRESHOLD;
	u32 value = _LLT_INIT_ADDR(address) | _LLT_INIT_DATA(data) | _LLT_OP(_LLT_WRITE_ACCESS);

	rtw_write32(padapter, REG_LLT_INIT, value);

	/* polling */
	do {
		value = rtw_read32(padapter, REG_LLT_INIT);
		if (_LLT_NO_ACTIVE == _LLT_OP_VALUE(value))
			break;
	} while (--count);

	if (count <= 0) {
		RTW_INFO("Failed to polling write LLT done at address %d!\n", address);
		status = _FAIL;
	}
	return status;
}

static u8 _LLTRead(
	IN PADAPTER padapter,
	IN u32 address
)
{
	int count = 0;
	u32 value = _LLT_INIT_ADDR(address) | _LLT_OP(_LLT_READ_ACCESS);

	rtw_write32(padapter, REG_LLT_INIT, value);

	/* polling and get value */
	do {
		value = rtw_read32(padapter, REG_LLT_INIT);
		if (_LLT_NO_ACTIVE == _LLT_OP_VALUE(value))
			return (u8)value;

		if (count > POLLING_LLT_THRESHOLD) {
			break;
		}
	} while (count++);

	return 0xFF;
}
#endif

/*
 * ---------------------------------------------------------------
 * MAC init functions
 * ---------------------------------------------------------------
 */

/*
 * USB has no hardware interrupt,
 * so no need to initialize HIMR.
 */
static void _InitInterrupt(PADAPTER padapter)
{
	HAL_DATA_TYPE *pHalData	= GET_HAL_DATA(padapter);

	// HIMR
	rtw_write32(padapter, REG_HIMR0_8710B, pHalData->IntrMask[0]&0xFFFFFFFF);
}

static void _InitQueueReservedPage(PADAPTER padapter)
{
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);
	struct registry_priv *pregistrypriv = &padapter->registrypriv;
	u32 outEPNum = (u32)pHalData->OutEpNumber;
	u32 numHQ = 0;
	u32 numLQ = 0;
	u32 numNQ = 0;
	u32 numPubQ;
	u32 value32;
	u8 value8;
	BOOLEAN bWiFiConfig = pregistrypriv->wifi_spec;

	if (pHalData->OutEpQueueSel & TX_SELE_HQ)
		numHQ = bWiFiConfig ? WMM_NORMAL_PAGE_NUM_HPQ_8710B : NORMAL_PAGE_NUM_HPQ_8710B;

	if (pHalData->OutEpQueueSel & TX_SELE_LQ)
		numLQ = bWiFiConfig ? WMM_NORMAL_PAGE_NUM_LPQ_8710B : NORMAL_PAGE_NUM_LPQ_8710B;

	/* NOTE: This step shall be proceed before writing REG_RQPN. */
	if (pHalData->OutEpQueueSel & TX_SELE_NQ)
		numNQ = bWiFiConfig ? WMM_NORMAL_PAGE_NUM_NPQ_8710B : NORMAL_PAGE_NUM_NPQ_8710B;
	value8 = (u8)_NPQ(numNQ);
	rtw_write8(padapter, REG_RQPN_NPQ, value8);

	numPubQ = TX_TOTAL_PAGE_NUMBER_8710B - numHQ - numLQ - numNQ;

	/* TX DMA */
	value32 = _HPQ(numHQ) | _LPQ(numLQ) | _PUBQ(numPubQ) | LD_RQPN;
	rtw_write32(padapter, REG_RQPN, value32);	
}

static void _InitTRxBufferBoundary(PADAPTER padapter)
{
	struct registry_priv *pregistrypriv = &padapter->registrypriv;
#ifdef CONFIG_CONCURRENT_MODE
	u8 val8;
#endif /* CONFIG_CONCURRENT_MODE */

	/* u16	txdmactrl; */
	u8 txpktbuf_bndy;

	if (!pregistrypriv->wifi_spec)
		txpktbuf_bndy = TX_PAGE_BOUNDARY_8710B;
	else {
		/* for WMM */
		txpktbuf_bndy = WMM_NORMAL_TX_PAGE_BOUNDARY_8710B;
	}

	rtw_write8(padapter, REG_TXPKTBUF_BCNQ_BDNY_8710B, txpktbuf_bndy);
	rtw_write8(padapter, REG_TXPKTBUF_MGQ_BDNY_8710B, txpktbuf_bndy);
	rtw_write8(padapter, REG_TXPKTBUF_WMAC_LBK_BF_HD_8710B, txpktbuf_bndy);
	rtw_write8(padapter, REG_TRXFF_BNDY, txpktbuf_bndy);
	rtw_write8(padapter, REG_TDECTRL + 1, txpktbuf_bndy);

	/* RX Page Boundary */
	rtw_write16(padapter, REG_TRXFF_BNDY + 2, RX_DMA_BOUNDARY_8710B);

#ifdef CONFIG_CONCURRENT_MODE
	val8 = txpktbuf_bndy + 8;
	rtw_write8(padapter, REG_BCNQ1_BDNY, val8);
	rtw_write8(padapter, REG_DWBCN1_CTRL_8710B + 1, val8); /* BCN1_HEAD */

	val8 = rtw_read8(padapter, REG_DWBCN1_CTRL_8710B + 2);
	val8 |= BIT(1); /* BIT1- BIT_SW_BCN_SEL_EN */
	rtw_write8(padapter, REG_DWBCN1_CTRL_8710B + 2, val8);
#endif /* CONFIG_CONCURRENT_MODE */
}


void _InitTransferPageSize_8710bu(PADAPTER padapter)
{
	u1Byte value8;

	value8 = _PSRX(PBP_256) | _PSTX(PBP_256);

	rtw_write8(padapter, REG_PBP, value8);
}


static void _InitNormalChipRegPriority(
	PADAPTER padapter,
	u16 beQ,
	u16 bkQ,
	u16 viQ,
	u16 voQ,
	u16 mgtQ,
	u16 hiQ
)
{
	u16 value16 = (rtw_read16(padapter, REG_TRXDMA_CTRL) & 0x7);

	value16 |= _TXDMA_BEQ_MAP(beQ) | _TXDMA_BKQ_MAP(bkQ) |
		   _TXDMA_VIQ_MAP(viQ) | _TXDMA_VOQ_MAP(voQ) |
		   _TXDMA_MGQ_MAP(mgtQ) | _TXDMA_HIQ_MAP(hiQ);

	rtw_write16(padapter, REG_TRXDMA_CTRL, value16);
}


static void _InitNormalChipTwoOutEpPriority(PADAPTER padapter)
{
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);
	struct registry_priv *pregistrypriv = &padapter->registrypriv;
	u16 beQ, bkQ, viQ, voQ, mgtQ, hiQ;

	u16 valueHi = 0;
	u16 valueLow = 0;

	switch (pHalData->OutEpQueueSel) {
	case (TX_SELE_HQ | TX_SELE_LQ):
		valueHi = QUEUE_HIGH;
		valueLow = QUEUE_LOW;
		break;
	case (TX_SELE_NQ | TX_SELE_LQ):
		valueHi = QUEUE_NORMAL;
		valueLow = QUEUE_LOW;
		break;
	case (TX_SELE_HQ | TX_SELE_NQ):
		valueHi = QUEUE_HIGH;
		valueLow = QUEUE_NORMAL;
		break;
	default:
		/* RT_ASSERT(FALSE,("Shall not reach here!\n")); */
		break;
	}

	if (!pregistrypriv->wifi_spec) {
		beQ = valueLow;
		bkQ = valueLow;
		viQ = valueHi;
		voQ = valueHi;
		mgtQ = valueHi;
		hiQ = valueHi;
	} else { /* for WMM ,CONFIG_OUT_EP_WIFI_MODE */
		beQ = valueLow;
		bkQ = valueHi;
		viQ = valueHi;
		voQ = valueLow;
		mgtQ = valueHi;
		hiQ = valueHi;
	}

	_InitNormalChipRegPriority(padapter, beQ, bkQ, viQ, voQ, mgtQ, hiQ);

}

static void
_InitNormalChipThreeOutEpPriority(
	IN PADAPTER padapter
)
{
	struct registry_priv *pregistrypriv = &padapter->registrypriv;
	u16 beQ, bkQ, viQ, voQ, mgtQ, hiQ;

	if (!pregistrypriv->wifi_spec) { /* typical setting */
		beQ = QUEUE_LOW;
		bkQ = QUEUE_LOW;
		viQ = QUEUE_NORMAL;
		voQ = QUEUE_HIGH;
		mgtQ = QUEUE_HIGH;
		hiQ = QUEUE_HIGH;
	} else { /* for WMM */
		beQ = QUEUE_LOW;
		bkQ = QUEUE_NORMAL;
		viQ = QUEUE_NORMAL;
		voQ = QUEUE_HIGH;
		mgtQ = QUEUE_HIGH;
		hiQ = QUEUE_HIGH;
	}
	_InitNormalChipRegPriority(padapter, beQ, bkQ, viQ, voQ, mgtQ, hiQ);
}

static void _InitQueuePriority(PADAPTER padapter)
{
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);

	switch (pHalData->OutEpNumber) {
	case 2:
		_InitNormalChipTwoOutEpPriority(padapter);
		break;
	case 3:
	case 4:
		_InitNormalChipThreeOutEpPriority(padapter);
		break;
	default:
		/* RT_ASSERT(FALSE,("Shall not reach here!\n")); */
		break;
	}
}

static void _InitHardwareDropIncorrectBulkOut(PADAPTER padapter)
{
	u32 value32 = rtw_read32(padapter, REG_TXDMA_OFFSET_CHK);

	value32 |= DROP_DATA_EN;

	rtw_write32(padapter, REG_TXDMA_OFFSET_CHK, value32);
}

static void _InitNetworkType(PADAPTER padapter)
{
	u32 value32;

	value32 = rtw_read32(padapter, REG_CR_8710B);

	/* TODO: use the other function to set network type */
#if 0 /* RTL8191C_FPGA_NETWORKTYPE_ADHOC */
	value32 = (value32 & ~MASK_NETTYPE) | _NETTYPE(NT_LINK_AD_HOC);
#else
	value32 = (value32 & ~MASK_NETTYPE) | _NETTYPE(NT_LINK_AP);
#endif
	rtw_write32(padapter, REG_CR_8710B, value32);
}


static void _InitDriverInfoSize(PADAPTER padapter, u8 drvInfoSize)
{
	u8 value8;

	/* BIT_DRVINFO_SZ [3:0] */
	value8 = rtw_read8(padapter, REG_RX_DRVINFO_SZ) & 0xF8;
	value8 |= drvInfoSize;
	rtw_write8(padapter, REG_RX_DRVINFO_SZ, value8);
}

static void _InitWMACSetting(PADAPTER padapter)
{
	u16 value16;
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);
	u32 rcr;

	rcr = RCR_APM | RCR_AM | RCR_AB | RCR_CBSSID_DATA | RCR_CBSSID_BCN | RCR_APP_ICV | RCR_AMF | RCR_HTC_LOC_CTRL | RCR_APP_MIC | RCR_APP_PHYST_RXFF;
	rtw_hal_set_hwreg(padapter, HW_VAR_RCR, (u8 *)&rcr);

	/* Accept all data frames */
	value16 = 0xFFFF;
	rtw_write16(padapter, REG_RXFLTMAP2_8710B, value16);

	/* 2010.09.08 hpfan */
	/* Since ADF is removed from RCR, ps-poll will not be indicate to driver, */
	/* RxFilterMap should mask ps-poll to gurantee AP mode can rx ps-poll. */

	value16 = 0x400;
	rtw_write16(padapter, REG_RXFLTMAP1_8710B, value16);

	/* Accept all management frames */
	value16 = 0xFFFF;
	rtw_write16(padapter, REG_RXFLTMAP0_8710B, value16);

}

static void _InitAdaptiveCtrl(PADAPTER padapter)
{
	u16 value16;
	u32 value32;

	/* Response Rate Set */
	value32 = rtw_read32(padapter, REG_RRSR);
	value32 &= ~RATE_BITMAP_ALL;
	value32 |= RATE_RRSR_CCK_ONLY_1M;
	rtw_write32(padapter, REG_RRSR, value32);

	/* CF-END Threshold */
	/* m_spIoBase->rtw_write8(REG_CFEND_TH, 0x1); */

	/* SIFS (used in NAV) */
	value16 = _SPEC_SIFS_CCK(0x10) | _SPEC_SIFS_OFDM(0x10);
	rtw_write16(padapter, REG_SPEC_SIFS_8710B, value16);

	/* Retry Limit */
	value16 = _LRL(RL_VAL_STA) | _SRL(RL_VAL_STA);
	rtw_write16(padapter, REG_RETRY_LIMIT_8710B, value16);

}

static void _InitEDCA(PADAPTER padapter)
{
	/* Set Spec SIFS (used in NAV) */
	rtw_write16(padapter, REG_SPEC_SIFS, 0x100a);
	rtw_write16(padapter, REG_MAC_SPEC_SIFS, 0x100a);

	/* Set SIFS for CCK */
	rtw_write16(padapter, REG_SIFS_CTX, 0x100a);

	/* Set SIFS for OFDM */
	rtw_write16(padapter, REG_SIFS_TRX, 0x100a);

	/* TXOP */
	rtw_write32(padapter, REG_EDCA_BE_PARAM, 0x005EA42B);
	rtw_write32(padapter, REG_EDCA_BK_PARAM, 0x0000A44F);
	rtw_write32(padapter, REG_EDCA_VI_PARAM, 0x005EA324);
	rtw_write32(padapter, REG_EDCA_VO_PARAM, 0x002FA226);

	// 0x50 for 80MHz clock
	rtw_write8(padapter, REG_USTIME_TSF_8710B, 0x50);
	rtw_write8(padapter, REG_USTIME_EDCA_8710B, 0x50);
}

#ifdef CONFIG_RTW_LED
static void _InitHWLed_8710bu(PADAPTER padapter)
{
	struct led_priv *pledpriv = &(padapter->ledpriv);

	if (pledpriv->LedStrategy != HW_LED)
		return;

	/* led initialize,suggest by jingjun_wu */
       //enable GPIOA_23
	hal_set_syson_reg_8710b(padapter, 0x02ac, 0x00ff0000, 0x00);
	hal_set_syson_reg_8710b(padapter, 0x02ac, 0xff000000, 0x03);
	hal_set_syson_reg_8710b(padapter, 0x021c, BIT8, 0x1);
	hal_set_syson_reg_8710b(padapter, 0x0230, BIT24, 0x1);
	//config as output mode
	hal_set_syson_reg_8710b(padapter, 0x1004, BIT23, 0x1); 
	
	/* HW led control */
	/* to do .... */
	/* must consider cases of antenna diversity/ commbo card/solo card/mini card */

}
#endif /* CONFIG_RTW_LED */

/* -------------------------------------------------------------------------
 *
 * LLT R/W/Init function
 *
 * ------------------------------------------------------------------------- */
s32 _InitLLTTable_8710bu(PADAPTER padapter)
{
	systime start;
	u32 passing_time;
	u32 val32;
	s32 ret;

	ret = _FAIL;
	
	val32 = rtw_read32(padapter, REG_AUTO_LLT_8710B);
	val32 |= BIT_AUTO_INIT_LLT;
	rtw_write32(padapter, REG_AUTO_LLT_8710B, val32);

	start = rtw_get_current_time();

	do {
		val32 = rtw_read32(padapter, REG_AUTO_LLT_8710B);
		if (!(val32 & BIT_AUTO_INIT_LLT)) {
			ret = _SUCCESS;
			break;
		}

		passing_time = rtw_get_passing_time_ms(start);
		if (passing_time > 1000) {
			RTW_INFO("%s: FAIL!! REG_AUTO_LLT(0x%X)=%08x\n",
				 __FUNCTION__, REG_AUTO_LLT_8710B, val32);
			break;
		}

		rtw_usleep_os(2);
	} while (1);

	return ret;
}

static void _InitRDGSetting_8710bu(PADAPTER padapter)
{
	rtw_write8(padapter, REG_RD_CTRL_8710B, 0xFF);
	rtw_write16(padapter, REG_RD_NAV_NXT_8710B, 0x200);
	rtw_write8(padapter, REG_RD_RESP_PKT_TH_8710B, 0x05);
}

static void _InitRetryFunction(PADAPTER padapter)
{
	u8 value8;

	value8 = rtw_read8(padapter, REG_FWHW_TXQ_CTRL);
	value8 |= EN_AMPDU_RTY_NEW;
	rtw_write8(padapter, REG_FWHW_TXQ_CTRL, value8);

	/* Set ACK timeout */
	rtw_write8(padapter, REG_ACKTO, 0x40);
}

static void _InitBurstPktLen(PADAPTER padapter)
{
	PHAL_DATA_TYPE pHalData = GET_HAL_DATA(padapter);
	u8 tmp8;

	tmp8 = rtw_read8(padapter, REG_RXDMA_MODE_CTRL_8710B);
	tmp8 &= ~(BIT(4) | BIT(5));
	switch (pHalData->UsbBulkOutSize) {
	case USB_HIGH_SPEED_BULK_SIZE:
		tmp8 |= BIT(4); /* set burst pkt len=512B */
		break;
	case USB_FULL_SPEED_BULK_SIZE:
	default:
		tmp8 |= BIT(5); /* set burst pkt len=64B */
		break;
	}
	tmp8 |= BIT(1) | BIT(2) | BIT(3);
	rtw_write8(padapter, REG_RXDMA_MODE_CTRL_8710B, tmp8);

	pHalData->bSupportUSB3 = _FALSE;

	tmp8 = rtw_read8(padapter, REG_HT_SINGLE_AMPDU_8710B);
	tmp8 |= BIT(7); /* enable single pkt ampdu */
	rtw_write8(padapter, REG_HT_SINGLE_AMPDU_8710B, tmp8);
	rtw_write16(padapter, REG_MAX_AGGR_NUM, 0x0C14);
	rtw_write8(padapter, REG_AMPDU_MAX_TIME_8710B, 0x5E);
	rtw_write32(padapter, REG_AMPDU_MAX_LENGTH_8710B, 0xffffffff);
	if (pHalData->AMPDUBurstMode)
		rtw_write8(padapter, REG_AMPDU_BURST_MODE_8710B, 0x5F);

	/* for VHT packet length 11K */
	rtw_write8(padapter, REG_RX_PKT_LIMIT, 0x18);

	rtw_write8(padapter, REG_PIFS, 0x00);
	rtw_write8(padapter, REG_FWHW_TXQ_CTRL, 0x80);
	rtw_write32(padapter, REG_FAST_EDCA_CTRL, 0x03086666);

	/* to prevent mac is reseted by bus. 20111208, by Page */
	tmp8 = rtw_read8(padapter, REG_RSV_CTRL);
	tmp8 |= BIT(5) | BIT(6);
	rtw_write8(padapter, REG_RSV_CTRL, tmp8);
}

/*-----------------------------------------------------------------------------
 * Function:	usb_AggSettingTxUpdate()
 *
 * Overview:	Separate TX/RX parameters update independent for TP detection and
 *			dynamic TX/RX aggreagtion parameters update.
 *
 * Input:			PADAPTER
 *
 * Output/Return:	NONE
 *
 * Revised History:
 *	When		Who		Remark
 *	12/10/2010	MHC		Separate to smaller function.
 *
 *---------------------------------------------------------------------------*/
static void usb_AggSettingTxUpdate(PADAPTER padapter)
{
#ifdef CONFIG_USB_TX_AGGREGATION
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);
	u32 value32;

	if (padapter->registrypriv.wifi_spec)
		pHalData->UsbTxAggMode = _FALSE;

	if (pHalData->UsbTxAggMode) {
		value32 = rtw_read32(padapter, REG_DWBCN0_CTRL_8710B);
		value32 = value32 & ~(BLK_DESC_NUM_MASK << BLK_DESC_NUM_SHIFT);
		value32 |= ((pHalData->UsbTxAggDescNum & BLK_DESC_NUM_MASK) << BLK_DESC_NUM_SHIFT);

		rtw_write32(padapter, REG_DWBCN0_CTRL_8710B, value32);
		//rtw_write8(padapter, REG_DWBCN1_CTRL_8710B, pHalData->UsbTxAggDescNum << 1);
	}
#endif
}   /* usb_AggSettingTxUpdate */


/*-----------------------------------------------------------------------------
 * Function:	usb_AggSettingRxUpdate()
 *
 * Overview:	Separate TX/RX parameters update independent for TP detection and
 *			dynamic TX/RX aggreagtion parameters update.
 *
 * Input:			PADAPTER
 *
 * Output/Return:	NONE
 *
 *---------------------------------------------------------------------------*/
static void usb_AggSettingRxUpdate(PADAPTER padapter)
{
	PHAL_DATA_TYPE pHalData;
	u8 aggctrl;
	u32 aggrx;
	u32 agg_size;

	pHalData = GET_HAL_DATA(padapter);

	aggctrl = rtw_read8(padapter, REG_TRXDMA_CTRL);
	aggctrl &= ~RXDMA_AGG_EN;

	aggrx = rtw_read32(padapter, REG_RXDMA_AGG_PG_TH);
	aggrx &= ~BIT_USB_RXDMA_AGG_EN;
	aggrx &= ~0xFF0F; /* reset agg size and timeout */

#ifdef CONFIG_USB_RX_AGGREGATION
	switch (pHalData->rxagg_mode) {
	case RX_AGG_DMA:
		agg_size = pHalData->rxagg_dma_size << 10;
		if (agg_size > RX_DMA_BOUNDARY_8710B)
			agg_size = RX_DMA_BOUNDARY_8710B >> 1;
		if ((agg_size + 2048) > MAX_RECVBUF_SZ)
			agg_size = MAX_RECVBUF_SZ - 2048;
		agg_size >>= 10; /* unit: 1K */
		if (agg_size > 0xF)
			agg_size = 0xF;

		aggctrl |= RXDMA_AGG_EN;
		aggrx |= BIT_USB_RXDMA_AGG_EN;
		aggrx |= agg_size;
		aggrx |= (pHalData->rxagg_dma_timeout << 8);
		RTW_INFO("%s: RX Agg-DMA mode, size=%dKB, timeout=%dus\n",
			__func__, agg_size, pHalData->rxagg_dma_timeout * 32);
		break;

	case RX_AGG_USB:
	case RX_AGG_MIX:
		agg_size = pHalData->rxagg_usb_size << 12;
		if ((agg_size + 2048) > MAX_RECVBUF_SZ)
			agg_size = MAX_RECVBUF_SZ - 2048;
		agg_size >>= 12; /* unit: 4K */
		if (agg_size > 0xF)
			agg_size = 0xF;

		aggctrl |= RXDMA_AGG_EN;
		aggrx &= ~BIT_USB_RXDMA_AGG_EN;
		aggrx |= agg_size;
		aggrx |= (pHalData->rxagg_usb_timeout << 8);
		RTW_INFO("%s: RX Agg-USB mode, size=%dKB, timeout=%dus\n",
				 __func__, agg_size * 4, pHalData->rxagg_usb_timeout * 32);
		break;

	case RX_AGG_DISABLE:
	default:
		RTW_INFO("%s: RX Aggregation Disable!\n", __func__);
		break;
	}
#endif /* CONFIG_USB_RX_AGGREGATION */

	rtw_write8(padapter, REG_TRXDMA_CTRL, aggctrl);
	rtw_write32(padapter, REG_RXDMA_AGG_PG_TH, aggrx);
}

static void _initUsbAggregationSetting(PADAPTER padapter)
{
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);

	/* Tx aggregation setting */
	usb_AggSettingTxUpdate(padapter);

	/* Rx aggregation setting */
	usb_AggSettingRxUpdate(padapter);

	/* 201/12/10 MH Add for USB agg mode dynamic switch. */
	pHalData->UsbRxHighSpeedMode = _FALSE;
}

VOID _InitOperationMode(PADAPTER padapter)
{
	u8 regBwOpMode = 0;
	u32 regRATR = 0, regRRSR = 0;

	// This part need to modified according to the rate set we filtered!!
	//
	// Set RRSR, RATR, and REG_BWOPMODE registers
	//
	switch(padapter->registrypriv.wireless_mode)
	{
		case WIRELESS_MODE_B:
			regBwOpMode = BW_OPMODE_20MHZ;
			regRATR = RATE_ALL_CCK;
			regRRSR = RATE_ALL_CCK;
			break;

		case WIRELESS_MODE_G:
			regBwOpMode = BW_OPMODE_20MHZ;
			regRATR = RATE_ALL_CCK | RATE_ALL_OFDM_AG;
			regRRSR = RATE_ALL_CCK | RATE_ALL_OFDM_AG;
			break;
			
		case WIRELESS_MODE_N_24G:
			// It support CCK rate by default.
			// CCK rate will be filtered out only when associated AP does not support it.
			regBwOpMode = BW_OPMODE_20MHZ;
				regRATR = RATE_ALL_CCK | RATE_ALL_OFDM_AG | RATE_ALL_OFDM_1SS | RATE_ALL_OFDM_2SS;
				regRRSR = RATE_ALL_CCK | RATE_ALL_OFDM_AG;
			break;
			
		default: //for MacOSX compiler warning.
			break;
	}

	rtw_write8(padapter, REG_BWOPMODE, regBwOpMode);
}

static void _InitRFType(PADAPTER padapter)
{
	struct registry_priv *pregpriv = &padapter->registrypriv;
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);

#if DISABLE_BB_RF
	pHalData->rf_chip = RF_PSEUDO_11N;
	pHalData->rf_type = RF_1T1R;
	return;
#endif

	pHalData->rf_chip = RF_6052;
	pHalData->rf_type = RF_1T1R;

	RTW_INFO("Set RF Chip ID to RF_6052 and RF type to %d.\n", pHalData->rf_type);
}

/* Set CCK and OFDM Block "ON" */
static void _BBTurnOnBlock(PADAPTER padapter)
{
#if (DISABLE_BB_RF)
	return;
#endif

	phy_set_bb_reg(padapter, rFPGA0_RFMOD, bCCKEn, 0x1);
	phy_set_bb_reg(padapter, rFPGA0_RFMOD, bOFDMEn, 0x1);
}

/* 2010/08/09 MH Add for power down check. */
static BOOLEAN HalDetectPwrDownMode(PADAPTER padapter)
{
	HAL_DATA_TYPE *pHalData	= GET_HAL_DATA(padapter);

	pHalData->pwrdown = FALSE;

	RTW_DBG("Hal_DetectPwrDownMode_8710B(): PDN=%d\n", pHalData->pwrdown);
	
	return pHalData->pwrdown;	
}

rt_rf_power_state RfOnOffDetect(PADAPTER padapter)
{
	/* HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter); */
	u8 val8;
	rt_rf_power_state rfpowerstate = rf_off;

	if (adapter_to_pwrctl(padapter)->bHWPowerdown) {
		val8 = rtw_read8(padapter, REG_HSISR);
		RTW_INFO("pwrdown, 0x5c(BIT(7))=%02x\n", val8);
		rfpowerstate = (val8 & BIT(7)) ? rf_off : rf_on;
	} else { /* rf on/off */
		rtw_write8(padapter, REG_MAC_PINMUX_CFG, rtw_read8(padapter, REG_MAC_PINMUX_CFG) & ~(BIT(3)));
		val8 = rtw_read8(padapter, REG_GPIO_IO_SEL);
		RTW_INFO("GPIO_IN=%02x\n", val8);
		rfpowerstate = (val8 & BIT(3)) ? rf_on : rf_off;
	}
	return rfpowerstate;
}   /* HalDetectPwrDownMode */

void _InitBBRegBackup_8710bu(PADAPTER padapter)
{
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);

	/* For Channel 1~11 (Default Value)*/
	pHalData->RegForRecover[0].offset = rCCK0_TxFilter2;
	pHalData->RegForRecover[0].value =
		phy_query_bb_reg(padapter, pHalData->RegForRecover[0].offset, bMaskDWord);

	pHalData->RegForRecover[1].offset = rCCK0_DebugPort;
	pHalData->RegForRecover[1].value =
 		phy_query_bb_reg(padapter, pHalData->RegForRecover[1].offset, bMaskDWord);

	pHalData->RegForRecover[2].offset = 0xAAC;
	pHalData->RegForRecover[2].value =
		phy_query_bb_reg(padapter, pHalData->RegForRecover[2].offset, bMaskDWord);

	pHalData->RegForRecover[3].offset = rCCK0_TxFilter1;  //0xa20
	pHalData->RegForRecover[3].value =
		phy_query_bb_reg(padapter, pHalData->RegForRecover[3].offset, bMaskDWord);
}

u32 rtl8710bu_hal_init(PADAPTER padapter)
{
	u8 value8 = 0, u1bRegCR;
	u32 status = _SUCCESS;
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);
	struct pwrctrl_priv *pwrctrlpriv = adapter_to_pwrctl(padapter);
	struct registry_priv *pregistrypriv = &padapter->registrypriv;
	rt_rf_power_state eRfPowerStateToSet;
	u32 NavUpper = WiFiNavUpperUs;
	u32 value32;
	systime init_start_time = rtw_get_current_time();

	value8 = rtw_read8(padapter, REG_8051FW_CTRL_V1_8710B);
	if(value8 == 0xC6)
		RTW_INFO(" Keep alive is TRUE.\n");

	/* Check if MAC has already power on. */
	rtw_write8(padapter, REG_USB_ACCESS_TIMEOUT, 0x80);  //set usb timeout to fix read mac register fail before power on
	u1bRegCR = rtw_read8(padapter, REG_CR_8710B);

	if (u1bRegCR != 0 && u1bRegCR != 0xEA)
		RTW_INFO(" MAC has already power on.\n");
	else 
		RTW_INFO(" MAC has not been powered on yet.\n");

#ifdef CONFIG_WOWLAN
	if (rtw_read8(padapter, REG_MCUFWDL) & BIT(7) &&
	    (pwrctrlpriv->wowlan_wake_reason & FW_DECISION_DISCONNECT)) {
		u8 reg_val = 0;

		RTW_INFO("+Reset Entry+\n");
		rtw_write8(padapter, REG_MCUFWDL, 0x00);
		_8051Reset8710(padapter);
		/* reset BB */
		reg_val = rtw_read8(padapter, REG_SYS_FUNC_EN);
		reg_val &= ~(BIT(0) | BIT(1));
		rtw_write8(padapter, REG_SYS_FUNC_EN, reg_val);
		/* reset RF */
		rtw_write8(padapter, REG_RF_CTRL, 0);
		/* reset TRX path */
		rtw_write16(padapter, REG_CR, 0);
		/* reset MAC, Digital Core */
		reg_val = rtw_read8(padapter, REG_SYS_FUNC_EN + 1);
		reg_val &= ~(BIT(4) | BIT(7));
		rtw_write8(padapter, REG_SYS_FUNC_EN + 1, reg_val);
		reg_val = rtw_read8(padapter, REG_SYS_FUNC_EN + 1);
		reg_val |= BIT(4) | BIT(7);
		rtw_write8(padapter, REG_SYS_FUNC_EN + 1, reg_val);
		RTW_INFO("-Reset Entry-\n");
	}
#endif /* CONFIG_WOWLAN */

	status = rtw_hal_power_on(padapter);
	if (status == _FAIL) {
		goto exit;
	}

	value8 = rtw_read8(padapter, 0xFEF9);
	value8 = value8 & (~BIT0);
	rtw_write8(padapter, 0xFEF9, value8);
	hal_set_syson_reg_8710b(padapter, 0x138, BIT5, 0x0);//add by ylb , clear the bit to prevent CM4 Suspend
	RTW_DBG("Clear the 0x40000138[5] to prevent CM4 Suspend! 0x40000138 = 0x%x\n", hal_query_syson_reg_8710b(padapter, 0x138, bMaskByte0));

	status = _InitLLTTable_8710bu(padapter);
	if (status == _FAIL) {
		goto exit;
	}

	if (pHalData->bRDGEnable)
		_InitRDGSetting_8710bu(padapter);

	/* Enable TX Report */
	/* Enable Tx Report Timer */
	value8 = rtw_read8(padapter, REG_TX_RPT_CTRL);
	rtw_write8(padapter, REG_TX_RPT_CTRL, value8 | BIT(1));
	
	/* Set MAX RPT MACID */
	rtw_write8(padapter, REG_TX_RPT_CTRL + 1, 2);
	
	/* Tx RPT Timer. Unit: 32us */
	rtw_write16(padapter, REG_TX_RPT_TIME, 0xCdf0);

#ifdef CONFIG_TX_EARLY_MODE
	if (pHalData->AMPDUBurstMode) {
		value8 = rtw_read8(padapter, REG_EARLY_MODE_CONTROL_8710B);
#if RTL8710B_EARLY_MODE_PKT_NUM_10 == 1
		value8 = value8 | 0x1f;
#else
		value8 = value8 | 0xf;
#endif
		rtw_write8(padapter, REG_EARLY_MODE_CONTROL_8710B, value8);

		rtw_write8(padapter, REG_EARLY_MODE_CONTROL_8710B + 3, 0x80);

		value8 = rtw_read8(padapter, REG_TCR_8710B + 1);
		value8 = value8 | 0x40;
		rtw_write8(padapter, REG_TCR_8710B + 1, value8);
	} else
#endif
		rtw_write8(padapter, REG_EARLY_MODE_CONTROL_8710B, 0);

	if (padapter->registrypriv.mp_mode == 0
		#if defined(CONFIG_MP_INCLUDED) && defined(CONFIG_RTW_CUSTOMER_STR)
		|| padapter->registrypriv.mp_customer_str
		#endif
	) {
		status = rtl8710b_FirmwareDownload(padapter, _FALSE);
		if (status != _SUCCESS) {
			pHalData->bFWReady = _FALSE;
			pHalData->fw_ractrl = _FALSE;
			goto exit;
		} else {
			pHalData->bFWReady = _TRUE;
			pHalData->fw_ractrl = _TRUE;
		}
	}

	if (pwrctrlpriv->reg_rfoff == _TRUE)
		pwrctrlpriv->rf_pwrstate = rf_off;

	/* Set RF type for BB/RF configuration */
	_InitRFType(padapter);

	HalDetectPwrDownMode(padapter);

#if (DISABLE_BB_RF == 1)
	/* fpga verification to open phy */
	rtw_write8(padapter, REG_SYS_FUNC_EN_8710B + 2, BIT0 | BIT1);
#endif

#if (HAL_MAC_ENABLE == 1)
	status = PHY_MACConfig8710B(padapter);
	if (status == _FAIL) {
		RTW_INFO("PHY_MACConfig8710B fault !!\n");
		goto exit;
	}
#endif
RTW_INFO("Config MAC succeed\n");

	/* d. Initialize BB related configurations. */
#if (HAL_BB_ENABLE == 1)
	status = PHY_BBConfig8710B(padapter);
	if (status == _FAIL) {
		RTW_INFO("PHY_BBConfig8710B fault !!\n");
		goto exit;
	}
#endif
RTW_INFO("Config BB succeed\n");

#if (HAL_RF_ENABLE == 1)
	status = PHY_RFConfig8710B(padapter);

	if (status == _FAIL) {
		RTW_INFO("PHY_RFConfig8710B fault !!\n");
		goto exit;
	}
	/*---- Set CCK and OFDM Block "ON"----*/
	phy_set_bb_reg(padapter, rFPGA0_RFMOD, bCCKEn, 0x1);
	phy_set_bb_reg(padapter, rFPGA0_RFMOD, bOFDMEn, 0x1);
#endif
RTW_INFO("Config RF succeed\n");

	_InitBBRegBackup_8710bu(padapter);
	_InitQueuePriority(padapter);
	_InitQueueReservedPage(padapter);
	_InitTRxBufferBoundary(padapter);
	_InitTransferPageSize_8710bu(padapter);

	/* Get Rx PHY status in order to report RSSI and others. */
	_InitDriverInfoSize(padapter, DRVINFO_SZ);

	_InitInterrupt(padapter);
	_InitNetworkType(padapter); /* set msr */
	_InitWMACSetting(padapter);
	_InitAdaptiveCtrl(padapter);
	_InitEDCA(padapter);

	_InitRetryFunction(padapter);
	_initUsbAggregationSetting(padapter);
	_InitOperationMode(padapter);
	rtl8710b_InitBeaconParameters(padapter);
	rtl8710b_InitBeaconMaxError(padapter, _TRUE);

	_InitBurstPktLen(padapter);

#ifdef ENABLE_USB_DROP_INCORRECT_OUT
	_InitHardwareDropIncorrectBulkOut(padapter);
#endif

#if defined(CONFIG_CONCURRENT_MODE) || defined(CONFIG_TX_MCAST2UNI)

#ifdef CONFIG_CHECK_AC_LIFETIME
	/* Enable lifetime check for the four ACs */
	rtw_write8(padapter, REG_LIFETIME_CTRL, rtw_read8(padapter, REG_LIFETIME_CTRL) | 0x0f);
#endif	/* CONFIG_CHECK_AC_LIFETIME */

#ifdef CONFIG_TX_MCAST2UNI
	rtw_write16(padapter, REG_PKT_VO_VI_LIFE_TIME, 0x0400); /* unit: 256us. 256ms */
	rtw_write16(padapter, REG_PKT_BE_BK_LIFE_TIME, 0x0400); /* unit: 256us. 256ms */
#else	/* CONFIG_TX_MCAST2UNI */
	rtw_write16(padapter, REG_PKT_VO_VI_LIFE_TIME, 0x3000); /* unit: 256us. 3s */
	rtw_write16(padapter, REG_PKT_BE_BK_LIFE_TIME, 0x3000); /* unit: 256us. 3s */
#endif	/* CONFIG_TX_MCAST2UNI */
#endif	/* CONFIG_CONCURRENT_MODE || CONFIG_TX_MCAST2UNI */


#ifdef CONFIG_RTW_LED
	_InitHWLed_8710bu(padapter);	
#endif /* CONFIG_RTW_LED */

	rtw_hal_set_chnl_bw(padapter, padapter->registrypriv.channel,
		CHANNEL_WIDTH_20, HAL_PRIME_CHNL_OFFSET_DONT_CARE, HAL_PRIME_CHNL_OFFSET_DONT_CARE);

	invalidate_cam_all(padapter);

	/* 2010/12/17 MH We need to set TX power according to EFUSE content at first. */
	/* PHY_SetTxPowerLevel8710B(padapter, pHalData->current_channel); */
	//rtl8710b_InitAntenna_Selection(padapter);

	/*
	 * Disable BAR, suggested by Scott
	 * 2010.04.09 add by hpfan
	 */
	rtw_write32(padapter, REG_BAR_MODE_CTRL, 0x0201ffff);

	if (pregistrypriv->wifi_spec)
		rtw_write16(padapter, REG_FAST_EDCA_CTRL, 0);

	/* Move by Neo for USB SS from above setp */

	/* _RfPowerSave(padapter); */

	rtl8710b_InitHalDm(padapter);

#if (MP_DRIVER == 1)
	if (padapter->registrypriv.mp_mode == 1) {
		padapter->mppriv.channel = pHalData->current_channel;
		MPT_InitializeAdapter(padapter, padapter->mppriv.channel);
	} else
#endif
	{
		pwrctrlpriv->rf_pwrstate = rf_on;

		if (pwrctrlpriv->rf_pwrstate == rf_on) {
			struct pwrctrl_priv *pwrpriv;
			systime start_time;
			u8 restore_iqk_rst;
			u8 b2Ant;
			u8 h2cCmdBuf;

			pwrpriv = adapter_to_pwrctl(padapter);

			/*phy_lc_calibrate_8710b(&pHalData->odmpriv);*/
			halrf_lck_trigger(&pHalData->odmpriv);

			restore_iqk_rst = (pwrpriv->bips_processing == _TRUE) ? _TRUE : _FALSE;

			halrf_iqk_trigger(&pHalData->odmpriv, _FALSE);
			//halrf_iqk_trigger(&pHalData->odmpriv, restore_iqk_rst);
			
			/*phy_iq_calibrate_8710b(padapter, _FALSE);*/

			pHalData->odmpriv.rf_calibrate_info.is_iqk_initialized = _TRUE;
			pHalData->bIQKInitialized = _TRUE;

			odm_txpowertracking_check(&pHalData->odmpriv);
		}
	}

	rtw_hal_set_hwreg(padapter, HW_VAR_NAV_UPPER, (u8 *)&NavUpper);

#ifdef CONFIG_XMIT_ACK
	/* ack for xmit mgmt frames. */
	rtw_write32(padapter, REG_FWHW_TXQ_CTRL, rtw_read32(padapter, REG_FWHW_TXQ_CTRL) | BIT(12));
#endif /*CONFIG_XMIT_ACK */

	phy_set_bb_reg(padapter, rOFDM0_XAAGCCore1, bMaskByte0, 0x50);
	phy_set_bb_reg(padapter, rOFDM0_XAAGCCore1, bMaskByte0, 0x20);

	/* 
	* 0x76D[5:4] is Port0,Port1 Enable Bit.
	* This is Only for 8710B, 2b'00 for MP and 2b'11 for Normal Driver 
	*/
	value8 = rtw_read8(padapter, REG_PORT_CONTROL_8710B);
	if (padapter->registrypriv.mp_mode == 1)
		rtw_write8(padapter, REG_PORT_CONTROL_8710B, value8&(~(BIT4|BIT5)));
	else
		rtw_write8(padapter, REG_PORT_CONTROL_8710B, value8|BIT4|BIT5);


	/* Set 0x5c[8] and [2:0] = 1,  LDO mode, suggest by RF tod_ji */
	value32 = rtw_read32(padapter, REG_WL_RF_PSS_8710B);
	rtw_write32(padapter, REG_WL_RF_PSS_8710B, value32 | (BIT0|BIT1|BIT2|BIT8)); //0x5c = 0x107

	if (padapter->registrypriv.mp_mode == 1) {
		//This is Only for 8710B MP , suggest by RF jerry_fan
		PlatformEFIOWrite4Byte(padapter, REG_EDCA_VO_PARAM_8710B, 0x00000080); // interval 128us
		PlatformEFIOWrite4Byte(padapter, REG_EDCA_VI_PARAM_8710B,  0x00000080); // interval 128us 
		PlatformEFIOWrite4Byte(padapter, REG_EDCA_BE_PARAM_8710B, 0x00000080); // interval 128us 
		PlatformEFIOWrite4Byte(padapter, REG_EDCA_BK_PARAM_8710B, 0x00000080); // interval 128us
	}

	if (padapter->registrypriv.mp_mode == 1) {
		value32 = indirect_read32_8710b(padapter, 0x10000318);
		indirect_write32_8710b(padapter, 0x10000318, value32&(~BIT19));//clear 0x10000318[19]=0
		RTW_DBG("%s : REG 0x10000318 = 0x%x!!\n", __FUNCTION__, indirect_read32_8710b(padapter, 0x10000318));
	}

	RTW_DBG("CM4 Image Version: 0x4000013c = 0x%x \n", hal_query_syson_reg_8710b(padapter, 0x013c, 0xFFFFFFFF));

	/* Enable MACTXEN/MACRXEN block,  2011.08.05. by tynli. */
	u1bRegCR = rtw_read8(padapter, REG_CR);
	u1bRegCR |= (MACTXEN | MACRXEN);
	rtw_write8(padapter, REG_CR, u1bRegCR);

	if (padapter->registrypriv.wifi_spec == 1)
		phy_set_bb_reg(padapter, rOFDM0_ECCAThreshold, 0x00ff00ff, 0x00250029);
	/*_dbg_dump_macreg(padapter); */

exit:

	RTW_INFO("%s in %dms\n", __func__, rtw_get_passing_time_ms(init_start_time));

	return status;
}

#if 0
static void
_DisableGPIO(
	IN PADAPTER padapter
)
{
	/*
	 * j. GPIO_PIN_CTRL 0x44[31:0]=0x000
	 * k. Value = GPIO_PIN_CTRL[7:0]
	 * l. GPIO_PIN_CTRL 0x44[31:0] = 0x00FF0000 | (value <<8); write external PIN level
	 * m. GPIO_MUXCFG 0x42 [15:0] = 0x0780
	 * n. LEDCFG 0x4C[15:0] = 0x8080
	 */
	u8 value8;
	u16 value16;
	u32 value32;

	/* 1. Disable GPIO[7:0] */
	rtw_write16(padapter, REG_GPIO_PIN_CTRL + 2, 0x0000);
	value32 = rtw_read32(padapter, REG_GPIO_PIN_CTRL) & 0xFFFF00FF;
	value8 = (u8)(value32 & 0x000000FF);
	value32 |= ((value8 << 8) | 0x00FF0000);
	rtw_write32(padapter, REG_GPIO_PIN_CTRL, value32);

	/* 2. Disable GPIO[10:8] */
	rtw_write8(padapter, REG_GPIO_MUXCFG + 3, 0x00);
	value16 = rtw_read16(padapter, REG_GPIO_MUXCFG + 2) & 0xFF0F;
	value8 = (u8)(value16 & 0x000F);
	value16 |= ((value8 << 4) | 0x0780);
	rtw_write16(padapter, REG_GPIO_MUXCFG + 2, value16);

	/* 3. Disable LED0 & 1 */
	rtw_write16(padapter, REG_LEDCFG0, 0x8080);


} /* end of _DisableGPIO() */

static void
_ResetFWDownloadRegister(
	IN PADAPTER padapter
)
{
	u32 value32;

	value32 = rtw_read32(padapter, REG_MCUFWDL);
	value32 &= ~(MCUFWDL_EN | MCUFWDL_RDY);
	rtw_write32(padapter, REG_MCUFWDL, value32);
}

static void
_ResetBB(
	IN PADAPTER padapter
)
{
	u16 value16;

	/* reset BB */
	value16 = rtw_read16(padapter, REG_SYS_FUNC_EN);
	value16 &= ~(FEN_BBRSTB | FEN_BB_GLB_RSTn);
	rtw_write16(padapter, REG_SYS_FUNC_EN, value16);
}

static void
_ResetMCU(
	IN PADAPTER padapter
)
{
	u16 value16;

	/* reset MCU */
	value16 = rtw_read16(padapter, REG_SYS_FUNC_EN);
	value16 &= ~FEN_CPUEN;
	rtw_write16(padapter, REG_SYS_FUNC_EN, value16);
}

static void
_DisableMAC_AFE_PLL(
	IN PADAPTER padapter
)
{
	u32 value32;

	/* disable MAC/ AFE PLL */
	value32 = rtw_read32(padapter, REG_APS_FSMCO);
	value32 |= APDM_MAC;
	rtw_write32(padapter, REG_APS_FSMCO, value32);

	value32 |= APFM_OFF;
	rtw_write32(padapter, REG_APS_FSMCO, value32);
}

static void
_AutoPowerDownToHostOff(
	IN PADAPTER padapter
)
{
	u32 value32;

	rtw_write8(padapter, REG_SPS0_CTRL, 0x22);

	value32 = rtw_read32(padapter, REG_APS_FSMCO);

	value32 |= APDM_HOST; /* card disable */
	rtw_write32(padapter, REG_APS_FSMCO, value32);

	/* set USB suspend */
	value32 = rtw_read32(padapter, REG_APS_FSMCO);
	value32 &= ~AFSM_PCIE;
	rtw_write32(padapter, REG_APS_FSMCO, value32);

}


static void
_DisableRFAFEAndResetBB(
	IN PADAPTER padapter
)
{
	/*
	 * a.	TXPAUSE 0x522[7:0] = 0xFF			Pause MAC TX queue
	 * b.	RF path 0 offset 0x00 = 0x00		disable RF
	 * c.	APSD_CTRL 0x600[7:0] = 0x40
	 * d.	SYS_FUNC_EN 0x02[7:0] = 0x16		reset BB state machine
	 * e.	SYS_FUNC_EN 0x02[7:0] = 0x14		reset BB state machine
	 */
	enum rf_path eRFPath = RF_PATH_A, value8 = 0;

	rtw_write8(padapter, REG_TXPAUSE, 0xFF);
	phy_set_rf_reg(padapter, eRFPath, 0x0, bMaskByte0, 0x0);

	value8 |= APSDOFF;
	rtw_write8(padapter, REG_APSD_CTRL, value8); /*0x40 */

	value8 = 0;
	value8 |= (FEN_USBD | FEN_USBA | FEN_BB_GLB_RSTn);
	rtw_write8(padapter, REG_SYS_FUNC_EN, value8); /*0x16 */

	value8 &= (~FEN_BB_GLB_RSTn);
	rtw_write8(padapter, REG_SYS_FUNC_EN, value8); /*0x14 */

}

static void
_DisableAnalog(
	IN PADAPTER padapter,
	IN BOOLEAN bWithoutHWSM
)
{
	u16 value16 = 0;
	u8 value8 = 0;
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);

	if (bWithoutHWSM) {
		/*
		 * n.	LDOA15_CTRL 0x20[7:0] = 0x04		disable A15 power
		 * o.	LDOV12D_CTRL 0x21[7:0] = 0x54		disable digital core power
		 * r.	When driver call disable, the ASIC will turn off remaining clock automatically
		 */

		rtw_write8(padapter, REG_LDOA15_CTRL, 0x04);
		/* PlatformIOWrite1Byte(padapter, REG_LDOV12D_CTRL, 0x54); */

		value8 = rtw_read8(padapter, REG_LDOV12D_CTRL);
		value8 &= (~LDV12_EN);
		rtw_write8(padapter, REG_LDOV12D_CTRL, value8);
	}

	/*
	 * h.	SPS0_CTRL 0x11[7:0] = 0x23			enter PFM mode
	 * i.	APS_FSMCO 0x04[15:0] = 0x4802		set USB suspend
	 */


	value8 = 0x23;

	rtw_write8(padapter, REG_SPS0_CTRL, value8);


	if (bWithoutHWSM) {
		/* 2010/08/31 According to Filen description, we need to use HW to shut down 8051 automatically. */
		/* Because suspend operation need the asistance of 8051 to wait for 3ms. */
		value16 |= (APDM_HOST | AFSM_HSUS | PFM_ALDN);
	} else
		value16 |= (APDM_HOST | AFSM_HSUS | PFM_ALDN);

	rtw_write16(padapter, REG_APS_FSMCO, value16); /* 0x4802 */

	rtw_write8(padapter, REG_RSV_CTRL, 0x0e);

#if 0
	/* tynli_test for suspend mode. */
	if (!bWithoutHWSM)
		rtw_write8(padapter, 0xfe10, 0x19);
#endif

}
#endif

void card_disable_8710bu(PADAPTER padapter)
{
	u8 value8;

	RTW_DBG("Enter card_disable_8710bu\n");
	
	hal_set_syson_reg_8710b(padapter, 0x138, BIT5, 0x1); //add by ylb , set the bit to allow CM4 Suspend
	RTW_DBG("Set the 0x40000138[5] to allow CM4 Suspend! 0x40000138 = 0x%x\n",hal_query_syson_reg_8710b(padapter, 0x138, bMaskByte0));
	
	// stop rx 
	rtw_write8(padapter, REG_CR_8710B, 0x0);

	// Run LPS WL RFOFF flow
	HalPwrSeqCmdParsing(padapter, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK, rtl8710B_enter_lps_flow);

	value8 = rtw_read8(padapter, (REG_8051FW_CTRL_V1_8710B+3));
	rtw_write8(padapter, (REG_8051FW_CTRL_V1_8710B+3), value8 & (~BIT0));

	// MCUFWDL 0x80[1:0]=0				
	// reset MCU ready status
	rtw_write8(padapter, REG_8051FW_CTRL_V1_8710B, 0x00);	

	// Card disable power action flow
	HalPwrSeqCmdParsing(padapter, PWR_CUT_ALL_MSK, PWR_FAB_ALL_MSK, PWR_INTF_USB_MSK, rtl8710B_card_disable_flow);	

}


u32 rtl8710bu_hal_deinit(PADAPTER padapter)
{
	PHAL_DATA_TYPE pHalData = GET_HAL_DATA(padapter);
	struct pwrctrl_priv *pwrctl = adapter_to_pwrctl(padapter);

	RTW_INFO("==> %s\n", __func__);

	rtw_write32(padapter, REG_HISR0_8710B | PAGE0_OFFSET, 0xFFFFFFFF);
	rtw_write32(padapter, REG_HIMR0_8710B | PAGE0_OFFSET, 0x0);
	
#ifdef CONFIG_MP_INCLUDED
	if (padapter->registrypriv.mp_mode == 1)
		MPT_DeInitAdapter(padapter);
#endif

	if (rtw_is_hw_init_completed(padapter)) {
		rtw_hal_power_off(padapter);
	}
	
	return _SUCCESS;
}


unsigned int rtl8710bu_inirp_init(PADAPTER padapter)
{
	u8 i;
	struct recv_buf *precvbuf;
	uint status;
	struct dvobj_priv *pdev = adapter_to_dvobj(padapter);
	struct intf_hdl *pintfhdl = &padapter->iopriv.intf;
	struct recv_priv *precvpriv = &(padapter->recvpriv);

	u32(*_read_port)(struct intf_hdl *pintfhdl, u32 addr, u32 cnt, u8 *pmem);
#ifdef CONFIG_USB_INTERRUPT_IN_PIPE
	u32(*_read_interrupt)(struct intf_hdl *pintfhdl, u32 addr);
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);
#endif /*CONFIG_USB_INTERRUPT_IN_PIPE */


	_read_port = pintfhdl->io_ops._read_port;

	status = _SUCCESS;

	precvpriv->ff_hwaddr = RECV_BULK_IN_ADDR;

	/* issue Rx irp to receive data */
	precvbuf = (struct recv_buf *)precvpriv->precv_buf;
	for (i = 0; i < NR_RECVBUFF; i++) {
		if (_read_port(pintfhdl, precvpriv->ff_hwaddr, 0, (unsigned char *)precvbuf) == _FALSE) {
			status = _FAIL;
			goto exit;
		}

		precvbuf++;
		precvpriv->free_recv_buf_queue_cnt--;
	}

#ifdef CONFIG_USB_INTERRUPT_IN_PIPE
	_read_interrupt = pintfhdl->io_ops._read_interrupt;
	if (_read_interrupt(pintfhdl, RECV_INT_IN_ADDR) == _FALSE) {
		status = _FAIL;
	}
	pHalData->IntrMask[0] = rtw_read32(padapter, REG_HIMR0_8710B);
	RTW_INFO("pHalData->IntrMask = 0x%04x\n", pHalData->IntrMask[0]);
	pHalData->IntrMask[0] |= UHIMR_C2HCMD | UHIMR_CPWM;
	rtw_write32(padapter, REG_HIMR0_8710B, pHalData->IntrMask[0]);
#endif /*CONFIG_USB_INTERRUPT_IN_PIPE */

exit:
	return status;

}

unsigned int rtl8710bu_inirp_deinit(PADAPTER padapter)
{
#ifdef CONFIG_USB_INTERRUPT_IN_PIPE
	u32(*_read_interrupt)(struct intf_hdl *pintfhdl, u32 addr);
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);
#endif /*CONFIG_USB_INTERRUPT_IN_PIPE */

	rtw_read_port_cancel(padapter);
#ifdef CONFIG_USB_INTERRUPT_IN_PIPE
	pHalData->IntrMask[0] = rtw_read32(padapter, REG_HIMR0_8710B);
	RTW_INFO("%s pHalData->IntrMask = 0x%04x\n", __func__, pHalData->IntrMask[0]);
	pHalData->IntrMask[0] = 0x0;
	rtw_write32(padapter, REG_HIMR0_8710B, pHalData->IntrMask[0]);
#endif /*CONFIG_USB_INTERRUPT_IN_PIPE */
	return _SUCCESS;
}

#if 0
static u32
_GetChannelGroup(
	IN u32 channel
)
{
	/*RT_ASSERT((channel < 14), ("Channel %d no is supported!\n")); */

	if (channel < 3)    /* Channel 1~3 */
		return 0;
	else if (channel < 9)   /* Channel 4~9 */
		return 1;

	return 2;               /* Channel 10~14 */
}

/*
 *-------------------------------------------------------------------
 *	EEPROM/EFUSE Content Parsing
 *-------------------------------------------------------------------
 */
static void
hal_EfuseParseLEDSetting(
	IN PADAPTER padapter,
	IN u8 *PROMContent,
	IN BOOLEAN AutoloadFail
)
{
	struct led_priv *pledpriv = &(padapter->ledpriv);
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);

#ifdef CONFIG_RTW_SW_LED
	pledpriv->bRegUseLed = _TRUE;

	/* Led mode */
	switch (pHalData->CustomerID) {
	case RT_CID_DEFAULT:
		pledpriv->LedStrategy = SW_LED_MODE1;
		pledpriv->bRegUseLed = _TRUE;
		break;

	case RT_CID_819x_HP:
		pledpriv->LedStrategy = SW_LED_MODE6;
		break;

	default:
		pledpriv->LedStrategy = SW_LED_MODE1;
		break;
	}

	pHalData->bLedOpenDrain = _TRUE; /* Support Open-drain arrangement for controlling the LED. Added by Roger, 2009.10.16. */
#else /* HW LED */
	pledpriv->LedStrategy = HW_LED;
#endif /*CONFIG_RTW_SW_LED */
}

static void
hal_EfuseParseRFSetting(
	IN PADAPTER padapter,
	IN u8 *PROMContent,
	IN BOOLEAN AutoloadFail
)
{
}

/* Read HW power down mode selection */
static void
hal_EfuseParsePowerSavingSetting(
	IN PADAPTER padapter,
	IN u8 *PROMContent,
	IN u8 AutoloadFail
)
{
	struct pwrctrl_priv *pwrctl = adapter_to_pwrctl(padapter);

	if (AutoloadFail) {
		pwrctl->bHWPowerdown = _FALSE;
		pwrctl->bSupportRemoteWakeup = _FALSE;
	} else {
		/*if(SUPPORT_HW_RADIO_DETECT(padapter)) */
		pwrctl->bHWPwrPindetect = padapter->registrypriv.hwpwrp_detect;
		/*else */
		/*pwrctl->bHWPwrPindetect = _FALSE; */ /*dongle not support new */


		/*hw power down mode selection , 0:rf-off / 1:power down */

		if (padapter->registrypriv.hwpdn_mode == 2)
			pwrctl->bHWPowerdown = (PROMContent[EEPROM_FEATURE_OPTION_8710B] & BIT(4));
		else
			pwrctl->bHWPowerdown = padapter->registrypriv.hwpdn_mode;

		/* decide hw if support remote wakeup function */
		/* if hw supported, 8051 (SIE) will generate WeakUP signal( D+/D- toggle) when autoresume */
		pwrctl->bSupportRemoteWakeup = (PROMContent[EEPROM_USB_OPTIONAL_FUNCTION0] & BIT(1)) ? _TRUE : _FALSE;

		/*if(SUPPORT_HW_RADIO_DETECT(padapter)) */
		/*padapter->registrypriv.usbss_enable = pwrctl->bSupportRemoteWakeup ; */

		RTW_INFO("%s...bHWPwrPindetect(%x)-bHWPowerdown(%x) ,bSupportRemoteWakeup(%x)\n", __func__,
			pwrctl->bHWPwrPindetect, pwrctl->bHWPowerdown, pwrctl->bSupportRemoteWakeup);

		RTW_INFO("### PS params=>  power_mgnt(%x),usbss_enable(%x) ###\n", padapter->registrypriv.power_mgnt, padapter->registrypriv.usbss_enable);

	}
}

static u8
InitpadapterVariablesByPROM_8710bu(
	IN PADAPTER padapter
)
{
	PHAL_DATA_TYPE pHalData = GET_HAL_DATA(padapter);
	u8 *hwinfo = NULL;
	u8 ret = _FAIL;

	if (sizeof(pHalData->efuse_eeprom_data) < HWSET_MAX_SIZE_8710B)
		RTW_INFO("[WARNING] size of efuse_eeprom_data is less than HWSET_MAX_SIZE_8710B!\n");

	hwinfo = pHalData->efuse_eeprom_data;

	Hal_InitPGData(padapter, hwinfo);
	Hal_EfuseParseIDCode(padapter, hwinfo);
	Hal_EfuseParseEEPROMVer_8710B(padapter, hwinfo, pHalData->bautoload_fail_flag);

	hal_EfuseParseIDs(padapter, hwinfo, pHalData->bautoload_fail_flag);
	hal_config_macaddr(padapter, pHalData->bautoload_fail_flag);
	hal_EfuseParsePowerSavingSetting(padapter, hwinfo, pHalData->bautoload_fail_flag);

	Hal_EfuseParseTxPowerInfo_8710B(padapter, hwinfo, pHalData->bautoload_fail_flag);
	Hal_EfuseParseBoardType_8710B(padapter, hwinfo, pHalData->bautoload_fail_flag);

	Hal_EfuseParseBTCoexistInfo_8710B(padapter, hwinfo, pHalData->bautoload_fail_flag);

	Hal_EfuseParseChnlPlan_8710B(padapter, hwinfo, pHalData->bautoload_fail_flag);
	Hal_EfuseParseXtal_8710B(padapter, hwinfo, pHalData->bautoload_fail_flag);
	Hal_EfuseParseThermalMeter_8710B(padapter, hwinfo, pHalData->bautoload_fail_flag);
	Hal_EfuseParseAntennaDiversity_8710B(padapter, hwinfo, pHalData->bautoload_fail_flag);
	Hal_EfuseParseCustomerID_8710B(padapter, hwinfo, pHalData->bautoload_fail_flag);

	hal_EfuseParseLEDSetting(padapter, hwinfo, pHalData->bautoload_fail_flag);

	/* set coex. ant info once efuse parsing is done */
	rtw_btcoex_set_ant_info(padapter);

	/* Hal_EfuseParseKFreeData_8710B(padapter, hwinfo, pHalData->bautoload_fail_flag); */
#ifdef CONFIG_RTW_MAC_HIDDEN_RPT
	if (hal_read_mac_hidden_rpt(padapter) != _SUCCESS)
		goto exit;
#endif

	ret = _SUCCESS;

exit:
	return ret;
}
 
static u8 hal_efuse_parse_prom_content(PADAPTER padapter)
{
	PHAL_DATA_TYPE pHalData = GET_HAL_DATA(padapter);

	u8 eeValue;
	u32 i;
	u16 value16;
	u8 ret = _FAIL;

	eeValue = rtw_read8(padapter, REG_9346CR);
	/* To check system boot selection. */
	pHalData->EepromOrEfuse = (eeValue & BOOT_FROM_EEPROM) ? _TRUE : _FALSE;
	pHalData->bautoload_fail_flag = (eeValue & EEPROM_EN) ? _FALSE : _TRUE;

	RTW_INFO("Boot from %s, Autoload %s !\n", (pHalData->EepromOrEfuse ? "EEPROM" : "EFUSE"),
		 (pHalData->bautoload_fail_flag ? "Fail" : "OK"));

	if (InitpadapterVariablesByPROM_8710bu(padapter) != _SUCCESS)
		goto exit;

	ret = _SUCCESS;

exit:
	return ret;
}



u8 get_eeprom_type_8710b(PADAPTER padapter)
{
	u8 size = 0;
	u32	value32;

	value32 = hal_query_syson_reg_8710b(padapter, REG_SYS_EEPROM_CTRL0, 0xFF);
	/* 6: EEPROM used is 93C46, 4: boot from E-Fuse. */
	size = (value32 & BOOT_FROM_EEPROM) ? 6 : 4;

	RTW_INFO("EEPROM type is %s\n", size == 4 ? "E-FUSE" : "93C46");

	return size;
}

/*
 * Description:
 *    We should set Efuse cell selection to WiFi cell in default.
 * Assumption:
 *    PASSIVE_LEVEL
 */
void hal_efuse_cellselection(PADAPTER padapter)
{
	u32 value32;

	value32 = rtw_read32(padapter, REG_EFUSE_TEST);
	value32 = (value32 & ~EFUSE_SEL_MASK) | EFUSE_SEL(EFUSE_WIFI_SEL_0);
	rtw_write32(padapter, EFUSE_TEST, value32);
}

static u8 read_adapter_info(PADAPTER padapter)
{
	u8 ret = _FAIL;

	/* Read EEPROM size before call any EEPROM function */
	padapter->EepromAddressSize = get_eeprom_type_8710b(padapter);

	/* Efuse_InitSomeVar(padapter); */

	/*hal_efuse_cellselection(padapter); 20171207 remove by Peter*/

	/* We need to define the RF type after all PROM value is recognized. */
	read_rftype(padapter);
	
	if (hal_efuse_parse_prom_content(padapter) != _SUCCESS)
		goto exit;

	ret = _SUCCESS;

exit:
	return ret;
}

void hal_init_pgdata(PADAPTER padapter,u8	*PROMContent)
{
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);
	u32	i;
	u16	value16;

	if (_FALSE == pHalData->bautoload_fail_flag) {
		/* autoload OK.
		*		if (IS_BOOT_FROM_EEPROM(padapter)) */
		if (_TRUE == pHalData->EepromOrEfuse) {
			/* Read all Content from EEPROM or EFUSE. */
			for (i = 0; i < HWSET_MAX_SIZE_8710B; i += 2) {
				/*	value16 = EF2Byte(ReadEEprom(pAdapter, (u2Byte) (i>>1)));
				 *	*((u16*)(&PROMContent[i])) = value16; */
			}
		} else {
			/* Read EFUSE real map to shadow. */
			EFUSE_ShadowMapUpdate(padapter, EFUSE_WIFI, _FALSE);
			_rtw_memcpy((void *)PROMContent, (void *)pHalData->efuse_eeprom_data, HWSET_MAX_SIZE_8710B);
		}
	} else {
		/* autoload fail */
		/*		pHalData->AutoloadFailFlag = _TRUE; */
		/* update to default value 0xFF */
		if (_FALSE == pHalData->EepromOrEfuse)
			EFUSE_ShadowMapUpdate(padapter, EFUSE_WIFI, _FALSE);
		_rtw_memcpy((void *)PROMContent, (void *)pHalData->efuse_eeprom_data, HWSET_MAX_SIZE_8710B);
	}

#ifdef CONFIG_EFUSE_CONFIG_FILE
	if (check_phy_efuse_tx_power_info_valid(padapter) == _FALSE) {
		if (Hal_readPGDataFromConfigFile(padapter) != _SUCCESS)
			RTW_ERR("invalid phy efuse and read from file fail, will use driver default!!\n");
	}
#endif
}
#endif

static void Hal_EfuseParseIDs(PADAPTER padapter, u8 *map, u8 mapvalid)
{
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);

	if (mapvalid) {
		/* VID, PID */
		pHalData->EEPROMVID = ReadLE2Byte(&map[EEPROM_VID_8710BU]);
		pHalData->EEPROMPID = ReadLE2Byte(&map[EEPROM_PID_8710BU]);

		/* Customer ID, 0x00 and 0xff are reserved for Realtek. */
		pHalData->EEPROMCustomerID = *(u8 *)&map[EEPROM_CUSTOM_ID_8710B];
		pHalData->EEPROMSubCustomerID = EEPROM_Default_SubCustomerID;
	} else {
		pHalData->EEPROMVID = EEPROM_Default_VID;
		pHalData->EEPROMPID = EEPROM_Default_PID;

		/* Customer ID, 0x00 and 0xff are reserved for Realtek. */
		pHalData->EEPROMCustomerID = EEPROM_Default_CustomerID;
		pHalData->EEPROMSubCustomerID = EEPROM_Default_SubCustomerID;
	}

	if ((pHalData->EEPROMVID == EEPROM_Default_VID)
	    && (pHalData->EEPROMPID == EEPROM_Default_PID)) {
		pHalData->CustomerID = EEPROM_Default_CustomerID;
		pHalData->EEPROMSubCustomerID = EEPROM_Default_SubCustomerID;
	}

	RTW_INFO("VID = 0x%04X, PID = 0x%04X\n", pHalData->EEPROMVID, pHalData->EEPROMPID);
	RTW_INFO("Customer ID: 0x%02X, SubCustomer ID: 0x%02X\n", pHalData->EEPROMCustomerID, pHalData->EEPROMSubCustomerID);
}

void Hal_EfuseParsePackageType(PADAPTER pAdapter, u8 *map, u8 mapvalid) 
{
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(pAdapter);
	u8 package = 0x7;

	pAdapter->hal_func.efuse_indirect_read4(pAdapter, EEPROM_PACKAGE_TYPE_8710B, &package);
	RTW_INFO("Efuse_indirect_read4 : package: 0x%x\n", package);
	
	switch (package) 
	{
		case 0xFE: //the correct efuse value, definiton 8188GU Dongle Package, Efuse Physical Address 0xF8 = 0xFE //SMIC  QFN48M
			pHalData->PackageType = PACKAGE_QFN48M_S;
			break;
			
		case 0xEE: //UMC  QFN48M
			pHalData->PackageType = PACKAGE_QFN48M_U;
			break;
			
		case 0xFF: //cover the efuse not PG case, revise later
			RTW_DBG("Waring: Efuse Physical Address 0xF8 = 0xFF. Package Type Not PG! Use SYSON 0x1F0!! \n");
			if(IS_CHIP_VENDOR_SMIC(pHalData->version_id)) //SMIC 
				pHalData->PackageType = PACKAGE_QFN48M_S;
			else if(IS_CHIP_VENDOR_UMC(pHalData->version_id)) //UMC 
				pHalData->PackageType = PACKAGE_QFN48M_U;
			else
				RTW_DBG("Error:Package Type Not PG! and  SYSON 0x1F0 is wrong!! \n");
			
			break;

		default:
			pHalData->PackageType = PACKAGE_DEFAULT;
			RTW_DBG("Error: Package Type Error! Efuse Physical Address 0xF8 = 0x%x\n",package);
			break;
	}

	RTW_INFO("PackageType = 0x%X, %s\n", pHalData->PackageType,(pHalData->PackageType == PACKAGE_QFN48M_S)?"QFN48M_S":"QFN48M_U");
}

static void read_rftype(PADAPTER padapter)
{
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);

#if DISABLE_BB_RF
	pHalData->rf_chip = RF_PSEUDO_11N;
#else
	pHalData->rf_chip = RF_6052;
#endif
	pHalData->BandSet = BAND_ON_2_4G;
}

static u8 Hal_EfuseParseIDCode(PADAPTER adapter, u8 *map)
{
	u16 EEPROMId;

	/* Check 0x8129 again for making sure autoload status!! */
	EEPROMId = le16_to_cpu(*(u16 *)map);
	RTW_INFO("EEPROM ID = 0x%04x\n", EEPROMId);
	if (EEPROMId == RTL_EEPROM_ID_8710B)
		return _TRUE;

	RTW_WARN("EEPROM ID is invalid!!\n");
	return _FALSE;
}

static void Hal_EfuseParseEEPROMVer(PADAPTER adapter, u8 *map, u8 mapvalid)
{
	PHAL_DATA_TYPE hal = GET_HAL_DATA(adapter);

	if (mapvalid)
		hal->EEPROMVersion = map[EEPROM_VERSION_8710B];
	else
		hal->EEPROMVersion = EEPROM_Default_Version;

	RTW_INFO("EEPROM Version = %d\n", hal->EEPROMVersion);
}

static void Hal_EfuseParseTxPowerInfo(PADAPTER adapter, u8 *map, u8 mapvalid)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(adapter);
	TxPowerInfo24G	pwrInfo24G;

	hal_load_txpwr_info(adapter, &pwrInfo24G, NULL, map);

	/* 2010/10/19 MH Add Regulator recognize for CU. */
	if (mapvalid) {
		pHalData->EEPROMRegulatory = (map[EEPROM_RF_BOARD_OPTION_8710B] & 0x7);	/* bit0~2 */
		if (map[EEPROM_RF_BOARD_OPTION_8710B] == 0xFF)
			pHalData->EEPROMRegulatory = (EEPROM_DEFAULT_BOARD_OPTION & 0x7);	/* bit0~2 */
	} else
		pHalData->EEPROMRegulatory = 0;
	
	RTW_INFO("EEPROMRegulatory = 0x%x\n", pHalData->EEPROMRegulatory);
}

static void Hal_EfuseParseBoardType(PADAPTER adapter, u8 *map, u8 mapvalid)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(adapter);

	if (mapvalid) {
		pHalData->InterfaceSel = (map[EEPROM_RF_BOARD_OPTION_8710B] & 0xE0) >> 5;
		if (map[EEPROM_RF_BOARD_OPTION_8710B] == 0xFF)
			pHalData->InterfaceSel = (EEPROM_DEFAULT_BOARD_OPTION & 0xE0) >> 5;
	} else
		pHalData->InterfaceSel = 0;

	RTW_INFO("Board Type: 0x%2x\n", pHalData->InterfaceSel);
}

static void Hal_EfuseParseChnlPlan(PADAPTER adapter, u8 *map, u8 autoloadfail)
{
	hal_com_config_channel_plan(
		adapter,
		map ? &map[EEPROM_COUNTRY_CODE_8710B] : NULL,
		map ? map[EEPROM_CHANNEL_PLAN_8710B] : 0xFF,
		adapter->registrypriv.alpha2,
		adapter->registrypriv.channel_plan,
		RTW_CHPLAN_REALTEK_DEFINE,
		autoloadfail
	);
}

static void Hal_EfuseParseXtal(PADAPTER adapter, u8 *map, u8 mapvalid)
{
	PHAL_DATA_TYPE hal = GET_HAL_DATA(adapter);

	if (mapvalid && map[EEPROM_XTAL_8710B] != 0xFF)
		hal->crystal_cap = map[EEPROM_XTAL_8710B];
	else
		hal->crystal_cap = EEPROM_Default_CrystalCap;

	RTW_INFO("EEPROM crystal_cap=0x%02x\n", hal->crystal_cap);
}

static void Hal_EfuseParseThermalMeter(PADAPTER adapter, u8 *map, u8 mapvalid)
{
	PHAL_DATA_TYPE hal = GET_HAL_DATA(adapter);

	/* ThermalMeter from EEPROM */
	if (mapvalid && (map[EEPROM_THERMAL_METER_8710B] != 0xFF))
		hal->eeprom_thermal_meter = map[EEPROM_THERMAL_METER_8710B];
	else {
		hal->eeprom_thermal_meter = EEPROM_Default_ThermalMeter;
		hal->odmpriv.rf_calibrate_info.is_apk_thermal_meter_ignore = _TRUE;
	}

	RTW_INFO("EEPROM ThermalMeter=0x%02x\n", hal->eeprom_thermal_meter);
}

static void Hal_EfuseParseAntennaDiversity(PADAPTER adapter, u8 *map, u8 mapvalid)
{
#ifdef CONFIG_ANTENNA_DIVERSITY
	PHAL_DATA_TYPE hal = GET_HAL_DATA(adapter);
	struct registry_priv *registry_par = &adapter->registrypriv;


	if (hal->EEPROMBluetoothAntNum == Ant_x1)
		hal->AntDivCfg = 0;
	else {
		if (registry_par->antdiv_cfg == 2)/* 0:OFF , 1:ON, 2:By EFUSE */
			hal->AntDivCfg = (map[EEPROM_RF_BOARD_OPTION_8710B] & BIT3) ? _TRUE : _FALSE;
		else
			hal->AntDivCfg = registry_par->antdiv_cfg;
	}
	/*hal->TRxAntDivType = S0S1_TRX_HW_ANTDIV;*/
	hal->with_extenal_ant_switch = ((map[EEPROM_RF_BT_SETTING_8710B] & BIT7) >> 7);

	RTW_INFO("%s:EEPROM AntDivCfg=%d, AntDivType=%d, extenal_ant_switch:%d\n",
		 __func__, hal->AntDivCfg, hal->TRxAntDivType, hal->with_extenal_ant_switch);
#endif /* CONFIG_ANTENNA_DIVERSITY */
}

static void Hal_EfuseTxBBSwing(PADAPTER adapter, u8 *map, u8 mapvalid)
{
	PHAL_DATA_TYPE hal_data = GET_HAL_DATA(adapter);

	if (_TRUE == mapvalid) {
		hal_data->tx_bbswing_24G = map[EEPROM_TX_BBSWING_2G_8710B];
		if (0xFF == hal_data->tx_bbswing_24G)
			hal_data->tx_bbswing_24G = 0;
		hal_data->tx_bbswing_5G = map[EEPROM_TX_BBSWING_5G_8710B];
		if (0xFF == hal_data->tx_bbswing_5G)
			hal_data->tx_bbswing_5G = 0;
	} else {
		hal_data->tx_bbswing_24G = 0;
		hal_data->tx_bbswing_5G = 0;
	}
	RTW_INFO("EEPROM tx_bbswing_24G =0x%02x\n", hal_data->tx_bbswing_24G);
	RTW_INFO("EEPROM tx_bbswing_5G =0x%02x\n", hal_data->tx_bbswing_5G);
}

static void Hal_EfuseParseCustomerID(PADAPTER adapter, u8 *map, u8 mapvalid)
{
	PHAL_DATA_TYPE hal = GET_HAL_DATA(adapter);

	if (mapvalid)
		hal->EEPROMCustomerID = map[EEPROM_CUSTOM_ID_8710B];
	else
		hal->EEPROMCustomerID = 0;
	
	RTW_INFO("EEPROM Customer ID=0x%02x\n", hal->EEPROMCustomerID);
}

static void Hal_DetectWoWMode(PADAPTER adapter)
{
#if defined(CONFIG_WOWLAN) || defined(CONFIG_AP_WOWLAN)
	adapter_to_pwrctl(adapter)->bSupportRemoteWakeup = _TRUE;
#else /* !(CONFIG_WOWLAN || CONFIG_AP_WOWLAN) */
	adapter_to_pwrctl(adapter)->bSupportRemoteWakeup = _FALSE;
#endif /* !(CONFIG_WOWLAN || CONFIG_AP_WOWLAN) */

	RTW_INFO("EEPROM SupportRemoteWakeup=%d\n", adapter_to_pwrctl(adapter)->bSupportRemoteWakeup);
}

static void hal_ReadPAType(PADAPTER adapter, u8 *map, u8 mapvalid)
{
	PHAL_DATA_TYPE hal_data = GET_HAL_DATA(adapter);

	if (mapvalid) {
		/* AUTO - Get INFO from eFuse*/
		if (GetRegAmplifierType2G(adapter) == 0) {
			switch (hal_data->rfe_type) {
			default:
					hal_data->PAType_2G = 0;
					hal_data->LNAType_2G = 0;
					hal_data->ExternalPA_2G = 0;
					hal_data->ExternalLNA_2G = 0;
				break;
			}
		} else {
			hal_data->ExternalPA_2G  = (GetRegAmplifierType2G(adapter) & ODM_BOARD_EXT_PA)  ? 1 : 0;
			hal_data->ExternalLNA_2G = (GetRegAmplifierType2G(adapter) & ODM_BOARD_EXT_LNA) ? 1 : 0;
		}
	} else {
		/*Get INFO from registry*/
		hal_data->ExternalPA_2G  = EEPROM_Default_PAType;
		hal_data->ExternalLNA_2G = EEPROM_Default_LNAType;


		if (GetRegAmplifierType2G(adapter) == 0) {
			hal_data->ExternalPA_2G  = 0;
			hal_data->ExternalLNA_2G = 0;
		} else {
			hal_data->ExternalPA_2G  = (GetRegAmplifierType2G(adapter) & ODM_BOARD_EXT_PA)  ? 1 : 0;
			hal_data->ExternalLNA_2G = (GetRegAmplifierType2G(adapter) & ODM_BOARD_EXT_LNA) ? 1 : 0;
		}
	}

	RTW_INFO("EEPROM PAType_2G is 0x%x, ExternalPA_2G = %d\n", hal_data->PAType_2G, hal_data->ExternalPA_2G);
	RTW_INFO("EEPROM LNAType_2G is 0x%x, ExternalLNA_2G = %d\n", hal_data->LNAType_2G, hal_data->ExternalLNA_2G);
}

#ifdef CONFIG_USB_HCI
static void Hal_ReadUsbModeSwitch(PADAPTER adapter, u8 *map, u8 mapvalid)
{
	PHAL_DATA_TYPE hal = GET_HAL_DATA(adapter);

	if (_TRUE == mapvalid)
		/* check efuse 0x06 bit7 */
		hal->EEPROMUsbSwitch = (map[EEPROM_USB_MODE_8821CU] & BIT7) >> 7;
	else
		hal->EEPROMUsbSwitch = _FALSE;

	RTW_INFO("EEPROM USB Switch=%d\n", hal->EEPROMUsbSwitch);
}
#endif /* CONFIG_USB_HCI */

static void Hal_ReadAmplifierType(PADAPTER adapter, u8 *map, u8 mapvalid)
{
	PHAL_DATA_TYPE hal_data = GET_HAL_DATA(adapter);

	if (hal_data->rfe_type < 8) { /*According to RF-EFUSE DOC : R15]*/
		RTW_INFO("WIFI Module is iPA/iLNA\n");
		return;
	}

	hal_ReadPAType(adapter, map, mapvalid);

	/* [2.4G] extPA */
	hal_data->TypeGPA  = hal_data->PAType_2G;

	/* [2.4G] extLNA */
	hal_data->TypeGLNA = hal_data->LNAType_2G;
	
	RTW_INFO("EEPROM TypeGPA = 0x%X\n", hal_data->TypeGPA);
	RTW_INFO("EEPROM TypeGLNA = 0x%X\n", hal_data->TypeGLNA);
}

/*
 * Description:
 *	Collect all information from efuse or files.
 *	This function will do
 *	1. Read registers to check hardware efuse available or not
 *	2. Read Efuse/EEPROM
 *	3. Read file if necessary
 *	4. Parsing Efuse data
 *
 *  Bautoload_fail_flag is used to present eFuse map is valid or not, 
 *  no matter the map comes from hardware or files.
 *
 */
u8 rtl8710b_read_efuse(PADAPTER padapter)
{
	PHAL_DATA_TYPE hal;
	u8 val8;
	u8 *efuse_map = NULL;
	u8 valid;
	u8 ret = _FAIL;

	hal = GET_HAL_DATA(padapter);
	efuse_map = hal->efuse_eeprom_data;

	/* 1. Read registers to check hardware eFuse available or not */
	val8 = hal_query_syson_reg_8710b(padapter, REG_SYS_EEPROM_CTRL0, 0xF);
	hal->EepromOrEfuse = (val8 & BIT_EERPOMSEL_8710B) ? _TRUE : _FALSE;
	hal->bautoload_fail_flag = (val8 & BIT_AUTOLOAD_SUS_8710B);

	/* 2. Read eFuse */
	EFUSE_ShadowMapUpdate(padapter, EFUSE_WIFI, 0);

	/* 3. Read Efuse file if necessary */
#ifdef CONFIG_EFUSE_CONFIG_FILE
	if (check_phy_efuse_tx_power_info_valid(padapter) == _FALSE) {
		if (Hal_readPGDataFromConfigFile(padapter) != _SUCCESS)
			RTW_WARN("%s: invalid phy efuse and read from file fail, will use driver default!!\n", __FUNCTION__);
	}
#endif /* CONFIG_EFUSE_CONFIG_FILE */

	/* 4. Parse Efuse data */
	valid = Hal_EfuseParseIDCode(padapter, efuse_map);
	if (_TRUE == valid)
		hal->bautoload_fail_flag = _FALSE;
	else
		hal->bautoload_fail_flag = _TRUE;

	Hal_EfuseParseEEPROMVer(padapter, efuse_map, valid);
	Hal_EfuseParseIDs(padapter, efuse_map, valid);
	hal_config_macaddr(padapter, hal->bautoload_fail_flag);
	Hal_EfuseParseTxPowerInfo(padapter, efuse_map, valid);
	Hal_EfuseParseBoardType(padapter, efuse_map, valid);
	
	Hal_EfuseParseChnlPlan(padapter, efuse_map, hal->bautoload_fail_flag);
	Hal_EfuseParseXtal(padapter, efuse_map, valid);
	Hal_EfuseParsePackageType(padapter, efuse_map, valid);
	Hal_EfuseParseThermalMeter(padapter, efuse_map, valid);
	Hal_EfuseParseAntennaDiversity(padapter, efuse_map, valid);
	Hal_EfuseParseCustomerID(padapter, efuse_map, valid);
	Hal_ReadAmplifierType(padapter, efuse_map, valid);
	
#ifdef CONFIG_WOWLAN
	Hal_DetectWoWMode(padapter);
#endif

#ifdef CONFIG_USB_HCI
	//Hal_ReadUsbModeSwitch(padapter, efuse_map, valid);
#endif /* CONFIG_USB_HCI */

#if 0
#ifdef CONFIG_RTW_MAC_HIDDEN_RPT
	hal_read_mac_hidden_rpt(padapter);
	{
		struct hal_spec_t *hal_spec = GET_HAL_SPEC(padapter);

		if (hal_spec->hci_type <= 3 && hal_spec->hci_type >= 1) {
			hal->EEPROMBluetoothCoexist = _FALSE;
			RTW_INFO("EEPROM Disable BT-coex by hal_spec\n");
			rtw_btcoex_wifionly_AntInfoSetting(padapter);
		}
	}
#endif
#endif

	ret = _SUCCESS;

exit:
	return ret;
}

/*
 * Description:
 *	Collect all hardware information, fill "HAL_DATA_TYPE".
 *	Sometimes this would be used to read MAC address.
 *	This function will do
 *	1. Read Efuse/EEPROM to initialize
 *	2. Read registers to initialize
 *	3. Other vaiables initialization
 */
static u8 read_adapter_info(PADAPTER padapter)
{
	u8 ret = _FAIL;

	/*
	 * 1. Read Efuse/EEPROM to initialize
	 */
	if (rtl8710b_read_efuse(padapter) != _SUCCESS)
		goto exit;

	/*
	 * 2. Read registers to initialize
	 */

	/*
	 * 3. Other Initialization
	 */
	read_rftype(padapter);
	
	ret = _SUCCESS;

exit:
	return ret;
}

#define GPIO_DEBUG_PORT_NUM 0
static void rtl8710bu_trigger_gpio_0(PADAPTER padapter)
{

	u32 gpioctrl;

	RTW_INFO("==> trigger_gpio_0...\n");
	rtw_write16_async(padapter, REG_GPIO_PIN_CTRL, 0);
	rtw_write8_async(padapter, REG_GPIO_PIN_CTRL + 2, 0xFF);
	gpioctrl = (BIT(GPIO_DEBUG_PORT_NUM) << 24) | (BIT(GPIO_DEBUG_PORT_NUM) << 16);
	rtw_write32_async(padapter, REG_GPIO_PIN_CTRL, gpioctrl);
	gpioctrl |= (BIT(GPIO_DEBUG_PORT_NUM) << 8);
	rtw_write32_async(padapter, REG_GPIO_PIN_CTRL, gpioctrl);
	RTW_INFO("<=== trigger_gpio_0...\n");

}

/*
 * If variable not handled here,
 * some variables will be processed in SetHwReg8723A()
 */
u8 SetHwReg8710bu(PADAPTER padapter, u8 variable, u8 *val)
{
	PHAL_DATA_TYPE pHalData = GET_HAL_DATA(padapter);
	u8 ret = _SUCCESS;

	switch (variable) {
	case HW_VAR_RXDMA_AGG_PG_TH:
#ifdef CONFIG_USB_RX_AGGREGATION
		{
			u8 threshold = *val;

			if (threshold == 0)
				threshold = pHalData->rxagg_dma_size;
			ret = SetHwReg8710B(padapter, HW_VAR_RXDMA_AGG_PG_TH, &threshold);
		}
#endif
		break;

	case HW_VAR_SET_RPWM:
		rtw_write8(padapter, REG_USB_HRPWM, *val);
		break;

	case HW_VAR_TRIGGER_GPIO_0:
		rtl8710bu_trigger_gpio_0(padapter);
		break;
#ifdef CONFIG_GPIO_WAKEUP
	case HW_SET_GPIO_WL_CTRL: {

		//SIC enable is controled by CM4 os
		#if 0
		u8 enable = *val;
		u8 value = 0;

		if (WAKEUP_GPIO_IDX != 6)
			break;

		value = rtw_read8(padapter, REG_GPIO_MUXCFG);

		if (enable && (value & BIT(3))) {
			value &= ~BIT(3);
			rtw_write8(padapter, REG_GPIO_MUXCFG, value);
		} else if (enable == _FALSE) {
			RTW_INFO("%s: keep WLAN ctrl\n", __func__);
		}
		/*0x66 bit4*/
		value = rtw_read8(padapter, REG_PAD_CTRL_1 + 2);
		if (enable && (value & BIT(4))) {
			value &= ~BIT(4);
			rtw_write8(padapter, REG_PAD_CTRL_1 + 2, value);
		} else if (enable == _FALSE) {
			value |= BIT(4);
			rtw_write8(padapter, REG_PAD_CTRL_1 + 2, value);
		}

		/*0x66 bit8*/
		value = rtw_read8(padapter, REG_PAD_CTRL_1 + 3);
		if (enable && (value & BIT(0))) {
			value &= ~BIT(0);
			rtw_write8(padapter, REG_PAD_CTRL_1 + 3, value);
		} else if (enable == _FALSE) {
			value |= BIT(0);
			rtw_write8(padapter, REG_PAD_CTRL_1 + 3, value);
		}

		RTW_INFO("%s: HW_SET_GPIO_WL_CTRL\n", __func__);
		#endif
	}
		break;
#endif
	default:
		ret = SetHwReg8710B(padapter, variable, val);
		break;
	}

	return ret;
}

/*
 * If variable not handled here,
 * some variables will be processed in GetHwReg8723A()
 */
void GetHwReg8710bu(PADAPTER padapter, u8 variable, u8 *val)
{
	PHAL_DATA_TYPE pHalData = GET_HAL_DATA(padapter);


	switch (variable) {
	default:
		GetHwReg8710B(padapter, variable, val);
		break;
	}

}

/*
 * Description:
 * Query setting of specified variable.
 */
u8 GetHalDefVar8710bu(PADAPTER padapter, HAL_DEF_VARIABLE eVariable, PVOID pValue)
{
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);
	u8 bResult = _SUCCESS;

	switch (eVariable) {
	case HAL_DEF_IS_SUPPORT_ANT_DIV:
#ifdef CONFIG_ANTENNA_DIVERSITY
		*((u8 *)pValue) = _FALSE;
#endif
		break;

	case HAL_DEF_DRVINFO_SZ:
		*((u32 *)pValue) = DRVINFO_SZ;
		break;
	case HAL_DEF_MAX_RECVBUF_SZ:
		*((u32 *)pValue) = MAX_RECVBUF_SZ;
		break;
	case HAL_DEF_RX_PACKET_OFFSET:
		*((u32 *)pValue) = RXDESC_SIZE + DRVINFO_SZ * 8;
		break;
	case HW_VAR_MAX_RX_AMPDU_FACTOR:
		*((HT_CAP_AMPDU_FACTOR *)pValue) = MAX_AMPDU_FACTOR_64K;
		break;
	default:
		bResult = GetHalDefVar8710B(padapter, eVariable, pValue);
		break;
	}

	return bResult;
}

/*
 * Description:
 * Change default setting of specified variable.
 */
u8 SetHalDefVar8710bu(PADAPTER padapter, HAL_DEF_VARIABLE eVariable, PVOID pValue)
{
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);
	u8 bResult = _SUCCESS;

	switch (eVariable) {
	default:
		bResult = SetHalDefVar8710B(padapter, eVariable, pValue);
		break;
	}

	return bResult;
}

void _update_response_rate(PADAPTER padapter, unsigned int mask)
{
	u8 RateIndex = 0;
	/* Set RRSR rate table. */
	rtw_write8(padapter, REG_RRSR, mask & 0xff);
	rtw_write8(padapter, REG_RRSR + 1, (mask >> 8) & 0xff);

	/* Set RTS initial rate */
	while (mask > 0x1) {
		mask = (mask >> 1);
		RateIndex++;
	}
	rtw_write8(padapter, REG_INIRTS_RATE_SEL, RateIndex);
}

static void rtl8710bu_init_default_value(PADAPTER padapter)
{
	rtl8710b_init_default_value(padapter);
}

static u8 rtl8710bu_ps_func(PADAPTER padapter, HAL_INTF_PS_FUNC efunc_id, u8 *val)
{
	u8 bResult = _TRUE;

	switch (efunc_id) {

#if defined(CONFIG_AUTOSUSPEND) && defined(SUPPORT_HW_RFOFF_DETECTED)
	case HAL_USB_SELECT_SUSPEND: {
		u8 bfwpoll = *((u8 *)val);

		rtl8710b_set_FwSelectSuspend_cmd(padapter, bfwpoll, 500); /*note fw to support hw power down ping detect */
	}
	break;
#endif /*CONFIG_AUTOSUSPEND && SUPPORT_HW_RFOFF_DETECTED */

	default:
		break;
	}
	return bResult;
}

#ifdef CONFIG_SUPPORT_USB_INT
extern void rtl8710bu_interrupt_handler(_adapter *padapter, u16 pkt_len, u8 *pbuf);
#endif

void rtl8710bu_set_hal_ops(PADAPTER padapter)
{
	struct hal_ops *pHalFunc = &padapter->hal_func;


	rtl8710b_set_hal_ops(pHalFunc);

	pHalFunc->hal_power_on = &_InitPowerOn_8710bu;
	pHalFunc->hal_power_off = &card_disable_8710bu;

	pHalFunc->hal_init = &rtl8710bu_hal_init;
	pHalFunc->hal_deinit = &rtl8710bu_hal_deinit;

	pHalFunc->inirp_init = &rtl8710bu_inirp_init;
	pHalFunc->inirp_deinit = &rtl8710bu_inirp_deinit;

	pHalFunc->init_xmit_priv = &rtl8710bu_init_xmit_priv;
	pHalFunc->free_xmit_priv = &rtl8710bu_free_xmit_priv;

	pHalFunc->init_recv_priv = &rtl8710bu_init_recv_priv;
	pHalFunc->free_recv_priv = &rtl8710bu_free_recv_priv;

#ifdef CONFIG_RTW_SW_LED
	pHalFunc->InitSwLeds = &rtl8710bu_InitSwLeds;
	pHalFunc->DeInitSwLeds = &rtl8710bu_DeInitSwLeds;
#endif/*CONFIG_RTW_SW_LED */

	pHalFunc->init_default_value = &rtl8710b_init_default_value;
	pHalFunc->intf_chip_configure = &rtl8710bu_interface_configure;
	pHalFunc->read_adapter_info = &read_adapter_info;

	pHalFunc->set_hw_reg_handler = &SetHwReg8710bu;
	pHalFunc->GetHwRegHandler = &GetHwReg8710bu;
	pHalFunc->get_hal_def_var_handler = &GetHalDefVar8710bu;
	pHalFunc->SetHalDefVarHandler = &SetHalDefVar8710bu;

	pHalFunc->hal_xmit = &rtl8710bu_hal_xmit;
	pHalFunc->mgnt_xmit = &rtl8710bu_mgnt_xmit;
	pHalFunc->hal_xmitframe_enqueue = &rtl8710bu_hal_xmitframe_enqueue;

#ifdef CONFIG_HOSTAPD_MLME
	pHalFunc->hostap_mgnt_xmit_entry = &rtl8710bu_hostap_mgnt_xmit_entry;
#endif
	pHalFunc->interface_ps_func = &rtl8710bu_ps_func;

#ifdef CONFIG_XMIT_THREAD_MODE
	pHalFunc->xmit_thread_handler = &rtl8710bu_xmit_buf_handler;
#endif
#ifdef CONFIG_SUPPORT_USB_INT
	pHalFunc->interrupt_handler = &rtl8710bu_interrupt_handler;
#endif

}
