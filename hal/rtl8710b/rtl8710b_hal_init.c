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
#define _HAL_INIT_C_

#include <rtl8710b_hal.h>
#include "hal_com_h2c.h"
#include <hal_com.h>
#include "hal8710b_fw.h"

#ifndef CONFIG_DLFW_TXPKT
#define DL_FW_MAX 5
#else
#define FW_DOWNLOAD_SIZE_8710B 8192
#endif

#ifdef CONFIG_FILE_FWIMG
	u8 FwBuffer[FW_8710B_SIZE];
#endif

static VOID _FWDownloadEnable(PADAPTER padapter, BOOLEAN enable)
{
	u8 tmp;

	if (enable) {
		// MCU firmware download enable.
		rtw_write8(padapter, REG_8051FW_CTRL_V1_8710B | PAGE0_OFFSET, 0x05);	
		tmp = rtw_read8(padapter, (REG_8051FW_CTRL_V1_8710B + 3) | PAGE0_OFFSET);
		tmp |= BIT0;
		rtw_write8(padapter, (REG_8051FW_CTRL_V1_8710B+3) | PAGE0_OFFSET, tmp);	
		
		// Clear Rom DL enable
		tmp = rtw_read8(padapter, (REG_8051FW_CTRL_V1_8710B+2) | PAGE0_OFFSET);
		rtw_write8(padapter, (REG_8051FW_CTRL_V1_8710B+2) | PAGE0_OFFSET, tmp&0xf7);
	} else {
		// MCU firmware download enable.
		tmp = rtw_read8(padapter, REG_8051FW_CTRL_V1_8710B | PAGE0_OFFSET);
		rtw_write8(padapter, REG_8051FW_CTRL_V1_8710B | PAGE0_OFFSET, tmp&0xfe);
	}
}

static int _BlockWrite(PADAPTER padapter, PVOID buffer, u32 buffSize)
{
	int ret = _SUCCESS;

	u32			blockSize_p1 = 4;	/* (Default) Phase #1 : PCI muse use 4-byte write to download FW */
	u32			blockSize_p2 = 8;	/* Phase #2 : Use 8-byte, if Phase#1 use big size to write FW. */
	u32			blockSize_p3 = 1;	/* Phase #3 : Use 1-byte, the remnant of FW image. */
	u32			blockCount_p1 = 0, blockCount_p2 = 0, blockCount_p3 = 0;
	u32			remainSize_p1 = 0, remainSize_p2 = 0;
	u8			*bufferPtr	= (u8 *)buffer;
	u32			i = 0, offset = 0;
#if 0
#ifdef CONFIG_USB_HCI
	blockSize_p1 = 254;
#endif
#endif
	/* 3 Phase #1 */
	blockCount_p1 = buffSize / blockSize_p1;
	remainSize_p1 = buffSize % blockSize_p1;

	for (i = 0; i < blockCount_p1; i++) {
#if 0
#ifdef CONFIG_USB_HCI
		ret = rtw_writeN(padapter, (FW_8710B_START_ADDRESS + i * blockSize_p1), blockSize_p1, (bufferPtr + i * blockSize_p1));
#else
		ret = rtw_write32(padapter, (FW_8710B_START_ADDRESS + i * blockSize_p1), le32_to_cpu(*((u32 *)(bufferPtr + i * blockSize_p1))));
#endif
#endif
		ret = rtw_write32(padapter, (FW_8710B_START_ADDRESS + i * blockSize_p1), le32_to_cpu(*((u32 *)(bufferPtr + i * blockSize_p1))));
		if (ret == _FAIL) {
			RTW_ERR("====>%s %d i:%d\n", __func__, __LINE__, i);
			goto exit;
		}
	}

	/* 3 Phase #2 */
	if (remainSize_p1) {
		offset = blockCount_p1 * blockSize_p1;

		blockCount_p2 = remainSize_p1 / blockSize_p2;
		remainSize_p2 = remainSize_p1 % blockSize_p2;
#if 0
#ifdef CONFIG_USB_HCI
		for (i = 0; i < blockCount_p2; i++) {
			ret = rtw_writeN(padapter, (FW_8710B_START_ADDRESS + offset + i * blockSize_p2), blockSize_p2, (bufferPtr + offset + i * blockSize_p2));

			if (ret == _FAIL)
				goto exit;
		}
#endif
#endif
	}

	/* 3 Phase #3 */
	if (remainSize_p2) {

		offset = (blockCount_p1 * blockSize_p1) + (blockCount_p2 * blockSize_p2);
		blockCount_p3 = remainSize_p2 / blockSize_p3;

		for (i = 0 ; i < blockCount_p3 ; i++) {
			ret = rtw_write8(padapter, (FW_8710B_START_ADDRESS + offset + i), *(bufferPtr + offset + i));

			if (ret == _FAIL) {
				RTW_ERR("====>%s %d i:%d\n", __func__, __LINE__, i);
				goto exit;
			}
		}
	}

exit:
	return ret;
}

static int _PageWrite(PADAPTER	padapter, u32 page, PVOID buffer, u32 size)
{
	u8 value8;
	u8 u8Page = (u8)(page & 0x07);

	value8 = (rtw_read8(padapter, REG_8051FW_CTRL_V1_8710B + 2) & 0xF8) | u8Page;
	rtw_write8(padapter, REG_8051FW_CTRL_V1_8710B + 2, value8);

	return _BlockWrite(padapter, buffer, size);
}

static int _WriteFW(PADAPTER padapter, PVOID buffer, u32 size)
{
	/* Since we need dynamic decide method of dwonload fw, so we call this function to get chip version. */
	int ret = _SUCCESS;
	u32 pageNums, remainSize;
	u32 page, offset;
	u8 *bufferPtr = (u8 *)buffer;

	pageNums = size / MAX_DLFW_PAGE_SIZE;
	/* RT_ASSERT((pageNums <= 4), ("Page numbers should not greater then 4\n")); */
	remainSize = size % MAX_DLFW_PAGE_SIZE;

	for (page = 0; page < pageNums; page++) {
		offset = page * MAX_DLFW_PAGE_SIZE;
		ret = _PageWrite(padapter, page, bufferPtr + offset, MAX_DLFW_PAGE_SIZE);

		if (ret == _FAIL) {
			RTW_ERR("====>%s %d\n", __func__, __LINE__);
			goto exit;
		}
	}
	if (remainSize) {
		offset = pageNums * MAX_DLFW_PAGE_SIZE;
		page = pageNums;
		ret = _PageWrite(padapter, page, bufferPtr + offset, remainSize);

		if (ret == _FAIL) {
			RTW_ERR("====>%s %d\n", __func__, __LINE__);
			goto exit;
		}
	}

exit:
	return ret;
}

void _8051Reset8710(PADAPTER padapter)
{
	u8 value8;
	
	value8 = rtw_read8(padapter, (REG_8051FW_CTRL_V1_8710B+3) | PAGE0_OFFSET);
	rtw_write8(padapter,  (REG_8051FW_CTRL_V1_8710B+3) | PAGE0_OFFSET, value8&(~BIT0));

	rtw_udelay_os(50);

	value8 = rtw_read8(padapter,  (REG_8051FW_CTRL_V1_8710B+3) | PAGE0_OFFSET);
	rtw_write8(padapter,  (REG_8051FW_CTRL_V1_8710B+3) | PAGE0_OFFSET, value8|(BIT0));
}

static s32 polling_fwdl_chksum(_adapter *adapter, u32 min_cnt, u32 timeout_ms)
{
	s32 ret = _FAIL;
	u32 value32;
	systime start = rtw_get_current_time();
	u32 cnt = 0;

	/* polling CheckSum report */
	do {
		cnt++;
		value32 = rtw_read32(adapter, REG_8051FW_CTRL_V1_8710B);
		if (value32 & FWDL_ChkSum_rpt || RTW_CANNOT_IO(adapter))
			break;
		rtw_yield_os();
	} while (rtw_get_passing_time_ms(start) < timeout_ms || cnt < min_cnt);

	if (!(value32 & FWDL_ChkSum_rpt))
		goto exit;

	if (rtw_fwdl_test_trigger_chksum_fail())
		goto exit;

	ret = _SUCCESS;

exit:
	RTW_INFO("%s: Checksum report %s! (%u, %dms), REG_MCUFWDL:0x%08x\n", __FUNCTION__
		, (ret == _SUCCESS) ? "OK" : "Fail", cnt, rtw_get_passing_time_ms(start), value32);

	return ret;
}

#ifdef PLATFORM_WINDOWS
static s32 _FWFreeToGo(_adapter *adapter, u32 min_cnt, u32 timeout_ms)
{
	s32 ret = _FAIL;
	u32 cnt = 0;
	u32	value32;
	systime start = rtw_get_current_time();
	u32 value_to_check = 0;
	u32 value_expected = (MCUFWDL_RDY | FWDL_ChkSum_rpt | WINTINI_RDY | RAM_DL_SEL);

	value32 = rtw_read32(adapter, REG_8051FW_CTRL_V1_8710B);
	value32 |= MCUFWDL_RDY;
	value32 &= ~WINTINI_RDY;
	rtw_write32(adapter, REG_8051FW_CTRL_V1_8710B, value32);

	_8051Reset8710(adapter);

	/*  polling for FW ready */
	do {
		cnt++;
		value32 = rtw_read32(adapter, REG_8051FW_CTRL_V1_8710B);
		value_to_check = value32 & value_expected;
		if ((value_to_check == value_expected) || RTW_CANNOT_IO(adapter))
			break;
		rtw_yield_os();
	} while (rtw_get_passing_time_ms(start) < timeout_ms || cnt < min_cnt * 100);

	if (value_to_check != value_expected)
		goto exit;

	if (rtw_fwdl_test_trigger_wintint_rdy_fail())
		goto exit;

	ret = _SUCCESS;

exit:
	RTW_INFO("%s: Polling FW ready %s! (%u, %dms), REG_MCUFWDL:0x%08x\n", __FUNCTION__
		, (ret == _SUCCESS) ? "OK" : "Fail", cnt, rtw_get_passing_time_ms(start), value32);

	return ret;
}
#endif

void rtl8710b_InitializeFirmwareVars(PADAPTER padapter)
{
	PHAL_DATA_TYPE pHalData = GET_HAL_DATA(padapter);

	/* Init Fw LPS related. */
	adapter_to_pwrctl(padapter)->bFwCurrentInPSMode = _FALSE;

	/* Init H2C cmd. */
	rtw_write8(padapter, REG_HMETFR, 0x0f);

	/* Init H2C counter. by tynli. 2009.12.09. */
	pHalData->LastHMEBoxNum = 0;
	/*	pHalData->H2CQueueHead = 0;
	 *	pHalData->H2CQueueTail = 0;
	 *	pHalData->H2CStopInsertQueue = _FALSE; */
}

static void dump_fw_page(PADAPTER padapter)
{	
	int index = 0, i = 1;
	RTW_ERR("Dump FW page 0x1000 ~ 0x10FF :\n");
	for (index = 0x1000 ; index < 0x10ff ; index ++) {
		printk("0x%02x, ", rtw_read8(padapter, index));
		if(i % 8 == 0)
			printk("\n");
		i++;	
	}
	printk("\n");
}
	
/*
 *	Description:
 *		Download 8192C firmware code.
 *
 *   */
s32 rtl8710b_FirmwareDownload(PADAPTER padapter, BOOLEAN  bUsedWoWLANFw)
{
	s32	rtStatus = _SUCCESS;
	u8 write_fw = 0;
	systime fwdl_start_time;
	PHAL_DATA_TYPE	pHalData = GET_HAL_DATA(padapter);
	u8			*FwImage;
	u32			FwImageLen;
	u8			*pFwImageFileName;
#ifdef CONFIG_WOWLAN
	u8			*FwImageWoWLAN;
	u32			FwImageWoWLANLen;
#endif
	u8			*pucMappedFile = NULL;
	PRT_FIRMWARE_8710B	pFirmware = NULL;
	PRT_8710B_FIRMWARE_HDR		pFwHdr = NULL;
	u8			*pFirmwareBuf;
	u32			FirmwareLen;
#ifdef CONFIG_FILE_FWIMG
	u8 *fwfilepath;
#endif /* CONFIG_FILE_FWIMG */
	u8			value8;
	u16			value16;
	u32			value32;
	u8			dma_iram_sel;
	u16		new_chk_sum = 0;
	u32		send_pkt_size, pkt_size_tmp;
	u32		mem_offset;
	u32			counter;
	struct dvobj_priv *psdpriv = padapter->dvobj;
	struct debug_priv *pdbgpriv = &psdpriv->drv_dbg;
	struct pwrctrl_priv *pwrpriv = adapter_to_pwrctl(padapter);


	pFirmware = (PRT_FIRMWARE_8710B)rtw_zmalloc(sizeof(RT_FIRMWARE_8710B));

	if (!pFirmware) {
		rtStatus = _FAIL;
		goto exit;
	}
#if 0
	{
		u8 tmp_ps = 0, tmp_rf = 0;

		tmp_ps = rtw_read8(padapter, 0xa3);
		tmp_ps &= 0xf8;
		tmp_ps |= 0x02;
		/* 1. write 0xA3[:2:0] = 3b'010 */
		rtw_write8(padapter, 0xa3, tmp_ps);
		/* 2. read power_state = 0xA0[1:0] */
		tmp_ps = rtw_read8(padapter, 0xa0);
		tmp_ps &= 0x03;
		if (tmp_ps != 0x01) {
			RTW_INFO(FUNC_ADPT_FMT" tmp_ps=%x\n",
				 FUNC_ADPT_ARG(padapter), tmp_ps);
			pdbgpriv->dbg_downloadfw_pwr_state_cnt++;
		}
	}
#endif

#ifdef CONFIG_FILE_FWIMG
#ifdef CONFIG_WOWLAN
	if (bUsedWoWLANFw)
		fwfilepath = rtw_fw_wow_file_path;
	else
#endif /* CONFIG_WOWLAN */
	{
		fwfilepath = rtw_fw_file_path;
	}
#endif /* CONFIG_FILE_FWIMG */

#ifdef CONFIG_FILE_FWIMG
	if (rtw_is_file_readable(fwfilepath) == _TRUE) {
		RTW_INFO("%s acquire FW from file:%s\n", __FUNCTION__, fwfilepath);
		pFirmware->eFWSource = FW_SOURCE_IMG_FILE;
	} else
#endif /* CONFIG_FILE_FWIMG */
	{
#ifdef CONFIG_EMBEDDED_FWIMG
		pFirmware->eFWSource = FW_SOURCE_HEADER_FILE;
#else /* !CONFIG_EMBEDDED_FWIMG */
		pFirmware->eFWSource = FW_SOURCE_IMG_FILE; /* We should decided by Reg. */
#endif /* !CONFIG_EMBEDDED_FWIMG */
	}

	switch (pFirmware->eFWSource) {
	case FW_SOURCE_IMG_FILE:
#ifdef CONFIG_FILE_FWIMG
		rtStatus = rtw_retrieve_from_file(fwfilepath, FwBuffer, FW_8710B_SIZE);
		pFirmware->ulFwLength = rtStatus >= 0 ? rtStatus : 0;
		pFirmware->szFwBuffer = FwBuffer;
#endif /* CONFIG_FILE_FWIMG */
		break;

	case FW_SOURCE_HEADER_FILE:
		if (bUsedWoWLANFw) {
	#ifdef CONFIG_WOWLAN
			if (pwrpriv->wowlan_mode) {
				pFirmware->szFwBuffer = array_mp_8710b_fw_wowlan;
				pFirmware->ulFwLength = array_length_mp_8710b_fw_wowlan;
				RTW_INFO(" ===> %s fw: %s, size: %d\n",
					 __FUNCTION__, "WoWLAN", pFirmware->ulFwLength);
			}
	#endif /* CONFIG_WOWLAN */

	#ifdef CONFIG_AP_WOWLAN
			if (pwrpriv->wowlan_ap_mode) {
				pFirmware->szFwBuffer = array_mp_8710b_fw_ap;
				pFirmware->ulFwLength = array_length_mp_8710b_fw_ap;
				RTW_INFO(" ===> %s fw: %s, size: %d\n",
					 __FUNCTION__, "AP_WoWLAN", pFirmware->ulFwLength);
			}
	#endif /* CONFIG_AP_WOWLAN */
		} else {
			pFirmware->szFwBuffer = array_mp_8710b_fw_nic;
			pFirmware->ulFwLength = array_length_mp_8710b_fw_nic;
			RTW_INFO("%s fw: %s, size: %d\n", __FUNCTION__, "FW_NIC", pFirmware->ulFwLength);
		}
		break;
	}

	if ((pFirmware->ulFwLength - 32) > FW_8710B_SIZE) {
		rtStatus = _FAIL;
		RTW_ERR("Firmware size:%u exceed %u\n",
			pFirmware->ulFwLength, FW_8710B_SIZE);
		goto exit;
	}

	pFirmwareBuf = pFirmware->szFwBuffer;
	FirmwareLen = pFirmware->ulFwLength;

	/* To Check Fw header. Added by tynli. 2009.12.04. */
	pFwHdr = (PRT_8710B_FIRMWARE_HDR)pFirmwareBuf;

	pHalData->firmware_version =  le16_to_cpu(pFwHdr->Version);
	pHalData->firmware_sub_version = le16_to_cpu(pFwHdr->Subversion);
	pHalData->FirmwareSignature = le16_to_cpu(pFwHdr->Signature);

	RTW_INFO("%s: fw_ver=%x fw_subver=%04x sig=0x%x, Month=%02x, Date=%02x, Hour=%02x, Minute=%02x\n",
		 __func__, pHalData->firmware_version,
		 pHalData->firmware_sub_version, pHalData->FirmwareSignature
		 , pFwHdr->Month, pFwHdr->Date, pFwHdr->Hour, pFwHdr->Minute);

	if ((pHalData->version_id.VendorType == CHIP_VENDOR_UMC
		&& pHalData->FirmwareSignature == 0x10B1) ||
		(pHalData->version_id.VendorType == CHIP_VENDOR_SMIC
		&& pHalData->FirmwareSignature == 0x10B1) ) {
		RTW_INFO("%s: Shift for fw header!\n", __FUNCTION__);
		/* Shift 32 bytes for FW header */
		pFirmwareBuf = pFirmwareBuf + 32;
		FirmwareLen = FirmwareLen - 32;
	} else {
		RTW_INFO("%s: Error! No shift for fw header! %04x\n", __FUNCTION__, pHalData->FirmwareSignature);
	}
#if 0
	if (IS_FW_HEADER_EXIST_8710B(pFwHdr, pHalData->version_id)) {
		RTW_INFO("%s(): Shift for fw header!\n", __FUNCTION__);
		/* Shift 32 bytes for FW header */
		pFirmwareBuf = pFirmwareBuf + 32;
		FirmwareLen = FirmwareLen - 32;
	}
#endif

	fwdl_start_time = rtw_get_current_time();

	/* To check if FW already exists before download FW */
	if (rtw_read8(padapter, REG_8051FW_CTRL_V1_8710B | PAGE0_OFFSET) & RAM_DL_SEL) {
		RTW_INFO("%s: FW exists before download FW\n", __func__);
		rtw_write8(padapter, REG_8051FW_CTRL_V1_8710B | PAGE0_OFFSET, 0x00);
		_8051Reset8710(padapter);
	}

#ifndef CONFIG_DLFW_TXPKT

	RTW_INFO("%s by IO write!\n", __func__);

	_FWDownloadEnable(padapter, _TRUE);

	/* We must set 0x90[8]=1 before download FW. Recommented by Ben, 20171221 */
	value8 = rtw_read8(padapter, REG_8051FW_CTRL_V1_8710B+1);
	value8 |= BIT(0);
	rtw_write8(padapter, REG_8051FW_CTRL_V1_8710B+1, value8);
	
	while (!RTW_CANNOT_IO(padapter) &&
	       (write_fw++ < DL_FW_MAX ||
		rtw_get_passing_time_ms(fwdl_start_time) < 500)) {
		/* reset FWDL chksum */
		rtw_write8(padapter, REG_8051FW_CTRL_V1_8710B,
			   rtw_read8(padapter, REG_8051FW_CTRL_V1_8710B) | FWDL_ChkSum_rpt);

		rtStatus = _WriteFW(padapter, pFirmwareBuf, FirmwareLen);
		if (rtStatus != _SUCCESS)
			continue;

		rtStatus = polling_fwdl_chksum(padapter, 2, 10);
		if (rtStatus == _SUCCESS) {
			RTW_INFO("%s: download FW count:%d\n", __func__, write_fw);
			break;
		} else {
			rtw_mdelay_os(10);
		}
	}
#else
#if 0
	RTW_INFO("%s by Tx pkt write!\n", __func__);

	if ((rtw_read8(padapter, REG_MCUFWDL) & MCUFWDL_RDY) == 0) {
		/*
		 * SDIO DMA condition:
		 * all queue must be 256 (0x100 = 0x20 + 0xE0)
		*/

		value32 = 0x802000E0;
		rtw_write32(padapter, REG_RQPN, value32);

		/* Set beacon boundary to TXFIFO header */
		rtw_write8(padapter, REG_BCNQ_BDNY, 0);
		rtw_write16(padapter, REG_DWBCN0_CTRL_8710B + 1, BIT(8));

		/* SDIO need read this register before send packet */
		rtw_read32(padapter, 0x10250020);

		_FWDownloadEnable(padapter, _TRUE);

		/* Get original check sum */
		new_chk_sum = *(pFirmwareBuf + FirmwareLen - 2) |
			      ((u16)*(pFirmwareBuf + FirmwareLen - 1) << 8);

		/* Send ram code flow */
		dma_iram_sel = 0;
		mem_offset = 0;
		pkt_size_tmp = FirmwareLen;
		while (0 != pkt_size_tmp) {
			if (pkt_size_tmp >= FW_DOWNLOAD_SIZE_8710B) {
				send_pkt_size = FW_DOWNLOAD_SIZE_8710B;
				/* Modify check sum value */
				new_chk_sum = (u16)(new_chk_sum ^ (((send_pkt_size - 1) << 2) - TXDESC_SIZE));
			} else {
				send_pkt_size = pkt_size_tmp;
				new_chk_sum = (u16)(new_chk_sum ^ ((send_pkt_size << 2) - TXDESC_SIZE));

			}

			if (send_pkt_size == pkt_size_tmp) {
				/* last partition packet, write new check sum to ram code file */
				*(pFirmwareBuf + FirmwareLen - 2) = new_chk_sum & 0xFF;
				*(pFirmwareBuf + FirmwareLen - 1) = (new_chk_sum & 0xFF00) >> 8;
			}

			/* IRAM select */
			rtw_write8(padapter, REG_MCUFWDL + 1,
				(rtw_read8(padapter, REG_MCUFWDL + 1) & 0x3F) | (dma_iram_sel << 6));
			/* Enable DMA */
			rtw_write8(padapter, REG_MCUFWDL + 1,
				rtw_read8(padapter, REG_MCUFWDL + 1) | BIT(5));

			if (_FALSE == send_fw_packet(padapter, pFirmwareBuf + mem_offset, send_pkt_size)) {
				RTW_INFO("%s: Send FW fail !\n", __FUNCTION__);
				rtStatus = _FAIL;
				goto DLFW_FAIL;
			}

			dma_iram_sel++;
			mem_offset += send_pkt_size;
			pkt_size_tmp -= send_pkt_size;
		}
	} else {
		RTW_INFO("%s: Download FW fail since MCUFWDL_RDY is not set!\n", __FUNCTION__);
		rtStatus = _FAIL;
		goto DLFW_FAIL;
	}
#endif
#endif

	_FWDownloadEnable(padapter, _FALSE);

	if (rtStatus == _SUCCESS) {
		// 20150128 Sinda/SD-Alex, OS will write 1dx during download FW to let driver stuck for waiting FW clear 0x1cc when resume from sleep.
		// Driver clear 0x1cc irrespective of download FW successful or failure to avoid driver stuck.
		rtw_write8(padapter, REG_HMETFR_8710B, 0xF);
#ifdef PLATFORM_WINDOWS
		rtStatus = _FWFreeToGo(padapter, 10, 200);
#endif
	}
	else
		goto DLFW_FAIL;

	if (_SUCCESS != rtStatus)
		goto DLFW_FAIL;

DLFW_FAIL:
	if (rtStatus == _FAIL) {

		rtw_write32(padapter, REG_8051FW_CTRL_V1_8710B , 0x01000105); // Can show FW page, suggested by Ben.
		RTW_INFO("DUMP REG 0x90=0x%x \n", rtw_read32(padapter, REG_8051FW_CTRL_V1_8710B));//dbg log
		dump_fw_page(padapter);

		/* Disable FWDL_EN */
		value8 = rtw_read8(padapter, REG_8051FW_CTRL_V1_8710B);
		value8 = (value8 & ~(BIT(0)) & ~(BIT(1)));
		rtw_write8(padapter, REG_8051FW_CTRL_V1_8710B, value8);
	}

	RTW_INFO("%s %s. write_fw:%u, %dms\n"
		 , __FUNCTION__, (rtStatus == _SUCCESS) ? "success" : "fail"
		 , write_fw
		 , rtw_get_passing_time_ms(fwdl_start_time)
		);

exit:
	if (pFirmware)
		rtw_mfree((u8 *)pFirmware, sizeof(RT_FIRMWARE_8710B));

	rtl8710b_InitializeFirmwareVars(padapter);

	RTW_INFO(" <=== %s()\n", __FUNCTION__);

	return rtStatus;
}

/* ***********************************************************
 *				Efuse related code
 * *********************************************************** */
static u8
hal_EfuseSwitchToBank(
	PADAPTER	padapter,
	u8			bank,
	u8			bPseudoTest)
{
	u8 bRet = _FALSE;
	u32 value32 = 0;
#ifdef HAL_EFUSE_MEMORY
	PHAL_DATA_TYPE pHalData = GET_HAL_DATA(padapter);
	PEFUSE_HAL pEfuseHal = &pHalData->EfuseHal;
#endif


	RTW_INFO("%s: Efuse switch bank to %d\n", __FUNCTION__, bank);
	if (bPseudoTest) {
#ifdef HAL_EFUSE_MEMORY
		pEfuseHal->fakeEfuseBank = bank;
#else
		fakeEfuseBank = bank;
#endif
		bRet = _TRUE;
	} else {
		value32 = rtw_read32(padapter, EFUSE_TEST);
		bRet = _TRUE;
		switch (bank) {
		case 0:
			value32 = (value32 & ~EFUSE_SEL_MASK) | EFUSE_SEL(EFUSE_WIFI_SEL_0);
			break;
		case 1:
			value32 = (value32 & ~EFUSE_SEL_MASK) | EFUSE_SEL(EFUSE_BT_SEL_0);
			break;
		case 2:
			value32 = (value32 & ~EFUSE_SEL_MASK) | EFUSE_SEL(EFUSE_BT_SEL_1);
			break;
		case 3:
			value32 = (value32 & ~EFUSE_SEL_MASK) | EFUSE_SEL(EFUSE_BT_SEL_2);
			break;
		default:
			value32 = (value32 & ~EFUSE_SEL_MASK) | EFUSE_SEL(EFUSE_WIFI_SEL_0);
			bRet = _FALSE;
			break;
		}
		rtw_write32(padapter, EFUSE_TEST, value32);
	}

	return bRet;
}

static void hal_get_efuse_definition(
	PADAPTER	padapter,
	u8			efuseType,
	u8			type,
	void		*pOut,
	u8			bPseudoTest)
{
	switch (type) {
	case TYPE_EFUSE_MAX_SECTION: {
		u8 *pMax_section;

		pMax_section = (u8 *)pOut;

		*pMax_section = EFUSE_MAX_SECTION_8710B;
	}
	break;

	case TYPE_EFUSE_REAL_CONTENT_LEN: {
		u16 *pu2Tmp;

		pu2Tmp = (u16 *)pOut;

		*pu2Tmp = EFUSE_REAL_CONTENT_LEN_8710B;
	}
	break;

	case TYPE_AVAILABLE_EFUSE_BYTES_BANK: {
		u16	*pu2Tmp;

		pu2Tmp = (u16 *)pOut;

		*pu2Tmp = (EFUSE_REAL_CONTENT_LEN_8710B - EFUSE_OOB_PROTECT_BYTES);
	}
	break;

	case TYPE_AVAILABLE_EFUSE_BYTES_TOTAL: {
		u16 *pu2Tmp;

		pu2Tmp = (u16 *)pOut;

		*pu2Tmp = (EFUSE_REAL_CONTENT_LEN_8710B - EFUSE_OOB_PROTECT_BYTES);
	}
	break;

	case TYPE_EFUSE_MAP_LEN: {
		u16 *pu2Tmp;

		pu2Tmp = (u16 *)pOut;

		*pu2Tmp = EFUSE_MAP_LEN_8710B;
	}
	break;

	case TYPE_EFUSE_PROTECT_BYTES_BANK: {
		u8 *pu1Tmp;

		pu1Tmp = (u8 *)pOut;

		*pu1Tmp = EFUSE_OOB_PROTECT_BYTES;
	}
	break;

	case TYPE_EFUSE_CONTENT_LEN_BANK: {
		u16 *pu2Tmp;

		pu2Tmp = (u16 *)pOut;

		*pu2Tmp = EFUSE_REAL_CONTENT_LEN_8710B;
	}
	break;

	default: {
		u8 *pu1Tmp;

		pu1Tmp = (u8 *)pOut;
		*pu1Tmp = 0;
	}
	break;
	}
}

static void hal_efuse_power_switch(PADAPTER padapter, u8 bWrite, u8 PwrState)
{
	/* 8710BU dont need to do power switch, so return null */
	return;
}

static void hal_read_efuse_8710b(PADAPTER padapter, u8 efuseType,u16 _offset,u16 _size_byte, u8 *pbuf, BOOLEAN bPseudoTest)
{
#ifdef HAL_EFUSE_MEMORY
	PHAL_DATA_TYPE	pHalData = GET_HAL_DATA(padapter);
	PEFUSE_HAL pEfuseHal = &pHalData->EfuseHal;
#endif
	u8	*efuseTbl = NULL;
	u16	eFuse_Addr = 0;
	u8	offset, wden;
	u8	efuseHeader, efuseExtHdr, efuseData;
	u16	i, total, used;
	u8	efuse_usage = 0;

	/* Do NOT excess total size of EFuse table. Added by Roger, 2008.11.10. */
	if ((_offset + _size_byte) > EFUSE_MAX_MAP_LEN) {
		RTW_INFO("%s: Invalid offset(%#x) with read bytes(%#x)!!\n", __FUNCTION__, _offset, _size_byte);
		return;
	}

	efuseTbl = (u8 *)rtw_malloc(EFUSE_MAX_MAP_LEN);
	if (efuseTbl == NULL) {
		RTW_INFO("%s: alloc efuseTbl fail!\n", __FUNCTION__);
		return;
	}
	/* 0xff will be efuse default value instead of 0x00. */
	_rtw_memset(efuseTbl, 0xFF, EFUSE_MAX_MAP_LEN);

	while (AVAILABLE_EFUSE_ADDR(eFuse_Addr)) {
		efuse_OneByteRead(padapter, eFuse_Addr++, &efuseHeader, bPseudoTest);
		if (efuseHeader == 0xFF) {
			//RTW_INFO("%s: data end at address=%#x\n", __FUNCTION__, eFuse_Addr - 1);
			break;
		}
		//RTW_INFO("%s: efuse[0x%X]=0x%02X\n", __FUNCTION__, eFuse_Addr-1, efuseHeader);

		/* Check PG header for section num. */
		if (EXT_HEADER(efuseHeader)) {	/* extended header */
			offset = GET_HDR_OFFSET_2_0(efuseHeader);
			/* RTW_INFO("%s: extended header offset=0x%X\n", __FUNCTION__, offset); */

			/* ReadEFuseByte(padapter, eFuse_Addr++, &efuseExtHdr, bPseudoTest); */
			efuse_OneByteRead(padapter, eFuse_Addr++, &efuseExtHdr, bPseudoTest);
			/* RTW_INFO("%s: efuse[0x%X]=0x%02X\n", __FUNCTION__, eFuse_Addr-1, efuseExtHdr); */
			if (ALL_WORDS_DISABLED(efuseExtHdr))
				continue;

			offset |= ((efuseExtHdr & 0xF0) >> 1);
			wden = (efuseExtHdr & 0x0F);
		} else {
			offset = ((efuseHeader >> 4) & 0x0f);
			wden = (efuseHeader & 0x0f);
		}
		/* RTW_INFO("%s: Offset=%d Worden=0x%X\n", __FUNCTION__, offset, wden); */

		if (offset < EFUSE_MAX_SECTION_8710B) {
			u16 addr;
			/* Get word enable value from PG header
			*			RTW_INFO("%s: Offset=%d Worden=0x%X\n", __FUNCTION__, offset, wden); */

			addr = offset * PGPKT_DATA_SIZE;
			for (i = 0; i < EFUSE_MAX_WORD_UNIT; i++) {
				/* Check word enable condition in the section */
				if (!(wden & (0x01 << i))) {
					efuseData = 0;
					efuse_OneByteRead(padapter, eFuse_Addr++, &efuseData, bPseudoTest);
					//RTW_INFO("%s: efuse[%#X]=0x%02X\n", __FUNCTION__, eFuse_Addr-1, efuseData);
					efuseTbl[addr] = efuseData;

					efuseData = 0;

					efuse_OneByteRead(padapter, eFuse_Addr++, &efuseData, bPseudoTest);
					//RTW_INFO("%s: efuse[%#X]=0x%02X\n", __FUNCTION__, eFuse_Addr-1, efuseData); 
					efuseTbl[addr + 1] = efuseData;
				}
				addr += 2;
			}
		} else {
			RTW_INFO(KERN_ERR "%s: offset(%d) is illegal!!\n", __FUNCTION__, offset);
			eFuse_Addr += Efuse_CalculateWordCnts(wden) * 2;
		}
	}

	/* Copy from Efuse map to output pointer memory!!! */
	for (i = 0; i < _size_byte; i++) {
		pbuf[i] = efuseTbl[_offset + i];
	}

	/* Calculate Efuse utilization */
	total = 0;
	EFUSE_GetEfuseDefinition(padapter, EFUSE_WIFI, TYPE_AVAILABLE_EFUSE_BYTES_TOTAL, &total, bPseudoTest);
	used = eFuse_Addr - 1;
	if (total)
		efuse_usage = (u8)((used * 100) / total);
	else
		efuse_usage = 100;

	if (bPseudoTest) {
#ifdef HAL_EFUSE_MEMORY
		pEfuseHal->fakeEfuseUsedBytes = used;
#else
		fakeEfuseUsedBytes = used;
#endif
	}
	
	if (efuseTbl)
		rtw_mfree(efuseTbl, EFUSE_MAX_MAP_LEN);
}

static u16 hal_EfuseGetCurrentSize_WiFi(PADAPTER padapter, u8	bPseudoTest)
{
#ifdef HAL_EFUSE_MEMORY
	PHAL_DATA_TYPE	pHalData = GET_HAL_DATA(padapter);
	PEFUSE_HAL		pEfuseHal = &pHalData->EfuseHal;
#endif
	u16	efuse_addr = 0;
	u16 start_addr = 0; /* for debug */
	u8	hoffset = 0, hworden = 0;
	u8	efuse_data, word_cnts = 0;
	u32 count = 0; /* for debug */


	if (bPseudoTest) {
#ifdef HAL_EFUSE_MEMORY
		efuse_addr = (u16)pEfuseHal->fakeEfuseUsedBytes;
#else
		efuse_addr = (u16)fakeEfuseUsedBytes;
#endif
	} else
		rtw_hal_get_hwreg(padapter, HW_VAR_EFUSE_BYTES, (u8 *)&efuse_addr);
	start_addr = efuse_addr;
	RTW_INFO("%s: start_efuse_addr=0x%X\n", __FUNCTION__, efuse_addr);

	/* switch bank back to bank 0 for later BT and wifi use. */
	hal_EfuseSwitchToBank(padapter, 0, bPseudoTest);

#if 0 /* for debug test */
	efuse_OneByteRead(padapter, 0x1FF, &efuse_data, bPseudoTest);
	RTW_INFO(FUNC_ADPT_FMT ": efuse raw 0x1FF=0x%02X\n",
		 FUNC_ADPT_ARG(padapter), efuse_data);
	efuse_data = 0xFF;
#endif /* for debug test */

	count = 0;
	while (AVAILABLE_EFUSE_ADDR(efuse_addr)) {
#if 1
		if (efuse_OneByteRead(padapter, efuse_addr, &efuse_data, bPseudoTest) == _FALSE) {
			RTW_INFO(KERN_ERR "%s: efuse_OneByteRead Fail! addr=0x%X !!\n", __FUNCTION__, efuse_addr);
			goto error;
		}
#else
		ReadEFuseByte(padapter, efuse_addr, &efuse_data, bPseudoTest);
#endif

		if (efuse_data == 0xFF)
			break;

		if ((start_addr != 0) && (efuse_addr == start_addr)) {
			count++;
			RTW_INFO(FUNC_ADPT_FMT ": [WARNING] efuse raw 0x%X=0x%02X not 0xFF!!(%d times)\n",
				FUNC_ADPT_ARG(padapter), efuse_addr, efuse_data, count);

			efuse_data = 0xFF;
			if (count < 4) {
				/* try again! */

				if (count > 2) {
					/* try again form address 0 */
					efuse_addr = 0;
					start_addr = 0;
				}

				continue;
			}

			goto error;
		}

		if (EXT_HEADER(efuse_data)) {
			hoffset = GET_HDR_OFFSET_2_0(efuse_data);
			efuse_addr++;
			efuse_OneByteRead(padapter, efuse_addr, &efuse_data, bPseudoTest);
			if (ALL_WORDS_DISABLED(efuse_data))
				continue;

			hoffset |= ((efuse_data & 0xF0) >> 1);
			hworden = efuse_data & 0x0F;
		} else {
			hoffset = (efuse_data >> 4) & 0x0F;
			hworden = efuse_data & 0x0F;
		}

		word_cnts = Efuse_CalculateWordCnts(hworden);
		efuse_addr += (word_cnts * 2) + 1;
	}

	if (bPseudoTest) {
#ifdef HAL_EFUSE_MEMORY
		pEfuseHal->fakeEfuseUsedBytes = efuse_addr;
#else
		fakeEfuseUsedBytes = efuse_addr;
#endif
	} else
		rtw_hal_set_hwreg(padapter, HW_VAR_EFUSE_BYTES, (u8 *)&efuse_addr);

	goto exit;

error:
	/* report max size to prevent write efuse */
	EFUSE_GetEfuseDefinition(padapter, EFUSE_WIFI, TYPE_AVAILABLE_EFUSE_BYTES_TOTAL, &efuse_addr, bPseudoTest);

exit:
	RTW_INFO("%s: CurrentSize=%d\n", __FUNCTION__, efuse_addr);

	return efuse_addr;
}

static u16 Hal_EfuseGetCurrentSize(PADAPTER	padapter, u8 	efuseType, u8 bPseudoTest)
{
	return hal_EfuseGetCurrentSize_WiFi(padapter, bPseudoTest);
}

static u8 Hal_EfuseWordEnableDataWrite(PADAPTER	padapter, u16 efuse_addr, u8 word_en, u8 *data, u8 bPseudoTest)
{
	u16	tmpaddr = 0;
	u16	start_addr = efuse_addr;
	u8	badworden = 0x0F;
	u8	tmpdata[PGPKT_DATA_SIZE];


	/*	RTW_INFO("%s: efuse_addr=%#x word_en=%#x\n", __FUNCTION__, efuse_addr, word_en); */
	_rtw_memset(tmpdata, 0xFF, PGPKT_DATA_SIZE);

	if (!(word_en & BIT(0))) {
		tmpaddr = start_addr;
		efuse_OneByteWrite(padapter, start_addr++, data[0], bPseudoTest);
		efuse_OneByteWrite(padapter, start_addr++, data[1], bPseudoTest);

		efuse_OneByteRead(padapter, tmpaddr, &tmpdata[0], bPseudoTest);
		efuse_OneByteRead(padapter, tmpaddr + 1, &tmpdata[1], bPseudoTest);
		if ((data[0] != tmpdata[0]) || (data[1] != tmpdata[1]))
			badworden &= (~BIT(0));
	}
	if (!(word_en & BIT(1))) {
		tmpaddr = start_addr;
		efuse_OneByteWrite(padapter, start_addr++, data[2], bPseudoTest);
		efuse_OneByteWrite(padapter, start_addr++, data[3], bPseudoTest);

		efuse_OneByteRead(padapter, tmpaddr, &tmpdata[2], bPseudoTest);
		efuse_OneByteRead(padapter, tmpaddr + 1, &tmpdata[3], bPseudoTest);
		if ((data[2] != tmpdata[2]) || (data[3] != tmpdata[3]))
			badworden &= (~BIT(1));
	}
	if (!(word_en & BIT(2))) {
		tmpaddr = start_addr;
		efuse_OneByteWrite(padapter, start_addr++, data[4], bPseudoTest);
		efuse_OneByteWrite(padapter, start_addr++, data[5], bPseudoTest);

		efuse_OneByteRead(padapter, tmpaddr, &tmpdata[4], bPseudoTest);
		efuse_OneByteRead(padapter, tmpaddr + 1, &tmpdata[5], bPseudoTest);
		if ((data[4] != tmpdata[4]) || (data[5] != tmpdata[5]))
			badworden &= (~BIT(2));
	}
	if (!(word_en & BIT(3))) {
		tmpaddr = start_addr;
		efuse_OneByteWrite(padapter, start_addr++, data[6], bPseudoTest);
		efuse_OneByteWrite(padapter, start_addr++, data[7], bPseudoTest);

		efuse_OneByteRead(padapter, tmpaddr, &tmpdata[6], bPseudoTest);
		efuse_OneByteRead(padapter, tmpaddr + 1, &tmpdata[7], bPseudoTest);
		if ((data[6] != tmpdata[6]) || (data[7] != tmpdata[7]))
			badworden &= (~BIT(3));
	}

	return badworden;
}

static s32
Hal_EfusePgPacketRead(
	PADAPTER	padapter,
	u8			offset,
	u8			*data,
	u8			bPseudoTest)
{
	u8	bDataEmpty = _TRUE;
	u8	efuse_data, word_cnts = 0;
	u16	efuse_addr = 0;
	u8	hoffset = 0, hworden = 0;
	u8	i;
	u8	max_section = 0;
	s32	ret;


	if (data == NULL)
		return _FALSE;

	EFUSE_GetEfuseDefinition(padapter, EFUSE_WIFI, TYPE_EFUSE_MAX_SECTION, &max_section, bPseudoTest);
	if (offset > max_section) {
		RTW_INFO("%s: Packet offset(%d) is illegal(>%d)!\n", __FUNCTION__, offset, max_section);
		return _FALSE;
	}

	_rtw_memset(data, 0xFF, PGPKT_DATA_SIZE);
	ret = _TRUE;

	/* */
	/* <Roger_TODO> Efuse has been pre-programmed dummy 5Bytes at the end of Efuse by CP. */
	/* Skip dummy parts to prevent unexpected data read from Efuse. */
	/* By pass right now. 2009.02.19. */
	/* */
	while (AVAILABLE_EFUSE_ADDR(efuse_addr)) {
		if (efuse_OneByteRead(padapter, efuse_addr++, &efuse_data, bPseudoTest) == _FALSE) {
			ret = _FALSE;
			break;
		}

		if (efuse_data == 0xFF)
			break;

		if (EXT_HEADER(efuse_data)) {
			hoffset = GET_HDR_OFFSET_2_0(efuse_data);
			efuse_OneByteRead(padapter, efuse_addr++, &efuse_data, bPseudoTest);
			if (ALL_WORDS_DISABLED(efuse_data)) {
				RTW_INFO("%s: Error!! All words disabled!\n", __FUNCTION__);
				continue;
			}

			hoffset |= ((efuse_data & 0xF0) >> 1);
			hworden = efuse_data & 0x0F;
		} else {
			hoffset = (efuse_data >> 4) & 0x0F;
			hworden =  efuse_data & 0x0F;
		}

		if (hoffset == offset) {
			for (i = 0; i < EFUSE_MAX_WORD_UNIT; i++) {
				/* Check word enable condition in the section */
				if (!(hworden & (0x01 << i))) {
					/* ReadEFuseByte(padapter, efuse_addr++, &efuse_data, bPseudoTest); */
					efuse_OneByteRead(padapter, efuse_addr++, &efuse_data, bPseudoTest);
					/*					RTW_INFO("%s: efuse[%#X]=0x%02X\n", __FUNCTION__, efuse_addr+tmpidx, efuse_data); */
					data[i * 2] = efuse_data;

					/* ReadEFuseByte(padapter, efuse_addr++, &efuse_data, bPseudoTest); */
					efuse_OneByteRead(padapter, efuse_addr++, &efuse_data, bPseudoTest);
					/*					RTW_INFO("%s: efuse[%#X]=0x%02X\n", __FUNCTION__, efuse_addr+tmpidx, efuse_data); */
					data[(i * 2) + 1] = efuse_data;
				}
			}
		} else {
			word_cnts = Efuse_CalculateWordCnts(hworden);
			efuse_addr += word_cnts * 2;
		}
	}

	return ret;
}

static u8
hal_EfusePgCheckAvailableAddr(
	PADAPTER	padapter,
	u8			efuseType,
	u8		bPseudoTest)
{
	u16	max_available = 0;
	u16 current_size;


	EFUSE_GetEfuseDefinition(padapter, efuseType, TYPE_AVAILABLE_EFUSE_BYTES_TOTAL, &max_available, bPseudoTest);
	/*	RTW_INFO("%s: max_available=%d\n", __FUNCTION__, max_available); */

	current_size = Efuse_GetCurrentSize(padapter, efuseType, bPseudoTest);
	if (current_size >= max_available) {
		RTW_INFO("%s: Error!! current_size(%d)>max_available(%d)\n", __FUNCTION__, current_size, max_available);
		return _FALSE;
	}
	return _TRUE;
}

static void
hal_EfuseConstructPGPkt(
	u8				offset,
	u8				word_en,
	u8				*pData,
	PPGPKT_STRUCT	pTargetPkt)
{
	_rtw_memset(pTargetPkt->data, 0xFF, PGPKT_DATA_SIZE);
	pTargetPkt->offset = offset;
	pTargetPkt->word_en = word_en;
	efuse_WordEnableDataRead(word_en, pData, pTargetPkt->data);
	pTargetPkt->word_cnts = Efuse_CalculateWordCnts(pTargetPkt->word_en);
}

#if 0
static u8
wordEnMatched(
	PPGPKT_STRUCT	pTargetPkt,
	PPGPKT_STRUCT	pCurPkt,
	u8				*pWden)
{
	u8	match_word_en = 0x0F;	/* default all words are disabled */
	u8	i;

	/* check if the same words are enabled both target and current PG packet */
	if (((pTargetPkt->word_en & BIT(0)) == 0) &&
	    ((pCurPkt->word_en & BIT(0)) == 0)) {
		match_word_en &= ~BIT(0);				/* enable word 0 */
	}
	if (((pTargetPkt->word_en & BIT(1)) == 0) &&
	    ((pCurPkt->word_en & BIT(1)) == 0)) {
		match_word_en &= ~BIT(1);				/* enable word 1 */
	}
	if (((pTargetPkt->word_en & BIT(2)) == 0) &&
	    ((pCurPkt->word_en & BIT(2)) == 0)) {
		match_word_en &= ~BIT(2);				/* enable word 2 */
	}
	if (((pTargetPkt->word_en & BIT(3)) == 0) &&
	    ((pCurPkt->word_en & BIT(3)) == 0)) {
		match_word_en &= ~BIT(3);				/* enable word 3 */
	}

	*pWden = match_word_en;

	if (match_word_en != 0xf)
		return _TRUE;
	else
		return _FALSE;
}

static u8
hal_EfuseCheckIfDatafollowed(
	PADAPTER		padapter,
	u8				word_cnts,
	u16				startAddr,
	u8				bPseudoTest)
{
	u8 bRet = _FALSE;
	u8 i, efuse_data;

	for (i = 0; i < (word_cnts * 2); i++) {
		if (efuse_OneByteRead(padapter, (startAddr + i) , &efuse_data, bPseudoTest) == _FALSE) {
			RTW_INFO("%s: efuse_OneByteRead FAIL!!\n", __FUNCTION__);
			bRet = _TRUE;
			break;
		}

		if (efuse_data != 0xFF) {
			bRet = _TRUE;
			break;
		}
	}

	return bRet;
}
#endif

static u8
hal_EfusePartialWriteCheck(
	PADAPTER		padapter,
	u8				efuseType,
	u16				*pAddr,
	PPGPKT_STRUCT	pTargetPkt,
	u8				bPseudoTest)
{
	PHAL_DATA_TYPE	pHalData = GET_HAL_DATA(padapter);
	PEFUSE_HAL		pEfuseHal = &pHalData->EfuseHal;
	u8	bRet = _FALSE;
	u16	startAddr = 0, efuse_max_available_len = 0, efuse_max = 0;
	u8	efuse_data = 0;
#if 0
	u8	i, cur_header = 0;
	u8	new_wden = 0, matched_wden = 0, badworden = 0;
	PGPKT_STRUCT	curPkt;
#endif


	EFUSE_GetEfuseDefinition(padapter, efuseType, TYPE_AVAILABLE_EFUSE_BYTES_TOTAL, &efuse_max_available_len, bPseudoTest);
	EFUSE_GetEfuseDefinition(padapter, efuseType, TYPE_EFUSE_CONTENT_LEN_BANK, &efuse_max, bPseudoTest);

	if (efuseType == EFUSE_WIFI) {
		if (bPseudoTest) {
#ifdef HAL_EFUSE_MEMORY
			startAddr = (u16)pEfuseHal->fakeEfuseUsedBytes;
#else
			startAddr = (u16)fakeEfuseUsedBytes;
#endif
		} else
			rtw_hal_get_hwreg(padapter, HW_VAR_EFUSE_BYTES, (u8 *)&startAddr);
	} else {
		if (bPseudoTest) {
#ifdef HAL_EFUSE_MEMORY
			startAddr = (u16)pEfuseHal->fakeBTEfuseUsedBytes;
#else
			startAddr = (u16)fakeBTEfuseUsedBytes;
#endif
		} else
			rtw_hal_get_hwreg(padapter, HW_VAR_EFUSE_BT_BYTES, (u8 *)&startAddr);
	}
	startAddr %= efuse_max;
	RTW_INFO("%s: startAddr=%#X\n", __FUNCTION__, startAddr);

	while (1) {
		if (startAddr >= efuse_max_available_len) {
			bRet = _FALSE;
			RTW_INFO("%s: startAddr(%d) >= efuse_max_available_len(%d)\n",
				__FUNCTION__, startAddr, efuse_max_available_len);
			break;
		}

		if (efuse_OneByteRead(padapter, startAddr, &efuse_data, bPseudoTest) && (efuse_data != 0xFF)) {
#if 1
			bRet = _FALSE;
			RTW_INFO("%s: Something Wrong! last bytes(%#X=0x%02X) is not 0xFF\n",
				 __FUNCTION__, startAddr, efuse_data);
			break;
#else
			if (EXT_HEADER(efuse_data)) {
				cur_header = efuse_data;
				startAddr++;
				efuse_OneByteRead(padapter, startAddr, &efuse_data, bPseudoTest);
				if (ALL_WORDS_DISABLED(efuse_data)) {
					RTW_INFO("%s: Error condition, all words disabled!", __FUNCTION__);
					bRet = _FALSE;
					break;
				} /*else*/
				{
					curPkt.offset = ((cur_header & 0xE0) >> 5) | ((efuse_data & 0xF0) >> 1);
					curPkt.word_en = efuse_data & 0x0F;
				}
			} else {
				cur_header  =  efuse_data;
				curPkt.offset = (cur_header >> 4) & 0x0F;
				curPkt.word_en = cur_header & 0x0F;
			}

			curPkt.word_cnts = Efuse_CalculateWordCnts(curPkt.word_en);
			/* if same header is found but no data followed */
			/* write some part of data followed by the header. */
			if ((curPkt.offset == pTargetPkt->offset) &&
			    (hal_EfuseCheckIfDatafollowed(padapter, curPkt.word_cnts, startAddr + 1, bPseudoTest) == _FALSE) &&
			    wordEnMatched(pTargetPkt, &curPkt, &matched_wden) == _TRUE) {
				RTW_INFO("%s: Need to partial write data by the previous wrote header\n", __FUNCTION__);
				/* Here to write partial data */
				badworden = Efuse_WordEnableDataWrite(padapter, startAddr + 1, matched_wden, pTargetPkt->data, bPseudoTest);
				if (badworden != 0x0F) {
					u32	PgWriteSuccess = 0;
					/* if write fail on some words, write these bad words again */
					if (efuseType == EFUSE_WIFI)
						PgWriteSuccess = Efuse_PgPacketWrite(padapter, pTargetPkt->offset, badworden, pTargetPkt->data, bPseudoTest);
					else
						PgWriteSuccess = Efuse_PgPacketWrite_BT(padapter, pTargetPkt->offset, badworden, pTargetPkt->data, bPseudoTest);

					if (!PgWriteSuccess) {
						bRet = _FALSE;	/* write fail, return */
						break;
					}
				}
				/* partial write ok, update the target packet for later use */
				for (i = 0; i < 4; i++) {
					if ((matched_wden & (0x1 << i)) == 0) {	/* this word has been written */
						pTargetPkt->word_en |= (0x1 << i);	/* disable the word */
					}
				}
				pTargetPkt->word_cnts = Efuse_CalculateWordCnts(pTargetPkt->word_en);
			}
			/* read from next header */
			startAddr = startAddr + (curPkt.word_cnts * 2) + 1;
#endif
		} else {
			/* not used header, 0xff */
			*pAddr = startAddr;
			/*			RTW_INFO("%s: Started from unused header offset=%d\n", __FUNCTION__, startAddr)); */
			bRet = _TRUE;
			break;
		}
	}

	return bRet;
}

static u8
hal_EfusePgPacketWrite1ByteHeader(
	PADAPTER		padapter,
	u8				efuseType,
	u16				*pAddr,
	PPGPKT_STRUCT	pTargetPkt,
	u8				bPseudoTest)
{
	u8	bRet = _FALSE;
	u8	pg_header = 0, tmp_header = 0;
	u16	efuse_addr = *pAddr;
	u8	repeatcnt = 0;


	/*	RTW_INFO("%s\n", __FUNCTION__); */
	pg_header = ((pTargetPkt->offset << 4) & 0xf0) | pTargetPkt->word_en;

	do {
		efuse_OneByteWrite(padapter, efuse_addr, pg_header, bPseudoTest);
		efuse_OneByteRead(padapter, efuse_addr, &tmp_header, bPseudoTest);
		if (tmp_header != 0xFF)
			break;
		if (repeatcnt++ > EFUSE_REPEAT_THRESHOLD_) {
			RTW_INFO("%s: Repeat over limit for pg_header!!\n", __FUNCTION__);
			return _FALSE;
		}
	} while (1);

	if (tmp_header != pg_header) {
		RTW_INFO(KERN_ERR "%s: PG Header Fail!!(pg=0x%02X read=0x%02X)\n", __FUNCTION__, pg_header, tmp_header);
		return _FALSE;
	}

	*pAddr = efuse_addr;

	return _TRUE;
}

static u8
hal_EfusePgPacketWrite2ByteHeader(
	PADAPTER		padapter,
	u8				efuseType,
	u16				*pAddr,
	PPGPKT_STRUCT	pTargetPkt,
	u8				bPseudoTest)
{
	u16	efuse_addr, efuse_max_available_len = 0;
	u8	pg_header = 0, tmp_header = 0;
	u8	repeatcnt = 0;


	/*	RTW_INFO("%s\n", __FUNCTION__); */
	EFUSE_GetEfuseDefinition(padapter, efuseType, TYPE_AVAILABLE_EFUSE_BYTES_BANK, &efuse_max_available_len, bPseudoTest);

	efuse_addr = *pAddr;
	if (efuse_addr >= efuse_max_available_len) {
		RTW_INFO("%s: addr(%d) over available(%d)!!\n", __FUNCTION__, efuse_addr, efuse_max_available_len);
		return _FALSE;
	}

	pg_header = ((pTargetPkt->offset & 0x07) << 5) | 0x0F;
	/*	RTW_INFO("%s: pg_header=0x%x\n", __FUNCTION__, pg_header); */

	do {
		efuse_OneByteWrite(padapter, efuse_addr, pg_header, bPseudoTest);
		efuse_OneByteRead(padapter, efuse_addr, &tmp_header, bPseudoTest);
		if (tmp_header != 0xFF)
			break;
		if (repeatcnt++ > EFUSE_REPEAT_THRESHOLD_) {
			RTW_INFO("%s: Repeat over limit for pg_header!!\n", __FUNCTION__);
			return _FALSE;
		}
	} while (1);

	if (tmp_header != pg_header) {
		RTW_INFO(KERN_ERR "%s: PG Header Fail!!(pg=0x%02X read=0x%02X)\n", __FUNCTION__, pg_header, tmp_header);
		return _FALSE;
	}

	/* to write ext_header */
	efuse_addr++;
	pg_header = ((pTargetPkt->offset & 0x78) << 1) | pTargetPkt->word_en;

	do {
		efuse_OneByteWrite(padapter, efuse_addr, pg_header, bPseudoTest);
		efuse_OneByteRead(padapter, efuse_addr, &tmp_header, bPseudoTest);
		if (tmp_header != 0xFF)
			break;
		if (repeatcnt++ > EFUSE_REPEAT_THRESHOLD_) {
			RTW_INFO("%s: Repeat over limit for ext_header!!\n", __FUNCTION__);
			return _FALSE;
		}
	} while (1);

	if (tmp_header != pg_header) {	/* offset PG fail */
		RTW_INFO(KERN_ERR "%s: PG EXT Header Fail!!(pg=0x%02X read=0x%02X)\n", __FUNCTION__, pg_header, tmp_header);
		return _FALSE;
	}

	*pAddr = efuse_addr;

	return _TRUE;
}

static u8
hal_EfusePgPacketWriteHeader(
	PADAPTER		padapter,
	u8				efuseType,
	u16				*pAddr,
	PPGPKT_STRUCT	pTargetPkt,
	u8				bPseudoTest)
{
	u8 bRet = _FALSE;

	if (pTargetPkt->offset >= EFUSE_MAX_SECTION_BASE)
		bRet = hal_EfusePgPacketWrite2ByteHeader(padapter, efuseType, pAddr, pTargetPkt, bPseudoTest);
	else
		bRet = hal_EfusePgPacketWrite1ByteHeader(padapter, efuseType, pAddr, pTargetPkt, bPseudoTest);

	return bRet;
}

static u8
hal_EfusePgPacketWriteData(
	PADAPTER		padapter,
	u8				efuseType,
	u16				*pAddr,
	PPGPKT_STRUCT	pTargetPkt,
	u8				bPseudoTest)
{
	u16	efuse_addr;
	u8	badworden;


	efuse_addr = *pAddr;
	badworden = Efuse_WordEnableDataWrite(padapter, efuse_addr + 1, pTargetPkt->word_en, pTargetPkt->data, bPseudoTest);
	if (badworden != 0x0F) {
		RTW_INFO("%s: Fail!!\n", __FUNCTION__);
		return _FALSE;
	}

	/*	RTW_INFO("%s: ok\n", __FUNCTION__); */
	return _TRUE;
}

static s32
Hal_EfusePgPacketWrite(
	PADAPTER	padapter,
	u8			offset,
	u8			word_en,
	u8			*pData,
	u8			bPseudoTest)
{
	PGPKT_STRUCT targetPkt;
	u16 startAddr = 0;
	u8 efuseType = EFUSE_WIFI;

	if (!hal_EfusePgCheckAvailableAddr(padapter, efuseType, bPseudoTest))
		return _FALSE;

	hal_EfuseConstructPGPkt(offset, word_en, pData, &targetPkt);

	if (!hal_EfusePartialWriteCheck(padapter, efuseType, &startAddr, &targetPkt, bPseudoTest))
		return _FALSE;

	if (!hal_EfusePgPacketWriteHeader(padapter, efuseType, &startAddr, &targetPkt, bPseudoTest))
		return _FALSE;

	if (!hal_EfusePgPacketWriteData(padapter, efuseType, &startAddr, &targetPkt, bPseudoTest))
		return _FALSE;

	return _TRUE;
}

/* 
* efuse read   											
* 1) write 0x80F8: 0x000000xx =>xx is efuse physical address  	
* 2) write 0x806C: 0x85000000  							
* 3) poll bit31  											
* 4) read 0x809C: 0x000000yy => yy is efuse physical data  		
*/
BOOLEAN hal_efuse_read4_8710b(PADAPTER padapter, u16 regaddr, u8 *value)
{
	BOOLEAN ret = _TRUE;
	u8 pollint_count;
	u32 value32, reg_value32;
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);

	reg_value32 =0xFFFFFFFF;
	pollint_count = 0xFF;

	rtw_write32(padapter, REG_USB_HOST_INDIRECT_ADDR_8710B, regaddr);
	rtw_write32(padapter, REG_EFUSE_INDIRECT_CTRL_8710B, EFUSE_READ_OFFSET);
	
	do {
		value32 = rtw_read32(padapter, REG_EFUSE_INDIRECT_CTRL_8710B);
	}
	while ((value32 & BIT31) && (pollint_count-- > 0));

	if (pollint_count == 0) {
		RTW_ERR("Error: Polling 0x6c[31] fail, 0x6c = 0x%x\n", value32);
		ret = _FALSE;
	}
	else {
		reg_value32 = rtw_read32(padapter, REG_USB_HOST_INDIRECT_DATA_8710B);
		*value = (u8)reg_value32 & 0xFF; 
	}
	
	return ret;
}

#if 0
// Peter, TODO : cant use. Need modify
/* efuse write												*/
/* 1) write 0x80F8: 0x000000xx =>xx is efuse physical address	*/
/* 2) write 0x809C: 0x000000yy => yy is efuse physical data		*/
/* 3) write 0x806C: 0x86000000								*/
/* 4) poll bit31											*/
BOOLEAN hal_efuse_write4(PADAPTER padapter, u16 offset, u8 value)
{
	BOOLEAN bRetVaule = TRUE;
	u4Byte	u4Temp = 0;
	u1Byte u1PollingCnt = 0xFF;
	RT_TRACE(COMP_EFUSE, DBG_LOUD, ("Hal_EfuseOneByteWrite_8710B() offset = 0x%x, value= 0x%x\n", offset,value));
	
	PlatformEFIOWrite4Byte(padapter, REG_USB_HOST_INDIRECT_ADDR_8710B, offset);
	PlatformEFIOWrite4Byte(padapter, REG_USB_HOST_INDIRECT_DATA_8710B, value);
	PlatformEFIOWrite4Byte(padapter, REG_EFUSE_INDIRECT_CTRL_8710B, 0x86000000);

	do
	{
		u4Temp = PlatformEFIORead4Byte(padapter, REG_EFUSE_INDIRECT_CTRL_8710B);
	}
	while((u4Temp & BIT31) && (u1PollingCnt-- > 0));

	if(u1PollingCnt == 0)
	{
		RT_TRACE(COMP_EFUSE, DBG_SERIOUS, ("Hal_EfuseOneByteWrite_8710B() Error: Polling 0x6c[31] fail, 0x6c = 0x%x\n",u4Temp));
		bRetVaule = FALSE;
	}

	return bRetVaule;
}
#endif

/**
* Function:	CalculateBitShift*
* OverView:	Get shifted position of the BitMask*
* Input:*		u4Byte		BitMask,	*
* Output:		none
* Return:		u4Byte		Return the shift bit bit position of the mask
*/
u32 calculate_bitshift(u32 bitmask)
{
	u32 i;

	for (i=0; i<=31; i++) {
		if (((bitmask>>i) &  0x1 ) == 1)
			break;
	}
	return i;
}

/* 
* read any register 
* 1) write 0x80F8: 0xxxxxxxxx =>xxxxxx is register complete address		
* 2) write 0x806C: 0x83000000													
* 3) poll bit31													
* 4) read 0x809C: 0xyyyyyyyy => yyyyyyyy is register data				
*/
u32 indirect_read32_8710b(PADAPTER padapter, u32 regaddr)
{
	u32 value32, reg_value32;
	u8 pollint_count;
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);
	_mutex *mutex = &adapter_to_dvobj(padapter)->syson_indirect_access_mutex;

	reg_value32 =0xFFFFFFFF;
	pollint_count = 0xFF;
	
	/* 4 byte alignment */
	if ((regaddr & 0x00000003) != 0)
	{
		RTW_DBG("skip read reg because offset(0x%x) is not 4 byte alignment!!!\n",regaddr);
		return reg_value32;
	}	
	
	_enter_critical_mutex(mutex, NULL);

	rtw_write32(padapter, REG_USB_HOST_INDIRECT_ADDR_8710B, regaddr);
	rtw_write32(padapter, REG_EFUSE_INDIRECT_CTRL_8710B, NORMAL_REG_READ_OFFSET);

	do {
		value32 = rtw_read32(padapter, REG_EFUSE_INDIRECT_CTRL_8710B);
	}
	while ((value32 & BIT31) && (pollint_count-- > 0));

	if (pollint_count == 0) 
		RTW_ERR("Indirect_read32_8710b fail, 0x806c = 0x%x\n", value32);
	else
		reg_value32 = rtw_read32(padapter, REG_USB_HOST_INDIRECT_DATA_8710B);

	_exit_critical_mutex(mutex, NULL);

	return reg_value32;
}

/* write any register												*/
/* 1) write 0x80F8: 0xxxxxxxxx =>xxxxxx is register complete address		*/
/* 2) write 0x809C: 0xyyyyyyyy =>yyyyyyyy is register data				*/
/* 3) write 0x806C: 84000000										*/
/* 4) poll bit31													*/
VOID indirect_write32_8710b(PADAPTER padapter, u32 regaddr, u32 data)
{
	u32 value32, reg_value32;
	u8 pollint_count;
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);
	_mutex *mutex = &adapter_to_dvobj(padapter)->syson_indirect_access_mutex;

	reg_value32 =0xFFFFFFFF;
	pollint_count = 0xFF;

	//  4 byte alignment
	if((regaddr & 0x00000003) != 0)
	{
		RTW_DBG("skip write reg because offset(0x%x) is not 4 byte alignment!!!\n", regaddr);
		return;
	}
	
	_enter_critical_mutex(mutex, NULL);
	
	rtw_write32(padapter, REG_USB_HOST_INDIRECT_ADDR_8710B, regaddr);
	rtw_write32(padapter, REG_USB_HOST_INDIRECT_DATA_8710B, data);
	rtw_write32(padapter, REG_EFUSE_INDIRECT_CTRL_8710B, NORMAL_REG_WRITE_OFFSET);

	do {
		value32 = rtw_read32(padapter, REG_EFUSE_INDIRECT_CTRL_8710B);
	}
	while((value32 & BIT31) && (pollint_count-- > 0));
	
	if(pollint_count == 0)
		RTW_ERR("Indirect_write32_8710b fail, 0x806c = 0x%x\n", value32);

	_exit_critical_mutex(mutex, NULL);
	
	return;	
}

u32 hal_query_syson_reg_8710b(PADAPTER padapter, u32 regaddr, u32 bitmask)
{
	u32	ret = 0;
	u32 value32, bitshift = 0;

	/* 4 byte alignment */
	if((regaddr & 0x00000003) != 0)
	{
		RTW_DBG("skip Query SYSOn Reg because RegAddr(0x%x) is not 4 byte alignment!!!\n", regaddr);
		goto exit;
	}	
	
	RTW_DBG("--->hal_query_syson_reg_8710b(): RegAddr(%#x), BitMask(%#x)\n", regaddr, bitmask);
	
	value32 = indirect_read32_8710b(padapter, regaddr | SYSON_REG_BASE_ADDR_8710B);
	bitshift = calculate_bitshift(bitmask);
	ret = (value32 & bitmask) >> bitshift;

	RTW_DBG("<---hal_query_syson_reg_8710b(): RegAddr(%#x), BitMask(%#x), OriginalValue(%#x)\n", regaddr, bitmask, value32);

exit:
	return ret;
}

VOID hal_set_syson_reg_8710b(PADAPTER padapter, u32 regaddr, u32 bitmask, u32 data)
{
	u32 value32, bitshift = 0;

	/* 4 byte alignment */
	if ((regaddr & 0x00000003) != 0)
	{
		RTW_DBG("skip Set SYSOn Reg because RegAddr(0x%x) is not 4 byte alignment!!!\n", regaddr);
		goto exit;
	}	
	
	RTW_DBG("--->HAL_SetSYSOnReg_8710B(): RegAddr(%#x), BitMask(%#x), Data(%#x)\n", regaddr, bitmask, data);
	
	if (bitmask != bMaskDWord)
	{
		value32 = indirect_read32_8710b(padapter, regaddr | SYSON_REG_BASE_ADDR_8710B);
		bitshift = calculate_bitshift(bitmask);
		data = ((value32) & (~bitmask)) | ((data << bitshift));
	}

	indirect_write32_8710b(padapter, regaddr | SYSON_REG_BASE_ADDR_8710B, data);
	RTW_DBG("<---HAL_SetSYSOnReg_8710B(): RegAddr(%#x), BitMask(%#x), Data(%#x)\n", regaddr, bitmask, data);

exit:
	return;
}

static void read_chip_version_8710b(PADAPTER padapter)
{
	u32				value32;
	HAL_DATA_TYPE	*pHalData;

	pHalData = GET_HAL_DATA(padapter);

	value32 = hal_query_syson_reg_8710b(padapter, REG_SYS_SYSTEM_CFG0, 0xFFFFFFFF);
	pHalData->version_id.ICType = CHIP_8710B;
	pHalData->version_id.ChipType = ((value32 & BIT_RTL_ID_8710B) ? TEST_CHIP : NORMAL_CHIP);
	pHalData->version_id.CUTVersion = BIT_GET_CHIP_VER_8710B(value32);
	pHalData->version_id.VendorType = BIT_GET_VENDOR_ID_8710B(value32);
	pHalData->version_id.VendorType >>= 2;
	switch (pHalData->version_id.VendorType) {
	case 0:
		pHalData->version_id.VendorType = CHIP_VENDOR_SMIC;
		break;
	case 1:
		pHalData->version_id.VendorType = CHIP_VENDOR_TSMC;
		break;
	case 2:
		pHalData->version_id.VendorType = CHIP_VENDOR_UMC;
		break;
	}
	pHalData->version_id.RFType = RF_TYPE_1T1R;

	value32 = hal_query_syson_reg_8710b(padapter, REG_SYS_SYSTEM_CFG1, 0xFFFFFFFF);
	pHalData->RegulatorMode = ((value32 & BIT_SPSLDO_SEL_8710B) ? RT_LDO_REGULATOR : RT_SWITCHING_REGULATOR);

	value32 = hal_query_syson_reg_8710b(padapter, REG_SYS_SYSTEM_CFG2, 0xFFFFFFFF);
	pHalData->version_id.ROMVer = BIT_GET_RF_RL_ID_8710B(value32);

	pHalData->MultiFunc = RT_MULTI_FUNC_NONE;

	rtw_hal_config_rftype(padapter);

	dump_chip_info(pHalData->version_id);
}


void rtl8710b_InitBeaconParameters(PADAPTER padapter)
{
	PHAL_DATA_TYPE pHalData = GET_HAL_DATA(padapter);
	u16 val16;
	u8 val8;
	
	val8 = DIS_TSF_UDT;
	val16 = val8 | (val8 << 8); /* port0 and port1 */

#ifdef CONFIG_CONCURRENT_MODE
	val16 |= (EN_BCN_FUNCTION << 8);
#endif
	rtw_write16(padapter, REG_BCN_CTRL, val16);

	/* TBTT setup time */
	rtw_write8(padapter, REG_TBTT_PROHIBIT, TBTT_PROHIBIT_SETUP_TIME);

	/* TBTT hold time: 0x540[19:8] */
	rtw_write8(padapter, REG_TBTT_PROHIBIT + 1, TBTT_PROHIBIT_HOLD_TIME_STOP_BCN & 0xFF);
	rtw_write8(padapter, REG_TBTT_PROHIBIT + 2,
		(rtw_read8(padapter, REG_TBTT_PROHIBIT + 2) & 0xF0) | (TBTT_PROHIBIT_HOLD_TIME_STOP_BCN >> 8));

	/* Firmware will control REG_DRVERLYINT when power saving is enable, */
	/* so don't set this register on STA mode. */
	if (check_fwstate(&padapter->mlmepriv, WIFI_STATION_STATE) == _FALSE)
		rtw_write8(padapter, REG_DRVERLYINT, DRIVER_EARLY_INT_TIME_8710B); /* 5ms */
	rtw_write8(padapter, REG_BCNDMATIM, BCN_DMA_ATIME_INT_TIME_8710B); /* 2ms */

	/* Suggested by designer timchen. Change beacon AIFS to the largest number */
	/* beacause test chip does not contension before sending beacon. by tynli. 2009.11.03 */
	rtw_write16(padapter, REG_BCNTCFG, 0x660F);

}

void rtl8710b_InitBeaconMaxError(PADAPTER padapter, u8 InfraMode)
{
#ifdef CONFIG_ADHOC_WORKAROUND_SETTING
	rtw_write8(padapter, REG_BCN_MAX_ERR, 0xFF);
#else
	/* rtw_write8(Adapter, REG_BCN_MAX_ERR, (InfraMode ? 0xFF : 0x10)); */
#endif
}

static void _BeaconFunctionEnable(PADAPTER padapter, u8 Enable, u8 Linked)
{
	rtw_write8(padapter, REG_BCN_CTRL, DIS_TSF_UDT | EN_BCN_FUNCTION | DIS_BCNQ_SUB);
	rtw_write8(padapter, REG_RD_CTRL + 1, 0x6F);
}

static void rtl8710b_SetBeaconRelatedRegisters(PADAPTER padapter)
{
	u8 val8;
	u32 value32;
	PHAL_DATA_TYPE pHalData = GET_HAL_DATA(padapter);
	struct mlme_ext_priv *pmlmeext = &padapter->mlmeextpriv;
	struct mlme_ext_info *pmlmeinfo = &pmlmeext->mlmext_info;
	u32 bcn_ctrl_reg;

	/* reset TSF, enable update TSF, correcting TSF On Beacon */

	/* REG_BCN_INTERVAL */
	/* REG_BCNDMATIM */
	/* REG_ATIMWND */
	/* REG_TBTT_PROHIBIT */
	/* REG_DRVERLYINT */
	/* REG_BCN_MAX_ERR */
	/* REG_BCNTCFG //(0x510) */
	/* REG_DUAL_TSF_RST */
	/* REG_BCN_CTRL //(0x550) */


	bcn_ctrl_reg = REG_BCN_CTRL;
#ifdef CONFIG_CONCURRENT_MODE
	if (padapter->hw_port == HW_PORT1)
		bcn_ctrl_reg = REG_BCN_CTRL_1;
#endif

	/* */
	/* ATIM window */
	/* */
	rtw_write16(padapter, REG_ATIMWND, 2);

	/* */
	/* Beacon interval (in unit of TU). */
	/* */
	rtw_write16(padapter, REG_BCN_INTERVAL, pmlmeinfo->bcn_interval);

	rtl8710b_InitBeaconParameters(padapter);

	rtw_write8(padapter, REG_SLOT, 0x09);

	/* */
	/* Reset TSF Timer to zero, added by Roger. 2008.06.24 */
	/* */
	value32 = rtw_read32(padapter, REG_TCR);
	value32 &= ~TSFRST;
	rtw_write32(padapter, REG_TCR, value32);

	value32 |= TSFRST;
	rtw_write32(padapter, REG_TCR, value32);

	/* NOTE: Fix test chip's bug (about contention windows's randomness) */
	if (check_fwstate(&padapter->mlmepriv, WIFI_ADHOC_STATE | WIFI_ADHOC_MASTER_STATE | WIFI_AP_STATE | WIFI_MESH_STATE) == _TRUE) {
		rtw_write8(padapter, REG_RXTSF_OFFSET_CCK, 0x50);
		rtw_write8(padapter, REG_RXTSF_OFFSET_OFDM, 0x50);
	}

	_BeaconFunctionEnable(padapter, _TRUE, _TRUE);

	ResumeTxBeacon(padapter);
	val8 = rtw_read8(padapter, bcn_ctrl_reg);
	val8 |= DIS_BCNQ_SUB;
	rtw_write8(padapter, bcn_ctrl_reg, val8);
}

void hal_notch_filter_8710b(_adapter *adapter, bool enable)
{
	if (enable) {
		RTW_INFO("Enable notch filter\n");
		rtw_write8(adapter, rOFDM0_RxDSP + 1, rtw_read8(adapter, rOFDM0_RxDSP + 1) | BIT(1));
	} else {
		RTW_INFO("Disable notch filter\n");
		rtw_write8(adapter, rOFDM0_RxDSP + 1, rtw_read8(adapter, rOFDM0_RxDSP + 1) & ~BIT(1));
	}
}

u8 rtl8710b_MRateIdxToARFRId(PADAPTER padapter, u8 rate_idx)
{
	u8 ret = 0;
	enum rf_type rftype = (enum rf_type)GET_RF_TYPE(padapter);

	switch (rate_idx) {

	case RATR_INX_WIRELESS_NGB:
		if (rftype == RF_1T1R)
			ret = 1;
		else
			ret = 0;
		break;

	case RATR_INX_WIRELESS_N:
	case RATR_INX_WIRELESS_NG:
		if (rftype == RF_1T1R)
			ret = 5;
		else
			ret = 4;
		break;

	case RATR_INX_WIRELESS_NB:
		if (rftype == RF_1T1R)
			ret = 3;
		else
			ret = 2;
		break;

	case RATR_INX_WIRELESS_GB:
		ret = 6;
		break;

	case RATR_INX_WIRELESS_G:
		ret = 7;
		break;

	case RATR_INX_WIRELESS_B:
		ret = 8;
		break;

	case RATR_INX_WIRELESS_MC:
		if (padapter->mlmeextpriv.cur_wireless_mode & WIRELESS_11BG_24N)
			ret = 6;
		else
			ret = 7;
		break;
	case RATR_INX_WIRELESS_AC_N:
		if (rftype == RF_1T1R) /* || padapter->MgntInfo.VHTHighestOperaRate <= MGN_VHT1SS_MCS9) */
			ret = 10;
		else
			ret = 9;
		break;

	default:
		ret = 0;
		break;
	}

	return ret;
}

/*
 * Description: In normal chip, we should send some packet to Hw which will be used by Fw
 *			in FW LPS mode. The function is to fill the Tx descriptor of this packets, then
 *			Fw can tell Hw to send these packet derectly.
 * Added by tynli. 2009.10.15.
 *
 * type1:pspoll, type2:null */
void rtl8710b_fill_fake_txdesc(
	PADAPTER	padapter,
	u8			*pDesc,
	u32			BufferLen,
	u8			IsPsPoll,
	u8			IsBTQosNull,
	u8			bDataFrame)
{
	/* Clear all status */
	_rtw_memset(pDesc, 0, TXDESC_SIZE);

	SET_TX_DESC_OFFSET_8710B(pDesc, 0x28); /* Offset = 32 */

	SET_TX_DESC_PKT_SIZE_8710B(pDesc, BufferLen); /* Buffer size + command header */
	SET_TX_DESC_QUEUE_SEL_8710B(pDesc, QSLT_MGNT); /* Fixed queue of Mgnt queue */

	/* Set NAVUSEHDR to prevent Ps-poll AId filed to be changed to error vlaue by Hw. */
	if (_TRUE == IsPsPoll)
		SET_TX_DESC_NAV_USE_HDR_8710B(pDesc, 1);
	else {
		SET_TX_DESC_HWSEQ_EN_8710B(pDesc, 1); /* Hw set sequence number */
		SET_TX_DESC_HWSEQ_SEL_8710B(pDesc, 0);
	}

	if (_TRUE == IsBTQosNull)
		SET_TX_DESC_BT_INT_8710B(pDesc, 1);

	SET_TX_DESC_USE_RATE_8710B(pDesc, 1); /* use data rate which is set by Sw */

	SET_TX_DESC_TX_RATE_8710B(pDesc, DESC8710B_RATE1M);

	/* */
	/* Encrypt the data frame if under security mode excepct null data. Suggested by CCW. */
	/* */
	if (_TRUE == bDataFrame) {
		u32 EncAlg;

		EncAlg = padapter->securitypriv.dot11PrivacyAlgrthm;
		switch (EncAlg) {
		case _NO_PRIVACY_:
			SET_TX_DESC_SEC_TYPE_8710B(pDesc, 0x0);
			break;
		case _WEP40_:
		case _WEP104_:
		case _TKIP_:
			SET_TX_DESC_SEC_TYPE_8710B(pDesc, 0x1);
			break;
		case _SMS4_:
			SET_TX_DESC_SEC_TYPE_8710B(pDesc, 0x2);
			break;
		case _AES_:
			SET_TX_DESC_SEC_TYPE_8710B(pDesc, 0x3);
			break;
		default:
			SET_TX_DESC_SEC_TYPE_8710B(pDesc, 0x0);
			break;
		}
	}

#if defined(CONFIG_USB_HCI)
	/*
	 * USB interface drop packet if the checksum of descriptor isn't correct.
	 * Using this checksum can let hardware recovery from packet bulk out error (e.g. Cancel URC, Bulk out error.).
	 */
	rtl8710b_cal_txdesc_chksum((struct tx_desc *)pDesc);
#endif
}

void rtl8710b_InitAntenna_Selection(PADAPTER padapter)
{
	rtw_write8(padapter, REG_LEDCFG2, 0x82);
}

void rtl8710b_CheckAntenna_Selection(PADAPTER padapter)
{
#if 0
	PHAL_DATA_TYPE pHalData;
	u8 val;


	pHalData = GET_HAL_DATA(padapter);

	val = rtw_read8(padapter, REG_LEDCFG2);
	/* Let 8051 take control antenna setting */
	if (!(val & BIT(7))) {
		val |= BIT(7); /* DPDT_SEL_EN, 0x4C[23] */
		rtw_write8(padapter, REG_LEDCFG2, val);
	}
#endif
}
void rtl8710b_DeinitAntenna_Selection(PADAPTER padapter)
{
#if 0
	PHAL_DATA_TYPE pHalData;
	u8 val;


	pHalData = GET_HAL_DATA(padapter);
	val = rtw_read8(padapter, REG_LEDCFG2);
	/* Let 8051 take control antenna setting */
	val &= ~BIT(7); /* DPDT_SEL_EN, clear 0x4C[23] */
	rtw_write8(padapter, REG_LEDCFG2, val);
#endif
}

void init_hal_spec_8710b(_adapter *adapter)
{
	struct hal_spec_t *hal_spec = GET_HAL_SPEC(adapter);

	hal_spec->ic_name = "rtl8710b";
	hal_spec->macid_num = 16;
	hal_spec->sec_cam_ent_num = 32;
	hal_spec->sec_cap = SEC_CAP_CHK_BMC;
	hal_spec->rfpath_num_2g = 2;
	hal_spec->rfpath_num_5g = 0;
	hal_spec->max_tx_cnt = 1;
	hal_spec->tx_nss_num = 1;
	hal_spec->rx_nss_num = 1;
	hal_spec->band_cap = BAND_CAP_2G;
	hal_spec->bw_cap = BW_CAP_20M | BW_CAP_40M;
	hal_spec->port_num = 3;
	hal_spec->proto_cap = PROTO_CAP_11B | PROTO_CAP_11G | PROTO_CAP_11N;

	hal_spec->wl_func = 0
			    | WL_FUNC_P2P
			    | WL_FUNC_MIRACAST
			    | WL_FUNC_TDLS
			    ;
#if 0
	rtw_macid_ctl_init_sleep_reg(adapter_to_macidctl(adapter)
		, REG_MACID_SLEEP, 0, 0, 0);
#endif
}

void rtl8710b_init_default_value(PADAPTER padapter)
{
	PHAL_DATA_TYPE pHalData;
	u8 i;

	pHalData = GET_HAL_DATA(padapter);

	padapter->registrypriv.wireless_mode = WIRELESS_11BG_24N;

	/* init default value */
	pHalData->fw_ractrl = _FALSE;
	if (!adapter_to_pwrctl(padapter)->bkeepfwalive)
		pHalData->LastHMEBoxNum = 0;

	/* init phydm default value */
	pHalData->bIQKInitialized = _FALSE;

	/* init Efuse variables */
	pHalData->EfuseUsedBytes = 0;
	pHalData->EfuseUsedPercentage = 0;
#ifdef HAL_EFUSE_MEMORY
	pHalData->EfuseHal.fakeEfuseBank = 0;
	pHalData->EfuseHal.fakeEfuseUsedBytes = 0;
	_rtw_memset(pHalData->EfuseHal.fakeEfuseContent, 0xFF, EFUSE_MAX_HW_SIZE);
	_rtw_memset(pHalData->EfuseHal.fakeEfuseInitMap, 0xFF, EFUSE_MAX_MAP_LEN);
	_rtw_memset(pHalData->EfuseHal.fakeEfuseModifiedMap, 0xFF, EFUSE_MAX_MAP_LEN);
#endif
}

u8 GetEEPROMSize8710B(PADAPTER padapter)
{
	u8 size = 0;
	u32	value32;

	value32 = hal_query_syson_reg_8710b(padapter, REG_SYS_EEPROM_CTRL0, 0xFFFFFFFF);
	/* 6: EEPROM used is 93C46, 4: boot from E-Fuse. */
	size = (value32 & BOOT_FROM_EEPROM) ? 6 : 4;

	RTW_INFO("EEPROM type is %s\n", size == 4 ? "E-FUSE" : "93C46");

	return size;
}

#if defined(CONFIG_USB_HCI) || defined(CONFIG_SDIO_HCI) || defined(CONFIG_GSPI_HCI)
void _DisableGPIO(PADAPTER	padapter)
{
	/***************************************
	j. GPIO_PIN_CTRL 0x44[31:0]=0x000
	k.Value = GPIO_PIN_CTRL[7:0]
	l. GPIO_PIN_CTRL 0x44[31:0] = 0x00FF0000 | (value <<8);
	m. GPIO_MUXCFG 0x42 [15:0] = 0x0780
	n. LEDCFG 0x4C[15:0] = 0x8080
	***************************************/
	u8	value8;
	u16	value16;
	u32	value32;
	u32	u4bTmp;


	/* 1. Disable GPIO[7:0] */
	rtw_write16(padapter, REG_GPIO_PIN_CTRL + 2, 0x0000);
	value32 = rtw_read32(padapter, REG_GPIO_PIN_CTRL) & 0xFFFF00FF;
	u4bTmp = value32 & 0x000000FF;
	value32 |= ((u4bTmp << 8) | 0x00FF0000);
	rtw_write32(padapter, REG_GPIO_PIN_CTRL, value32);


	/* 2. Disable GPIO[10:8] */
	rtw_write8(padapter, REG_MAC_PINMUX_CFG, 0x00);
	value16 = rtw_read16(padapter, REG_GPIO_IO_SEL) & 0xFF0F;
	value8 = (u8)(value16 & 0x000F);
	value16 |= ((value8 << 4) | 0x0780);
	rtw_write16(padapter, REG_GPIO_IO_SEL, value16);


	/* 3. Disable LED0 & 1 */
	rtw_write16(padapter, REG_LEDCFG0, 0x8080);

} /* end of _DisableGPIO() */

void _DisableRFAFEAndResetBB8710B(PADAPTER padapter)
{
	/**************************************
	a.	TXPAUSE 0x522[7:0] = 0xFF
	b.	RF path 0 offset 0x00 = 0x00
	c.	APSD_CTRL 0x600[7:0] = 0x40
	d.	SYS_FUNC_EN 0x02[7:0] = 0x16
	e.	SYS_FUNC_EN 0x02[7:0] = 0x14
	***************************************/
	enum rf_path eRFPath = RF_PATH_A, value8 = 0;

	rtw_write8(padapter, REG_TXPAUSE, 0xFF);

	phy_set_rf_reg(padapter, eRFPath, 0x0, bMaskByte0, 0x0);

	value8 |= APSDOFF;
	rtw_write8(padapter, REG_APSD_CTRL, value8);/* 0x40 */

	/* Set BB reset at first */
	value8 = 0;
	value8 |= (FEN_USBD | FEN_USBA | FEN_BB_GLB_RSTn);
	rtw_write8(padapter, REG_SYS_FUNC_EN, value8); /* 0x16 */

	/* Set global reset. */
	value8 &= ~FEN_BB_GLB_RSTn;
	rtw_write8(padapter, REG_SYS_FUNC_EN, value8); /* 0x14 */

	/* 2010/08/12 MH We need to set BB/GLBAL reset to save power for SS mode. */

}

void _DisableRFAFEAndResetBB(PADAPTER padapter)
{
	_DisableRFAFEAndResetBB8710B(padapter);
}

void _DisableAnalog(PADAPTER padapter, BOOLEAN bWithoutHWSM)
{
	HAL_DATA_TYPE	*pHalData	= GET_HAL_DATA(padapter);
	u16 value16 = 0;
	u8 value8 = 0;


	if (bWithoutHWSM) {
		/*****************************
		n.	LDOA15_CTRL 0x20[7:0] = 0x04
		o.	LDOV12D_CTRL 0x21[7:0] = 0x54
		r.	When driver call disable, the ASIC will turn off remaining clock automatically
		******************************/

		rtw_write8(padapter, REG_LDOA15_CTRL, 0x04);
		/* rtw_write8(padapter, REG_LDOV12D_CTRL, 0x54); */

		value8 = rtw_read8(padapter, REG_LDOV12D_CTRL);
		value8 &= (~LDV12_EN);
		rtw_write8(padapter, REG_LDOV12D_CTRL, value8);
	}

	/*****************************
	h.	SPS0_CTRL 0x11[7:0] = 0x23
	i.	APS_FSMCO 0x04[15:0] = 0x4802
	******************************/
	value8 = 0x23;

	rtw_write8(padapter, REG_SPS0_CTRL, value8);

	if (bWithoutHWSM) {
		/* value16 |= (APDM_HOST | AFSM_HSUS |PFM_ALDN); */
		/* 2010/08/31 According to Filen description, we need to use HW to shut down 8051 automatically. */
		/* Because suspend operation need the asistance of 8051 to wait for 3ms. */
		value16 |= (APDM_HOST | AFSM_HSUS | PFM_ALDN);
	} else
		value16 |= (APDM_HOST | AFSM_HSUS | PFM_ALDN);

	rtw_write16(padapter, REG_APS_FSMCO, value16);/* 0x4802 */

	rtw_write8(padapter, REG_RSV_CTRL, 0x0e);

#if 0
	/* tynli_test for suspend mode. */
	if (!bWithoutHWSM)
		rtw_write8(padapter, 0xfe10, 0x19);
#endif

}

#endif /* CONFIG_USB_HCI || CONFIG_SDIO_HCI || CONFIG_GSPI_HCI */

#if 0
void
Hal_InitPGData(
	PADAPTER	padapter,
	u8			*PROMContent)
{

	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	u32			i;
	u16			value16;

	if (_FALSE == pHalData->bautoload_fail_flag) {
		/* autoload OK.
		*		if (IS_BOOT_FROM_EEPROM(padapter)) */
		if (_TRUE == pHalData->EepromOrEfuse) {
			/* Read all Content from EEPROM or EFUSE. */
			for (i = 0; i < HWSET_MAX_SIZE_8710B; i += 2) {
				/*				value16 = EF2Byte(ReadEEprom(padapter, (u2Byte) (i>>1)));
				 *				*((u16*)(&PROMContent[i])) = value16; */
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

static void
Hal_EEValueCheck(
	IN		u8		EEType,
	IN		PVOID		pInValue,
	OUT		PVOID		pOutValue
)
{
	switch (EEType) {
	case EETYPE_TX_PWR: {
		u8	*pIn, *pOut;

		pIn = (u8 *)pInValue;
		pOut = (u8 *)pOutValue;
		if (*pIn <= 63)
			*pOut = *pIn;
		else {
			*pOut = EEPROM_Default_TxPowerLevel;
		}
	}
	break;
	default:
		break;
	}
}

void
Hal_EfuseParseTxPowerInfo_8710B(
	IN	PADAPTER		padapter,
	IN	u8			*PROMContent,
	IN	BOOLEAN			AutoLoadFail
)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	TxPowerInfo24G	pwrInfo24G;

	hal_load_txpwr_info(padapter, &pwrInfo24G, NULL, PROMContent);

	/* 2010/10/19 MH Add Regulator recognize for CU. */
	if (!AutoLoadFail) {
		pHalData->EEPROMRegulatory = (PROMContent[EEPROM_RF_BOARD_OPTION_8710B] & 0x7);	/* bit0~2 */
		if (PROMContent[EEPROM_RF_BOARD_OPTION_8710B] == 0xFF)
			pHalData->EEPROMRegulatory = (EEPROM_DEFAULT_BOARD_OPTION & 0x7);	/* bit0~2 */
	} else
		pHalData->EEPROMRegulatory = 0;
}

VOID
Hal_EfuseParseBoardType_8710B(
	IN	PADAPTER	Adapter,
	IN	u8			*PROMContent,
	IN	BOOLEAN		AutoloadFail
)
{


	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);

	if (!AutoloadFail) {
		pHalData->InterfaceSel = (PROMContent[EEPROM_RF_BOARD_OPTION_8710B] & 0xE0) >> 5;
		if (PROMContent[EEPROM_RF_BOARD_OPTION_8710B] == 0xFF)
			pHalData->InterfaceSel = (EEPROM_DEFAULT_BOARD_OPTION & 0xE0) >> 5;
	} else
		pHalData->InterfaceSel = 0;

}

VOID
Hal_EfuseParsePackageType_8710B(
	IN	PADAPTER		padapter,
	IN	u8				*hwinfo,
	IN	BOOLEAN		AutoLoadFail
)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	u1Byte			package;
	u8 efuseContent;

	Efuse_PowerSwitch(padapter, _FALSE, _TRUE);
	efuse_OneByteRead(padapter, 0x1FB, &efuseContent, FALSE);
	RTW_INFO("%s phy efuse read 0x1FB =%x\n", __func__, efuseContent);
	Efuse_PowerSwitch(padapter, _FALSE, _FALSE);

	package = efuseContent & 0x7;
	switch (package) {
	case 0x4:
		pHalData->PackageType = PACKAGE_TFBGA79;
		break;
	case 0x5:
		pHalData->PackageType = PACKAGE_TFBGA90;
		break;
	case 0x6:
		pHalData->PackageType = PACKAGE_QFN68;
		break;
	case 0x7:
		pHalData->PackageType = PACKAGE_TFBGA80;
		break;

	default:
		pHalData->PackageType = PACKAGE_DEFAULT;
		break;
	}

	RTW_INFO("PackageType = 0x%X\n", pHalData->PackageType);

#ifdef CONFIG_SDIO_HCI
	/* RTL8703AS: 0x1FB[5:4] 2b'10 */
	/* RTL8710BS: 0x1FB[5:4] 2b'11 */
	if ((efuseContent & 0x30) == 0x20) {
		padapter->registrypriv.bw_mode &= 0xF0;
		RTW_INFO("This is the case of 8703AS\n");
	}
#endif
}

VOID
Hal_EfuseParseChnlPlan_8710B(
	IN	PADAPTER		padapter,
	IN	u8			*hwinfo,
	IN	BOOLEAN			AutoLoadFail
)
{
	hal_com_config_channel_plan(
		padapter
		, hwinfo ? &hwinfo[EEPROM_COUNTRY_CODE_8710B] : NULL
		, hwinfo ? hwinfo[EEPROM_ChannelPlan_8710B] : 0xFF
		, padapter->registrypriv.alpha2
		, padapter->registrypriv.channel_plan
		, RTW_CHPLAN_WORLD_NULL
		, AutoLoadFail
	);
}

VOID
Hal_EfuseParseCustomerID_8710B(
	IN	PADAPTER		padapter,
	IN	u8			*hwinfo,
	IN	BOOLEAN			AutoLoadFail
)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);

	if (!AutoLoadFail)
		pHalData->EEPROMCustomerID = hwinfo[EEPROM_CustomID_8710B];
	else
		pHalData->EEPROMCustomerID = 0;
}

VOID
Hal_EfuseParseAntennaDiversity_8710B(
	IN	PADAPTER		padapter,
	IN	u8				*hwinfo,
	IN	BOOLEAN			AutoLoadFail
)
{
#ifdef CONFIG_ANTENNA_DIVERSITY
	PHAL_DATA_TYPE		pHalData = GET_HAL_DATA(padapter);
	struct registry_priv	*registry_par = &padapter->registrypriv;
	u8				get_efuse_div_type;

	if (pHalData->EEPROMBluetoothAntNum == Ant_x1)
		pHalData->AntDivCfg = 0;
	else {
		if (registry_par->antdiv_cfg == 2) /* 0:OFF , 1:ON, 2:By EFUSE */
			pHalData->AntDivCfg = 1;
		else
			pHalData->AntDivCfg = registry_par->antdiv_cfg;
	}

	pHalData->TRxAntDivType = S0S1_TRX_HW_ANTDIV; /* it's the only diversity-type for 8710B*/
	pHalData->with_extenal_ant_switch = ((hwinfo[EEPROM_RF_BT_SETTING_8710B] & BIT7) >> 7);

	if (pHalData->AntDivCfg != 0) {

		get_efuse_div_type = hwinfo[EEPROM_RFE_OPTION_8710B];

		if (get_efuse_div_type == 0x11) {
			pHalData->b_fix_tx_ant = NO_FIX_TX_ANT;
		} else if (get_efuse_div_type == 0x13) {
			pHalData->b_fix_tx_ant = FIX_TX_AT_MAIN;/* RX diversity only*/
		} else
			pHalData->AntDivCfg = FALSE;
	}

	RTW_INFO("%s: AntDivCfg=%d, AntDivType=%d\n",
		 __FUNCTION__, pHalData->AntDivCfg, pHalData->TRxAntDivType);
#endif
}

VOID
Hal_EfuseParseXtal_8710B(
	IN	PADAPTER		padapter,
	IN	u8			*hwinfo,
	IN	BOOLEAN		AutoLoadFail
)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);

	if (!AutoLoadFail) {
		pHalData->crystal_cap = hwinfo[EEPROM_XTAL_8710B];
		if (pHalData->crystal_cap == 0xFF)
			pHalData->crystal_cap = EEPROM_Default_CrystalCap_8710B;	   /* what value should 8812 set? */
	} else
		pHalData->crystal_cap = EEPROM_Default_CrystalCap_8710B;
}


void
Hal_EfuseParseThermalMeter_8710B(
	PADAPTER	padapter,
	u8			*PROMContent,
	u8			AutoLoadFail
)
{
	PHAL_DATA_TYPE	pHalData = GET_HAL_DATA(padapter);

	/* */
	/* ThermalMeter from EEPROM */
	/* */
	if (_FALSE == AutoLoadFail)
		pHalData->eeprom_thermal_meter = PROMContent[EEPROM_THERMAL_METER_8710B];
	else
		pHalData->eeprom_thermal_meter = EEPROM_Default_ThermalMeter_8710B;

	if ((pHalData->eeprom_thermal_meter == 0xff) || (_TRUE == AutoLoadFail)) {
		pHalData->odmpriv.rf_calibrate_info.is_apk_thermal_meter_ignore = _TRUE;
		pHalData->eeprom_thermal_meter = EEPROM_Default_ThermalMeter_8710B;
	}

}

void Hal_ReadRFGainOffset(
	IN		PADAPTER		Adapter,
	IN		u8			*PROMContent,
	IN		BOOLEAN		AutoloadFail)
{
#ifdef CONFIG_RF_POWER_TRIM
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	struct kfree_data_t *kfree_data = &pHalData->kfree_data;
	u8 pg_pwrtrim = 0xFF, pg_therm = 0xFF;

	efuse_OneByteRead(Adapter,
		PPG_BB_GAIN_2G_TX_OFFSET_8710B, &pg_pwrtrim, _FALSE);
	efuse_OneByteRead(Adapter,
		PPG_THERMAL_OFFSET_8710B, &pg_therm, _FALSE);

	if (pg_pwrtrim != 0xFF) {
		kfree_data->bb_gain[BB_GAIN_2G][PPG_8710B_S1]
			= KFREE_BB_GAIN_2G_TX_OFFSET(pg_pwrtrim & PPG_BB_GAIN_2G_TX_OFFSET_MASK);
		kfree_data->bb_gain[BB_GAIN_2G][PPG_8710B_S0]
			= KFREE_BB_GAIN_2G_TXB_OFFSET(pg_pwrtrim & PPG_BB_GAIN_2G_TXB_OFFSET_MASK);
		kfree_data->flag |= KFREE_FLAG_ON;
	}

	if (pg_therm != 0xFF) {
		kfree_data->thermal
			= KFREE_THERMAL_OFFSET(pg_therm  & PPG_THERMAL_OFFSET_MASK);
		/* kfree_data->flag |= KFREE_FLAG_THERMAL_K_ON; */ /* Default disable thermel kfree by realsil Alan 20160428 */
	}

	if (kfree_data->flag & KFREE_FLAG_THERMAL_K_ON)
		pHalData->eeprom_thermal_meter += kfree_data->thermal;

	RTW_INFO("kfree Pwr Trim flag:%u\n", kfree_data->flag);
	if (kfree_data->flag & KFREE_FLAG_ON) {
		RTW_INFO("bb_gain(S1):%d\n", kfree_data->bb_gain[BB_GAIN_2G][PPG_8710B_S1]);
		RTW_INFO("bb_gain(S0):%d\n", kfree_data->bb_gain[BB_GAIN_2G][PPG_8710B_S0]);
	}
	if (kfree_data->flag & KFREE_FLAG_THERMAL_K_ON)
		RTW_INFO("thermal:%d\n", kfree_data->thermal);
#endif /*CONFIG_RF_POWER_TRIM */
}
#endif

u8
BWMapping_8710B(
	IN	PADAPTER		Adapter,
	IN	struct pkt_attrib	*pattrib
)
{
	u8	BWSettingOfDesc = 0;
	PHAL_DATA_TYPE	pHalData = GET_HAL_DATA(Adapter);

	/* RTW_INFO("BWMapping pHalData->current_channel_bw %d, pattrib->bwmode %d\n",pHalData->current_channel_bw,pattrib->bwmode); */

	if (pHalData->current_channel_bw == CHANNEL_WIDTH_80) {
		if (pattrib->bwmode == CHANNEL_WIDTH_80)
			BWSettingOfDesc = 2;
		else if (pattrib->bwmode == CHANNEL_WIDTH_40)
			BWSettingOfDesc = 1;
		else
			BWSettingOfDesc = 0;
	} else if (pHalData->current_channel_bw == CHANNEL_WIDTH_40) {
		if ((pattrib->bwmode == CHANNEL_WIDTH_40) || (pattrib->bwmode == CHANNEL_WIDTH_80))
			BWSettingOfDesc = 1;
		else
			BWSettingOfDesc = 0;
	} else
		BWSettingOfDesc = 0;

	/* if(pTcb->bBTTxPacket) */
	/*	BWSettingOfDesc = 0; */

	return BWSettingOfDesc;
}

u8	SCMapping_8710B(PADAPTER Adapter, struct pkt_attrib *pattrib)
{
	u8	SCSettingOfDesc = 0;
	PHAL_DATA_TYPE	pHalData = GET_HAL_DATA(Adapter);

	/* RTW_INFO("SCMapping: pHalData->current_channel_bw %d, pHalData->nCur80MhzPrimeSC %d, pHalData->nCur40MhzPrimeSC %d\n",pHalData->current_channel_bw,pHalData->nCur80MhzPrimeSC,pHalData->nCur40MhzPrimeSC); */

	if (pHalData->current_channel_bw == CHANNEL_WIDTH_80) {
		if (pattrib->bwmode == CHANNEL_WIDTH_80)
			SCSettingOfDesc = VHT_DATA_SC_DONOT_CARE;
		else if (pattrib->bwmode == CHANNEL_WIDTH_40) {
			if (pHalData->nCur80MhzPrimeSC == HAL_PRIME_CHNL_OFFSET_LOWER)
				SCSettingOfDesc = VHT_DATA_SC_40_LOWER_OF_80MHZ;
			else if (pHalData->nCur80MhzPrimeSC == HAL_PRIME_CHNL_OFFSET_UPPER)
				SCSettingOfDesc = VHT_DATA_SC_40_UPPER_OF_80MHZ;
			else
				RTW_INFO("SCMapping: DONOT CARE Mode Setting\n");
		} else {
			if ((pHalData->nCur40MhzPrimeSC == HAL_PRIME_CHNL_OFFSET_LOWER) && (pHalData->nCur80MhzPrimeSC == HAL_PRIME_CHNL_OFFSET_LOWER))
				SCSettingOfDesc = VHT_DATA_SC_20_LOWEST_OF_80MHZ;
			else if ((pHalData->nCur40MhzPrimeSC == HAL_PRIME_CHNL_OFFSET_UPPER) && (pHalData->nCur80MhzPrimeSC == HAL_PRIME_CHNL_OFFSET_LOWER))
				SCSettingOfDesc = VHT_DATA_SC_20_LOWER_OF_80MHZ;
			else if ((pHalData->nCur40MhzPrimeSC == HAL_PRIME_CHNL_OFFSET_LOWER) && (pHalData->nCur80MhzPrimeSC == HAL_PRIME_CHNL_OFFSET_UPPER))
				SCSettingOfDesc = VHT_DATA_SC_20_UPPER_OF_80MHZ;
			else if ((pHalData->nCur40MhzPrimeSC == HAL_PRIME_CHNL_OFFSET_UPPER) && (pHalData->nCur80MhzPrimeSC == HAL_PRIME_CHNL_OFFSET_UPPER))
				SCSettingOfDesc = VHT_DATA_SC_20_UPPERST_OF_80MHZ;
			else
				RTW_INFO("SCMapping: DONOT CARE Mode Setting\n");
		}
	} else if (pHalData->current_channel_bw == CHANNEL_WIDTH_40) {
		/* RTW_INFO("SCMapping: HT Case: pHalData->current_channel_bw %d, pHalData->nCur40MhzPrimeSC %d\n",pHalData->current_channel_bw,pHalData->nCur40MhzPrimeSC); */

		if (pattrib->bwmode == CHANNEL_WIDTH_40)
			SCSettingOfDesc = VHT_DATA_SC_DONOT_CARE;
		else if (pattrib->bwmode == CHANNEL_WIDTH_20) {
			if (pHalData->nCur40MhzPrimeSC == HAL_PRIME_CHNL_OFFSET_UPPER)
				SCSettingOfDesc = VHT_DATA_SC_20_UPPER_OF_80MHZ;
			else if (pHalData->nCur40MhzPrimeSC == HAL_PRIME_CHNL_OFFSET_LOWER)
				SCSettingOfDesc = VHT_DATA_SC_20_LOWER_OF_80MHZ;
			else
				SCSettingOfDesc = VHT_DATA_SC_DONOT_CARE;
		}
	} else
		SCSettingOfDesc = VHT_DATA_SC_DONOT_CARE;

	return SCSettingOfDesc;
}

#if defined(CONFIG_CONCURRENT_MODE)
void fill_txdesc_force_bmc_camid(struct pkt_attrib *pattrib, u8 *ptxdesc)
{
	if ((pattrib->encrypt > 0) && (!pattrib->bswenc)
	    && (pattrib->bmc_camid != INVALID_SEC_MAC_CAM_ID)) {

		SET_TX_DESC_EN_DESC_ID_8710B(ptxdesc, 1);
		SET_TX_DESC_MACID_8710B(ptxdesc, pattrib->bmc_camid);
	}
}
#endif
void fill_txdesc_bmc_tx_rate(struct pkt_attrib *pattrib, u8 *ptxdesc)
{
	SET_TX_DESC_USE_RATE_8710B(ptxdesc, 1);
	SET_TX_DESC_TX_RATE_8710B(ptxdesc, MRateToHwRate(pattrib->rate));
	SET_TX_DESC_DISABLE_FB_8710B(ptxdesc, 1);
}

static u8 fill_txdesc_sectype(struct pkt_attrib *pattrib)
{
	u8 sectype = 0;

	if ((pattrib->encrypt > 0) && !pattrib->bswenc) {
		switch (pattrib->encrypt) {
		/* SEC_TYPE */
		case _WEP40_:
		case _WEP104_:
		case _TKIP_:
		case _TKIP_WTMIC_:
			sectype = 1;
			break;

#ifdef CONFIG_WAPI_SUPPORT
		case _SMS4_:
			sectype = 2;
			break;
#endif
		case _AES_:
			sectype = 3;
			break;

		case _NO_PRIVACY_:
		default:
			break;
		}
	}
	return sectype;
}

static void fill_txdesc_vcs_8710b(PADAPTER padapter, struct pkt_attrib *pattrib, u8 *ptxdesc)
{
	/* RTW_INFO("cvs_mode=%d\n", pattrib->vcs_mode); */

	if (pattrib->vcs_mode) {
		switch (pattrib->vcs_mode) {
		case RTS_CTS:
			SET_TX_DESC_RTS_ENABLE_8710B(ptxdesc, 1);
			SET_TX_DESC_HW_RTS_ENABLE_8710B(ptxdesc, 1);
			break;

		case CTS_TO_SELF:
			SET_TX_DESC_CTS2SELF_8710B(ptxdesc, 1);
			break;

		case NONE_VCS:
		default:
			break;
		}

		SET_TX_DESC_RTS_RATE_8710B(ptxdesc, 8); /* RTS Rate=24M */
		SET_TX_DESC_RTS_RATE_FB_LIMIT_8710B(ptxdesc, 0xF);

		if (padapter->mlmeextpriv.mlmext_info.preamble_mode == PREAMBLE_SHORT)
			SET_TX_DESC_RTS_SHORT_8710B(ptxdesc, 1);

		/* Set RTS BW */
		if (pattrib->ht_en)
			SET_TX_DESC_RTS_SC_8710B(ptxdesc, SCMapping_8710B(padapter, pattrib));
	}
}

static void fill_txdesc_phy_8710b(PADAPTER padapter, struct pkt_attrib *pattrib, u8 *ptxdesc)
{
	/* RTW_INFO("bwmode=%d, ch_off=%d\n", pattrib->bwmode, pattrib->ch_offset); */

	if (pattrib->ht_en) {
		SET_TX_DESC_DATA_BW_8710B(ptxdesc, BWMapping_8710B(padapter, pattrib));
		SET_TX_DESC_DATA_SC_8710B(ptxdesc, SCMapping_8710B(padapter, pattrib));
	}
}

static void rtl8710b_fill_default_txdesc(
	struct xmit_frame *pxmitframe,
	u8 *pbuf)
{
	PADAPTER padapter;
	HAL_DATA_TYPE *pHalData;
	struct mlme_ext_priv *pmlmeext;
	struct mlme_ext_info *pmlmeinfo;
	struct pkt_attrib *pattrib;
	s32 bmcst;

	_rtw_memset(pbuf, 0, TXDESC_SIZE);

	padapter = pxmitframe->padapter;
	pHalData = GET_HAL_DATA(padapter);
	pmlmeext = &padapter->mlmeextpriv;
	pmlmeinfo = &(pmlmeext->mlmext_info);

	pattrib = &pxmitframe->attrib;
	bmcst = IS_MCAST(pattrib->ra);

	if (pHalData->rf_type == RF_1T1R)
		SET_TX_DESC_PATH_A_EN_8710B(pbuf, 1);

	if (pxmitframe->frame_tag == DATA_FRAMETAG) {
		u8 drv_userate = 0;

		SET_TX_DESC_MACID_8710B(pbuf, pattrib->mac_id);
		SET_TX_DESC_RATE_ID_8710B(pbuf, pattrib->raid);
		SET_TX_DESC_QUEUE_SEL_8710B(pbuf, pattrib->qsel);
		SET_TX_DESC_SEQ_8710B(pbuf, pattrib->seqnum);

		SET_TX_DESC_SEC_TYPE_8710B(pbuf, fill_txdesc_sectype(pattrib));
#if defined(CONFIG_CONCURRENT_MODE)
		if (bmcst)
			fill_txdesc_force_bmc_camid(pattrib, pbuf);
#endif
		fill_txdesc_vcs_8710b(padapter, pattrib, pbuf);

#ifdef CONFIG_P2P
		if (!rtw_p2p_chk_state(&padapter->wdinfo, P2P_STATE_NONE)) {
			if (pattrib->icmp_pkt == 1 && padapter->registrypriv.wifi_spec == 1)
				drv_userate = 1;
		}
#endif

		if ((pattrib->ether_type != 0x888e) &&
		    (pattrib->ether_type != 0x0806) &&
		    (pattrib->ether_type != 0x88B4) &&
		    (pattrib->dhcp_pkt != 1) &&
		    (drv_userate != 1)
#ifdef CONFIG_AUTO_AP_MODE
		    && (pattrib->pctrl != _TRUE)
#endif
		   ) {
			/* Non EAP & ARP & DHCP type data packet */

			if (pattrib->ampdu_en == _TRUE) {
				SET_TX_DESC_AGG_ENABLE_8710B(pbuf, 1);
				SET_TX_DESC_MAX_AGG_NUM_8710B(pbuf, 0x1F);
				SET_TX_DESC_AMPDU_DENSITY_8710B(pbuf, pattrib->ampdu_spacing);
			} else
				SET_TX_DESC_BK_8710B(pbuf, 1);

			fill_txdesc_phy_8710b(padapter, pattrib, pbuf);

			SET_TX_DESC_DATA_RATE_FB_LIMIT_8710B(pbuf, 0x1F);

			if (pHalData->fw_ractrl == _FALSE) {
				SET_TX_DESC_USE_RATE_8710B(pbuf, 1);

				if (pHalData->INIDATA_RATE[pattrib->mac_id] & BIT(7))
					SET_TX_DESC_DATA_SHORT_8710B(pbuf, 1);

				SET_TX_DESC_TX_RATE_8710B(pbuf, pHalData->INIDATA_RATE[pattrib->mac_id] & 0x7F);
			}
			if (bmcst)
				fill_txdesc_bmc_tx_rate(pattrib, pbuf);

			/* modify data rate by iwpriv */
			if (padapter->fix_rate != 0xFF) {
				SET_TX_DESC_USE_RATE_8710B(pbuf, 1);
				if (padapter->fix_rate & BIT(7))
					SET_TX_DESC_DATA_SHORT_8710B(pbuf, 1);
				SET_TX_DESC_TX_RATE_8710B(pbuf, padapter->fix_rate & 0x7F);
				if (!padapter->data_fb)
					SET_TX_DESC_DISABLE_FB_8710B(pbuf, 1);
			}

			if (pattrib->stbc)
				SET_TX_DESC_DATA_STBC_8710B(pbuf, 1);

#ifdef CONFIG_CMCC_TEST
			SET_TX_DESC_DATA_SHORT_8710B(pbuf, 1); /* use cck short premble */
#endif
		} else {
			/* EAP data packet and ARP packet. */
			/* Use the 1M data rate to send the EAP/ARP packet. */
			/* This will maybe make the handshake smooth. */

			SET_TX_DESC_BK_8710B(pbuf, 1);
			SET_TX_DESC_USE_RATE_8710B(pbuf, 1);
			if (pmlmeinfo->preamble_mode == PREAMBLE_SHORT)
				SET_TX_DESC_DATA_SHORT_8710B(pbuf, 1);
			SET_TX_DESC_TX_RATE_8710B(pbuf, MRateToHwRate(pmlmeext->tx_rate));

			RTW_INFO(FUNC_ADPT_FMT ": SP Packet(0x%04X) rate=0x%x\n",
				FUNC_ADPT_ARG(padapter), pattrib->ether_type, MRateToHwRate(pmlmeext->tx_rate));
		}

#if defined(CONFIG_USB_TX_AGGREGATION) || defined(CONFIG_SDIO_HCI) || defined(CONFIG_GSPI_HCI)
		SET_TX_DESC_USB_TXAGG_NUM_8710B(pbuf, pxmitframe->agg_num);
#endif

#ifdef CONFIG_TDLS
#ifdef CONFIG_XMIT_ACK
		/* CCX-TXRPT ack for xmit mgmt frames. */
		if (pxmitframe->ack_report) {
#ifdef DBG_CCX
			RTW_INFO("%s set spe_rpt\n", __func__);
#endif
			SET_TX_DESC_CCX_8710B(pbuf, 1);
			SET_TX_DESC_SW_DEFINE_8710B(pbuf, (u8)(GET_PRIMARY_ADAPTER(padapter)->xmitpriv.seq_no));
		}
#endif /* CONFIG_XMIT_ACK */
#endif
	} else if (pxmitframe->frame_tag == MGNT_FRAMETAG) {

		SET_TX_DESC_MACID_8710B(pbuf, pattrib->mac_id);
		SET_TX_DESC_QUEUE_SEL_8710B(pbuf, pattrib->qsel);
		SET_TX_DESC_RATE_ID_8710B(pbuf, pattrib->raid);
		SET_TX_DESC_SEQ_8710B(pbuf, pattrib->seqnum);
		SET_TX_DESC_USE_RATE_8710B(pbuf, 1);

		SET_TX_DESC_MBSSID_8710B(pbuf, pattrib->mbssid & 0xF);

		SET_TX_DESC_RETRY_LIMIT_ENABLE_8710B(pbuf, 1);
		if (pattrib->retry_ctrl == _TRUE)
			SET_TX_DESC_DATA_RETRY_LIMIT_8710B(pbuf, 6);
		else
			SET_TX_DESC_DATA_RETRY_LIMIT_8710B(pbuf, 12);

		SET_TX_DESC_TX_RATE_8710B(pbuf, MRateToHwRate(pattrib->rate));

#ifdef CONFIG_XMIT_ACK
		/* CCX-TXRPT ack for xmit mgmt frames. */
		if (pxmitframe->ack_report) {
#ifdef DBG_CCX
			RTW_INFO("%s set spe_rpt\n", __FUNCTION__);
#endif
			SET_TX_DESC_CCX_8710B(pbuf, 1);
			SET_TX_DESC_SW_DEFINE_8710B(pbuf, (u8)(GET_PRIMARY_ADAPTER(padapter)->xmitpriv.seq_no));
		}
#endif /* CONFIG_XMIT_ACK */
	} else if (pxmitframe->frame_tag == TXAGG_FRAMETAG) {
	}
#ifdef CONFIG_MP_INCLUDED
	else if (pxmitframe->frame_tag == MP_FRAMETAG) {
		fill_txdesc_for_mp(padapter, pbuf);
	}
#endif
	else {

		SET_TX_DESC_MACID_8710B(pbuf, pattrib->mac_id);
		SET_TX_DESC_RATE_ID_8710B(pbuf, pattrib->raid);
		SET_TX_DESC_QUEUE_SEL_8710B(pbuf, pattrib->qsel);
		SET_TX_DESC_SEQ_8710B(pbuf, pattrib->seqnum);
		SET_TX_DESC_USE_RATE_8710B(pbuf, 1);
		SET_TX_DESC_TX_RATE_8710B(pbuf, MRateToHwRate(pmlmeext->tx_rate));
	}

	SET_TX_DESC_PKT_SIZE_8710B(pbuf, pattrib->last_txcmdsz);

	{
		u8 pkt_offset, offset;

		pkt_offset = 0;
		offset = TXDESC_SIZE;
#ifdef CONFIG_USB_HCI
		pkt_offset = pxmitframe->pkt_offset;
		offset += (pxmitframe->pkt_offset >> 3);
#endif /* CONFIG_USB_HCI */

#ifdef CONFIG_TX_EARLY_MODE
		if (pxmitframe->frame_tag == DATA_FRAMETAG) {
			pkt_offset = 1;
			offset += EARLY_MODE_INFO_SIZE;
		}
#endif /* CONFIG_TX_EARLY_MODE */

		SET_TX_DESC_PKT_OFFSET_8710B(pbuf, pkt_offset);
		SET_TX_DESC_OFFSET_8710B(pbuf, offset);
	}

	if (bmcst)
		SET_TX_DESC_BMC_8710B(pbuf, 1);

	/* 2009.11.05. tynli_test. Suggested by SD4 Filen for FW LPS. */
	/* (1) The sequence number of each non-Qos frame / broadcast / multicast / */
	/* mgnt frame should be controlled by Hw because Fw will also send null data */
	/* which we cannot control when Fw LPS enable. */
	/* --> default enable non-Qos data sequense number. 2010.06.23. by tynli. */
	/* (2) Enable HW SEQ control for beacon packet, because we use Hw beacon. */
	/* (3) Use HW Qos SEQ to control the seq num of Ext port non-Qos packets. */
	/* 2010.06.23. Added by tynli. */
	if (!pattrib->qos_en)
		SET_TX_DESC_HWSEQ_EN_8710B(pbuf, 1);

#ifdef CONFIG_ANTENNA_DIVERSITY
	if (!bmcst && pattrib->psta)
		odm_set_tx_ant_by_tx_info(adapter_to_phydm(padapter), pbuf, pattrib->psta->cmn.mac_id);
#endif
}

/*
 *	Description:
 *
 *	Parameters:
 *		pxmitframe	xmitframe
 *		pbuf		where to fill tx desc
 */
void rtl8710b_update_txdesc(struct xmit_frame *pxmitframe, u8 *pbuf)
{
	rtl8710b_fill_default_txdesc(pxmitframe, pbuf);

#if defined(CONFIG_USB_HCI)
	rtl8710b_cal_txdesc_chksum((struct tx_desc *)pbuf);
#endif
}

static void hw_var_set_monitor(PADAPTER Adapter, u8 variable, u8 *val)
{
	u32	rcr_bits;
	u16	value_rxfltmap2;
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(Adapter);
	struct mlme_priv *pmlmepriv = &(Adapter->mlmepriv);

	if (*((u8 *)val) == _HW_STATE_MONITOR_) {


		/* Receive all type */
		rcr_bits = RCR_AAP | RCR_APM | RCR_AM | RCR_AB | RCR_APWRMGT | RCR_ADF | RCR_ACF | RCR_AMF | RCR_APP_PHYST_RXFF;

		/* Append FCS */
		rcr_bits |= RCR_APPFCS;

#if 0
		/*
		   CRC and ICV packet will drop in recvbuf2recvframe()
		   We no turn on it.
		 */
		rcr_bits |= (RCR_ACRC32 | RCR_AICV);
#endif

		rtw_hal_get_hwreg(Adapter, HW_VAR_RCR, (u8 *)&pHalData->rcr_backup);
		rtw_hal_set_hwreg(Adapter, HW_VAR_RCR, (u8 *)&rcr_bits);

		/* Receive all data frames */
		value_rxfltmap2 = 0xFFFF;
		rtw_write16(Adapter, REG_RXFLTMAP2, value_rxfltmap2);

#if 0
		/* tx pause */
		rtw_write8(padapter, REG_TXPAUSE, 0xFF);
#endif
	} else {
		/* do nothing */
	}

}

static void hw_var_set_opmode(PADAPTER padapter, u8 variable, u8 *val)
{
	u8 val8;
	u8 mode = *((u8 *)val);
	static u8 isMonitor = _FALSE;

	HAL_DATA_TYPE			*pHalData = GET_HAL_DATA(padapter);

	if (isMonitor == _TRUE) {
		/* reset RCR from backup */
		rtw_hal_set_hwreg(padapter, HW_VAR_RCR, (u8 *)&pHalData->rcr_backup);
		rtw_hal_rcr_set_chk_bssid(padapter, MLME_ACTION_NONE);
		isMonitor = _FALSE;
	}

	if (mode == _HW_STATE_MONITOR_) {
		isMonitor = _TRUE;
		/* set net_type */
		Set_MSR(padapter, _HW_STATE_NOLINK_);

		hw_var_set_monitor(padapter, variable, val);
		return;
	}
	/* set mac addr to mac register */
	rtw_hal_set_hwreg(padapter, HW_VAR_MAC_ADDR,
			  adapter_mac_addr(padapter));

#ifdef CONFIG_CONCURRENT_MODE
	if (padapter->hw_port == HW_PORT1) {
		/* disable Port1 TSF update */
		val8 = rtw_read8(padapter, REG_BCN_CTRL_1);
		val8 |= DIS_TSF_UDT;
		rtw_write8(padapter, REG_BCN_CTRL_1, val8);

		Set_MSR(padapter, mode);

		RTW_INFO("#### %s()-%d hw_port(%d) mode=%d ####\n",
			 __func__, __LINE__, padapter->hw_port, mode);

		if ((mode == _HW_STATE_STATION_) || (mode == _HW_STATE_NOLINK_)) {
			if (!rtw_mi_get_ap_num(padapter) && !rtw_mi_get_mesh_num(padapter)) {
				StopTxBeacon(padapter);
#ifdef CONFIG_PCI_HCI
				UpdateInterruptMask8710BE(padapter, 0, 0, RT_BCN_INT_MASKS, 0);
#else /* !CONFIG_PCI_HCI */
#ifdef CONFIG_INTERRUPT_BASED_TXBCN

#ifdef CONFIG_INTERRUPT_BASED_TXBCN_EARLY_INT
				rtw_write8(padapter, REG_DRVERLYINT, 0x05);/* restore early int time to 5ms */
				UpdateInterruptMask8710BU(padapter, _TRUE, 0, IMR_BCNDMAINT0_8710B);
#endif /* CONFIG_INTERRUPT_BASED_TXBCN_EARLY_INT */

#ifdef CONFIG_INTERRUPT_BASED_TXBCN_BCN_OK_ERR
				UpdateInterruptMask8710BU(padapter, _TRUE , 0, (IMR_TXBCN0ERR_8710B | IMR_TXBCN0OK_8710B));
#endif /* CONFIG_INTERRUPT_BASED_TXBCN_BCN_OK_ERR */

#endif /* CONFIG_INTERRUPT_BASED_TXBCN */
#endif /* !CONFIG_PCI_HCI */
			}

			/* disable atim wnd */
			rtw_write8(padapter, REG_BCN_CTRL_1, DIS_TSF_UDT | DIS_ATIM | EN_BCN_FUNCTION);
		} else if (mode == _HW_STATE_ADHOC_) {
			ResumeTxBeacon(padapter);
			rtw_write8(padapter, REG_BCN_CTRL_1, DIS_TSF_UDT | EN_BCN_FUNCTION | DIS_BCNQ_SUB);
		} else if (mode == _HW_STATE_AP_) {
#ifdef CONFIG_PCI_HCI
			UpdateInterruptMask8710BE(padapter, RT_BCN_INT_MASKS, 0, 0, 0);
#else /* !CONFIG_PCI_HCI */
#ifdef CONFIG_INTERRUPT_BASED_TXBCN

#ifdef CONFIG_INTERRUPT_BASED_TXBCN_EARLY_INT
			UpdateInterruptMask8710BU(padapter, _TRUE, IMR_BCNDMAINT0_8710B, 0);
#endif /* CONFIG_INTERRUPT_BASED_TXBCN_EARLY_INT */

#ifdef CONFIG_INTERRUPT_BASED_TXBCN_BCN_OK_ERR
			UpdateInterruptMask8710BU(padapter, _TRUE, (IMR_TXBCN0ERR_8710B | IMR_TXBCN0OK_8710B), 0);
#endif /* CONFIG_INTERRUPT_BASED_TXBCN_BCN_OK_ERR */

#endif /* CONFIG_INTERRUPT_BASED_TXBCN */
#endif /* !CONFIG_PCI_HCI */

			ResumeTxBeacon(padapter);

			rtw_write8(padapter, REG_BCN_CTRL_1, DIS_TSF_UDT | DIS_BCNQ_SUB);

			/* enable to rx data frame*/
			rtw_write16(padapter, REG_RXFLTMAP2, 0xFFFF);
			/* enable to rx ps-poll */
			rtw_write16(padapter, REG_RXFLTMAP1, 0x0400);

			/* Beacon Control related register for first time */
			rtw_write8(padapter, REG_BCNDMATIM, 0x02); /* 2ms */

			/* rtw_write8(padapter, REG_BCN_MAX_ERR, 0xFF); */
			rtw_write8(padapter, REG_ATIMWND_1, 0x0c); /* 13ms for port1 */
			rtw_write16(padapter, REG_BCNTCFG, 0x00);

			rtw_write16(padapter, REG_TSFTR_SYN_OFFSET, 0x7fff);/* +32767 (~32ms) */

			/* reset TSF2 */
			rtw_write8(padapter, REG_DUAL_TSF_RST, BIT(1));

			/* enable BCN1 Function for if2 */
			/* don't enable update TSF1 for if2 (due to TSF update when beacon/probe rsp are received) */
			rtw_write8(padapter, REG_BCN_CTRL_1, (DIS_TSF_UDT | EN_BCN_FUNCTION | EN_TXBCN_RPT | DIS_BCNQ_SUB));

			/* SW_BCN_SEL - Port1 */
			/* rtw_write8(Adapter, REG_DWBCN1_CTRL_8192E+2, rtw_read8(Adapter, REG_DWBCN1_CTRL_8192E+2)|BIT4); */
			rtw_hal_set_hwreg(padapter, HW_VAR_DL_BCN_SEL, NULL);

			/* select BCN on port 1 */
			rtw_write8(padapter, REG_CCK_CHECK_8710B,
				(rtw_read8(padapter, REG_CCK_CHECK_8710B) | BIT_BCN_PORT_SEL));

			/* BCN1 TSF will sync to BCN0 TSF with offset(0x518) if if1_sta linked */
			/* rtw_write8(padapter, REG_BCN_CTRL_1, rtw_read8(padapter, REG_BCN_CTRL_1)|BIT(5)); */
			/* rtw_write8(padapter, REG_DUAL_TSF_RST, BIT(3)); */

			/* dis BCN0 ATIM  WND if if1 is station */
			rtw_write8(padapter, REG_BCN_CTRL, rtw_read8(padapter, REG_BCN_CTRL) | DIS_ATIM);

#ifdef CONFIG_TSF_RESET_OFFLOAD
			/* Reset TSF for STA+AP concurrent mode */
			if (rtw_mi_buddy_check_fwstate(padapter,
				(WIFI_STATION_STATE | WIFI_ASOC_STATE))) {
				if (rtw_hal_reset_tsf(padapter, HW_PORT1) == _FAIL)
					RTW_INFO("ERROR! %s()-%d: Reset port1 TSF fail\n",
						 __FUNCTION__, __LINE__);
			}
#endif /* CONFIG_TSF_RESET_OFFLOAD */
		}
	} else /* else for port0 */
#endif /* CONFIG_CONCURRENT_MODE */
	{
#ifdef CONFIG_MI_WITH_MBSSID_CAM /*For Port0 - MBSS CAM*/
		hw_var_set_opmode_mbid(padapter, mode);
#else
		/* disable Port0 TSF update */
		val8 = rtw_read8(padapter, REG_BCN_CTRL);
		val8 |= DIS_TSF_UDT;
		rtw_write8(padapter, REG_BCN_CTRL, val8);

		/* set net_type */
		Set_MSR(padapter, mode);
		RTW_INFO("#### %s() -%d hw_port(0) mode = %d ####\n",
			 __func__, __LINE__, mode);

		if ((mode == _HW_STATE_STATION_) || (mode == _HW_STATE_NOLINK_)) {
#ifdef CONFIG_CONCURRENT_MODE
			if (!rtw_mi_get_ap_num(padapter) && !rtw_mi_get_mesh_num(padapter)) {
#else
			{
#endif /* CONFIG_CONCURRENT_MODE */
				StopTxBeacon(padapter);
#ifdef CONFIG_PCI_HCI
				UpdateInterruptMask8710BE(padapter, 0, 0, RT_BCN_INT_MASKS, 0);
#else /* !CONFIG_PCI_HCI */
#ifdef CONFIG_INTERRUPT_BASED_TXBCN
#ifdef CONFIG_INTERRUPT_BASED_TXBCN_EARLY_INT
				rtw_write8(padapter, REG_DRVERLYINT, 0x05); /* restore early int time to 5ms */
				UpdateInterruptMask8812AU(padapter, _TRUE, 0, IMR_BCNDMAINT0_8710B);
#endif /* CONFIG_INTERRUPT_BASED_TXBCN_EARLY_INT */

#ifdef CONFIG_INTERRUPT_BASED_TXBCN_BCN_OK_ERR
				UpdateInterruptMask8812AU(padapter, _TRUE , 0, (IMR_TXBCN0ERR_8710B | IMR_TXBCN0OK_8710B));
#endif /* CONFIG_INTERRUPT_BASED_TXBCN_BCN_OK_ERR */

#endif /* CONFIG_INTERRUPT_BASED_TXBCN */
#endif /* !CONFIG_PCI_HCI */
			}

			/* disable atim wnd */
			rtw_write8(padapter, REG_BCN_CTRL, DIS_TSF_UDT | EN_BCN_FUNCTION | DIS_ATIM);
			/* rtw_write8(padapter,REG_BCN_CTRL, 0x18); */
		} else if (mode == _HW_STATE_ADHOC_) {
			ResumeTxBeacon(padapter);
			rtw_write8(padapter, REG_BCN_CTRL, DIS_TSF_UDT | EN_BCN_FUNCTION | DIS_BCNQ_SUB);
		} else if (mode == _HW_STATE_AP_) {
#ifdef CONFIG_PCI_HCI
			UpdateInterruptMask8710BE(padapter, RT_BCN_INT_MASKS, 0, 0, 0);
#else /* !CONFIG_PCI_HCI */
#ifdef CONFIG_INTERRUPT_BASED_TXBCN
#ifdef CONFIG_INTERRUPT_BASED_TXBCN_EARLY_INT
			UpdateInterruptMask8710BU(padapter, _TRUE , IMR_BCNDMAINT0_8710B, 0);
#endif /* CONFIG_INTERRUPT_BASED_TXBCN_EARLY_INT */

#ifdef CONFIG_INTERRUPT_BASED_TXBCN_BCN_OK_ERR
			UpdateInterruptMask8710BU(padapter, _TRUE , (IMR_TXBCN0ERR_8710B | IMR_TXBCN0OK_8710B), 0);
#endif /* CONFIG_INTERRUPT_BASED_TXBCN_BCN_OK_ERR */

#endif /* CONFIG_INTERRUPT_BASED_TXBCN */
#endif

			ResumeTxBeacon(padapter);

			rtw_write8(padapter, REG_BCN_CTRL, DIS_TSF_UDT | DIS_BCNQ_SUB);

			/* enable to rx data frame */
			rtw_write16(padapter, REG_RXFLTMAP2, 0xFFFF);
			/* enable to rx ps-poll */
			rtw_write16(padapter, REG_RXFLTMAP1, 0x0400);

			/* Beacon Control related register for first time */
			rtw_write8(padapter, REG_BCNDMATIM, 0x02); /* 2ms */

			/* rtw_write8(padapter, REG_BCN_MAX_ERR, 0xFF); */
			rtw_write8(padapter, REG_ATIMWND, 0x0c); /* 13ms */
			rtw_write16(padapter, REG_BCNTCFG, 0x00);

			rtw_write16(padapter, REG_TSFTR_SYN_OFFSET, 0x7fff);/* +32767 (~32ms) */

			/* reset TSF */
			rtw_write8(padapter, REG_DUAL_TSF_RST, BIT(0));

			/* enable BCN0 Function for if1 */
			/* don't enable update TSF0 for if1 (due to TSF update when beacon/probe rsp are received) */
			rtw_write8(padapter, REG_BCN_CTRL, (DIS_TSF_UDT | EN_BCN_FUNCTION | EN_TXBCN_RPT | DIS_BCNQ_SUB));

			/* SW_BCN_SEL - Port0 */
			/* rtw_write8(Adapter, REG_DWBCN1_CTRL_8192E+2, rtw_read8(Adapter, REG_DWBCN1_CTRL_8192E+2) & ~BIT4); */
			rtw_hal_set_hwreg(padapter, HW_VAR_DL_BCN_SEL, NULL);

			/* select BCN on port 0 */
			rtw_write8(padapter, REG_CCK_CHECK_8710B,
				(rtw_read8(padapter, REG_CCK_CHECK_8710B) & ~BIT_BCN_PORT_SEL));

			/* dis BCN1 ATIM  WND if if2 is station */
			val8 = rtw_read8(padapter, REG_BCN_CTRL_1);
			val8 |= DIS_ATIM;
			rtw_write8(padapter, REG_BCN_CTRL_1, val8);
#ifdef CONFIG_TSF_RESET_OFFLOAD
			/* Reset TSF for STA+AP concurrent mode */
			if (rtw_mi_buddy_check_fwstate(padapter,
				(WIFI_STATION_STATE | WIFI_ASOC_STATE))) {
				if (rtw_hal_reset_tsf(padapter, HW_PORT0) == _FAIL)
					RTW_INFO("ERROR! %s()-%d: Reset port0 TSF fail\n",
						 __FUNCTION__, __LINE__);
			}
#endif /* CONFIG_TSF_RESET_OFFLOAD */
		}
#endif /* !CONFIG_MI_WITH_MBSSID_CAM */
	}
}

static void hw_var_set_bcn_func(PADAPTER padapter, u8 variable, u8 *val)
{
	u32 bcn_ctrl_reg;

#ifdef CONFIG_CONCURRENT_MODE
	if (padapter->hw_port == HW_PORT1)
		bcn_ctrl_reg = REG_BCN_CTRL_1;
	else
#endif
	{
		bcn_ctrl_reg = REG_BCN_CTRL;
	}

	if (*(u8 *)val)
		rtw_write8(padapter, bcn_ctrl_reg, (EN_BCN_FUNCTION | EN_TXBCN_RPT));
	else {
		u8 val8;

		val8 = rtw_read8(padapter, bcn_ctrl_reg);
		val8 &= ~(EN_BCN_FUNCTION | EN_TXBCN_RPT);
#ifdef CONFIG_BT_COEXIST
		/* Always enable port0 beacon function for PSTDMA */
		if (REG_BCN_CTRL == bcn_ctrl_reg)
			val8 |= EN_BCN_FUNCTION;
#endif
		rtw_write8(padapter, bcn_ctrl_reg, val8);
	}
}

static void hw_var_set_mlme_disconnect(PADAPTER padapter, u8 variable, u8 *val)
{
	u8 val8;

	/* reject all data frames */
#ifdef CONFIG_CONCURRENT_MODE
	if (rtw_mi_check_status(padapter, MI_LINKED) == _FALSE)
#endif
		rtw_write16(padapter, REG_RXFLTMAP2, 0);

#ifdef CONFIG_CONCURRENT_MODE
	if (padapter->hw_port == HW_PORT1) {
	/* reset TSF1 */
		rtw_write8(padapter, REG_DUAL_TSF_RST, BIT(1));

	/* disable update TSF1 */
		val8 = rtw_read8(padapter, REG_BCN_CTRL_1);
		val8 |= DIS_TSF_UDT;
		rtw_write8(padapter, REG_BCN_CTRL_1, val8);
	} else
#endif
	{
	/* reset TSF */
		rtw_write8(padapter, REG_DUAL_TSF_RST, BIT(0));

	/* disable update TSF */
		val8 = rtw_read8(padapter, REG_BCN_CTRL);
		val8 |= DIS_TSF_UDT;
		rtw_write8(padapter, REG_BCN_CTRL, val8);
	}
}

static void hw_var_set_mlme_join(PADAPTER padapter, u8 variable, u8 *val)
{
	u8 val8;
	u16 val16;
	u32 val32;
	u8 RetryLimit;
	u8 type;
	PHAL_DATA_TYPE pHalData;
	struct mlme_priv *pmlmepriv;

	RetryLimit = RL_VAL_STA;
	type = *(u8 *)val;
	pHalData = GET_HAL_DATA(padapter);
	pmlmepriv = &padapter->mlmepriv;
#ifdef CONFIG_CONCURRENT_MODE
	if (type == 0) {
		/* prepare to join */
		if (rtw_mi_get_ap_num(padapter) || rtw_mi_get_mesh_num(padapter))
			StopTxBeacon(padapter);

		/* enable to rx data frame.Accept all data frame */
		rtw_write16(padapter, REG_RXFLTMAP2, 0xFFFF);

		if (check_fwstate(pmlmepriv, WIFI_STATION_STATE) == _TRUE)
			RetryLimit = (pHalData->CustomerID == RT_CID_CCX) ? RL_VAL_AP : RL_VAL_STA;
		else /* Ad-hoc Mode */
			RetryLimit = RL_VAL_AP;
		} else if (type == 1) {
			/* joinbss_event call back when join res < 0 */
			if (rtw_mi_check_status(padapter, MI_LINKED) == _FALSE)
				rtw_write16(padapter, REG_RXFLTMAP2, 0x00);

			if (rtw_mi_get_ap_num(padapter) || rtw_mi_get_mesh_num(padapter)) {
				ResumeTxBeacon(padapter);
				/* reset TSF 1/2 after ResumeTxBeacon */
				rtw_write8(padapter, REG_DUAL_TSF_RST, BIT(1) | BIT(0));
			}
		} else if (type == 2) {
			/* sta add event call back */
#ifdef CONFIG_MI_WITH_MBSSID_CAM
			/*if (check_fwstate(pmlmepriv, WIFI_STATION_STATE) && (rtw_mi_get_assoced_sta_num(padapter) == 1))
				rtw_write8(padapter, REG_BCN_CTRL, rtw_read8(padapter, REG_BCN_CTRL)&(~DIS_TSF_UDT));*/
#else
			/* enable update TSF */
			if (padapter->hw_port == HW_PORT1) {
				val8 = rtw_read8(padapter, REG_BCN_CTRL_1);
				val8 &= ~DIS_TSF_UDT;
				rtw_write8(padapter, REG_BCN_CTRL_1, val8);
			} else {
				val8 = rtw_read8(padapter, REG_BCN_CTRL);
				val8 &= ~DIS_TSF_UDT;
				rtw_write8(padapter, REG_BCN_CTRL, val8);
			}
#endif
			if (check_fwstate(pmlmepriv,
				WIFI_ADHOC_STATE | WIFI_ADHOC_MASTER_STATE)) {
				rtw_write8(padapter, 0x542 , 0x02);
				RetryLimit = RL_VAL_AP;
			}

			if (rtw_mi_get_ap_num(padapter) || rtw_mi_get_mesh_num(padapter)) {
				ResumeTxBeacon(padapter);

			/* reset TSF 1/2 after ResumeTxBeacon */
				rtw_write8(padapter, REG_DUAL_TSF_RST, BIT(1) | BIT(0));
			}
		}

		val16 = (RetryLimit << RETRY_LIMIT_SHORT_SHIFT) | (RetryLimit << RETRY_LIMIT_LONG_SHIFT);
		rtw_write16(padapter, REG_RL, val16);
#else /* !CONFIG_CONCURRENT_MODE */
		if (type == 0) { /* prepare to join */
			/* enable to rx data frame.Accept all data frame */
			rtw_write16(padapter, REG_RXFLTMAP2, 0xFFFF);

			if (check_fwstate(pmlmepriv, WIFI_STATION_STATE) == _TRUE)
				RetryLimit = (pHalData->CustomerID == RT_CID_CCX) ? RL_VAL_AP : RL_VAL_STA;
			else /* Ad-hoc Mode */
				RetryLimit = RL_VAL_AP;
		} else if (type == 1) /* joinbss_event call back when join res < 0 */
			rtw_write16(padapter, REG_RXFLTMAP2, 0x00);
		else if (type == 2) { /* sta add event call back */
			/* enable update TSF */
			val8 = rtw_read8(padapter, REG_BCN_CTRL);
			val8 &= ~DIS_TSF_UDT;
			rtw_write8(padapter, REG_BCN_CTRL, val8);

			if (check_fwstate(pmlmepriv, WIFI_ADHOC_STATE | WIFI_ADHOC_MASTER_STATE))
				RetryLimit = RL_VAL_AP;
		}

	val16 = (RetryLimit << RETRY_LIMIT_SHORT_SHIFT) | (RetryLimit << RETRY_LIMIT_LONG_SHIFT);
	rtw_write16(padapter, REG_RL, val16);
#endif /* !CONFIG_CONCURRENT_MODE */
}

void CCX_FwC2HTxRpt_8710b(PADAPTER padapter, u8 *pdata, u8 len)
{
	u8 seq_no;

#define	GET_8710B_C2H_TX_RPT_LIFE_TIME_OVER(_Header)	LE_BITS_TO_1BYTE((_Header + 0), 6, 1)
#define	GET_8710B_C2H_TX_RPT_RETRY_OVER(_Header)	LE_BITS_TO_1BYTE((_Header + 0), 7, 1)

	/* RTW_INFO("%s, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", __func__,  */
	/**pdata, *(pdata+1), *(pdata+2), *(pdata+3), *(pdata+4), *(pdata+5), *(pdata+6), *(pdata+7)); */

	seq_no = *(pdata + 6);

#ifdef CONFIG_XMIT_ACK
	if (GET_8710B_C2H_TX_RPT_RETRY_OVER(pdata) | GET_8710B_C2H_TX_RPT_LIFE_TIME_OVER(pdata))
		rtw_ack_tx_done(&padapter->xmitpriv, RTW_SCTX_DONE_CCX_PKT_FAIL);
	/*
	else if(seq_no != padapter->xmitpriv.seq_no) {
		RTW_INFO("tx_seq_no=%d, rpt_seq_no=%d\n", padapter->xmitpriv.seq_no, seq_no);
		rtw_ack_tx_done(&padapter->xmitpriv, RTW_SCTX_DONE_CCX_PKT_FAIL);
	}
	*/
	else
		rtw_ack_tx_done(&padapter->xmitpriv, RTW_SCTX_DONE_SUCCESS);
#endif
}

s32 c2h_handler_8710b(_adapter *adapter, u8 id, u8 seq, u8 plen, u8 *payload)
{
	s32 ret = _SUCCESS;

	switch (id) {
	case C2H_CCX_TX_RPT:
		CCX_FwC2HTxRpt_8710b(adapter, payload, plen);
		break;
	default:
		ret = _FAIL;
		break;
	}

exit:
	return ret;
}

u8 SetHwReg8710B(PADAPTER padapter, u8 variable, u8 *val)
{
	PHAL_DATA_TYPE	pHalData = GET_HAL_DATA(padapter);
	u8 ret = _SUCCESS;
	u8 val8;
	u16 val16;
	u32 val32;


	switch (variable) {
	case HW_VAR_SET_OPMODE:
		hw_var_set_opmode(padapter, variable, val);
		break;

	case HW_VAR_BASIC_RATE:
	{
		struct mlme_ext_info *mlmext_info = &padapter->mlmeextpriv.mlmext_info;
		u16 input_b = 0, masked = 0, ioted = 0, BrateCfg = 0;
		u16 rrsr_2g_force_mask = RRSR_CCK_RATES;
		u16 rrsr_2g_allow_mask = (RRSR_24M | RRSR_12M | RRSR_6M | RRSR_CCK_RATES);

		HalSetBrateCfg(padapter, val, &BrateCfg);
		input_b = BrateCfg;

		/* apply force and allow mask */
		BrateCfg |= rrsr_2g_force_mask;
		BrateCfg &= rrsr_2g_allow_mask;
		masked = BrateCfg;

#ifdef CONFIG_CMCC_TEST
		BrateCfg |= (RRSR_11M | RRSR_5_5M | RRSR_1M); /* use 11M to send ACK */
		BrateCfg |= (RRSR_24M | RRSR_18M | RRSR_12M); /* CMCC_OFDM_ACK 12/18/24M */
#endif

		/* IOT consideration */
		if (mlmext_info->assoc_AP_vendor == HT_IOT_PEER_CISCO) {
			/* if peer is cisco and didn't use ofdm rate, we enable 6M ack */
			if ((BrateCfg & (RRSR_24M | RRSR_12M | RRSR_6M)) == 0)
				BrateCfg |= RRSR_6M;
		}
		ioted = BrateCfg;

		pHalData->BasicRateSet = BrateCfg;

		RTW_INFO("HW_VAR_BASIC_RATE: %#x->%#x->%#x\n", input_b, masked, ioted);

		/* Set RRSR rate table. */
		rtw_write16(padapter, REG_RRSR, BrateCfg);
		rtw_write8(padapter, REG_RRSR + 2, rtw_read8(padapter, REG_RRSR + 2) & 0xf0);
	}
		break;

	case HW_VAR_TXPAUSE:
		rtw_write8(padapter, REG_TXPAUSE, *val);
		break;

	case HW_VAR_BCN_FUNC:
		hw_var_set_bcn_func(padapter, variable, val);
		break;

	case HW_VAR_MLME_DISCONNECT:
		hw_var_set_mlme_disconnect(padapter, variable, val);
		break;

	case HW_VAR_MLME_JOIN:
		hw_var_set_mlme_join(padapter, variable, val);

#ifdef CONFIG_BT_COEXIST
		switch (*val) {
		case 0:
			/* Notify coex. mechanism before join */
			rtw_btcoex_ConnectNotify(padapter, _TRUE);
			break;
		case 1:
		case 2:
			/* Notify coex. mechanism after join, whether successful or failed */
			rtw_btcoex_ConnectNotify(padapter, _FALSE);
			break;
		}
#endif /* CONFIG_BT_COEXIST */
		break;

	case HW_VAR_BEACON_INTERVAL:
		{
			u16 bcn_interval = *((u16 *)val);

			#ifdef CONFIG_SWTIMER_BASED_TXBCN
			bcn_interval = rtw_hal_bcn_interval_adjust(padapter, bcn_interval);
			#endif

			rtw_write16(padapter, REG_BCN_INTERVAL, bcn_interval);

			#ifdef CONFIG_INTERRUPT_BASED_TXBCN_EARLY_INT
			{
				struct mlme_ext_priv	*pmlmeext = &padapter->mlmeextpriv;
				struct mlme_ext_info	*pmlmeinfo = &(pmlmeext->mlmext_info);

				if ((pmlmeinfo->state & 0x03) == WIFI_FW_AP_STATE) {
					RTW_INFO("%s==> bcn_interval:%d, eraly_int:%d\n", __func__, bcn_interval, bcn_interval >> 1);
					rtw_write8(padapter, REG_DRVERLYINT, bcn_interval >> 1);
				}
			}
			#endif/* CONFIG_INTERRUPT_BASED_TXBCN_EARLY_INT */
		}
		break;

	case HW_VAR_SLOT_TIME:
		rtw_write8(padapter, REG_SLOT, *val);
		break;

	case HW_VAR_RESP_SIFS:
#if 0
		/* SIFS for OFDM Data ACK */
		rtw_write8(padapter, REG_SIFS_CTX + 1, val[0]);
		/* SIFS for OFDM consecutive tx like CTS data! */
		rtw_write8(padapter, REG_SIFS_TRX + 1, val[1]);

		rtw_write8(padapter, REG_SPEC_SIFS + 1, val[0]);
		rtw_write8(padapter, REG_MAC_SPEC_SIFS + 1, val[0]);

		/* 20100719 Joseph: Revise SIFS setting due to Hardware register definition change. */
		rtw_write8(padapter, REG_R2T_SIFS + 1, val[0]);
		rtw_write8(padapter, REG_T2T_SIFS + 1, val[0]);

#else
		/* SIFS_Timer = 0x0a0a0808; */
		/* RESP_SIFS for CCK */
		rtw_write8(padapter, REG_RESP_SIFS_CCK, val[0]); /* SIFS_T2T_CCK (0x08) */
		rtw_write8(padapter, REG_RESP_SIFS_CCK + 1, val[1]); /* SIFS_R2T_CCK(0x08) */
		/* RESP_SIFS for OFDM */
		rtw_write8(padapter, REG_RESP_SIFS_OFDM, val[2]); /* SIFS_T2T_OFDM (0x0a) */
		rtw_write8(padapter, REG_RESP_SIFS_OFDM + 1, val[3]); /* SIFS_R2T_OFDM(0x0a) */
#endif
		break;

	case HW_VAR_ACK_PREAMBLE: {
		u8 regTmp;
		u8 bShortPreamble = *val;

		/* Joseph marked out for Netgear 3500 TKIP channel 7 issue.(Temporarily) */
		/* regTmp = (pHalData->nCur40MhzPrimeSC)<<5; */
		regTmp = 0;
		if (bShortPreamble)
			regTmp |= 0x80;
			rtw_write8(padapter, REG_RRSR + 2, regTmp);
	}
		break;

	case HW_VAR_CAM_EMPTY_ENTRY: {
		u8	ucIndex = *val;
		u8	i;
		u32	ulCommand = 0;
		u32	ulContent = 0;
		u32	ulEncAlgo = CAM_AES;

		for (i = 0; i < CAM_CONTENT_COUNT; i++) {
		/* filled id in CAM config 2 byte */
			if (i == 0) {
				ulContent |= (ucIndex & 0x03) | ((u16)(ulEncAlgo) << 2);
				/* ulContent |= CAM_VALID; */
			} else {
				ulContent = 0;
			}
			/* polling bit, and No Write enable, and address */
			ulCommand = CAM_CONTENT_COUNT * ucIndex + i;
			ulCommand = ulCommand | CAM_POLLINIG | CAM_WRITE;
			/* write content 0 is equall to mark invalid */
			rtw_write32(padapter, WCAMI, ulContent);  /* delay_ms(40); */
			rtw_write32(padapter, RWCAM, ulCommand);  /* delay_ms(40); */
		}
	}
		break;

	case HW_VAR_CAM_INVALID_ALL:
		rtw_write32(padapter, RWCAM, BIT(31) | BIT(30));
		break;

	case HW_VAR_AC_PARAM_VO:
		rtw_write32(padapter, REG_EDCA_VO_PARAM, *((u32 *)val));
		break;

	case HW_VAR_AC_PARAM_VI:
		rtw_write32(padapter, REG_EDCA_VI_PARAM, *((u32 *)val));
		break;

	case HW_VAR_AC_PARAM_BE:
		pHalData->ac_param_be = ((u32 *)(val))[0];
		rtw_write32(padapter, REG_EDCA_BE_PARAM, *((u32 *)val));
		break;

	case HW_VAR_AC_PARAM_BK:
		rtw_write32(padapter, REG_EDCA_BK_PARAM, *((u32 *)val));
		break;

	case HW_VAR_ACM_CTRL: {
		u8 ctrl = *((u8 *)val);
		u8 hwctrl = 0;

		if (ctrl != 0) {
			hwctrl |= AcmHw_HwEn;

		if (ctrl & BIT(1)) /* BE */
			hwctrl |= AcmHw_BeqEn;

		if (ctrl & BIT(2)) /* VI */
			hwctrl |= AcmHw_ViqEn;

		if (ctrl & BIT(3)) /* VO */
			hwctrl |= AcmHw_VoqEn;
		}

		RTW_INFO("[HW_VAR_ACM_CTRL] Write 0x%02X\n", hwctrl);
		rtw_write8(padapter, REG_ACMHWCTRL, hwctrl);
	}
		break;

	case HW_VAR_AMPDU_FACTOR: {
		u32	AMPDULen = (*((u8 *)val));

		if (AMPDULen < HT_AGG_SIZE_32K)
			AMPDULen = (0x2000 << (*((u8 *)val))) - 1;
		else
			AMPDULen = 0x7fff;

		rtw_write32(padapter, REG_AMPDU_MAX_LENGTH_8710B, AMPDULen);
	}
		break;

#if 0
	case HW_VAR_RXDMA_AGG_PG_TH:
		rtw_write8(padapter, REG_RXDMA_AGG_PG_TH, *val);
		break;
#endif

	case HW_VAR_H2C_FW_PWRMODE: {
		u8 psmode = *val;

		/* if (psmode != PS_MODE_ACTIVE)	{ */
			/*rtl8710b_set_lowpwr_lps_cmd(padapter, _TRUE); */
		/* } else { */
			/*	rtl8710b_set_lowpwr_lps_cmd(padapter, _FALSE); */
		/* } */
		rtl8710b_set_FwPwrMode_cmd(padapter, psmode);
	}
		break;
	case HW_VAR_H2C_PS_TUNE_PARAM:
		rtl8710b_set_FwPsTuneParam_cmd(padapter);
		break;

	case HW_VAR_H2C_FW_JOINBSSRPT:
		rtl8710b_set_FwJoinBssRpt_cmd(padapter, *val);
#ifdef CONFIG_LPS_POFF
		rtl8710b_lps_poff_h2c_ctrl(padapter, *val);
#endif
		break;

#ifdef CONFIG_P2P
	case HW_VAR_H2C_FW_P2P_PS_OFFLOAD:
		rtl8710b_set_p2p_ps_offload_cmd(padapter, *val);
		break;
#endif /* CONFIG_P2P */
#ifdef CONFIG_LPS_POFF
	case HW_VAR_LPS_POFF_INIT:
		rtl8710b_lps_poff_init(padapter);
		break;
	case HW_VAR_LPS_POFF_DEINIT:
		rtl8710b_lps_poff_deinit(padapter);
		break;
	case HW_VAR_LPS_POFF_SET_MODE:
		rtl8710b_lps_poff_set_ps_mode(padapter, *val);
		break;
	case HW_VAR_LPS_POFF_WOW_EN:
		rtl8710b_lps_poff_h2c_ctrl(padapter, *val);
		break;
#endif /*CONFIG_LPS_POFF*/

	case HW_VAR_EFUSE_USAGE:
		pHalData->EfuseUsedPercentage = *val;
		break;

	case HW_VAR_EFUSE_BYTES:
		pHalData->EfuseUsedBytes = *((u16 *)val);
		break;

	case HW_VAR_EFUSE_BT_USAGE:
#ifdef HAL_EFUSE_MEMORY
		pHalData->EfuseHal.BTEfuseUsedPercentage = *val;
#endif
		break;

	case HW_VAR_EFUSE_BT_BYTES:
#ifdef HAL_EFUSE_MEMORY
		pHalData->EfuseHal.BTEfuseUsedBytes = *((u16 *)val);
#else
		BTEfuseUsedBytes = *((u16 *)val);
#endif
		break;

	case HW_VAR_FIFO_CLEARN_UP: {
#define RW_RELEASE_EN		BIT(18)
#define RXDMA_IDLE			BIT(17)

		struct pwrctrl_priv *pwrpriv = adapter_to_pwrctl(padapter);
		u8 trycnt = 100;

		/* pause tx */
		rtw_write8(padapter, REG_TXPAUSE, 0xff);

		/* keep sn */
		padapter->xmitpriv.nqos_ssn = rtw_read16(padapter, REG_NQOS_SEQ);

		if (pwrpriv->bkeepfwalive != _TRUE) {
			/* RX DMA stop */
			val32 = rtw_read32(padapter, REG_RXPKT_NUM);
			val32 |= RW_RELEASE_EN;
			rtw_write32(padapter, REG_RXPKT_NUM, val32);
			do {
				val32 = rtw_read32(padapter, REG_RXPKT_NUM);
				val32 &= RXDMA_IDLE;
				if (val32)
					break;

				RTW_INFO("%s: [HW_VAR_FIFO_CLEARN_UP] val=%x times:%d\n", __FUNCTION__, val32, trycnt);
			} while (--trycnt);
			if (trycnt == 0)
				RTW_INFO("[HW_VAR_FIFO_CLEARN_UP] Stop RX DMA failed......\n");

			/* RQPN Load 0 */
			rtw_write16(padapter, REG_RQPN_NPQ, 0);
			rtw_write32(padapter, REG_RQPN, 0x80000000);
			rtw_mdelay_os(2);
		}
	}
	break;

	case HW_VAR_RESTORE_HW_SEQ:
		/* restore Sequence No. */
		rtw_write8(padapter, 0x4dc, padapter->xmitpriv.nqos_ssn);
		break;

#ifdef CONFIG_CONCURRENT_MODE
	case HW_VAR_CHECK_TXBUF: {
		u32 i;
		u8 RetryLimit = 0x01;
		u32 reg_200, reg_204;

		val16 = RetryLimit << RETRY_LIMIT_SHORT_SHIFT | RetryLimit << RETRY_LIMIT_LONG_SHIFT;
		rtw_write16(padapter, REG_RL, val16);

		for (i = 0; i < 200; i++) { /* polling 200x10=2000 msec */
			reg_200 = rtw_read32(padapter, 0x200);
			reg_204 = rtw_read32(padapter, 0x204);
			if (reg_200 != reg_204) {
			/* RTW_INFO("packet in tx packet buffer - 0x204=%x, 0x200=%x (%d)\n", rtw_read32(padapter, 0x204), rtw_read32(padapter, 0x200), i); */
				rtw_msleep_os(10);
			} else {
				RTW_INFO("[HW_VAR_CHECK_TXBUF] no packet in tx packet buffer (%d)\n", i);
				break;
			}
		}

		if (reg_200 != reg_204)
			RTW_INFO("packets in tx buffer - 0x204=%x, 0x200=%x\n", reg_204, reg_200);

		RetryLimit = RL_VAL_STA;
		val16 = RetryLimit << RETRY_LIMIT_SHORT_SHIFT | RetryLimit << RETRY_LIMIT_LONG_SHIFT;
		rtw_write16(padapter, REG_RL, val16);
	}
		break;
#endif /* CONFIG_CONCURRENT_MODE */

	case HW_VAR_NAV_UPPER: {
		u32 usNavUpper = *((u32 *)val);

		if (usNavUpper > HAL_NAV_UPPER_UNIT_8710B * 0xFF) {
		break;
	}

		/* The value of ((usNavUpper + HAL_NAV_UPPER_UNIT_8710B - 1) / HAL_NAV_UPPER_UNIT_8710B) */
		/* is getting the upper integer. */
		usNavUpper = (usNavUpper + HAL_NAV_UPPER_UNIT_8710B - 1) / HAL_NAV_UPPER_UNIT_8710B;
		rtw_write8(padapter, REG_NAV_UPPER, (u8)usNavUpper);
	}
		break;

	case HW_VAR_BCN_VALID:
#ifdef CONFIG_CONCURRENT_MODE
		if (padapter->hw_port == HW_PORT1) {
			val8 = rtw_read8(padapter,  REG_DWBCN1_CTRL_8710B + 2);
			val8 |= BIT(0);
			rtw_write8(padapter, REG_DWBCN1_CTRL_8710B + 2, val8);
		} else
#endif /* CONFIG_CONCURRENT_MODE */
		{
		/* BCN_VALID, BIT16 of REG_TDECTRL = BIT0 of REG_TDECTRL+2, write 1 to clear, Clear by sw */
			val8 = rtw_read8(padapter, REG_TDECTRL + 2);
			val8 |= BIT(0);
			rtw_write8(padapter, REG_TDECTRL + 2, val8);
		}
		break;
	case HW_VAR_DL_BCN_SEL:
#ifdef CONFIG_CONCURRENT_MODE
		if (padapter->hw_port == HW_PORT1) {
			/* SW_BCN_SEL - Port1 */
			val8 = rtw_read8(padapter, REG_DWBCN1_CTRL_8710B + 2);
			val8 |= BIT(4);
			rtw_write8(padapter, REG_DWBCN1_CTRL_8710B + 2, val8);
		} else
#endif /* CONFIG_CONCURRENT_MODE */
		{
			/* SW_BCN_SEL - Port0 */
			val8 = rtw_read8(padapter, REG_DWBCN1_CTRL_8710B + 2);
			val8 &= ~BIT(4);
			rtw_write8(padapter, REG_DWBCN1_CTRL_8710B + 2, val8);
		}
		break;
	case HW_VAR_DO_IQK:
		if (*val)
			pHalData->bNeedIQK = _TRUE;
		else
			pHalData->bNeedIQK = _FALSE;
		break;

	case HW_VAR_DL_RSVD_PAGE:
#ifdef CONFIG_BT_COEXIST
		if (check_fwstate(&padapter->mlmepriv, WIFI_AP_STATE) == _TRUE)
			rtl8710b_download_BTCoex_AP_mode_rsvd_page(padapter);
		else
#endif /* CONFIG_BT_COEXIST */
		{
			rtl8710b_download_rsvd_page(padapter, RT_MEDIA_CONNECT);
		}
		break;

#if defined(CONFIG_TDLS) && defined(CONFIG_TDLS_CH_SW)
	case HW_VAR_TDLS_BCN_EARLY_C2H_RPT:
		rtl8710b_set_BcnEarly_C2H_Rpt_cmd(padapter, *val);
		break;
#endif	
	default:
		ret = SetHwReg(padapter, variable, val);
		break;
	}

	return ret;
}

struct qinfo_8710b {
	u32 head:8;
	u32 pkt_num:7;
	u32 tail:8;
	u32 ac:2;
	u32 macid:7;
};

struct bcn_qinfo_8710b {
	u16 head:8;
	u16 pkt_num:8;
};

void dump_qinfo_8710b(void *sel, struct qinfo_8710b *info, const char *tag)
{
	/* if (info->pkt_num) */
	RTW_PRINT_SEL(sel, "%shead:0x%02x, tail:0x%02x, pkt_num:%u, macid:%u, ac:%u\n"
		, tag ? tag : "", info->head, info->tail,
		info->pkt_num, info->macid, info->ac);
}

void dump_bcn_qinfo_8710b(void *sel, struct bcn_qinfo_8710b *info, const char *tag)
{
	/* if (info->pkt_num) */
	RTW_PRINT_SEL(sel, "%shead:0x%02x, pkt_num:%u\n"
		      , tag ? tag : "", info->head, info->pkt_num);
}

void dump_mac_qinfo_8710b(void *sel, _adapter *adapter)
{
	u32 q0_info;
	u32 q1_info;
	u32 q2_info;
	u32 q3_info;
	u32 q4_info;
	u32 q5_info;
	u32 q6_info;
	u32 q7_info;
	u32 mg_q_info;
	u32 hi_q_info;
	u16 bcn_q_info;

	q0_info = rtw_read32(adapter, REG_Q0_INFO);
	q1_info = rtw_read32(adapter, REG_Q1_INFO);
	q2_info = rtw_read32(adapter, REG_Q2_INFO);
	q3_info = rtw_read32(adapter, REG_Q3_INFO);
	q4_info = rtw_read32(adapter, REG_Q4_INFO);
	q5_info = rtw_read32(adapter, REG_Q5_INFO);
	q6_info = rtw_read32(adapter, REG_Q6_INFO);
	q7_info = rtw_read32(adapter, REG_Q7_INFO);
	mg_q_info = rtw_read32(adapter, REG_MGQ_INFO);
	hi_q_info = rtw_read32(adapter, REG_HGQ_INFO);
	bcn_q_info = rtw_read16(adapter, REG_BCNQ_INFO);

	dump_qinfo_8710b(sel, (struct qinfo_8710b *)&q0_info, "Q0 ");
	dump_qinfo_8710b(sel, (struct qinfo_8710b *)&q1_info, "Q1 ");
	dump_qinfo_8710b(sel, (struct qinfo_8710b *)&q2_info, "Q2 ");
	dump_qinfo_8710b(sel, (struct qinfo_8710b *)&q3_info, "Q3 ");
	dump_qinfo_8710b(sel, (struct qinfo_8710b *)&q4_info, "Q4 ");
	dump_qinfo_8710b(sel, (struct qinfo_8710b *)&q5_info, "Q5 ");
	dump_qinfo_8710b(sel, (struct qinfo_8710b *)&q6_info, "Q6 ");
	dump_qinfo_8710b(sel, (struct qinfo_8710b *)&q7_info, "Q7 ");
	dump_qinfo_8710b(sel, (struct qinfo_8710b *)&mg_q_info, "MG ");
	dump_qinfo_8710b(sel, (struct qinfo_8710b *)&hi_q_info, "HI ");
	dump_bcn_qinfo_8710b(sel, (struct bcn_qinfo_8710b *)&bcn_q_info, "BCN ");
}

static void dump_mac_txfifo_8710b(void *sel, _adapter *adapter)
{
	u32 rqpn, rqpn_npq;
	u32 hpq, lpq, npq, epq, pubq;

	rqpn = rtw_read32(adapter, REG_FIFOPAGE);
	rqpn_npq = rtw_read32(adapter, REG_RQPN_NPQ);

	hpq = (rqpn & 0xFF);
	lpq = ((rqpn & 0xFF00)>>8);
	pubq = ((rqpn & 0xFF0000)>>16);
	npq = ((rqpn_npq & 0xFF00)>>8);
	epq = ((rqpn_npq & 0xFF000000)>>24);

	RTW_PRINT_SEL(sel, "Tx: available page num: ");
	if ((hpq == 0xEA) && (hpq == lpq) && (hpq == pubq))
		RTW_PRINT_SEL(sel, "N/A (reg val = 0xea)\n");
	else
		RTW_PRINT_SEL(sel, "HPQ: %d, LPQ: %d, NPQ: %d, EPQ: %d, PUBQ: %d\n"
			, hpq, lpq, npq, epq, pubq);
}

void rtl8710b_read_wmmedca_reg(PADAPTER adapter, u16 *vo_params, u16 *vi_params, u16 *be_params, u16 *bk_params)
{
	u8 vo_reg_params[4];
	u8 vi_reg_params[4];
	u8 be_reg_params[4];
	u8 bk_reg_params[4];

	GetHwReg8710B(adapter, HW_VAR_AC_PARAM_VO, vo_reg_params);
	GetHwReg8710B(adapter, HW_VAR_AC_PARAM_VI, vi_reg_params);
	GetHwReg8710B(adapter, HW_VAR_AC_PARAM_BE, be_reg_params);
	GetHwReg8710B(adapter, HW_VAR_AC_PARAM_BK, bk_reg_params);

	vo_params[0] = vo_reg_params[0];
	vo_params[1] = vo_reg_params[1] & 0x0F;
	vo_params[2] = (vo_reg_params[1] & 0xF0) >> 4;
	vo_params[3] = ((vo_reg_params[3] << 8) | (vo_reg_params[2])) * 32;

	vi_params[0] = vi_reg_params[0];
	vi_params[1] = vi_reg_params[1] & 0x0F;
	vi_params[2] = (vi_reg_params[1] & 0xF0) >> 4;
	vi_params[3] = ((vi_reg_params[3] << 8) | (vi_reg_params[2])) * 32;

	be_params[0] = be_reg_params[0];
	be_params[1] = be_reg_params[1] & 0x0F;
	be_params[2] = (be_reg_params[1] & 0xF0) >> 4;
	be_params[3] = ((be_reg_params[3] << 8) | (be_reg_params[2])) * 32;

	bk_params[0] = bk_reg_params[0];
	bk_params[1] = bk_reg_params[1] & 0x0F;
	bk_params[2] = (bk_reg_params[1] & 0xF0) >> 4;
	bk_params[3] = ((bk_reg_params[3] << 8) | (bk_reg_params[2])) * 32;

	vo_params[1] = (1 << vo_params[1]) - 1;
	vo_params[2] = (1 << vo_params[2]) - 1;
	vi_params[1] = (1 << vi_params[1]) - 1;
	vi_params[2] = (1 << vi_params[2]) - 1;
	be_params[1] = (1 << be_params[1]) - 1;
	be_params[2] = (1 << be_params[2]) - 1;
	bk_params[1] = (1 << bk_params[1]) - 1;
	bk_params[2] = (1 << bk_params[2]) - 1;
}

void GetHwReg8710B(PADAPTER padapter, u8 variable, u8 *val)
{
	PHAL_DATA_TYPE pHalData = GET_HAL_DATA(padapter);
	u8 val8;
	u16 val16;
	u32 val32;

	switch (variable) {
	case HW_VAR_TXPAUSE:
		*val = rtw_read8(padapter, REG_TXPAUSE);
		break;

	case HW_VAR_BCN_VALID:
#ifdef CONFIG_CONCURRENT_MODE
		if (padapter->hw_port == HW_PORT1) {
			val8 = rtw_read8(padapter, REG_DWBCN1_CTRL_8710B + 2);
			*val = (BIT(0) & val8) ? _TRUE : _FALSE;
		} else
#endif
		{
			/* BCN_VALID, BIT16 of REG_TDECTRL = BIT0 of REG_TDECTRL+2 */
			val8 = rtw_read8(padapter, REG_TDECTRL + 2);
			*val = (BIT(0) & val8) ? _TRUE : _FALSE;
		}
		break;

	case HW_VAR_AC_PARAM_VO:
		val32 = rtw_read32(padapter, REG_EDCA_VO_PARAM);
		val[0] = val32 & 0xFF;
		val[1] = (val32 >> 8) & 0xFF;
		val[2] = (val32 >> 16) & 0xFF;
		val[3] = (val32 >> 24) & 0x07;
		break;

	case HW_VAR_AC_PARAM_VI:
		val32 = rtw_read32(padapter, REG_EDCA_VI_PARAM);
		val[0] = val32 & 0xFF;
		val[1] = (val32 >> 8) & 0xFF;
		val[2] = (val32 >> 16) & 0xFF;
		val[3] = (val32 >> 24) & 0x07;
		break;

	case HW_VAR_AC_PARAM_BE:
		val32 = rtw_read32(padapter, REG_EDCA_BE_PARAM);
		val[0] = val32 & 0xFF;
		val[1] = (val32 >> 8) & 0xFF;
		val[2] = (val32 >> 16) & 0xFF;
		val[3] = (val32 >> 24) & 0x07;
		break;

	case HW_VAR_AC_PARAM_BK:
		val32 = rtw_read32(padapter, REG_EDCA_BK_PARAM);
		val[0] = val32 & 0xFF;
		val[1] = (val32 >> 8) & 0xFF;
		val[2] = (val32 >> 16) & 0xFF;
		val[3] = (val32 >> 24) & 0x07;
		break;

	case HW_VAR_EFUSE_USAGE:
		*val = pHalData->EfuseUsedPercentage;
		break;

	case HW_VAR_EFUSE_BYTES:
		*((u16 *)val) = pHalData->EfuseUsedBytes;
		break;

	case HW_VAR_EFUSE_BT_USAGE:
#ifdef HAL_EFUSE_MEMORY
		*val = pHalData->EfuseHal.BTEfuseUsedPercentage;
#endif
		break;

	case HW_VAR_EFUSE_BT_BYTES:
#ifdef HAL_EFUSE_MEMORY
		*((u16 *)val) = pHalData->EfuseHal.BTEfuseUsedBytes;
#else
		*((u16 *)val) = BTEfuseUsedBytes;
#endif
		break;

	case HW_VAR_CHK_HI_QUEUE_EMPTY:
		val16 = rtw_read16(padapter, REG_TXPKT_EMPTY);
		*val = (val16 & BIT(10)) ? _TRUE : _FALSE;
		break;
	case HW_VAR_CHK_MGQ_CPU_EMPTY:
		val16 = rtw_read16(padapter, REG_TXPKT_EMPTY);
		*val = (val16 & BIT(8)) ? _TRUE : _FALSE;
		break;
#ifdef CONFIG_WOWLAN
	case HW_VAR_RPWM_TOG:
		*val = rtw_read8(padapter, SDIO_LOCAL_BASE | SDIO_REG_HRPWM1) & BIT(7);
		break;
	case HW_VAR_WAKEUP_REASON:
		*val = rtw_read8(padapter, REG_WOWLAN_WAKE_REASON);
		if (*val == 0xEA)
			*val = 0;
		break;
	case HW_VAR_SYS_CLKR:
		*val = rtw_read8(padapter, REG_SYS_CLKR);
		break;
#endif
	case HW_VAR_DUMP_MAC_QUEUE_INFO:
		dump_mac_qinfo_8710b(val, padapter);
		break;
	case HW_VAR_DUMP_MAC_TXFIFO:
		dump_mac_txfifo_8710b(val, padapter);
		break;
	default:
		GetHwReg(padapter, variable, val);
		break;
	}
}

/*
 *	Description:
 *		Change default setting of specified variable.
 */
u8 SetHalDefVar8710B(PADAPTER padapter, HAL_DEF_VARIABLE variable, void *pval)
{
	PHAL_DATA_TYPE pHalData;
	u8 bResult;

	bResult = _SUCCESS;

	switch (variable) {
	default:
		bResult = SetHalDefVar(padapter, variable, pval);
		break;
	}

	return bResult;
}

void hal_ra_info_dump(_adapter *padapter , void *sel)
{
	int i;
	u8 mac_id;
	u32 cmd;
	u32 ra_info1, ra_info2, bw_set;
	u32 rate_mask1, rate_mask2;
	u8 curr_tx_rate, curr_tx_sgi, hight_rate, lowest_rate;
	struct dvobj_priv *dvobj = adapter_to_dvobj(padapter);
	struct macid_ctl_t *macid_ctl = dvobj_to_macidctl(dvobj);
	HAL_DATA_TYPE *HalData = GET_HAL_DATA(padapter);

	for (i = 0; i < macid_ctl->num; i++) {

		if (rtw_macid_is_used(macid_ctl, i) && !rtw_macid_is_bmc(macid_ctl, i)) {

			mac_id = (u8) i;
			_RTW_PRINT_SEL(sel , "============ RA status check  Mac_id:%d ===================\n", mac_id);

			cmd = 0x40000100 | mac_id;
			rtw_write32(padapter, REG_HMEBOX_DBG_2_8710B, cmd);
			rtw_msleep_os(10);
			ra_info1 = rtw_read32(padapter, 0x2F0);
			curr_tx_rate = ra_info1 & 0x7F;
			curr_tx_sgi = (ra_info1 >> 7) & 0x01;

			_RTW_PRINT_SEL(sel , "[ ra_info1:0x%08x ] =>cur_tx_rate= %s,cur_sgi:%d\n", ra_info1, HDATA_RATE(curr_tx_rate), curr_tx_sgi);
			_RTW_PRINT_SEL(sel , "[ ra_info1:0x%08x ] => PWRSTS = 0x%02x\n", ra_info1, (ra_info1 >> 8)  & 0x07);

			cmd = 0x40000400 | mac_id;
			rtw_write32(padapter, REG_HMEBOX_DBG_2_8710B, cmd);
			rtw_msleep_os(10);
			ra_info1 = rtw_read32(padapter, 0x2F0);
			ra_info2 = rtw_read32(padapter, 0x2F4);
			rate_mask1 = rtw_read32(padapter, 0x2F8);
			rate_mask2 = rtw_read32(padapter, 0x2FC);
			hight_rate = ra_info2 & 0xFF;
			lowest_rate = (ra_info2 >> 8)  & 0xFF;
			bw_set = (ra_info1 >> 8)  & 0xFF;

			_RTW_PRINT_SEL(sel , "[ ra_info1:0x%08x ] => VHT_EN=0x%02x, ", ra_info1, (ra_info1 >> 24) & 0xFF);


			switch (bw_set) {

			case CHANNEL_WIDTH_20:
				_RTW_PRINT_SEL(sel , "BW_setting=20M\n");
				break;

			case CHANNEL_WIDTH_40:
				_RTW_PRINT_SEL(sel , "BW_setting=40M\n");
				break;

			case CHANNEL_WIDTH_80:
				_RTW_PRINT_SEL(sel , "BW_setting=80M\n");
				break;

			case CHANNEL_WIDTH_160:
				_RTW_PRINT_SEL(sel , "BW_setting=160M\n");
				break;

			default:
				_RTW_PRINT_SEL(sel , "BW_setting=0x%02x\n", bw_set);
				break;

			}

			_RTW_PRINT_SEL(sel , "[ ra_info1:0x%08x ] =>RSSI=%d, DISRA=0x%02x\n",
				       ra_info1,
				       ra_info1 & 0xFF,
				       (ra_info1 >> 16) & 0xFF);

			_RTW_PRINT_SEL(sel , "[ ra_info2:0x%08x ] =>hight_rate=%s, lowest_rate=%s, SGI=0x%02x, RateID=%d\n",
				       ra_info2,
				       HDATA_RATE(hight_rate),
				       HDATA_RATE(lowest_rate),
				       (ra_info2 >> 16) & 0xFF,
				       (ra_info2 >> 24) & 0xFF);

			_RTW_PRINT_SEL(sel , "rate_mask2=0x%08x, rate_mask1=0x%08x\n", rate_mask2, rate_mask1);

		}
	}
}

/*
 *	Description:
 *		Query setting of specified variable.
 */
u8 GetHalDefVar8710B(PADAPTER padapter, HAL_DEF_VARIABLE variable, void *pval)
{
	PHAL_DATA_TYPE pHalData;
	u8 bResult;

	pHalData = GET_HAL_DATA(padapter);
	bResult = _SUCCESS;

	switch (variable) {
	case HAL_DEF_MAX_RECVBUF_SZ:
		*((u32 *)pval) = MAX_RECVBUF_SZ;
		break;
	case HAL_DEF_RX_PACKET_OFFSET:
#ifdef CONFIG_TRX_BD_ARCH
		*((u32 *)pval) = RX_WIFI_INFO_SIZE + DRVINFO_SZ * 8;
#else
		*((u32 *)pval) = RXDESC_SIZE + DRVINFO_SZ * 8;
#endif
		break;

	case HW_VAR_MAX_RX_AMPDU_FACTOR:
		/* Stanley@BB.SD3 suggests 16K can get stable performance */
		/* The experiment was done on SDIO interface */
		/* coding by Lucas@20130730 */
		*(HT_CAP_AMPDU_FACTOR *)pval = MAX_AMPDU_FACTOR_16K;
		break;
	case HW_VAR_BEST_AMPDU_DENSITY:
		*((u32 *)pval) = AMPDU_DENSITY_VALUE_7;
		break;
	case HAL_DEF_TX_LDPC:
	case HAL_DEF_RX_LDPC:
		*((u8 *)pval) = _FALSE;
		break;
	case HAL_DEF_TX_STBC:
		*((u8 *)pval) = 0;
		break;
	case HAL_DEF_RX_STBC:
		*((u8 *)pval) = 1;
		break;
	case HAL_DEF_EXPLICIT_BEAMFORMER:
	case HAL_DEF_EXPLICIT_BEAMFORMEE:
		*((u8 *)pval) = _FALSE;
		break;

	case HW_DEF_RA_INFO_DUMP:
		hal_ra_info_dump(padapter, pval);
		break;

	case HAL_DEF_TX_PAGE_BOUNDARY:
		if (!padapter->registrypriv.wifi_spec)
			*(u8 *)pval = TX_PAGE_BOUNDARY_8710B;
		else
			*(u8 *)pval = WMM_NORMAL_TX_PAGE_BOUNDARY_8710B;
		break;
	case HAL_DEF_TX_PAGE_SIZE:
		*((u32 *)pval) = PAGE_SIZE_128;
		break;
	case HAL_DEF_RX_DMA_SZ_WOW:
		*(u32 *)pval = RX_DMA_SIZE_8710B - RESV_FMWF;
		break;
	case HAL_DEF_RX_DMA_SZ:
		*(u32 *)pval = RX_DMA_BOUNDARY_8710B + 1;
		break;
	case HAL_DEF_RX_PAGE_SIZE:
		*((u32 *)pval) = 8;
		break;
	default:
		bResult = GetHalDefVar(padapter, variable, pval);
		break;
	}

	return bResult;
}

void rtl8710b_start_thread(_adapter *padapter)
{
#if (defined CONFIG_SDIO_HCI) || (defined CONFIG_GSPI_HCI)
#ifndef CONFIG_SDIO_TX_TASKLET
	struct xmit_priv *xmitpriv = &padapter->xmitpriv;

	xmitpriv->SdioXmitThread = kthread_run(rtl8710bs_xmit_thread, padapter, "RTWHALXT");
	if (IS_ERR(xmitpriv->SdioXmitThread)) {
		RTW_ERR("%s: start rtl8710bs_xmit_thread FAIL!!\n", __func__);
		xmitpriv->SdioXmitThread = NULL;
	}
#endif
#endif
}

void rtl8710b_stop_thread(_adapter *padapter)
{
#if (defined CONFIG_SDIO_HCI) || (defined CONFIG_GSPI_HCI)
#ifndef CONFIG_SDIO_TX_TASKLET
	struct xmit_priv *xmitpriv = &padapter->xmitpriv;

	/* stop xmit_buf_thread */
	if (xmitpriv->SdioXmitThread) {
		_rtw_up_sema(&xmitpriv->SdioXmitSema);
		rtw_thread_stop(xmitpriv->SdioXmitThread);
		xmitpriv->SdioXmitThread = NULL;
	}
#endif
#endif
}

#if defined(CONFIG_CHECK_BT_HANG) && defined(CONFIG_BT_COEXIST)
void rtl8710bs_init_checkbthang_workqueue(_adapter *adapter)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37))
	adapter->priv_checkbt_wq = alloc_workqueue("sdio_wq", 0, 0);
#else
	adapter->priv_checkbt_wq = create_workqueue("sdio_wq");
#endif
	INIT_DELAYED_WORK(&adapter->checkbt_work, (void *)check_bt_status_work);
}

void rtl8710bs_free_checkbthang_workqueue(_adapter *adapter)
{
	if (adapter->priv_checkbt_wq) {
		cancel_delayed_work_sync(&adapter->checkbt_work);
		flush_workqueue(adapter->priv_checkbt_wq);
		destroy_workqueue(adapter->priv_checkbt_wq);
		adapter->priv_checkbt_wq = NULL;
	}
}

void rtl8710bs_cancle_checkbthang_workqueue(_adapter *adapter)
{
	if (adapter->priv_checkbt_wq)
		cancel_delayed_work_sync(&adapter->checkbt_work);
}

void rtl8710bs_hal_check_bt_hang(_adapter *adapter)
{
	if (adapter->priv_checkbt_wq)
		queue_delayed_work(adapter->priv_checkbt_wq, &(adapter->checkbt_work), 0);
}
#endif

void rtl8710b_set_hal_ops(struct hal_ops *pHalFunc)
{
	pHalFunc->read_chip_version = &read_chip_version_8710b;
	
	pHalFunc->dm_init = &rtl8710b_init_dm_priv;
	pHalFunc->dm_deinit = &rtl8710b_deinit_dm_priv;
	pHalFunc->set_chnl_bw_handler = &PHY_SetSwChnlBWMode8710B;
	pHalFunc->set_tx_power_level_handler = &PHY_SetTxPowerLevel8710B;
	pHalFunc->get_tx_power_level_handler = &PHY_GetTxPowerLevel8710B;
	pHalFunc->get_tx_power_index_handler = &PHY_GetTxPowerIndex_8710B;
	pHalFunc->hal_dm_watchdog = &rtl8710b_HalDmWatchDog;

	pHalFunc->SetBeaconRelatedRegistersHandler = &rtl8710b_SetBeaconRelatedRegisters;
	pHalFunc->run_thread = &rtl8710b_start_thread;
	pHalFunc->cancel_thread = &rtl8710b_stop_thread;
	pHalFunc->read_bbreg = &PHY_QueryBBReg_8710B;
	pHalFunc->write_bbreg = &PHY_SetBBReg_8710B;
	pHalFunc->read_rfreg = &PHY_QueryRFReg_8710B;
	pHalFunc->write_rfreg = &PHY_SetRFReg_8710B;
	pHalFunc->read_syson_reg = &hal_query_syson_reg_8710b;
	pHalFunc->write_syson_reg = &hal_set_syson_reg_8710b;
//	pHalFunc->read_wmmedca_reg = &rtl8710b_read_wmmedca_reg;

	/* Efuse related function */
	pHalFunc->ReadEFuse = &hal_read_efuse_8710b;
	pHalFunc->EfusePowerSwitch = &hal_efuse_power_switch;
	pHalFunc->EFUSEGetEfuseDefinition = &hal_get_efuse_definition;
	pHalFunc->EfuseGetCurrentSize = &Hal_EfuseGetCurrentSize;
	pHalFunc->Efuse_PgPacketRead = &Hal_EfusePgPacketRead;
	pHalFunc->Efuse_PgPacketWrite = &Hal_EfusePgPacketWrite;
	pHalFunc->Efuse_WordEnableDataWrite = &Hal_EfuseWordEnableDataWrite;
	pHalFunc->efuse_indirect_read4 = &hal_efuse_read4_8710b;
		
#ifdef DBG_CONFIG_ERROR_DETECT
	pHalFunc->sreset_init_value = &sreset_init_value;
	pHalFunc->sreset_reset_value = &sreset_reset_value;
	pHalFunc->silentreset = &sreset_reset;
	pHalFunc->sreset_xmit_status_check = &rtl8710b_sreset_xmit_status_check;
	pHalFunc->sreset_linked_status_check  = &rtl8710b_sreset_linked_status_check;
	pHalFunc->sreset_get_wifi_status  = &sreset_get_wifi_status;
	pHalFunc->sreset_inprogress = &sreset_inprogress;
#endif
	pHalFunc->GetHalODMVarHandler = GetHalODMVar;
	pHalFunc->SetHalODMVarHandler = SetHalODMVar;

#ifdef CONFIG_XMIT_THREAD_MODE
	pHalFunc->xmit_thread_handler = &hal_xmit_handler;
#endif
	pHalFunc->hal_notch_filter = &hal_notch_filter_8710b;
	pHalFunc->c2h_handler = c2h_handler_8710b;
	pHalFunc->fill_h2c_cmd = &FillH2CCmd8710B;
	pHalFunc->fill_fake_txdesc = &rtl8710b_fill_fake_txdesc;
	pHalFunc->fw_dl = &rtl8710b_FirmwareDownload;
	pHalFunc->hal_get_tx_buff_rsvd_page_num = &GetTxBufferRsvdPageNum8710B;
}

