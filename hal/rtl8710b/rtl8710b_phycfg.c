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
#define _RTL8710B_PHYCFG_C_

#include <rtl8710b_hal.h>


/*---------------------------Define Local Constant---------------------------*/
/* Channel switch:The size of command tables for switch channel*/
#define MAX_PRECMD_CNT 16
#define MAX_RFDEPENDCMD_CNT 16
#define MAX_POSTCMD_CNT 16

#define MAX_DOZE_WAITING_TIMES_9x 64

/*---------------------------Define Local Constant---------------------------*/


/*------------------------Define global variable-----------------------------*/

/*------------------------Define local variable------------------------------*/


/*--------------------Define export function prototype-----------------------*/
/* Please refer to header file
 *--------------------Define export function prototype-----------------------*/

/*----------------------------Function Body----------------------------------*/
/*
 * 1. BB register R/W API
 *   */

/**
* Function:	phy_CalculateBitShift
*
* OverView:	Get shifted position of the BitMask
*
* Input:
*			u4Byte		BitMask,
*
* Output:	none
* Return:		u4Byte		Return the shift bit bit position of the mask
*/
static	u32
phy_CalculateBitShift(
	u32 BitMask
)
{
	u32 i;

	for (i = 0; i <= 31; i++) {
		if (((BitMask >> i) &  0x1) == 1)
			break;
	}

	return i;
}


/**
* Function:	PHY_QueryBBReg
*
* OverView:	Read "sepcific bits" from BB register
*
* Input:
*			PADAPTER		Adapter,
*			u4Byte			RegAddr,
*			u4Byte			BitMask
*
* Output:	None
* Return:		u4Byte			Data
* Note:		This function is equal to "GetRegSetting" in PHY programming guide
*/
u32
PHY_QueryBBReg_8710B(
	IN	PADAPTER	Adapter,
	IN	u32		RegAddr,
	IN	u32		BitMask
)
{
	u32	ReturnValue = 0, OriginalValue, BitShift;
	u16	BBWaitCounter = 0;

#if (DISABLE_BB_RF == 1)
	return 0;
#endif


	OriginalValue = rtw_read32(Adapter, RegAddr);
	BitShift = phy_CalculateBitShift(BitMask);
	ReturnValue = (OriginalValue & BitMask) >> BitShift;
	//RTW_INFO("PHY_QueryBBReg_8710B : RegAddr=0x%x \n", RegAddr);
	return ReturnValue;

}


/**
* Function:	PHY_SetBBReg
*
* OverView:	Write "Specific bits" to BB register (page 8~)
*
* Input:
*			PADAPTER		Adapter,
*			u4Byte			RegAddr,
*			u4Byte			BitMask
*
*			u4Byte			Data
*
*
* Output:	None
* Return:		None
* Note:		This function is equal to "PutRegSetting" in PHY programming guide
*/

VOID
PHY_SetBBReg_8710B(
	IN	PADAPTER	Adapter,
	IN	u32		RegAddr,
	IN	u32		BitMask,
	IN	u32		Data
)
{
	HAL_DATA_TYPE	*pHalData		= GET_HAL_DATA(Adapter);
	/* u16			BBWaitCounter	= 0; */
	u32			OriginalValue, BitShift;

#if (DISABLE_BB_RF == 1)
	return;
#endif


	if (BitMask != bMaskDWord) { /* if not "double word" write */
		OriginalValue = rtw_read32(Adapter, RegAddr);
		BitShift = phy_CalculateBitShift(BitMask);
		Data = ((OriginalValue & (~BitMask)) | ((Data << BitShift) & BitMask));
	}
	
	//RTW_INFO("PHY_SetBBReg_8710B : RegAddr=0x%x  Data=0x%x\n", RegAddr, Data);
	rtw_write32(Adapter, RegAddr, Data);

}


/*
 * 2. RF register R/W API
 *   */

/*-----------------------------------------------------------------------------
 * Function:	phy_FwRFSerialRead()
 *
 * Overview:	We support firmware to execute RF-R/W.
 *
 * Input:		NONE
 *
 * Output:		NONE
 *
 * Return:		NONE
 *
 * Revised History:
 *	When		Who		Remark
 *	01/21/2008	MHC		Create Version 0.
 *
 *---------------------------------------------------------------------------*/
static	u32
phy_FwRFSerialRead(
	IN	PADAPTER			Adapter,
	IN	enum rf_path			eRFPath,
	IN	u32				Offset)
{
	u32		retValue = 0;
	/* RT_ASSERT(FALSE,("deprecate!\n")); */
	return	retValue;

}	/* phy_FwRFSerialRead */


/*-----------------------------------------------------------------------------
 * Function:	phy_FwRFSerialWrite()
 *
 * Overview:	We support firmware to execute RF-R/W.
 *
 * Input:		NONE
 *
 * Output:		NONE
 *
 * Return:		NONE
 *
 * Revised History:
 *	When		Who		Remark
 *	01/21/2008	MHC		Create Version 0.
 *
 *---------------------------------------------------------------------------*/
static	VOID
phy_FwRFSerialWrite(
	IN	PADAPTER			Adapter,
	IN	enum rf_path			eRFPath,
	IN	u32				Offset,
	IN	u32				Data)
{
	/* RT_ASSERT(FALSE,("deprecate!\n")); */
}

static	u32
phy_RFSerialRead_8710B(
	IN	PADAPTER			Adapter,
	IN	enum rf_path			eRFPath,
	IN	u32				Offset
)
{
	u32						retValue = 0;
	HAL_DATA_TYPE				*pHalData = GET_HAL_DATA(Adapter);
	BB_REGISTER_DEFINITION_T	*pPhyReg = &pHalData->PHYRegDef[eRFPath];
	u32						NewOffset;
	u32						tmplong, tmplong2;
	u8					RfPiEnable = 0;
	u32						MaskforPhySet = 0;
	int i = 0;

	u8 cnt = 0;
	u32 reg_addr = pPhyReg->rfLSSIReadBack;//0x8a0
	u32 value32;
	BOOLEAN err_flag = _TRUE;
	
	_enter_critical_mutex(&(adapter_to_dvobj(Adapter)->rf_read_reg_mutex) , NULL);

	Offset &= 0xff;

	value32 = phy_query_bb_reg(Adapter, rFPGA0_XA_HSSIParameter2, bMaskDWord);
	value32 = (value32 & (~bLSSIReadAddress)) | (Offset<<23);
	phy_set_bb_reg(Adapter, rFPGA0_XA_HSSIParameter2, bMaskDWord, value32 & (~bLSSIReadEdge));			
	phy_set_bb_reg(Adapter, rFPGA0_XA_HSSIParameter2, bMaskDWord, value32 | bLSSIReadEdge);		


	//===>add by ylb 20170322, delay 120us is not enough, so wait for BB(%x)[20] is 1
	if(phy_query_bb_reg(Adapter, REG_FPGA0_XA_HSSI_PARAMETER1, BIT8))
		reg_addr = pPhyReg->rfLSSIReadBackPi;//0x8b8

	do
	{
		rtw_udelay_os(120);
		
		retValue = phy_query_bb_reg(Adapter, reg_addr, bMaskDWord);
		if (retValue & BIT20)
		{
			err_flag = _FALSE;
			break;

		}
		RTW_DBG("phy_RFSerialRead, BB(%x) = %x, Cnt=%x\n", reg_addr, retValue, cnt);
		
	}while(cnt++ < 0x5);
	//<===add by ylb 20170322, delay 120us is not enough, so wait for BB(%x)[20] is 1

	retValue &= bLSSIReadBackData;
	if((0x18 == Offset) && (err_flag == _TRUE))
	{
		retValue &= 0xFFF;
		RTW_DBG("phy_RFSerialRead(): Offset(%#x), retValue(%#x), eRFPath(%#x)\n", Offset, retValue, eRFPath);
	}
	
#if 0
	NewOffset = Offset;

	if (eRFPath == RF_PATH_A) {
		tmplong2 = phy_query_bb_reg(Adapter, rFPGA0_XA_HSSIParameter2 | MaskforPhySet, bMaskDWord);
		tmplong2 = (tmplong2 & (~bLSSIReadAddress)) | (NewOffset << 23) | bLSSIReadEdge;	/* T65 RF */
		phy_set_bb_reg(Adapter, rFPGA0_XA_HSSIParameter2 | MaskforPhySet, bMaskDWord, tmplong2 & (~bLSSIReadEdge));
	} else {
		RTW_ERR("%s : RF path error = %u\n", phy_RFSerialRead_8710B, eRFPath);
	}

	tmplong2 = phy_query_bb_reg(Adapter, rFPGA0_XA_HSSIParameter2 | MaskforPhySet, bMaskDWord);
	phy_set_bb_reg(Adapter, rFPGA0_XA_HSSIParameter2 | MaskforPhySet, bMaskDWord, tmplong2 & (~bLSSIReadEdge));
	phy_set_bb_reg(Adapter, rFPGA0_XA_HSSIParameter2 | MaskforPhySet, bMaskDWord, tmplong2 | bLSSIReadEdge);

	rtw_udelay_os(10);

	for (i = 0; i < 2; i++)
		rtw_udelay_os(MAX_STALL_TIME);
	rtw_udelay_os(10);

	if (eRFPath == RF_PATH_A)
		RfPiEnable = (u1Byte)phy_query_bb_reg(Adapter, rFPGA0_XA_HSSIParameter1 | MaskforPhySet, BIT(8));
	else if (eRFPath == RF_PATH_B)
		RfPiEnable = (u1Byte)phy_query_bb_reg(Adapter, rFPGA0_XB_HSSIParameter1 | MaskforPhySet, BIT(8));

	if (RfPiEnable) {
		/* Read from BBreg8b8, 12 bits for 8190, 20bits for T65 RF */
		retValue = phy_query_bb_reg(Adapter, pPhyReg->rfLSSIReadBackPi | MaskforPhySet, bLSSIReadBackData);

		/* RT_DISP(FINIT, INIT_RF, ("Readback from RF-PI : 0x%x\n", retValue)); */
	} else {
		/* Read from BBreg8a0, 12 bits for 8190, 20 bits for T65 RF */
		retValue = phy_query_bb_reg(Adapter, pPhyReg->rfLSSIReadBack | MaskforPhySet, bLSSIReadBackData);

		/* RT_DISP(FINIT, INIT_RF,("Readback from RF-SI : 0x%x\n", retValue)); */
	}
	
#endif
	_exit_critical_mutex(&(adapter_to_dvobj(Adapter)->rf_read_reg_mutex) , NULL);

	return retValue;

}

/**
* Function:	phy_RFSerialWrite_8710B
*
* OverView:	Write data to RF register (page 8~)
*
* Input:
*			PADAPTER		Adapter,
			enum rf_path		eRFPath,
*			u4Byte			Offset,
*			u4Byte			Data
*
*
* Output:	None
* Return:		None
* Note:		Threre are three types of serial operations:
*			1. Software serial write
*			2. Hardware LSSI-Low Speed Serial Interface
*			3. Hardware HSSI-High speed
*			serial write. Driver need to implement (1) and (2).
*			This function is equal to the combination of RF_ReadReg() and  RFLSSIRead()
 *
 * Note:		  For RF8256 only
 *			 The total count of RTL8256(Zebra4) register is around 36 bit it only employs
 *			 4-bit RF address. RTL8256 uses "register mode control bit" (Reg00[12], Reg00[10])
 *			 to access register address bigger than 0xf. See "Appendix-4 in PHY Configuration
 *			 programming guide" for more details.
 *			 Thus, we define a sub-finction for RTL8526 register address conversion
 *		       ===========================================================
 *			 Register Mode		RegCTL[1]		RegCTL[0]		Note
 *								(Reg00[12])		(Reg00[10])
 *		       ===========================================================
 *			 Reg_Mode0				0				x			Reg 0 ~15(0x0 ~ 0xf)
 *		       ------------------------------------------------------------------
 *			 Reg_Mode1				1				0			Reg 16 ~30(0x1 ~ 0xf)
 *		       ------------------------------------------------------------------
 *			 Reg_Mode2				1				1			Reg 31 ~ 45(0x1 ~ 0xf)
 *		       ------------------------------------------------------------------
 *
 *	2008/09/02	MH	Add 92S RF definition
 *
 *
 *
*/
static	VOID
phy_RFSerialWrite_8710B(
	IN	PADAPTER			Adapter,
	IN	enum rf_path			eRFPath,
	IN	u32				Offset,
	IN	u32				Data
)
{
	u32						DataAndAddr = 0;
	HAL_DATA_TYPE				*pHalData = GET_HAL_DATA(Adapter);
	BB_REGISTER_DEFINITION_T	*pPhyReg = &pHalData->PHYRegDef[eRFPath];
	u32						NewOffset;
	
	Offset &= 0xff;

	/* */
	/* Shadow Update */
	/* */
	/* PHY_RFShadowWrite(Adapter, eRFPath, Offset, Data); */

	/* */
	/* Switch page for 8256 RF IC */
	/* */
	NewOffset = Offset;

	/* */
	/* Put write addr in [5:0]  and write data in [31:16] */
	/* */
	/* DataAndAddr = (Data<<16) | (NewOffset&0x3f); */
	DataAndAddr = ((NewOffset << 20) | (Data & 0x000fffff)) & 0x0fffffff;	/* T65 RF */

	/* */
	/* Write Operation */
	/* */
	phy_set_bb_reg(Adapter, pPhyReg->rf3wireOffset, bMaskDWord, DataAndAddr);
}


/**
* Function:	PHY_QueryRFReg
*
* OverView:	Query "Specific bits" to RF register (page 8~)
*
* Input:
*			PADAPTER		Adapter,
			enum rf_path			eRFPath,
*			u4Byte			RegAddr,
*			u4Byte			BitMask
*
*
* Output:	None
* Return:		u4Byte			Readback value
* Note:		This function is equal to "GetRFRegSetting" in PHY programming guide
*/
u32
PHY_QueryRFReg_8710B(
	IN	PADAPTER			Adapter,
	IN	enum rf_path			eRFPath,
	IN	u32				RegAddr,
	IN	u32				BitMask
)
{
	u32 Original_Value, Readback_Value, BitShift;

#if (DISABLE_BB_RF == 1)
	return 0;
#endif

	Original_Value = phy_RFSerialRead_8710B(Adapter, eRFPath, RegAddr);

	BitShift =  phy_CalculateBitShift(BitMask);
	Readback_Value = (Original_Value & BitMask) >> BitShift;

	return Readback_Value;
}

/**
* Function:	PHY_SetRFReg
*
* OverView:	Write "Specific bits" to RF register (page 8~)
*
* Input:
*			PADAPTER		Adapter,
*			RF_PATH			eRFPath,
*			u4Byte			RegAddr,
*			u4Byte			BitMask
*
*			u4Byte			Data
*
*
* Output:	None
* Return:		None
* Note:		This function is equal to "PutRFRegSetting" in PHY programming guide
*/
VOID
PHY_SetRFReg_8710B(
	IN	PADAPTER			Adapter,
	IN	enum rf_path			eRFPath,
	IN	u32				RegAddr,
	IN	u32				BitMask,
	IN	u32				Data
)
{
	u32		Original_Value, BitShift;

#if (DISABLE_BB_RF == 1)
	return;
#endif

	/* RF data is 12 bits only */
	if (BitMask != bRFRegOffsetMask) {
		Original_Value = phy_RFSerialRead_8710B(Adapter, eRFPath, RegAddr);
		BitShift =  phy_CalculateBitShift(BitMask);
		Data = ((Original_Value & (~BitMask)) | (Data << BitShift));
	}

	phy_RFSerialWrite_8710B(Adapter, eRFPath, RegAddr, Data);
}


/*
 * 3. Initial MAC/BB/RF config by reading MAC/BB/RF txt.
 *   */


/*-----------------------------------------------------------------------------
 * Function:    PHY_MACConfig8192C
 *
 * Overview:	Condig MAC by header file or parameter file.
 *
 * Input:       NONE
 *
 * Output:      NONE
 *
 * Return:      NONE
 *
 * Revised History:
 *  When		Who		Remark
 *  08/12/2008	MHC		Create Version 0.
 *
 *---------------------------------------------------------------------------*/
s32 PHY_MACConfig8710B(PADAPTER Adapter)
{
	int rtStatus = _SUCCESS;
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(Adapter);

#ifdef CONFIG_LOAD_PHY_PARA_FROM_FILE
	rtStatus = phy_ConfigMACWithParaFile(Adapter, PHY_FILE_MAC_REG);
	if (rtStatus == _FAIL)
#endif
	{
#ifdef CONFIG_EMBEDDED_FWIMG
		odm_config_mac_with_header_file(&pHalData->odmpriv);
		rtStatus = _SUCCESS;
#endif/* CONFIG_EMBEDDED_FWIMG */
	}

	return rtStatus;
}

/**
* Function:	phy_InitBBRFRegisterDefinition
*
* OverView:	Initialize Register definition offset for Radio Path A/B/C/D
*
* Input:
*			PADAPTER		Adapter,
*
* Output:	None
* Return:		None
* Note:		The initialization value is constant and it should never be changes
*/
static	VOID
phy_InitBBRFRegisterDefinition(
	IN	PADAPTER		Adapter
)
{
	HAL_DATA_TYPE		*pHalData = GET_HAL_DATA(Adapter);

	/* RF Interface Sowrtware Control */
	pHalData->PHYRegDef[RF_PATH_A].rfintfs = rFPGA0_XAB_RFInterfaceSW; /* 16 LSBs if read 32-bit from 0x870 */
	pHalData->PHYRegDef[RF_PATH_B].rfintfs = rFPGA0_XAB_RFInterfaceSW; /* 16 MSBs if read 32-bit from 0x870 (16-bit for 0x872) */

	/* RF Interface Output (and Enable) */
	pHalData->PHYRegDef[RF_PATH_A].rfintfo = rFPGA0_XA_RFInterfaceOE; /* 16 LSBs if read 32-bit from 0x860 */
	pHalData->PHYRegDef[RF_PATH_B].rfintfo = rFPGA0_XB_RFInterfaceOE; /* 16 LSBs if read 32-bit from 0x864 */

	/* RF Interface (Output and)  Enable */
	pHalData->PHYRegDef[RF_PATH_A].rfintfe = rFPGA0_XA_RFInterfaceOE; /* 16 MSBs if read 32-bit from 0x860 (16-bit for 0x862) */
	pHalData->PHYRegDef[RF_PATH_B].rfintfe = rFPGA0_XB_RFInterfaceOE; /* 16 MSBs if read 32-bit from 0x864 (16-bit for 0x866) */

	pHalData->PHYRegDef[RF_PATH_A].rf3wireOffset = rFPGA0_XA_LSSIParameter; /* LSSI Parameter */
	pHalData->PHYRegDef[RF_PATH_B].rf3wireOffset = rFPGA0_XB_LSSIParameter;

	pHalData->PHYRegDef[RF_PATH_A].rfHSSIPara2 = rFPGA0_XA_HSSIParameter2;  /* wire control parameter2 */
	pHalData->PHYRegDef[RF_PATH_B].rfHSSIPara2 = rFPGA0_XB_HSSIParameter2;  /* wire control parameter2 */

	/* Tranceiver Readback LSSI/HSPI mode */
	pHalData->PHYRegDef[RF_PATH_A].rfLSSIReadBack = rFPGA0_XA_LSSIReadBack;
	pHalData->PHYRegDef[RF_PATH_B].rfLSSIReadBack = rFPGA0_XB_LSSIReadBack;
	pHalData->PHYRegDef[RF_PATH_A].rfLSSIReadBackPi = TransceiverA_HSPI_Readback;
	pHalData->PHYRegDef[RF_PATH_B].rfLSSIReadBackPi = TransceiverB_HSPI_Readback;

}

static	int
phy_BB8710b_Config_ParaFile( // Peter, TODO
	IN	PADAPTER	Adapter
)
{
	HAL_DATA_TYPE		*pHalData = GET_HAL_DATA(Adapter);
	int			rtStatus = _SUCCESS;

	/* */
	/* 1. Read PHY_REG.TXT BB INIT!! */
	/* */
#ifdef CONFIG_LOAD_PHY_PARA_FROM_FILE
	if (phy_ConfigBBWithParaFile(Adapter, PHY_FILE_PHY_REG, CONFIG_BB_PHY_REG) == _FAIL)
#endif
	{
#ifdef CONFIG_EMBEDDED_FWIMG
		if (HAL_STATUS_SUCCESS != odm_config_bb_with_header_file(&pHalData->odmpriv, CONFIG_BB_PHY_REG))
			rtStatus = _FAIL;
#endif
	}

	if (rtStatus != _SUCCESS) {
		RTW_INFO("%s():Write BB Reg Fail!!", __func__);
		goto phy_BB8190_Config_ParaFile_Fail;
	}

#if MP_DRIVER == 1
	if (Adapter->registrypriv.mp_mode == 1) {
		/*20160504, Suggested by jessica_wang. To Fix CCK ACPR issue*/
		phy_set_bb_reg(Adapter, 0xCE0, BIT1|BIT0, 0);/*RXHP=low corner*/
		phy_set_bb_reg(Adapter, 0xC3C, 0xFF, 0xCC);/*make sure low rate sensitivity*/
	}
#endif	/*  #if (MP_DRIVER == 1) */

	/* */
	/* 2. Read BB AGC table Initialization */
	/* */
#ifdef CONFIG_LOAD_PHY_PARA_FROM_FILE
	if (phy_ConfigBBWithParaFile(Adapter, PHY_FILE_AGC_TAB, CONFIG_BB_AGC_TAB) == _FAIL)
#endif
	{
#ifdef CONFIG_EMBEDDED_FWIMG
		if (HAL_STATUS_SUCCESS != odm_config_bb_with_header_file(&pHalData->odmpriv, CONFIG_BB_AGC_TAB))
			rtStatus = _FAIL;
#endif
	}

	if (rtStatus != _SUCCESS) {
		RTW_INFO("%s():AGC Table Fail\n", __func__);
		goto phy_BB8190_Config_ParaFile_Fail;
	}

phy_BB8190_Config_ParaFile_Fail:

	return rtStatus;
}


int
PHY_BBConfig8710B(
	IN PADAPTER Adapter
)
{
	int	rtStatus = _SUCCESS;
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	u32	RegVal;
	u8	TmpU1B = 0;
	u8	value8;

	phy_InitBBRFRegisterDefinition(Adapter);

	/* Enable BB and RF */
	RegVal = rtw_read32(Adapter,REG_SYS_FUNC_EN_8710B);
	rtw_write32(Adapter,REG_SYS_FUNC_EN_8710B, RegVal|BIT16|BIT17|BIT24|BIT25|BIT26);

	/* Config BB and AGC */
	rtStatus = phy_BB8710b_Config_ParaFile(Adapter);

	hal_set_crystal_cap(Adapter, pHalData->crystal_cap);

	return rtStatus;
}
#if 0
/* Block & Path enable */
#define		rOFDMCCKEN_Jaguar		0x808 /* OFDM/CCK block enable */
#define		bOFDMEN_Jaguar			0x20000000
#define		bCCKEN_Jaguar			0x10000000
#define		rRxPath_Jaguar			0x808	/* Rx antenna */
#define		bRxPath_Jaguar			0xff
#define		rTxPath_Jaguar			0x80c	/* Tx antenna */
#define		bTxPath_Jaguar			0x0fffffff
#define		rCCK_RX_Jaguar			0xa04	/* for cck rx path selection */
#define		bCCK_RX_Jaguar			0x0c000000
#define		rVhtlen_Use_Lsig_Jaguar	0x8c3	/* Use LSIG for VHT length */
VOID
PHY_BB8710B_Config_1T(
	IN PADAPTER Adapter
)
{
	/* BB OFDM RX Path_A */
	phy_set_bb_reg(Adapter, rRxPath_Jaguar, bRxPath_Jaguar, 0x11);
	/* BB OFDM TX Path_A */
	phy_set_bb_reg(Adapter, rTxPath_Jaguar, bMaskLWord, 0x1111);
	/* BB CCK R/Rx Path_A */
	phy_set_bb_reg(Adapter, rCCK_RX_Jaguar, bCCK_RX_Jaguar, 0x0);
	/* MCS support */
	phy_set_bb_reg(Adapter, 0x8bc, 0xc0000060, 0x4);
	/* RF Path_B HSSI OFF */
	phy_set_bb_reg(Adapter, 0xe00, 0xf, 0x4);
	/* RF Path_B Power Down */
	phy_set_bb_reg(Adapter, 0xe90, bMaskDWord, 0);
	/* ADDA Path_B OFF */
	phy_set_bb_reg(Adapter, 0xe60, bMaskDWord, 0);
	phy_set_bb_reg(Adapter, 0xe64, bMaskDWord, 0);
}
#endif

int
PHY_RFConfig8710B(
	IN	PADAPTER	Adapter
)
{
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(Adapter);
	int rtStatus = _SUCCESS;

	/* RF config */
	rtStatus = PHY_RF6052_Config8710B(Adapter);

	return rtStatus;
}

/*-----------------------------------------------------------------------------
 * Function:    PHY_ConfigRFWithParaFile()
 *
 * Overview:    This function read RF parameters from general file format, and do RF 3-wire
 *
 * Input:	PADAPTER			Adapter
 *			ps1Byte				pFileName
 *			enum rf_path				eRFPath
 *
 * Output:      NONE
 *
 * Return:      RT_STATUS_SUCCESS: configuration file exist
 *
 * Note:		Delay may be required for RF configuration
 *---------------------------------------------------------------------------*/
int
PHY_ConfigRFWithParaFile_8710B(
	IN	PADAPTER			Adapter,
	IN	u8					*pFileName,
	enum rf_path				eRFPath
)
{
	return _SUCCESS;
}

/**************************************************************************************************************
 *   Description:
 *       The low-level interface to set TxAGC , called by both MP and Normal Driver.
 *
 *                                                                                    <20120830, Kordan>
 **************************************************************************************************************/

VOID
PHY_SetTxPowerIndex_8710B(
	IN	PADAPTER			Adapter,
	IN	u32					PowerIndex,
	IN	enum rf_path			RFPath,
	IN	u8					Rate
)
{
	if (RFPath == RF_PATH_A || RFPath == RF_PATH_B) {
		switch (Rate) {
		case MGN_1M:
			phy_set_bb_reg(Adapter, rTxAGC_A_CCK1_Mcs32,      bMaskByte1, PowerIndex);
			break;
		case MGN_2M:
			phy_set_bb_reg(Adapter, rTxAGC_B_CCK11_A_CCK2_11, bMaskByte1, PowerIndex);
			break;
		case MGN_5_5M:
			phy_set_bb_reg(Adapter, rTxAGC_B_CCK11_A_CCK2_11, bMaskByte2, PowerIndex);
			break;
		case MGN_11M:
			phy_set_bb_reg(Adapter, rTxAGC_B_CCK11_A_CCK2_11, bMaskByte3, PowerIndex);
			break;

		case MGN_6M:
			phy_set_bb_reg(Adapter, rTxAGC_A_Rate18_06, bMaskByte0, PowerIndex);
			break;
		case MGN_9M:
			phy_set_bb_reg(Adapter, rTxAGC_A_Rate18_06, bMaskByte1, PowerIndex);
			break;
		case MGN_12M:
			phy_set_bb_reg(Adapter, rTxAGC_A_Rate18_06, bMaskByte2, PowerIndex);
			break;
		case MGN_18M:
			phy_set_bb_reg(Adapter, rTxAGC_A_Rate18_06, bMaskByte3, PowerIndex);
			break;

		case MGN_24M:
			phy_set_bb_reg(Adapter, rTxAGC_A_Rate54_24, bMaskByte0, PowerIndex);
			break;
		case MGN_36M:
			phy_set_bb_reg(Adapter, rTxAGC_A_Rate54_24, bMaskByte1, PowerIndex);
			break;
		case MGN_48M:
			phy_set_bb_reg(Adapter, rTxAGC_A_Rate54_24, bMaskByte2, PowerIndex);
			break;
		case MGN_54M:
			phy_set_bb_reg(Adapter, rTxAGC_A_Rate54_24, bMaskByte3, PowerIndex);
			break;

		case MGN_MCS0:
			phy_set_bb_reg(Adapter, rTxAGC_A_Mcs03_Mcs00, bMaskByte0, PowerIndex);
			break;
		case MGN_MCS1:
			phy_set_bb_reg(Adapter, rTxAGC_A_Mcs03_Mcs00, bMaskByte1, PowerIndex);
			break;
		case MGN_MCS2:
			phy_set_bb_reg(Adapter, rTxAGC_A_Mcs03_Mcs00, bMaskByte2, PowerIndex);
			break;
		case MGN_MCS3:
			phy_set_bb_reg(Adapter, rTxAGC_A_Mcs03_Mcs00, bMaskByte3, PowerIndex);
			break;

		case MGN_MCS4:
			phy_set_bb_reg(Adapter, rTxAGC_A_Mcs07_Mcs04, bMaskByte0, PowerIndex);
			break;
		case MGN_MCS5:
			phy_set_bb_reg(Adapter, rTxAGC_A_Mcs07_Mcs04, bMaskByte1, PowerIndex);
			break;
		case MGN_MCS6:
			phy_set_bb_reg(Adapter, rTxAGC_A_Mcs07_Mcs04, bMaskByte2, PowerIndex);
			break;
		case MGN_MCS7:
			phy_set_bb_reg(Adapter, rTxAGC_A_Mcs07_Mcs04, bMaskByte3, PowerIndex);
			break;

		default:
			RTW_INFO("Invalid Rate!!\n");
			break;
		}
	}
}

u8
PHY_GetTxPowerIndex_8710B(
	IN	PADAPTER			pAdapter,
	IN	enum rf_path			RFPath,
	IN	u8					Rate,
	IN	u8					BandWidth,
	IN	u8					Channel,
	struct txpwr_idx_comp *tic
)
{
	PHAL_DATA_TYPE pHalData = GET_HAL_DATA(pAdapter);
	s16 power_idx;
	u8 base_idx = 0;
	s8 by_rate_diff = 0, limit = 0, tpt_offset = 0, extra_bias = 0;
	BOOLEAN bIn24G = _FALSE;

	base_idx = PHY_GetTxPowerIndexBase(pAdapter, RFPath, Rate, RF_1TX, BandWidth, Channel, &bIn24G);

	by_rate_diff = PHY_GetTxPowerByRate(pAdapter, BAND_ON_2_4G, RF_PATH_A, Rate);
	limit = PHY_GetTxPowerLimit(pAdapter, NULL, (u8)(!bIn24G), pHalData->current_channel_bw, RFPath, Rate, RF_1TX, pHalData->current_channel);

	tpt_offset = PHY_GetTxPowerTrackingOffset(pAdapter, RFPath, Rate);

	if (tic) {
		tic->ntx_idx = RF_1TX;
		tic->base = base_idx;
		tic->by_rate = by_rate_diff;
		tic->limit = limit;
		tic->tpt = tpt_offset;
		tic->ebias = extra_bias;
	}

	by_rate_diff = by_rate_diff > limit ? limit : by_rate_diff;
	power_idx = base_idx + by_rate_diff + tpt_offset + extra_bias;

	if (power_idx < 0)
		power_idx = 0;
	else if (power_idx > MAX_POWER_INDEX)
		power_idx = MAX_POWER_INDEX;

	return power_idx;
}

VOID
PHY_SetTxPowerLevel8710B(
	IN	PADAPTER		Adapter,
	IN	u8				Channel
)
{
	PHAL_DATA_TYPE	pHalData = GET_HAL_DATA(Adapter);
	u8				cur_antenna;
	enum rf_path		RFPath = RF_PATH_A;

#ifdef CONFIG_ANTENNA_DIVERSITY
	rtw_hal_get_odm_var(Adapter, HAL_ODM_ANTDIV_SELECT, &cur_antenna, NULL);

	if (pHalData->AntDivCfg)  /* antenna diversity Enable */
		RFPath = ((cur_antenna == MAIN_ANT) ? RF_PATH_A : RF_PATH_B);
	else   /* antenna diversity disable */
#endif
		RFPath = pHalData->ant_path;



	phy_set_tx_power_level_by_path(Adapter, Channel, RFPath);

}

VOID
PHY_GetTxPowerLevel8710B(
	IN	PADAPTER		Adapter,
	OUT s32				*powerlevel
)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	s32				TxPwrDbm = 13;
#if 0

	if (pMgntInfo->ClientConfigPwrInDbm != UNSPECIFIED_PWR_DBM)
		*powerlevel = pMgntInfo->ClientConfigPwrInDbm;
	else
		*powerlevel = TxPwrDbm;
#endif
}


/* <20160217, Jessica> A workaround to eliminate the 2472MHz & 2484MHz spur of 8723D. */
VOID
phy_SpurCalibration_8710B(
	IN	PADAPTER					pAdapter,
	IN	u1Byte						ToChannel,
	IN	u1Byte						threshold
)
{
	u4Byte		freq[2] = {0xFCCD, 0xFF9A}; /* {chnl 13, 14} */
	u1Byte		idx = 0xFF;
	u1Byte		b_doNotch = FALSE;
	u1Byte		initial_gain;

	/* add for notch */
	u4Byte				wlan_channel, CurrentChannel;
	HAL_DATA_TYPE		*pHalData	= GET_HAL_DATA(pAdapter);
	struct PHY_DM_STRUCT		*pDM_Odm = &(pHalData->odmpriv);

	/* check threshold */
	if (threshold <= 0x0)
		threshold = 0x16;

	RTW_DBG("===>phy_SpurCalibration_8710B: Channel = %d\n", ToChannel);

	if (ToChannel == 13)
		idx = 0;
	else if (ToChannel == 14)
		idx = 1;

	/* If current channel=13,14 */
	if (idx < 0xFF) {
		initial_gain = (u1Byte)(odm_get_bb_reg(pDM_Odm, rOFDM0_XAAGCCore1, bMaskByte0) & 0x7f);
		odm_pause_dig(pDM_Odm, PHYDM_PAUSE, PHYDM_PAUSE_LEVEL_1, 0x30);
		phy_set_bb_reg(pAdapter, rFPGA0_AnalogParameter4, bMaskDWord, 0xccf000c0);		/* disable 3-wire */

		phy_set_bb_reg(pAdapter, rFPGA0_PSDFunction, bMaskDWord, freq[idx]);				/* Setup PSD */
		phy_set_bb_reg(pAdapter, rFPGA0_PSDFunction, bMaskDWord, 0x400000 | freq[idx]); /* Start PSD	 */

		rtw_msleep_os(30);

		if (phy_query_bb_reg(pAdapter, rFPGA0_PSDReport, bMaskDWord) >= threshold)
			b_doNotch = TRUE;

		phy_set_bb_reg(pAdapter, rFPGA0_PSDFunction, bMaskDWord, freq[idx]); /* turn off PSD */
		phy_set_bb_reg(pAdapter, rFPGA0_AnalogParameter4, bMaskDWord, 0xccc000c0);	/* enable 3-wire */
		odm_pause_dig(pDM_Odm, PHYDM_RESUME, PHYDM_PAUSE_LEVEL_1, NONE);
	}

	/* --- Notch Filter --- Asked by Rock	 */
	if (b_doNotch) {
		CurrentChannel = odm_get_rf_reg(pDM_Odm, RF_PATH_A, RF_CHNLBW, bRFRegOffsetMask);
		wlan_channel   = CurrentChannel & 0x0f;						    /* Get center frequency */

		switch (wlan_channel) {											/* Set notch filter				 */
		case 13:
			odm_set_bb_reg(pDM_Odm, 0xC40, BIT(28) | BIT(27) | BIT(26) | BIT(25) | BIT(24), 0xB);
			odm_set_bb_reg(pDM_Odm, 0xC40, BIT(9), 0x1);                    /* enable notch filter */
			odm_set_bb_reg(pDM_Odm, 0xD40, bMaskDWord, 0x04000000);
			odm_set_bb_reg(pDM_Odm, 0xD44, bMaskDWord, 0x00000000);
			odm_set_bb_reg(pDM_Odm, 0xD48, bMaskDWord, 0x00000000);
			odm_set_bb_reg(pDM_Odm, 0xD4C, bMaskDWord, 0x00000000);
			odm_set_bb_reg(pDM_Odm, 0xD2C, BIT(28), 0x1);                   /* enable CSI mask */
			break;
		case 14:
			odm_set_bb_reg(pDM_Odm, 0xC40, BIT(28) | BIT(27) | BIT(26) | BIT(25) | BIT(24), 0x5);
			odm_set_bb_reg(pDM_Odm, 0xC40, BIT(9), 0x1);                   /* enable notch filter */
			odm_set_bb_reg(pDM_Odm, 0xD40, bMaskDWord, 0x00000000);
			odm_set_bb_reg(pDM_Odm, 0xD44, bMaskDWord, 0x00000000);
			odm_set_bb_reg(pDM_Odm, 0xD48, bMaskDWord, 0x00000000);
			odm_set_bb_reg(pDM_Odm, 0xD4C, bMaskDWord, 0x00080000);
			odm_set_bb_reg(pDM_Odm, 0xD2C, BIT(28), 0x1);                   /* enable CSI mask */
			break;
		default:
			odm_set_bb_reg(pDM_Odm, 0xC40, BIT(9), 0x0);						/* disable notch filter */
			odm_set_bb_reg(pDM_Odm, 0xD2C, BIT(28), 0x0);                   /* disable CSI mask	function */
			break;
		} /* switch(wlan_channel)	 */
		return;
	}

	odm_set_bb_reg(pDM_Odm, 0xC40, BIT(28) | BIT(27) | BIT(26) | BIT(25) | BIT(24), 0x1f);
	odm_set_bb_reg(pDM_Odm, 0xC40, BIT(9), 0x0);                     /* disable notch filter */
	odm_set_bb_reg(pDM_Odm, 0xD40, bMaskDWord, 0x00000000);
	odm_set_bb_reg(pDM_Odm, 0xD44, bMaskDWord, 0x00000000);
	odm_set_bb_reg(pDM_Odm, 0xD48, bMaskDWord, 0x00000000);
	odm_set_bb_reg(pDM_Odm, 0xD4C, bMaskDWord, 0x00000000);
	odm_set_bb_reg(pDM_Odm, 0xD2C, BIT(28), 0x0);                    /* disable CSI mask */
}

VOID
phy_SetRegBW_8710B(
	IN	PADAPTER		Adapter,
	enum channel_width	CurrentBW
)
{
	u16	RegRfMod_BW, u2tmp = 0;

	RegRfMod_BW = rtw_read16(Adapter, REG_TRXPTCL_CTL_8710B);

	switch (CurrentBW) {
	case CHANNEL_WIDTH_20:
		rtw_write16(Adapter, REG_TRXPTCL_CTL_8710B, (RegRfMod_BW & 0xFE7F)); /* BIT 7 = 0, BIT 8 = 0 */
		break;

	case CHANNEL_WIDTH_40:
		u2tmp = RegRfMod_BW | BIT(7);
		rtw_write16(Adapter, REG_TRXPTCL_CTL_8710B, (u2tmp & 0xFEFF)); /* BIT 7 = 1, BIT 8 = 0 */
		break;

	default:
		RTW_INFO("phy_PostSetBWMode8710B(): unknown Bandwidth: %#X\n", CurrentBW);
		break;
	}
}

u8
phy_GetSecondaryChnl_8710B(
	IN	PADAPTER	Adapter
)
{
	u8	SCSettingOf40 = 0, SCSettingOf20 = 0;
	PHAL_DATA_TYPE	pHalData = GET_HAL_DATA(Adapter);

	if (pHalData->current_channel_bw == CHANNEL_WIDTH_80) {
		if (pHalData->nCur80MhzPrimeSC == HAL_PRIME_CHNL_OFFSET_LOWER)
			SCSettingOf40 = VHT_DATA_SC_40_LOWER_OF_80MHZ;
		else if (pHalData->nCur80MhzPrimeSC == HAL_PRIME_CHNL_OFFSET_UPPER)
			SCSettingOf40 = VHT_DATA_SC_40_UPPER_OF_80MHZ;


		if ((pHalData->nCur40MhzPrimeSC == HAL_PRIME_CHNL_OFFSET_LOWER) && (pHalData->nCur80MhzPrimeSC == HAL_PRIME_CHNL_OFFSET_LOWER))
			SCSettingOf20 = VHT_DATA_SC_20_LOWEST_OF_80MHZ;
		else if ((pHalData->nCur40MhzPrimeSC == HAL_PRIME_CHNL_OFFSET_UPPER) && (pHalData->nCur80MhzPrimeSC == HAL_PRIME_CHNL_OFFSET_LOWER))
			SCSettingOf20 = VHT_DATA_SC_20_LOWER_OF_80MHZ;
		else if ((pHalData->nCur40MhzPrimeSC == HAL_PRIME_CHNL_OFFSET_LOWER) && (pHalData->nCur80MhzPrimeSC == HAL_PRIME_CHNL_OFFSET_UPPER))
			SCSettingOf20 = VHT_DATA_SC_20_UPPER_OF_80MHZ;
		else if ((pHalData->nCur40MhzPrimeSC == HAL_PRIME_CHNL_OFFSET_UPPER) && (pHalData->nCur80MhzPrimeSC == HAL_PRIME_CHNL_OFFSET_UPPER))
			SCSettingOf20 = VHT_DATA_SC_20_UPPERST_OF_80MHZ;

	} else if (pHalData->current_channel_bw == CHANNEL_WIDTH_40) {

		if (pHalData->nCur40MhzPrimeSC == HAL_PRIME_CHNL_OFFSET_UPPER)
			SCSettingOf20 = VHT_DATA_SC_20_UPPER_OF_80MHZ;
		else if (pHalData->nCur40MhzPrimeSC == HAL_PRIME_CHNL_OFFSET_LOWER)
			SCSettingOf20 = VHT_DATA_SC_20_LOWER_OF_80MHZ;

	}

	return (SCSettingOf40 << 4) | SCSettingOf20;
}

void
phy_PostSetBwMode8710B(
	IN PADAPTER padapter
)
{
	u1Byte SubChnlNum = 0;
	HAL_DATA_TYPE *pHalData = GET_HAL_DATA(padapter);

	phy_SetRegBW_8710B(padapter, pHalData->current_channel_bw);

	SubChnlNum = phy_GetSecondaryChnl_8710B(padapter);
	rtw_write8(padapter, REG_DATA_SC_8710B, SubChnlNum);

	switch (pHalData->current_channel_bw) {
	case CHANNEL_WIDTH_20:
		phy_set_bb_reg(padapter, REG_FPGA0_RFMOD, bRFMOD, 0x0);			
		phy_set_bb_reg(padapter, rFPGA1_RFMOD, bRFMOD, 0x0);
		phy_set_bb_reg(padapter, REG_FPGA0_RFMOD, BIT10|BIT9|BIT8, 0x7);	//0x800[10:8] = 3'b111		
		phy_set_bb_reg(padapter, REG_FPGA0_RFMOD, BIT14|BIT13|BIT12, 0x5); //0x800[14:12] = 3'b101 	
		phy_set_bb_reg(padapter, rOFDM0_TxPseudoNoiseWgt, BIT31|BIT30, 0x0); //0xCE4[31:30] = 2'00 		
		phy_set_bb_reg(padapter, rOFDM0_TxPseudoNoiseWgt, BIT29|BIT28, 0x1); //0xCE4[29:28] = 2'01 	
		phy_set_bb_reg(padapter, rOFDM0_XARxAFE, BIT29|BIT28, 0x1); //0xC10[29:28] = 2'01	
		phy_set_bb_reg(padapter, REG_FPGA0_XB_RF_INTERFACE_OE, BIT30|BIT29, 0x1); //0x864[30:29] = 2'01	
		phy_set_bb_reg(padapter, rBBrx_DFIR, BIT19, 1); //0x954[19] = 1
		phy_set_bb_reg(padapter, rBBrx_DFIR, (BIT23|BIT22|BIT21|BIT20), 0x8); //0x954[23:20] = 4'1000
		phy_set_bb_reg(padapter, rBBrx_DFIR, (BIT27|BIT26|BIT25|BIT24), 0xa); //0x954[27:24] = 4'1010

		phy_set_rf_reg(padapter, RF_PATH_A, RF_CHNLBW, (BIT11|BIT10), 0x3);
		break;

	case CHANNEL_WIDTH_40:
		phy_set_bb_reg(padapter, REG_FPGA0_RFMOD, bRFMOD, 0x1);			
		phy_set_bb_reg(padapter, rFPGA1_RFMOD, bRFMOD, 0x1);
		phy_set_bb_reg(padapter, rCCK0_System, bCCKSideBand, (pHalData->nCur40MhzPrimeSC>>1));
		phy_set_bb_reg(padapter, REG_FPGA0_RFMOD, BIT10|BIT9|BIT8, 0x7);	//0x800[10:8] = 3'b111		
		phy_set_bb_reg(padapter, REG_FPGA0_RFMOD, BIT14|BIT13|BIT12, 0x5); //0x800[14:12] = 3'b101 	
		phy_set_bb_reg(padapter, rOFDM0_TxPseudoNoiseWgt, BIT31|BIT30, 0x0); //0xCE4[31:30] = 2'00 		
		phy_set_bb_reg(padapter, rOFDM0_TxPseudoNoiseWgt, BIT29|BIT28, 0x1); //0xCE4[29:28] = 2'01 	
		phy_set_bb_reg(padapter, rOFDM0_XARxAFE, BIT29|BIT28, 0x1); //0xC10[29:28] = 2'01	
		phy_set_bb_reg(padapter, REG_FPGA0_XB_RF_INTERFACE_OE, BIT30|BIT29, 0x1); //0x864[30:29] = 2'01	
		phy_set_bb_reg(padapter, rBBrx_DFIR, BIT19, 0); //0x954[19] = 0
		phy_set_bb_reg(padapter, rBBrx_DFIR, (BIT23|BIT22|BIT21|BIT20), 0x0); //0x954[23:20] = 0
		phy_set_bb_reg(padapter, rBBrx_DFIR, (BIT27|BIT26|BIT25|BIT24), 0x0); //0x954[27:24] = 0

		phy_set_rf_reg(padapter, RF_PATH_A, RF_CHNLBW, (BIT11|BIT10), 0x1);
		break;
		
	default:
		break;
	}
}

VOID
phy_ReviseCCKTxPSF8710B(	
	IN	PADAPTER		pAdapter, 
	IN	u1Byte			channelToSW
	)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	
	if(channelToSW == 14)
	{
		phy_set_bb_reg(pAdapter, pHalData->RegForRecover[0].offset, bMaskDWord, 0x0000B81C);//0xA24
		phy_set_bb_reg(pAdapter, pHalData->RegForRecover[1].offset, bMaskDWord, 0x00000000); //0xA28
		phy_set_bb_reg(pAdapter, pHalData->RegForRecover[2].offset, bMaskDWord, 0x00003667); //0xAAC
		phy_set_bb_reg(pAdapter, pHalData->RegForRecover[3].offset, bMaskDWord, pHalData->RegForRecover[3].value); //0xA20
	}
	else if(channelToSW == 13)
	{
		phy_set_bb_reg(pAdapter, pHalData->RegForRecover[0].offset, bMaskDWord, pHalData->RegForRecover[0].value);
		phy_set_bb_reg(pAdapter, pHalData->RegForRecover[1].offset, bMaskDWord, pHalData->RegForRecover[1].value);
		phy_set_bb_reg(pAdapter, pHalData->RegForRecover[2].offset, bMaskDWord, pHalData->RegForRecover[2].value);
		phy_set_bb_reg(pAdapter, pHalData->RegForRecover[3].offset, bMaskDWord, 0xd1d80001);
	}
	else
	{
		phy_set_bb_reg(pAdapter, pHalData->RegForRecover[0].offset, bMaskDWord, pHalData->RegForRecover[0].value);
		phy_set_bb_reg(pAdapter, pHalData->RegForRecover[1].offset, bMaskDWord, pHalData->RegForRecover[1].value);
		phy_set_bb_reg(pAdapter, pHalData->RegForRecover[2].offset, bMaskDWord, pHalData->RegForRecover[2].value);
		phy_set_bb_reg(pAdapter, pHalData->RegForRecover[3].offset, bMaskDWord, pHalData->RegForRecover[3].value);
	}
}

VOID
phy_SwChnl8710B(
	IN	PADAPTER					pAdapter
)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(pAdapter);
	u8		channelToSW = pHalData->current_channel;
	u8		i = 0;

	if (pHalData->rf_chip == RF_PSEUDO_11N) {
		RTW_WARN("phy_SwChnl8710B: return for PSEUDO\n");
		return;
	}
	
	//phydm_config_kfree(GET_PDM_ODM(pAdapter), channelToSW);
	phy_set_rf_reg(pAdapter, RF_PATH_A, RF_CHNLBW, 0x3FF, channelToSW);
	
	//suggested by mingzhi_guo, 20170906
	phy_ReviseCCKTxPSF8710B(pAdapter, channelToSW);
	
	//phy_SpurCalibration_8710B(pAdapter, channelToSW, 0x16);

	RTW_DBG("===>phy_SwChnl8710B: Channel = %d\n", channelToSW);
}

VOID
phy_SwChnlAndSetBwMode8710B(
	IN  PADAPTER		Adapter
)
{
	HAL_DATA_TYPE		*pHalData = GET_HAL_DATA(Adapter);

	if (Adapter->bNotifyChannelChange) {
		RTW_INFO("[%s] bSwChnl=%d, ch=%d, bSetChnlBW=%d, bw=%d\n",
			 __func__,
			 pHalData->bSwChnl,
			 pHalData->current_channel,
			 pHalData->bSetChnlBW,
			 pHalData->current_channel_bw);
	}

	if (RTW_CANNOT_RUN(Adapter))
		return;

	if (pHalData->bSwChnl) {
		phy_SwChnl8710B(Adapter);
		pHalData->bSwChnl = _FALSE;
	}

	if (pHalData->bSetChnlBW) {
		phy_PostSetBwMode8710B(Adapter);
		pHalData->bSetChnlBW = _FALSE;
	}

	PHY_SetTxPowerLevel8710B(Adapter, pHalData->current_channel);
}

VOID
PHY_HandleSwChnlAndSetBW8710B(
	IN	PADAPTER			Adapter,
	IN	BOOLEAN				bSwitchChannel,
	IN	BOOLEAN				bSetBandWidth,
	IN	u8					ChannelNum,
	IN	enum channel_width	ChnlWidth,
	IN	EXTCHNL_OFFSET	ExtChnlOffsetOf40MHz,
	IN	EXTCHNL_OFFSET	ExtChnlOffsetOf80MHz,
	IN	u8					CenterFrequencyIndex1
)
{
	/* static BOOLEAN		bInitialzed = _FALSE; */
	PHAL_DATA_TYPE		pHalData = GET_HAL_DATA(Adapter);
	u8					tmpChannel = pHalData->current_channel;
	enum channel_width	tmpBW = pHalData->current_channel_bw;
	u8					tmpnCur40MhzPrimeSC = pHalData->nCur40MhzPrimeSC;
	u8					tmpnCur80MhzPrimeSC = pHalData->nCur80MhzPrimeSC;
	u8					tmpCenterFrequencyIndex1 = pHalData->CurrentCenterFrequencyIndex1;
	struct mlme_ext_priv	*pmlmeext = &Adapter->mlmeextpriv;

	/* RTW_INFO("=> PHY_HandleSwChnlAndSetBW8812: bSwitchChannel %d, bSetBandWidth %d\n",bSwitchChannel,bSetBandWidth); */

	/* check is swchnl or setbw */
	if (!bSwitchChannel && !bSetBandWidth) {
		RTW_INFO("PHY_HandleSwChnlAndSetBW8812:  not switch channel and not set bandwidth\n");
		return;
	}

	/* skip change for channel or bandwidth is the same */
	if (bSwitchChannel) {
		/* if(pHalData->current_channel != ChannelNum) */
		{
			if (HAL_IsLegalChannel(Adapter, ChannelNum))
				pHalData->bSwChnl = _TRUE;
		}
	}

	if (bSetBandWidth) {
#if 0
		if (bInitialzed == _FALSE) {
			bInitialzed = _TRUE;
			pHalData->bSetChnlBW = _TRUE;
		} else if ((pHalData->current_channel_bw != ChnlWidth) || (pHalData->nCur40MhzPrimeSC != ExtChnlOffsetOf40MHz) || (pHalData->CurrentCenterFrequencyIndex1 != CenterFrequencyIndex1))
			pHalData->bSetChnlBW = _TRUE;
#else
		pHalData->bSetChnlBW = _TRUE;
#endif
	}

	if (!pHalData->bSetChnlBW && !pHalData->bSwChnl) {
		/* RTW_INFO("<= PHY_HandleSwChnlAndSetBW8812: bSwChnl %d, bSetChnlBW %d\n",pHalData->bSwChnl,pHalData->bSetChnlBW); */
		return;
	}


	if (pHalData->bSwChnl) {
		pHalData->current_channel = ChannelNum;
		pHalData->CurrentCenterFrequencyIndex1 = ChannelNum;
	}


	if (pHalData->bSetChnlBW) {
		pHalData->current_channel_bw = ChnlWidth;
#if 0
		if (ExtChnlOffsetOf40MHz == EXTCHNL_OFFSET_LOWER)
			pHalData->nCur40MhzPrimeSC = HAL_PRIME_CHNL_OFFSET_UPPER;
		else if (ExtChnlOffsetOf40MHz == EXTCHNL_OFFSET_UPPER)
			pHalData->nCur40MhzPrimeSC = HAL_PRIME_CHNL_OFFSET_LOWER;
		else
			pHalData->nCur40MhzPrimeSC = HAL_PRIME_CHNL_OFFSET_DONT_CARE;

		if (ExtChnlOffsetOf80MHz == EXTCHNL_OFFSET_LOWER)
			pHalData->nCur80MhzPrimeSC = HAL_PRIME_CHNL_OFFSET_UPPER;
		else if (ExtChnlOffsetOf80MHz == EXTCHNL_OFFSET_UPPER)
			pHalData->nCur80MhzPrimeSC = HAL_PRIME_CHNL_OFFSET_LOWER;
		else
			pHalData->nCur80MhzPrimeSC = HAL_PRIME_CHNL_OFFSET_DONT_CARE;
#else
		pHalData->nCur40MhzPrimeSC = ExtChnlOffsetOf40MHz;
		pHalData->nCur80MhzPrimeSC = ExtChnlOffsetOf80MHz;
#endif

		pHalData->CurrentCenterFrequencyIndex1 = CenterFrequencyIndex1;
	}

	/* Switch workitem or set timer to do switch channel or setbandwidth operation */
	if (!RTW_CANNOT_RUN(Adapter))
		phy_SwChnlAndSetBwMode8710B(Adapter);
	else {
		if (pHalData->bSwChnl) {
			pHalData->current_channel = tmpChannel;
			pHalData->CurrentCenterFrequencyIndex1 = tmpChannel;
		}
		if (pHalData->bSetChnlBW) {
			pHalData->current_channel_bw = tmpBW;
			pHalData->nCur40MhzPrimeSC = tmpnCur40MhzPrimeSC;
			pHalData->nCur80MhzPrimeSC = tmpnCur80MhzPrimeSC;
			pHalData->CurrentCenterFrequencyIndex1 = tmpCenterFrequencyIndex1;
		}
	}

	/* RTW_INFO("Channel %d ChannelBW %d ",pHalData->current_channel, pHalData->current_channel_bw); */
	/* RTW_INFO("40MhzPrimeSC %d 80MhzPrimeSC %d ",pHalData->nCur40MhzPrimeSC, pHalData->nCur80MhzPrimeSC); */
	/* RTW_INFO("CenterFrequencyIndex1 %d\n",pHalData->CurrentCenterFrequencyIndex1); */

	/* RTW_INFO("<= PHY_HandleSwChnlAndSetBW8812: bSwChnl %d, bSetChnlBW %d\n",pHalData->bSwChnl,pHalData->bSetChnlBW); */

}

VOID
PHY_SetSwChnlBWMode8710B(
	IN	PADAPTER			Adapter,
	IN	u8					channel,
	IN	enum channel_width	Bandwidth,
	IN	u8					Offset40,
	IN	u8					Offset80
)
{
	/* RTW_INFO("%s()===>\n",__FUNCTION__); */

	PHY_HandleSwChnlAndSetBW8710B(Adapter, _TRUE, _TRUE, channel, Bandwidth, Offset40, Offset80, channel);

	/* RTW_INFO("<==%s()\n",__FUNCTION__); */
}

static VOID
_PHY_DumpRFReg_8710B(IN	PADAPTER	pAdapter)
{
	u32 rfRegValue, rfRegOffset;


	for (rfRegOffset = 0x00; rfRegOffset <= 0x30; rfRegOffset++) {
		rfRegValue = PHY_QueryRFReg_8710B(pAdapter, RF_PATH_A, rfRegOffset, bMaskDWord);
	}
}
