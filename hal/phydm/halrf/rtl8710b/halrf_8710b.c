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

#include "mp_precomp.h"
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	#if RT_PLATFORM==PLATFORM_MACOSX
	#include "phydm_precomp.h"
	#else
	#include "../phydm_precomp.h"
	#endif
#else
#include "../../phydm_precomp.h"
#endif


#if (RTL8710B_SUPPORT == 1)

/*---------------------------Define Local Constant---------------------------*/
/*IQK*/
#define IQK_DELAY_TIME_8710B	10

/* 2010/04/25 MH Define the max tx power tracking tx agc power.*/
#define		ODM_TXPWRTRACK_MAX_IDX_8710B		6

#define     PATH_S1                         0
#define     idx_0xc94                       0
#define     idx_0xc80                       1
#define     idx_0xc4c                       2

#define     idx_0xc14                       0
#define     idx_0xca0                       1


#define     PATH_S0                         1
#define     idx_0xcd0                       0
#define     idx_0xcd4                       1

#define     idx_0xcd8                       0
#define     idx_0xcdc                       1

#define     KEY                             0
#define     VAL                             1



/*---------------------------Define Local Constant---------------------------*/


/* Tx Power Tracking*/


void set_iqk_matrix_8710b(
	struct PHY_DM_STRUCT	*p_dm,
	u8		OFDM_index,
	u8		rf_path,
	s32		iqk_result_x,
	s32		iqk_result_y
)
{
	s32			ele_A = 0, ele_D = 0, ele_C = 0, value32;
	s32			ele_A_ext = 0, ele_C_ext = 0, ele_D_ext = 0;

	if (OFDM_index >= OFDM_TABLE_SIZE)
		OFDM_index = OFDM_TABLE_SIZE - 1;
	else if (OFDM_index < 0)
		OFDM_index = 0;

	if ((iqk_result_x != 0) && (*(p_dm->p_band_type) == ODM_BAND_2_4G)) {

		/* new element D */
		ele_D = (ofdm_swing_table_new[OFDM_index] & 0xFFC00000) >> 22;
		ele_D_ext = (((iqk_result_x * ele_D) >> 7) & 0x01);
		/* new element A */
		if ((iqk_result_x & 0x00000200) != 0)		/* consider minus */
			iqk_result_x = iqk_result_x | 0xFFFFFC00;
		ele_A = ((iqk_result_x * ele_D) >> 8) & 0x000003FF;
		ele_A_ext = ((iqk_result_x * ele_D) >> 7) & 0x1;
		/* new element C */
		if ((iqk_result_y & 0x00000200) != 0)
			iqk_result_y = iqk_result_y | 0xFFFFFC00;
		ele_C = ((iqk_result_y * ele_D) >> 8) & 0x000003FF;
		ele_C_ext = ((iqk_result_y * ele_D) >> 7) & 0x1;

		switch (rf_path) {
		case RF_PATH_A:
			/* write new elements A, C, D to regC80, regC94, reg0xc4c, and element B is always 0 */
			/* write 0xc80 */
			value32 = (ele_D << 22) | ((ele_C & 0x3F) << 16) | ele_A;
			odm_set_bb_reg(p_dm, REG_OFDM_0_XA_TX_IQ_IMBALANCE, MASKDWORD, value32);
			/* write 0xc94 */
			value32 = (ele_C & 0x000003C0) >> 6;
			odm_set_bb_reg(p_dm, REG_OFDM_0_XC_TX_AFE, MASKH4BITS, value32);
			/* write 0xc4c */
			value32 = (ele_D_ext << 28) | (ele_A_ext << 31) | (ele_C_ext << 29);
			value32 = (odm_get_bb_reg(p_dm, REG_OFDM_0_ECCA_THRESHOLD, MASKDWORD) & (~(BIT(31) | BIT(29) | BIT(28)))) | value32;
			odm_set_bb_reg(p_dm, REG_OFDM_0_ECCA_THRESHOLD, MASKDWORD, value32);
			break;

		case RF_PATH_B:
			/*wirte new elements A, C, D to regCd0 and regCd4, element B is always 0*/
			value32 = ele_D;
			odm_set_bb_reg(p_dm, 0xCd4, 0x007FE000, value32);

			value32 = ele_C;
			odm_set_bb_reg(p_dm, 0xCd4, 0x000007FE, value32);

			value32 = ele_A;
			odm_set_bb_reg(p_dm, 0xCd0, 0x000007FE, value32);

			odm_set_bb_reg(p_dm, 0xCd4, BIT(12), ele_D_ext);
			odm_set_bb_reg(p_dm, 0xCd0, BIT(0), ele_A_ext);
			odm_set_bb_reg(p_dm, 0xCd4, BIT(0), ele_C_ext);
			break;
		default:
			break;
		}
	} else {
		switch (rf_path) {
		case RF_PATH_A:
			odm_set_bb_reg(p_dm, REG_OFDM_0_XA_TX_IQ_IMBALANCE, MASKDWORD, ofdm_swing_table_new[OFDM_index]);
			odm_set_bb_reg(p_dm, REG_OFDM_0_XC_TX_AFE, MASKH4BITS, 0x00);
			value32 = odm_get_bb_reg(p_dm, REG_OFDM_0_ECCA_THRESHOLD, MASKDWORD) & (~(BIT(31) | BIT(29) | BIT(28)));
			odm_set_bb_reg(p_dm, REG_OFDM_0_ECCA_THRESHOLD, MASKDWORD, value32);
			break;

		case RF_PATH_B:
			/*image S1:c80 to S0:Cd0 and Cd4*/
			odm_set_bb_reg(p_dm, 0xcd0, 0x000007FE, ofdm_swing_table_new[OFDM_index] & 0x000003FF);
			odm_set_bb_reg(p_dm, 0xcd0, 0x0007E000, (ofdm_swing_table_new[OFDM_index] & 0x0000FC00) >> 10);
			odm_set_bb_reg(p_dm, 0xcd4, 0x0000007E, (ofdm_swing_table_new[OFDM_index] & 0x003F0000) >> 16);
			odm_set_bb_reg(p_dm, 0xcd4, 0x007FE000, (ofdm_swing_table_new[OFDM_index] & 0xFFC00000) >> 22);

			odm_set_bb_reg(p_dm, 0xcd4, 0x00000780, 0x00);

			odm_set_bb_reg(p_dm, 0xcd4, BIT(12), 0x0);
			odm_set_bb_reg(p_dm, 0xcd4, BIT(0), 0x0);
			odm_set_bb_reg(p_dm, 0xcd0, BIT(0), 0x0);
			break;
		default:
			break;
		}
	}
	ODM_RT_TRACE(p_dm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("TxPwrTracking path %c: X = 0x%x, Y = 0x%x ele_A = 0x%x ele_C = 0x%x ele_D = 0x%x ele_A_ext = 0x%x ele_C_ext = 0x%x ele_D_ext = 0x%x\n",
		(rf_path == RF_PATH_A ? 'A' : 'B'), (u32)iqk_result_x, (u32)iqk_result_y, (u32)ele_A, (u32)ele_C, (u32)ele_D, (u32)ele_A_ext, (u32)ele_C_ext, (u32)ele_D_ext));
}

void
set_cck_filter_coefficient_8710b(
	struct PHY_DM_STRUCT	*p_dm,
	u8		cck_swing_index
)
{
	odm_set_bb_reg(p_dm, 0xab4, 0x000007FF, cck_swing_table_ch1_ch14_8710b[cck_swing_index]);
}

void do_iqk_8710b(
	void		*p_dm_void,
	u8		delta_thermal_index,
	u8		thermal_value,
	u8		threshold
)
{
	//u32  is_bt_enable = 0;
	struct PHY_DM_STRUCT	*p_dm = (struct PHY_DM_STRUCT *)p_dm_void;
#if 0
	if (*(p_dm->p_mp_mode) == false)
		is_bt_enable = odm_get_mac_reg(p_dm, 0xa8, MASKDWORD) & BIT(17);

	if (is_bt_enable) {
		ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]Skip IQK because BT is enable\n"));
		return;
	} else
		ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]Do IQK because BT is disable\n"));
#endif
	odm_reset_iqk_result(p_dm);
	p_dm->rf_calibrate_info.thermal_value_iqk = thermal_value;
	halrf_iqk_trigger(p_dm, false);
}

/*-----------------------------------------------------------------------------
 * Function:	odm_tx_pwr_track_set_pwr_8710b()
 *
 * Overview:	8710B change all channel tx power accordign to flag.
 *				OFDM & CCK are all different.
 *
 * Input:		NONE
 *
 * Output:		NONE
 *
 * Return:		NONE
 *
 * Revised History:
 *	When		Who		Remark
 *	04/23/2012	MHC		Create version 0.
 *
 *---------------------------------------------------------------------------*/
void
odm_tx_pwr_track_set_pwr_8710b(
	void		*p_dm_void,
	enum pwrtrack_method	method,
	u8				rf_path,
	u8				channel_mapped_index
)
{

	struct PHY_DM_STRUCT	*p_dm = (struct PHY_DM_STRUCT *)p_dm_void;

	struct _ADAPTER				*adapter = p_dm->adapter;
	//PHAL_DATA_TYPE			p_hal_data = GET_HAL_DATA(adapter);
	struct odm_rf_calibration_structure			*p_rf_calibrate_info = &(p_dm->rf_calibrate_info);
	struct _hal_rf_	*p_rf = &(p_dm->rf_table);
	u8					pwr_tracking_limit_ofdm = 32;
	u8					pwr_tracking_limit_cck = 36;
	u8					tx_rate = 0xFF;
	u8					final_ofdm_swing_index = 0;
	u8					final_cck_swing_index = 0;
	u8					i = 0;

if (*(p_dm->p_mp_mode) == true) {
#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN | ODM_CE))
#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)
#if (MP_DRIVER == 1)
		PMPT_CONTEXT p_mpt_ctx = &(adapter->MptCtx);

		tx_rate = MptToMgntRate(p_mpt_ctx->MptRateIndex);
#endif
#elif (DM_ODM_SUPPORT_TYPE & ODM_CE)
#ifdef CONFIG_MP_INCLUDED
		PMPT_CONTEXT p_mpt_ctx = &(adapter->mppriv.mpt_ctx);

		tx_rate = mpt_to_mgnt_rate(p_mpt_ctx->mpt_rate_index);
#endif
#endif
#endif
	} else {
		u16	rate	 = *(p_dm->p_forced_data_rate);

		if (!rate) { /*auto rate*/
			if (rate != 0xFF && p_dm->tx_rate != 0xFF) {
#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)
				tx_rate = adapter->HalFunc.GetHwRateFromMRateHandler(p_dm->tx_rate);
#elif (DM_ODM_SUPPORT_TYPE & ODM_CE)
				if (p_dm->number_linked_client != 0)
					tx_rate = hw_rate_to_m_rate(p_dm->tx_rate);
				else
					tx_rate = p_rf->p_rate_index;

#endif
			}
		} else   /*force rate*/
			tx_rate = (u8)rate;
	}


	ODM_RT_TRACE(p_dm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("===>ODM_TxPwrTrackSetPwr8710BA\n"));

	if (tx_rate != 0xFF) {
		/*CCK*/
		if (((tx_rate >= MGN_1M) && (tx_rate <= MGN_5_5M)) || tx_rate == MGN_11M)
			pwr_tracking_limit_cck = 36;
		/*OFDM*/
		else if ((tx_rate >= MGN_6M) && (tx_rate <= MGN_48M))

			pwr_tracking_limit_ofdm = 32;

		else if (tx_rate == MGN_54M)

			pwr_tracking_limit_ofdm = 32;


		/* HT*/
		else if ((tx_rate >= MGN_MCS0) && (tx_rate <= MGN_MCS2))

			pwr_tracking_limit_ofdm = 32;

		else if ((tx_rate >= MGN_MCS3) && (tx_rate <= MGN_MCS4))

			pwr_tracking_limit_ofdm = 32;

		else if ((tx_rate >= MGN_MCS5) && (tx_rate <= MGN_MCS7))

			pwr_tracking_limit_ofdm = 32;

		else if ((tx_rate >= MGN_MCS8) && (tx_rate <= MGN_MCS10))

			pwr_tracking_limit_ofdm = 32;

		else if ((tx_rate >= MGN_MCS11) && (tx_rate <= MGN_MCS12))

			pwr_tracking_limit_ofdm = 32;

		else if ((tx_rate >= MGN_MCS13) && (tx_rate <= MGN_MCS15))

			pwr_tracking_limit_ofdm = 32;

		else
			pwr_tracking_limit_ofdm =  p_rf_calibrate_info->default_ofdm_index;   /*Default OFDM index = 30 */
	}

	if (method == TXAGC) {
		u8	rf = 0;
		u32	pwr = 0, tx_agc = 0;
		//struct _ADAPTER *adapter = p_dm->adapter;

		ODM_RT_TRACE(p_dm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("odm_TxPwrTrackSetPwr_8710B CH=%d\n", *(p_dm->p_channel)));

		p_rf_calibrate_info->remnant_ofdm_swing_idx[rf_path] = p_rf_calibrate_info->absolute_ofdm_swing_idx[rf_path];   /* Remnant index equal to aboslute compensate value. */

#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN | ODM_CE))

#if (MP_DRIVER != 1)
#if 0
		PHY_SetTxPowerLevelByPath8710B(adapter, *p_dm->p_channel, rf_path);   /* Using new set power function */
		/* PHY_SetTxPowerLevel8710B(p_dm->adapter, *p_dm->p_channel); */
#endif
		p_rf_calibrate_info->modify_tx_agc_flag_path_a = true;
		p_rf_calibrate_info->modify_tx_agc_flag_path_b = true;
		p_rf_calibrate_info->modify_tx_agc_flag_path_a_cck = true;
		if (rf_path == RF_PATH_A) {
			odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_A, *p_dm->p_channel, CCK);
			odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_A, *p_dm->p_channel, OFDM);
			odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_A, *p_dm->p_channel, HT_MCS0_MCS7);
			odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_A, *p_dm->p_channel, HT_MCS8_MCS15);
		} else {
			odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_B, *p_dm->p_channel, CCK);
			odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_B, *p_dm->p_channel, OFDM);
			odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_B, *p_dm->p_channel, HT_MCS0_MCS7);
			odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_B, *p_dm->p_channel, HT_MCS8_MCS15);
		}
#else

		if (rf_path == RF_PATH_A) {
			/*CCK path S1*/
			pwr = odm_get_bb_reg(p_dm, REG_TX_AGC_A_RATE18_06, 0xFF);
			pwr += p_rf_calibrate_info->power_index_offset[RF_PATH_A];
			odm_set_bb_reg(p_dm, REG_TX_AGC_A_CCK_1_MCS32, MASKBYTE1, pwr);
			tx_agc = (pwr << 16) | (pwr << 8) | (pwr);
			odm_set_bb_reg(p_dm, REG_TX_AGC_B_CCK_11_A_CCK_2_11, 0x00ffffff, tx_agc);
			RT_DISP(FPHY, PHY_TXPWR, ("odm_tx_pwr_track_set_pwr_8710b: CCK Tx-rf(A) Power = 0x%x\n", tx_agc));

			/*OFDM path S1*/
			pwr = odm_get_bb_reg(p_dm, REG_TX_AGC_A_RATE18_06, 0xFF);
			pwr += (p_rf_calibrate_info->bb_swing_idx_ofdm[RF_PATH_A] - p_rf_calibrate_info->bb_swing_idx_ofdm_base[RF_PATH_A]);
			tx_agc = ((pwr << 24) | (pwr << 16) | (pwr << 8) | pwr);
			odm_set_bb_reg(p_dm, REG_TX_AGC_A_RATE18_06, MASKDWORD, tx_agc);
			odm_set_bb_reg(p_dm, REG_TX_AGC_A_RATE54_24, MASKDWORD, tx_agc);
			odm_set_bb_reg(p_dm, REG_TX_AGC_A_MCS03_MCS00, MASKDWORD, tx_agc);
			odm_set_bb_reg(p_dm, REG_TX_AGC_A_MCS07_MCS04, MASKDWORD, tx_agc);
			odm_set_bb_reg(p_dm, REG_TX_AGC_A_MCS11_MCS08, MASKDWORD, tx_agc);
			odm_set_bb_reg(p_dm, REG_TX_AGC_A_MCS15_MCS12, MASKDWORD, tx_agc);
			RT_DISP(FPHY, PHY_TXPWR, ("odm_tx_pwr_track_set_pwr_8710b: OFDM Tx-rf(A) Power = 0x%x\n", tx_agc));
		} else if (rf_path == RF_PATH_B) {

			pwr = odm_get_bb_reg(p_dm, REG_TX_AGC_B_RATE18_06, 0xFF);
			pwr += p_rf_calibrate_info->power_index_offset[RF_PATH_B];
			odm_set_bb_reg(p_dm, REG_TX_AGC_B_CCK_1_55_MCS32, MASKBYTE3, pwr);
			odm_set_bb_reg(p_dm, REG_TX_AGC_B_CCK_11_A_CCK_2_11, 0xff000000, pwr);
			RT_DISP(FPHY, PHY_TXPWR, ("odm_tx_pwr_track_set_pwr_8710b: CCK Tx-rf(B) Power = 0x%x\n", pwr));


			pwr = odm_get_bb_reg(p_dm, REG_TX_AGC_B_RATE18_06, 0xFF);
			pwr += (p_rf_calibrate_info->bb_swing_idx_ofdm[RF_PATH_B] - p_rf_calibrate_info->bb_swing_idx_ofdm_base[RF_PATH_B]);
			tx_agc = ((pwr << 24) | (pwr << 16) | (pwr << 8) | pwr);
			odm_set_bb_reg(p_dm, REG_TX_AGC_B_RATE18_06, MASKDWORD, tx_agc);
			odm_set_bb_reg(p_dm, REG_TX_AGC_B_RATE54_24, MASKDWORD, tx_agc);
			odm_set_bb_reg(p_dm, REG_TX_AGC_B_MCS03_MCS00, MASKDWORD, tx_agc);
			odm_set_bb_reg(p_dm, REG_TX_AGC_B_MCS07_MCS04, MASKDWORD, tx_agc);
			odm_set_bb_reg(p_dm, REG_TX_AGC_B_MCS11_MCS08, MASKDWORD, tx_agc);
			odm_set_bb_reg(p_dm, REG_TX_AGC_B_MCS15_MCS12, MASKDWORD, tx_agc);
			RT_DISP(FPHY, PHY_TXPWR, ("odm_tx_pwr_track_set_pwr_8710b: OFDM Tx-rf(B) Power = 0x%x\n", tx_agc));
		}
#endif

#endif
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
		/*phy_rf6052_set_cck_tx_power(p_dm->priv, *(p_dm->p_channel));
		  phy_rf6052_set_ofdm_tx_power(p_dm->priv, *(p_dm->p_channel));*/
#endif

	} else if (method == BBSWING) {
		final_ofdm_swing_index = p_rf_calibrate_info->default_ofdm_index + p_rf_calibrate_info->absolute_ofdm_swing_idx[rf_path];
		final_cck_swing_index = p_rf_calibrate_info->default_cck_index + p_rf_calibrate_info->absolute_ofdm_swing_idx[rf_path];

		ODM_RT_TRACE(p_dm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD,
			(" p_rf_calibrate_info->default_ofdm_index=%d,  p_rf_calibrate_info->DefaultCCKIndex=%d, p_rf_calibrate_info->absolute_ofdm_swing_idx[rf_path]=%d, p_rf_calibrate_info->remnant_cck_swing_idx=%d   rf_path = %d\n",
			p_rf_calibrate_info->default_ofdm_index, p_rf_calibrate_info->default_cck_index, p_rf_calibrate_info->absolute_ofdm_swing_idx[rf_path], p_rf_calibrate_info->remnant_cck_swing_idx, rf_path));

		/* Adjust BB swing by OFDM IQ matrix */
		if (final_ofdm_swing_index >= pwr_tracking_limit_ofdm)
			final_ofdm_swing_index = pwr_tracking_limit_ofdm;
		else if (final_ofdm_swing_index < 0)
			final_ofdm_swing_index = 0;

		if (final_cck_swing_index >= CCK_TABLE_SIZE_8710B)
			final_cck_swing_index = CCK_TABLE_SIZE_8710B - 1;
		else if (p_rf_calibrate_info->bb_swing_idx_cck < 0)
			final_cck_swing_index = 0;

		set_iqk_matrix_8710b(p_dm, final_ofdm_swing_index, RF_PATH_A,
			p_rf_calibrate_info->iqk_matrix_reg_setting[channel_mapped_index].value[0][0],
			p_rf_calibrate_info->iqk_matrix_reg_setting[channel_mapped_index].value[0][1]);

		set_iqk_matrix_8710b(p_dm, final_ofdm_swing_index, RF_PATH_B,
			p_rf_calibrate_info->iqk_matrix_reg_setting[channel_mapped_index].value[0][4],
			p_rf_calibrate_info->iqk_matrix_reg_setting[channel_mapped_index].value[0][5]);

		set_cck_filter_coefficient_8710b(p_dm, final_cck_swing_index);

		p_rf_calibrate_info->modify_tx_agc_flag_path_a = true;

		odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_A, *p_dm->p_channel, CCK);
		odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_A, *p_dm->p_channel, OFDM);
		odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_A, *p_dm->p_channel, HT_MCS0_MCS7);

		ODM_RT_TRACE(p_dm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("final_cck_swing_index=%d\n", final_cck_swing_index));

	} else if (method == MIX_MODE) {
#if (MP_DRIVER == 1)
		u32	tx_agc = 0;			  /*add by Mingzhi.Guo 2015-04-10*/
		s32	pwr = 0;
#endif
		ODM_RT_TRACE(p_dm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("p_dm->default_ofdm_index=%d,  p_dm->DefaultCCKIndex=%d, p_dm->absolute_ofdm_swing_idx[rf_path]=%d, rf_path = %d\n",
			p_rf_calibrate_info->default_ofdm_index, p_rf_calibrate_info->default_cck_index, p_rf_calibrate_info->absolute_ofdm_swing_idx[rf_path], rf_path));

		final_ofdm_swing_index = p_rf_calibrate_info->default_ofdm_index + p_rf_calibrate_info->absolute_ofdm_swing_idx[rf_path];

		if (rf_path == RF_PATH_A) {
			final_cck_swing_index = p_rf_calibrate_info->default_cck_index + p_rf_calibrate_info->absolute_ofdm_swing_idx[rf_path];    /*CCK Follow path-A and lower CCK index means higher power.*/

			if (final_ofdm_swing_index > pwr_tracking_limit_ofdm) {
				p_rf_calibrate_info->remnant_ofdm_swing_idx[rf_path] = final_ofdm_swing_index - pwr_tracking_limit_ofdm;

				set_iqk_matrix_8710b(p_dm, pwr_tracking_limit_ofdm, RF_PATH_A,
					p_rf_calibrate_info->iqk_matrix_reg_setting[channel_mapped_index].value[0][0],
					p_rf_calibrate_info->iqk_matrix_reg_setting[channel_mapped_index].value[0][1]);
				set_iqk_matrix_8710b(p_dm, pwr_tracking_limit_ofdm, RF_PATH_B,
					p_rf_calibrate_info->iqk_matrix_reg_setting[channel_mapped_index].value[0][4],
					p_rf_calibrate_info->iqk_matrix_reg_setting[channel_mapped_index].value[0][5]);

				p_rf_calibrate_info->modify_tx_agc_flag_path_a = true;

				/*Set tx_agc Page C{};*/
				/*odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_A, *p_dm->p_channel, OFDM);*/
				/*odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_A,  *p_dm->p_channel, HT_MCS0_MCS7);*/

				ODM_RT_TRACE(p_dm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("******Path_A Over BBSwing Limit, pwr_tracking_limit = %d, Remnant tx_agc value = %d\n", pwr_tracking_limit_ofdm, p_rf_calibrate_info->remnant_ofdm_swing_idx[rf_path]));
			} else if (final_ofdm_swing_index < 0) {
				p_rf_calibrate_info->remnant_ofdm_swing_idx[rf_path] = final_ofdm_swing_index ;

				set_iqk_matrix_8710b(p_dm, 0, RF_PATH_A,
					p_rf_calibrate_info->iqk_matrix_reg_setting[channel_mapped_index].value[0][0],
					p_rf_calibrate_info->iqk_matrix_reg_setting[channel_mapped_index].value[0][1]);
				set_iqk_matrix_8710b(p_dm, 0, RF_PATH_B,
					p_rf_calibrate_info->iqk_matrix_reg_setting[channel_mapped_index].value[0][4],
					p_rf_calibrate_info->iqk_matrix_reg_setting[channel_mapped_index].value[0][5]);

				p_rf_calibrate_info->modify_tx_agc_flag_path_a = true;

				/*Set tx_agc Page C{};*/
				/*odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_A, *p_dm->p_channel, OFDM);*/
				/*odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_A, *p_dm->p_channel, HT_MCS0_MCS7);*/

				ODM_RT_TRACE(p_dm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("******Path_A Lower then BBSwing lower bound  0, Remnant tx_agc value = %d\n", p_rf_calibrate_info->remnant_ofdm_swing_idx[rf_path]));
			} else {
				set_iqk_matrix_8710b(p_dm, final_ofdm_swing_index, RF_PATH_A,
					p_rf_calibrate_info->iqk_matrix_reg_setting[channel_mapped_index].value[0][0],
					p_rf_calibrate_info->iqk_matrix_reg_setting[channel_mapped_index].value[0][1]);
				set_iqk_matrix_8710b(p_dm, final_ofdm_swing_index, RF_PATH_B,
					p_rf_calibrate_info->iqk_matrix_reg_setting[channel_mapped_index].value[0][4],
					p_rf_calibrate_info->iqk_matrix_reg_setting[channel_mapped_index].value[0][5]);

				ODM_RT_TRACE(p_dm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("******Path_A Compensate with BBSwing, final_ofdm_swing_index = %d\n", final_ofdm_swing_index));

				if (p_rf_calibrate_info->modify_tx_agc_flag_path_a) {
					p_rf_calibrate_info->remnant_ofdm_swing_idx[rf_path] = 0;

					/*Set tx_agc Page C{};*/
					/*odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_A, *p_dm->p_channel, OFDM );*/
					/*odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_A, *p_dm->p_channel, HT_MCS0_MCS7 );*/
					/*odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_A, p_hal_data->current_channel, HT_MCS8_MCS15 );*/

					p_rf_calibrate_info->modify_tx_agc_flag_path_a = false;

					ODM_RT_TRACE(p_dm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("******Path_A p_dm->Modify_TxAGC_Flag = false\n"));
				}
			}
#if (0)  /*MP_DRIVER == 1*/
			if ((p_dm->mp_mode) == 1) {
				
				ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("~~~~~beforeAGC~~~~~~0xc80 = 0x%x  0xe14= 0x%x rf0x0 = 0x%x \n", odm_get_bb_reg(p_dm, 0xc80, MASKDWORD), odm_get_bb_reg(p_dm, 0xe14, MASKDWORD), odm_get_rf_reg(p_dm, RF_PATH_A, 0x0, bRFRegOffsetMask)));
				pwr = odm_get_bb_reg(p_dm, REG_TX_AGC_A_RATE18_06, 0xFF);
				pwr += (p_rf_calibrate_info->remnant_ofdm_swing_idx[RF_PATH_A] - p_rf_calibrate_info->modify_tx_agc_value_ofdm);

				if (pwr > 0x3F)
					pwr = 0x3F;
				else if (pwr < 0)
					pwr = 0;

				tx_agc |= ((pwr << 24) | (pwr << 16) | (pwr << 8) | pwr);
				odm_set_bb_reg(p_dm, REG_TX_AGC_A_RATE18_06, MASKDWORD, tx_agc);
				odm_set_bb_reg(p_dm, REG_TX_AGC_A_RATE54_24, MASKDWORD, tx_agc);
				odm_set_bb_reg(p_dm, REG_TX_AGC_A_MCS03_MCS00, MASKDWORD, tx_agc);
				odm_set_bb_reg(p_dm, REG_TX_AGC_A_MCS07_MCS04, MASKDWORD, tx_agc);

				ODM_RT_TRACE(p_dm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("ODM_TxPwrTrackSetPwr8188F: OFDM Tx-rf(A) Power = 0x%x\n", tx_agc));
				ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("~~~~~afterAGC~~~~~~0xc80 = 0x%x  0xe14= 0x%x rf0x0 = 0x%x \n", odm_get_bb_reg(p_dm, 0xc80, MASKDWORD), odm_get_bb_reg(p_dm, 0xe14, MASKDWORD), odm_get_rf_reg(p_dm, RF_PATH_A, 0x0, bRFRegOffsetMask)));

			} else
#endif
			/*{
				ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("~~~~~beforeAGC~~~~~~0xc80 = 0x%x  0xe14= 0x%x rf0x0 = 0x%x \n", odm_get_bb_reg(p_dm, 0xc80, MASKDWORD), odm_get_bb_reg(p_dm, 0xe14, MASKDWORD), odm_get_rf_reg(p_dm, RF_PATH_A, 0x0, bRFRegOffsetMask)));
			*/
				odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_A, *p_dm->p_channel, OFDM);
				odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_A, *p_dm->p_channel, HT_MCS0_MCS7);
			/*	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("~~~~~afterAGC~~~~~~0xc80 = 0x%x  0xe14= 0x%x rf0x0 = 0x%x \n", odm_get_bb_reg(p_dm, 0xc80, MASKDWORD), odm_get_bb_reg(p_dm, 0xe14, MASKDWORD), odm_get_rf_reg(p_dm, RF_PATH_A, 0x0, bRFRegOffsetMask)));

			} */
			p_rf_calibrate_info->modify_tx_agc_value_ofdm = p_rf_calibrate_info->remnant_ofdm_swing_idx[RF_PATH_A] ;

			if (final_cck_swing_index > pwr_tracking_limit_cck) {
				p_rf_calibrate_info->remnant_cck_swing_idx = final_cck_swing_index - pwr_tracking_limit_cck;

				ODM_RT_TRACE(p_dm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("******Path_A CCK Over Limit, pwr_tracking_limit_cck = %d, p_dm->remnant_cck_swing_idx  = %d\n", pwr_tracking_limit_cck, p_rf_calibrate_info->remnant_cck_swing_idx));

				/* Adjust BB swing by CCK filter coefficient*/
				odm_set_bb_reg(p_dm, 0xab4, 0x000007FF, cck_swing_table_ch1_ch14_8710b[pwr_tracking_limit_cck]);

				p_rf_calibrate_info->modify_tx_agc_flag_path_a_cck = true;

				/*Set tx_agc Page C{};*/
					/*odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_A, *p_dm->p_channel, CCK);*/
					/*odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_B, *p_dm->p_channel, CCK);*/

			} else if (final_cck_swing_index < 0) {
				p_rf_calibrate_info->remnant_cck_swing_idx = final_cck_swing_index;

				ODM_RT_TRACE(p_dm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("******Path_A CCK Under Limit, pwr_tracking_limit_cck = %d, p_dm->remnant_cck_swing_idx  = %d\n", 0, p_rf_calibrate_info->remnant_cck_swing_idx));

				odm_set_bb_reg(p_dm, 0xab4, 0x000007FF, cck_swing_table_ch1_ch14_8710b[0]);

				p_rf_calibrate_info->modify_tx_agc_flag_path_a_cck = true;


				/*odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_A, *p_dm->p_channel, CCK);*/
				/*odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_B, *p_dm->p_channel, CCK);*/

			} else {
				ODM_RT_TRACE(p_dm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("******Path_A CCK Compensate with BBSwing, final_cck_swing_index = %d\n", final_cck_swing_index));

				odm_set_bb_reg(p_dm, 0xab4, 0x000007FF, cck_swing_table_ch1_ch14_8710b[final_cck_swing_index]);

				/*	if (p_rf_calibrate_info->modify_tx_agc_flag_path_a_cck) {*/
				p_rf_calibrate_info->remnant_cck_swing_idx = 0;


				/*odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_A, *p_dm->p_channel, CCK );*/
				/*odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_B, *p_dm->p_channel, CCK );*/

				p_rf_calibrate_info->modify_tx_agc_flag_path_a_cck = false;

				ODM_RT_TRACE(p_dm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("******Path_A p_dm->Modify_TxAGC_Flag_CCK = false\n"));
			
				}
#if (MP_DRIVER == 1)
			if ((*(p_dm->p_mp_mode)) == 1) {
				pwr = odm_get_bb_reg(p_dm, REG_TX_AGC_B_CCK_11_A_CCK_2_11, MASKBYTE1);
				pwr += p_rf_calibrate_info->remnant_cck_swing_idx - p_rf_calibrate_info->modify_tx_agc_value_cck;

				if (pwr > 0x3F)
					pwr = 0x3F;
				else if (pwr < 0)
					pwr = 0;

				odm_set_bb_reg(p_dm, REG_TX_AGC_A_CCK_1_MCS32, MASKBYTE1, pwr);
				tx_agc = (pwr << 16) | (pwr << 8) | (pwr);
				odm_set_bb_reg(p_dm, REG_TX_AGC_B_CCK_11_A_CCK_2_11, 0xffffff00, tx_agc);
				ODM_RT_TRACE(p_dm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("ODM_TxPwrTrackSetPwr8710B: CCK Tx-rf(A) Power = 0x%x\n", tx_agc));
			} else
#endif

				odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_A, *p_dm->p_channel, CCK);

			p_rf_calibrate_info->modify_tx_agc_value_cck = p_rf_calibrate_info->remnant_cck_swing_idx;
		}
#if 0
		if (rf_path == RF_PATH_B) {
			if (final_ofdm_swing_index > pwr_tracking_limit_ofdm) {
				p_rf_calibrate_info->remnant_ofdm_swing_idx[rf_path] = final_ofdm_swing_index - pwr_tracking_limit_ofdm;

				set_iqk_matrix_8710b(p_dm, pwr_tracking_limit_ofdm, RF_PATH_B,
					p_rf_calibrate_info->iqk_matrix_reg_setting[channel_mapped_index].value[0][4],
					p_rf_calibrate_info->iqk_matrix_reg_setting[channel_mapped_index].value[0][5]);

				p_rf_calibrate_info->modify_tx_agc_flag_path_a = true;


				/*odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_B, p_hal_data->current_channel, OFDM);
				odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_B, p_hal_data->current_channel, HT_MCS0_MCS7);*/

				ODM_RT_TRACE(p_dm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("******Path_B Over BBSwing Limit, pwr_tracking_limit = %d, Remnant tx_agc value = %d\n", pwr_tracking_limit_ofdm, p_rf_calibrate_info->remnant_ofdm_swing_idx[rf_path]));
			} else if (final_ofdm_swing_index < 0) {
				p_rf_calibrate_info->remnant_ofdm_swing_idx[rf_path] = final_ofdm_swing_index ;

				set_iqk_matrix_8710b(p_dm, 0, RF_PATH_B,
					p_rf_calibrate_info->iqk_matrix_reg_setting[channel_mapped_index].value[0][4],
					p_rf_calibrate_info->iqk_matrix_reg_setting[channel_mapped_index].value[0][5]);

				p_rf_calibrate_info->modify_tx_agc_flag_path_a = true;


				/*odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_B, p_hal_data->current_channel, OFDM);
				odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_B, p_hal_data->current_channel, HT_MCS0_MCS7);*/

				ODM_RT_TRACE(p_dm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("******Path_B Lower then BBSwing lower bound  0, Remnant tx_agc value = %d\n", p_rf_calibrate_info->remnant_ofdm_swing_idx[rf_path]));
			} else {
				set_iqk_matrix_8710b(p_dm, final_ofdm_swing_index, RF_PATH_B,
					p_rf_calibrate_info->iqk_matrix_reg_setting[channel_mapped_index].value[0][4],
					p_rf_calibrate_info->iqk_matrix_reg_setting[channel_mapped_index].value[0][5]);

				ODM_RT_TRACE(p_dm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("******Path_B Compensate with BBSwing, final_ofdm_swing_index = %d\n", final_ofdm_swing_index));

				if (p_rf_calibrate_info->modify_tx_agc_flag_path_b) {
					p_rf_calibrate_info->remnant_ofdm_swing_idx[rf_path] = 0;

					/*odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_B, p_hal_data->current_channel, OFDM);
					odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_B, p_hal_data->current_channel, HT_MCS0_MCS7);*/

					p_rf_calibrate_info->modify_tx_agc_flag_path_a = false;

					ODM_RT_TRACE(p_dm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("******Path_B p_dm->Modify_TxAGC_Flag = false\n"));
				}
			}
#if (MP_DRIVER == 1)
			if ((*(p_dm->p_mp_mode)) == 1) {
				pwr = odm_get_bb_reg(p_dm, REG_TX_AGC_A_RATE18_06, 0xFF);
				pwr += (p_rf_calibrate_info->remnant_ofdm_swing_idx[RF_PATH_B] - p_rf_calibrate_info->modify_tx_agc_value_ofdm);

				if (pwr > 0x3F)
					pwr = 0x3F;
				else if (pwr < 0)
					pwr = 0;

				tx_agc |= ((pwr << 24) | (pwr << 16) | (pwr << 8) | pwr);
				odm_set_bb_reg(p_dm, REG_TX_AGC_A_RATE18_06, MASKDWORD, tx_agc);
				odm_set_bb_reg(p_dm, REG_TX_AGC_A_RATE54_24, MASKDWORD, tx_agc);
				odm_set_bb_reg(p_dm, REG_TX_AGC_A_MCS03_MCS00, MASKDWORD, tx_agc);
				odm_set_bb_reg(p_dm, REG_TX_AGC_A_MCS07_MCS04, MASKDWORD, tx_agc);

				ODM_RT_TRACE(p_dm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("ODM_TxPwrTrackSetPwr8710B: OFDM Tx-rf(A) Power = 0x%x\n", tx_agc));

			} else
#endif
			{

				odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_B, *p_dm->p_channel, OFDM);
				odm_set_tx_power_index_by_rate_section(p_dm, RF_PATH_B, *p_dm->p_channel, HT_MCS0_MCS7);
			}
			p_rf_calibrate_info->modify_tx_agc_value_ofdm = p_rf_calibrate_info->remnant_ofdm_swing_idx[RF_PATH_B] ;
		}
#endif
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("~~~~~~~~~~~~~~~0xc80 = 0x%x 0xe14 = 0x%x 0xab4 = 0x%x 0x86c = 0x%x\n", odm_get_bb_reg(p_dm, 0xc80, MASKDWORD), odm_get_bb_reg(p_dm, 0xe14, MASKDWORD), odm_get_bb_reg(p_dm, 0xab4, MASKDWORD), odm_get_bb_reg(p_dm, 0x86c, MASKDWORD)));

	} else
		return;
}

void
get_delta_swing_table_8710b(
	void		*p_dm_void,
	u8 **temperature_up_a,
	u8 **temperature_down_a,
	u8 **temperature_up_b,
	u8 **temperature_down_b
)
{
	struct PHY_DM_STRUCT	*p_dm = (struct PHY_DM_STRUCT *)p_dm_void;
	struct _ADAPTER		*adapter		 = p_dm->adapter;
	struct odm_rf_calibration_structure	*p_rf_calibrate_info = &(p_dm->rf_calibrate_info);
	struct _hal_rf_ *p_rf = &(p_dm->rf_table);
	u8			tx_rate			= 0xFF;
	u8			channel		 = *p_dm->p_channel;

	if (*(p_dm->p_mp_mode) == true) {
#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN | ODM_CE))
#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)
#if (MP_DRIVER == 1)
		PMPT_CONTEXT p_mpt_ctx = &(adapter->MptCtx);

		tx_rate = MptToMgntRate(p_mpt_ctx->MptRateIndex);
#endif
#elif (DM_ODM_SUPPORT_TYPE & ODM_CE)
#ifdef CONFIG_MP_INCLUDED
		PMPT_CONTEXT p_mpt_ctx = &(adapter->mppriv.mpt_ctx);

		tx_rate = mpt_to_mgnt_rate(p_mpt_ctx->mpt_rate_index);
#endif
#endif
#endif
	} else {
		u16	rate	 = *(p_dm->p_forced_data_rate);

		if (!rate) { /*auto rate*/
			if (rate != 0xFF && p_dm->tx_rate != 0xFF) {
#if (DM_ODM_SUPPORT_TYPE & ODM_WIN)
				tx_rate = adapter->HalFunc.GetHwRateFromMRateHandler(p_dm->tx_rate);
#elif (DM_ODM_SUPPORT_TYPE & ODM_CE)
				if (p_dm->number_linked_client != 0)
					tx_rate = hw_rate_to_m_rate(p_dm->tx_rate);
				else
					tx_rate = p_rf->p_rate_index;
#endif
			}
		} else   /*force rate*/
			tx_rate = (u8)rate;
	}

	ODM_RT_TRACE(p_dm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD, ("Power Tracking tx_rate=0x%X\n", tx_rate));

	if (1 <= channel && channel <= 14) {
		if (IS_CCK_RATE(tx_rate)) {
			*temperature_up_a   = p_rf_calibrate_info->delta_swing_table_idx_2g_cck_a_p;
			*temperature_down_a = p_rf_calibrate_info->delta_swing_table_idx_2g_cck_a_n;
			*temperature_up_b   = p_rf_calibrate_info->delta_swing_table_idx_2g_cck_b_p;
			*temperature_down_b = p_rf_calibrate_info->delta_swing_table_idx_2g_cck_b_n;
		} else {
			*temperature_up_a   = p_rf_calibrate_info->delta_swing_table_idx_2ga_p;
			*temperature_down_a = p_rf_calibrate_info->delta_swing_table_idx_2ga_n;
			*temperature_up_b   = p_rf_calibrate_info->delta_swing_table_idx_2gb_p;
			*temperature_down_b = p_rf_calibrate_info->delta_swing_table_idx_2gb_n;
		}
	} else {
		*temperature_up_a   = (u8 *)delta_swing_table_idx_2ga_p_8188e;
		*temperature_down_a = (u8 *)delta_swing_table_idx_2ga_n_8188e;
		*temperature_up_b   = (u8 *)delta_swing_table_idx_2ga_p_8188e;
		*temperature_down_b = (u8 *)delta_swing_table_idx_2ga_n_8188e;
	}

	return;
}

void
get_delta_swing_xtal_table_8710b(
	void		*p_dm_void,
	s8 **temperature_up_xtal,
	s8 **temperature_down_xtal
)
{
	struct PHY_DM_STRUCT	*p_dm = (struct PHY_DM_STRUCT *)p_dm_void;
	struct odm_rf_calibration_structure	*p_rf_calibrate_info = &(p_dm->rf_calibrate_info);

	*temperature_up_xtal   = p_rf_calibrate_info->delta_swing_table_xtal_p;
	*temperature_down_xtal = p_rf_calibrate_info->delta_swing_table_xtal_n;
}



void
odm_txxtaltrack_set_xtal_8710b(
	void		*p_dm_void
)
{
	struct PHY_DM_STRUCT		*p_dm		= (struct PHY_DM_STRUCT *)p_dm_void;
	struct odm_rf_calibration_structure	*p_rf_calibrate_info	= &(p_dm->rf_calibrate_info);
	struct _ADAPTER		*adapter			= p_dm->adapter;
	HAL_DATA_TYPE	*p_hal_data		 = GET_HAL_DATA(adapter);

	s8	crystal_cap;


	crystal_cap = p_hal_data->crystal_cap & 0x3F;
	crystal_cap = crystal_cap + p_rf_calibrate_info->xtal_offset;

	if (crystal_cap < 0)
		crystal_cap = 0;
	else if (crystal_cap > 63)
		crystal_cap = 63;


	ODM_RT_TRACE(p_dm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD,
		("crystal_cap(%d)= p_hal_data->crystal_cap(%d) + p_rf_calibrate_info->xtal_offset(%d)\n", crystal_cap, p_hal_data->crystal_cap, p_rf_calibrate_info->xtal_offset));

	odm_set_bb_reg(p_dm, REG_MAC_PHY_CTRL, 0xFFF000, (crystal_cap | (crystal_cap << 6)));

	ODM_RT_TRACE(p_dm, ODM_COMP_TX_PWR_TRACK, ODM_DBG_LOUD,
		("crystal_cap(0x2c)  0x%X\n", odm_get_bb_reg(p_dm, REG_MAC_PHY_CTRL, 0xFFF000)));

}

void configure_txpower_track_8710b(
	struct _TXPWRTRACK_CFG	*p_config
)
{
	p_config->swing_table_size_cck = CCK_TABLE_SIZE_8710B;
	p_config->swing_table_size_ofdm = OFDM_TABLE_SIZE;
	p_config->threshold_iqk = IQK_THRESHOLD;
	p_config->average_thermal_num = AVG_THERMAL_NUM_8710B;
	p_config->rf_path_count = MAX_PATH_NUM_8710B;
	p_config->thermal_reg_addr = RF_T_METER_88E;

	p_config->odm_tx_pwr_track_set_pwr = odm_tx_pwr_track_set_pwr_8710b;
	p_config->do_iqk = do_iqk_8710b;
	p_config->phy_lc_calibrate = halrf_lck_trigger;
	p_config->get_delta_swing_table = get_delta_swing_table_8710b;
	p_config->get_delta_swing_xtal_table = get_delta_swing_xtal_table_8710b;
	p_config->odm_txxtaltrack_set_xtal = odm_txxtaltrack_set_xtal_8710b;


}


#define MAX_TOLERANCE		5
#define IQK_DELAY_TIME		1

u8
phy_path_s1_iqk_8710b(
	struct PHY_DM_STRUCT		*p_dm,
	boolean	config_path_s0
)
{
	u32 reg_eac, reg_e94, reg_e9c, path_sel_bb;
	u8 result = 0x00;//, ktime;
	u32 original_path, original_gnt;

	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]path S1 TXIQK!!\n"));
	/*ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0x67 @S1 TXIQK = 0x%x\n", odm_get_mac_reg(p_dm, 0x64, MASKBYTE3)));*/
	/*save RF path*/
	path_sel_bb = odm_get_bb_reg(p_dm, 0x948, MASKDWORD);
	/*ODM_RT_TRACE(p_dm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0x1e6@S1 TXIQK = 0x%x\n", platform_efio_read_1byte(p_adapter, 0x1e6)));*/
	odm_set_bb_reg(p_dm, 0x948, MASKDWORD, 0x99000000);

	/*IQK setting*/
	/*leave IQK mode*/
	odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x000000);
	/* --- \A7\EF\BCgTXIQK mode table ---//*/
	odm_set_rf_reg(p_dm, RF_PATH_A, 0xef, 0x80000, 0x1);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x30, RFREGOFFSETMASK, 0x20000);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x31, RFREGOFFSETMASK, 0x0000f);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x32, RFREGOFFSETMASK, 0x07ff7);

	/*PA, PAD setting*/
	odm_set_rf_reg(p_dm, RF_PATH_A, 0xdf, BIT11, 0x1);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x56, 0x00fff, 0x1ED);

	/*path-A IQK setting*/
	odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x808000);
	
	odm_set_bb_reg(p_dm, REG_TX_IQK_TONE_A, MASKDWORD, 0x18008c1c);
	odm_set_bb_reg(p_dm, REG_RX_IQK_TONE_A, MASKDWORD, 0x38008c1c);
	odm_set_bb_reg(p_dm, REG_TX_IQK_PI_A, MASKDWORD, 0x821403ff);
	odm_set_bb_reg(p_dm, REG_RX_IQK_PI_A, MASKDWORD, 0x28160c06);

	/*odm_set_bb_reg(p_dm, REG_TX_IQK, MASKDWORD, 0x01007c00);*/
	/*odm_set_bb_reg(p_dm, REG_RX_IQK, MASKDWORD, 0x01004800);*/

	/*LO calibration setting*/
	odm_set_bb_reg(p_dm, REG_IQK_AGC_RSP, MASKDWORD, 0x02002911);

	

	/*LOK setting  added for 8710B*/
#if 0
	odm_set_rf_reg(p_dm, RF_PATH_A, 0xef, 0x10, 0x1);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x54, 0x1, 0x1);

#if 0
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x1, RFREGOFFSETMASK, 0xe0d);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x2, RFREGOFFSETMASK, 0x60d);
#endif

	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("RF0x1 @S1 TXIQK = 0x%x\n", odm_get_rf_reg(p_dm, RF_PATH_A, 0x1, RFREGOFFSETMASK)));
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("RF0x2 @S1 TXIQK = 0x%x\n", odm_get_rf_reg(p_dm, RF_PATH_A, 0x2, RFREGOFFSETMASK)));
#endif

	/*enter IQK mode*/
	/*odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x808000);*/

#if 0
	/*backup path & GNT value */
	original_path = odm_get_mac_reg(p_dm, REG_LTECOEX_PATH_CONTROL, MASKDWORD);  /*save 0x70*/
	odm_set_bb_reg(p_dm, REG_LTECOEX_CTRL, MASKDWORD, 0x800f0038);
	ODM_delay_ms(1);
	original_gnt = odm_get_bb_reg(p_dm, REG_LTECOEX_READ_DATA, MASKDWORD);  /*save 0x38*/
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]OriginalGNT = 0x%x\n", original_gnt));

	/*set GNT_WL=1/GNT_BT=1  and path owner to WiFi for pause BT traffic*/
	odm_set_bb_reg(p_dm, REG_LTECOEX_WRITE_DATA, MASKDWORD, 0x0000ff00);
	odm_set_bb_reg(p_dm, REG_LTECOEX_CTRL, MASKDWORD, 0xc0020038);	/*0x38[15:8] = 0x77*/
	odm_set_mac_reg(p_dm, REG_LTECOEX_PATH_CONTROL, BIT(26), 0x1);
#endif

#if 0
	odm_set_bb_reg(p_dm, REG_LTECOEX_CTRL, MASKDWORD, 0x800f0054);  
	ODM_delay_ms(1);
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]GNT_BT @S1 TXIQK = 0x%x\n", odm_get_bb_reg(p_dm, REG_LTECOEX_READ_DATA, MASKDWORD)));
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0x948 @S1 TXIQK = 0x%x\n", odm_get_bb_reg(p_dm, 0x948, MASKDWORD)));
#endif

	/*One shot, path S1 LOK & IQK*/
	odm_set_bb_reg(p_dm, REG_IQK_AGC_PTS, MASKDWORD, 0xfa000000);
	odm_set_bb_reg(p_dm, REG_IQK_AGC_PTS, MASKDWORD, 0xf8000000);

	/* delay x ms */
	ODM_delay_ms(IQK_DELAY_TIME_8710B);

	/*ktime = 0;
	while ((!odm_get_bb_reg(p_dm, 0xeac, BIT(26))) && ktime < 10) {
		ODM_delay_ms(1);
		ktime++;
	}
	*/
#if 0
	/*Restore GNT_WL/GNT_BT  and path owner*/
	odm_set_bb_reg(p_dm, REG_LTECOEX_WRITE_DATA, MASKDWORD, original_gnt);
	odm_set_bb_reg(p_dm, REG_LTECOEX_CTRL, MASKDWORD, 0xc00f0038);
	odm_set_mac_reg(p_dm, REG_LTECOEX_PATH_CONTROL, 0xffffffff, original_path);
#endif


	/*reload RF path*/
	odm_set_bb_reg(p_dm, 0x948, MASKDWORD, path_sel_bb);

	/*leave IQK mode*/
	odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x000000);
	/*PA/PAD controlled by 0x0*/
	odm_set_rf_reg(p_dm, RF_PATH_A, 0xdf, BIT11, 0x0);
	/*odm_set_rf_reg(p_dm, RF_PATH_A, 0x1, BIT(0), 0x0); //?*/
	/*odm_set_rf_reg(p_dm, RF_PATH_A, 0x2, BIT(0), 0x0); //?*/

	
	/*save LOK result*/
	p_dm->rf_calibrate_info.lok_result = odm_get_rf_reg(p_dm, RF_PATH_A, 0x8, 0xfffff);

	/* Check failed*/
	reg_eac = odm_get_bb_reg(p_dm, REG_RX_POWER_AFTER_IQK_A_2, MASKDWORD);
	reg_e94 = odm_get_bb_reg(p_dm, REG_TX_POWER_BEFORE_IQK_A, MASKDWORD);
	reg_e9c = odm_get_bb_reg(p_dm, REG_TX_POWER_AFTER_IQK_A, MASKDWORD);
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xeac = 0x%x\n", reg_eac));
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xe94 = 0x%x, 0xe9c = 0x%x\n", reg_e94, reg_e9c));
	/*monitor image power before & after IQK*/
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xe90(before IQK)= 0x%x, 0xe98(afer IQK) = 0x%x\n",
		odm_get_bb_reg(p_dm, 0xe90, MASKDWORD), odm_get_bb_reg(p_dm, 0xe98, MASKDWORD)));

	if (!(reg_eac & BIT(28)) &&
	    (((reg_e94 & 0x03FF0000) >> 16) != 0x142) &&
	    (((reg_e9c & 0x03FF0000) >> 16) != 0x42))

		result |= 0x01;
	else
		ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("S1 TXIQK FAIL\n"));


	return result;

}

u8
phy_path_s1_rx_iqk_8710b(
	struct PHY_DM_STRUCT		*p_dm,
	boolean	config_path_s0
)
{
	u32 reg_eac, reg_e94, reg_e9c, reg_ea4, u4tmp, tmp, path_sel_bb;
	u8 result = 0x00, ktime;
	u32 original_path, original_gnt;

	path_sel_bb = odm_get_bb_reg(p_dm, 0x948, MASKDWORD);

	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]path S1 RXIQK Step1!!\n"));
	/*ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0x67 @S1 RXIQK1 = 0x%x\n", odm_get_mac_reg(p_dm, 0x64, MASKBYTE3)));*/
	odm_set_bb_reg(p_dm, 0x948, MASKDWORD, 0x99000000);
	/*ODM_RT_TRACE(p_dm,ODM_COMP_INIT, ODM_DBG_LOUD, ("[IQK]0x1e6@S1 RXIQK1 = 0x%x\n", platform_efio_read_1byte(p_adapter, 0x1e6)));	*/
	odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x000000);
	
	/*modify RXIQK mode table*/
	odm_set_rf_reg(p_dm, RF_PATH_A, 0xef, 0x80000, 0x1);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x30, RFREGOFFSETMASK, 0x30000);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x31, RFREGOFFSETMASK, 0x0000f);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x32, RFREGOFFSETMASK, 0xf1173);

	/*---------PA/PAD=0----------*/
	odm_set_rf_reg(p_dm, RF_PATH_A, 0xdf, BIT11, 0x1);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x56, 0x003E0, 0xf);
	
	/*enter IQK mode*/
	odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x808000);

	/*path-A IQK setting*/
	odm_set_bb_reg(p_dm, REG_TX_IQK_TONE_A, MASKDWORD, 0x18008c1c);
	odm_set_bb_reg(p_dm, REG_RX_IQK_TONE_A, MASKDWORD, 0x38008c1c);
	/*odm_set_bb_reg(p_dm, REG_TX_IQK_TONE_B, MASKDWORD, 0x38008c1c);*/
	/*odm_set_bb_reg(p_dm, REG_RX_IQK_TONE_B, MASKDWORD, 0x38008c1c);*/

	odm_set_bb_reg(p_dm, REG_TX_IQK_PI_A, MASKDWORD, 0x8216129f);
	odm_set_bb_reg(p_dm, REG_RX_IQK_PI_A, MASKDWORD, 0x28160c00);
	/*IQK setting*/
	odm_set_bb_reg(p_dm, REG_TX_IQK, MASKDWORD, 0x01007c00);
	odm_set_bb_reg(p_dm, REG_RX_IQK, MASKDWORD, 0x01004800);


	/*LO calibration setting*/
	odm_set_bb_reg(p_dm, REG_IQK_AGC_RSP, MASKDWORD, 0x0046a911);

#if 0
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x1, RFREGOFFSETMASK, 0xe0d);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x2, RFREGOFFSETMASK, 0x60d);
#endif

	/*ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("RF0x1@ path S1 RXIQK1 = 0x%x\n", odm_get_rf_reg(p_dm, RF_PATH_A, 0x1, RFREGOFFSETMASK)));*/
	/*ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("RF0x2@ path S1 RXIQK1 = 0x%x\n", odm_get_rf_reg(p_dm, RF_PATH_A, 0x2, RFREGOFFSETMASK)));*/



#if 0
	/*backup path & GNT value */
	original_path = odm_get_mac_reg(p_dm, REG_LTECOEX_PATH_CONTROL, MASKDWORD);  /*save 0x70*/
	odm_set_bb_reg(p_dm, REG_LTECOEX_CTRL, MASKDWORD, 0x800f0038);
	ODM_delay_ms(1);
	original_gnt = odm_get_bb_reg(p_dm, REG_LTECOEX_READ_DATA, MASKDWORD);  /*save 0x38*/
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]OriginalGNT = 0x%x\n", original_gnt));

	/*set GNT_WL=1/GNT_BT=1  and path owner to WiFi for pause BT traffic*/
	odm_set_bb_reg(p_dm, REG_LTECOEX_WRITE_DATA, MASKDWORD, 0x0000ff00);
	odm_set_bb_reg(p_dm, REG_LTECOEX_CTRL, MASKDWORD, 0xc0020038);	/*0x38[15:8] = 0x77*/
	odm_set_mac_reg(p_dm, REG_LTECOEX_PATH_CONTROL, BIT(26), 0x1);
#endif

#if 0

	odm_set_bb_reg(p_dm, REG_LTECOEX_CTRL, MASKDWORD, 0x800f0054);
	ODM_delay_ms(1);
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]GNT_BT @S1 RXIQK1 = 0x%x\n", odm_get_bb_reg(p_dm, REG_LTECOEX_READ_DATA, MASKDWORD)));
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0x948 @S1 RXIQK1 = 0x%x\n", odm_get_bb_reg(p_dm, 0x948, MASKDWORD)));
#endif

	/*One shot, path S1 LOK & IQK*/
	odm_set_bb_reg(p_dm, REG_IQK_AGC_PTS, MASKDWORD, 0xf9000000);
	odm_set_bb_reg(p_dm, REG_IQK_AGC_PTS, MASKDWORD, 0xf8000000);

	/*delay x ms*/
	ODM_delay_ms(IQK_DELAY_TIME_8710B);

	/*ktime = 0;
	while ((!odm_get_bb_reg(p_dm, 0xeac, BIT(26))) && ktime < 10) {
		ODM_delay_ms(1);
		ktime++;
	}
	*/


	reg_eac = odm_get_bb_reg(p_dm, REG_RX_POWER_AFTER_IQK_A_2, MASKDWORD);
	reg_e94 = odm_get_bb_reg(p_dm, REG_TX_POWER_BEFORE_IQK_A, MASKDWORD);
	reg_e9c = odm_get_bb_reg(p_dm, REG_TX_POWER_AFTER_IQK_A, MASKDWORD);
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xeac = 0x%x\n", reg_eac));
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xe94 = 0x%x, 0xe9c = 0x%x\n", reg_e94, reg_e9c));
	/*monitor image power before & after IQK*/
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xe90(before IQK)= 0x%x, 0xe98(afer IQK) = 0x%x\n",
		odm_get_bb_reg(p_dm, 0xe90, MASKDWORD), odm_get_bb_reg(p_dm, 0xe98, MASKDWORD)));


	tmp = (reg_e9c & 0x03FF0000) >> 16;
	if ((tmp & 0x200) > 0)
		tmp = 0x400 - tmp;

	if (!(reg_eac & BIT(28)) &&
	    (((reg_e94 & 0x03FF0000) >> 16) != 0x142) &&
	    (((reg_e9c & 0x03FF0000) >> 16) != 0x42))

		result |= 0x01;
	else {
		ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("S1 RXIQK STEP1 FAIL\n"));
#if 0
		/*Restore GNT_WL/GNT_BT  and path owner*/
		odm_set_bb_reg(p_dm, REG_LTECOEX_WRITE_DATA, MASKDWORD, original_gnt);
		odm_set_bb_reg(p_dm, REG_LTECOEX_CTRL, MASKDWORD, 0xc00f0038);
		odm_set_mac_reg(p_dm, REG_LTECOEX_PATH_CONTROL, 0xffffffff, original_path);
#endif
		/*reload RF path*/
		odm_set_bb_reg(p_dm, 0x948, MASKDWORD, path_sel_bb);
		odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x000000);
		odm_set_rf_reg(p_dm, RF_PATH_A, 0xdf, BIT11, 0x0);
		/*odm_set_rf_reg(p_dm, RF_PATH_A, 0x1, BIT(0), 0x0); //?*/
		/*odm_set_rf_reg(p_dm, RF_PATH_A, 0x2, BIT(0), 0x0); //?*/
		return result;
	}

	u4tmp = 0x80007C00 | (reg_e94 & 0x3FF0000)  | ((reg_e9c & 0x3FF0000) >> 16);
	odm_set_bb_reg(p_dm, REG_TX_IQK, MASKDWORD, u4tmp);
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xe40 = 0x%x u4tmp = 0x%x\n", odm_get_bb_reg(p_dm, REG_TX_IQK, MASKDWORD), u4tmp));

	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]path S1 RXIQK STEP2!!\n"));
	/*ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0x67 @S1 RXIQK2 = 0x%x\n", odm_get_mac_reg(p_dm, 0x64, MASKBYTE3)));*/
	/*ODM_RT_TRACE(p_dm,ODM_COMP_INIT, ODM_DBG_LOUD, ("[IQK]0x1e6@S1 RXIQK2 = 0x%x\n", platform_efio_read_1byte(p_adapter, 0x1e6)));	*/

/*enter RX step2*/
	/*modify RXIQK mode table*/
	odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x000000);
	odm_set_rf_reg(p_dm, RF_PATH_A, RF_WE_LUT, 0x80000, 0x1);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x30, RFREGOFFSETMASK, 0x30000);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x31, RFREGOFFSETMASK, 0x0000f);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x32, RFREGOFFSETMASK, 0xf7ff2);
		/*---------PA/PAD=0----------*/
	odm_set_rf_reg(p_dm, RF_PATH_A, 0xdf, BIT11, 0x1);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x56, 0xfff, 0x2A);
		/*enter IQK mode*/
	odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x808000);

		//IQK setting
	odm_set_bb_reg(p_dm, REG_RX_IQK, MASKDWORD, 0x01004800);

	odm_set_bb_reg(p_dm, REG_TX_IQK_TONE_A, MASKDWORD, 0x38008c1c);
	odm_set_bb_reg(p_dm, REG_RX_IQK_TONE_A, MASKDWORD, 0x18008c1c);
	/*odm_set_bb_reg(p_dm, REG_TX_IQK_TONE_B, MASKDWORD, 0x38008c1c);*/
	/*odm_set_bb_reg(p_dm, REG_RX_IQK_TONE_B, MASKDWORD, 0x38008c1c);*/

	/*odm_set_bb_reg(p_dm, REG_TX_IQK_PI_A, MASKDWORD, 0x82170000);*/
	odm_set_bb_reg(p_dm, REG_RX_IQK_PI_A, MASKDWORD, 0x2816169f);

	/*LO calibration setting*/
	odm_set_bb_reg(p_dm, REG_IQK_AGC_RSP, MASKDWORD, 0x0046a911);

	/*ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("RF0x1 @S1 RXIQK2 = 0x%x\n", odm_get_rf_reg(p_dm, RF_PATH_A, 0x1, RFREGOFFSETMASK)));*/
	/*ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("RF0x2 @S1 RXIQK2 = 0x%x\n", odm_get_rf_reg(p_dm, RF_PATH_A, 0x2, RFREGOFFSETMASK)));*/

#if 0
	/*backup path & GNT value */
	original_path = odm_get_mac_reg(p_dm, REG_LTECOEX_PATH_CONTROL, MASKDWORD);  /*save 0x70*/
	odm_set_bb_reg(p_dm, REG_LTECOEX_CTRL, MASKDWORD, 0x800f0038);
	ODM_delay_ms(1);
	original_gnt = odm_get_bb_reg(p_dm, REG_LTECOEX_READ_DATA, MASKDWORD);  /*save 0x38*/
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]OriginalGNT = 0x%x\n", original_gnt));

	/*set GNT_WL=1/GNT_BT=1  and path owner to WiFi for pause BT traffic*/
	odm_set_bb_reg(p_dm, REG_LTECOEX_WRITE_DATA, MASKDWORD, 0x0000ff00);
	odm_set_bb_reg(p_dm, REG_LTECOEX_CTRL, MASKDWORD, 0xc0020038);	/*0x38[15:8] = 0x77*/
	odm_set_mac_reg(p_dm, REG_LTECOEX_PATH_CONTROL, BIT(26), 0x1);
#endif

#if 0
	odm_set_bb_reg(p_dm, REG_LTECOEX_CTRL, MASKDWORD, 0x800f0054);
	ODM_delay_ms(1);
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]GNT_BT @S1 RXIQK2 = 0x%x\n", odm_get_bb_reg(p_dm, REG_LTECOEX_READ_DATA, MASKDWORD)));
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0x948 @S1 RXIQK2 = 0x%x\n", odm_get_bb_reg(p_dm, 0x948, MASKDWORD)));
#endif

	odm_set_bb_reg(p_dm, REG_IQK_AGC_PTS, MASKDWORD, 0xf9000000);
	odm_set_bb_reg(p_dm, REG_IQK_AGC_PTS, MASKDWORD, 0xf8000000);


	ODM_delay_ms(IQK_DELAY_TIME_8710B);

	/*ktime = 0;
	while ((!odm_get_bb_reg(p_dm, 0xeac, BIT(26))) && ktime < 10) {
		ODM_delay_ms(1);
		ktime++;
	}
	*/

#if 0
	/*Restore GNT_WL/GNT_BT  and path owner*/
	odm_set_bb_reg(p_dm, REG_LTECOEX_WRITE_DATA, MASKDWORD, original_gnt);
	odm_set_bb_reg(p_dm, REG_LTECOEX_CTRL, MASKDWORD, 0xc00f0038);
	odm_set_mac_reg(p_dm, REG_LTECOEX_PATH_CONTROL, 0xffffffff, original_path);
#endif


	/*reload RF path*/
	odm_set_bb_reg(p_dm, 0x948, MASKDWORD, path_sel_bb);

	/*leave IQK mode*/
	odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x000000);
	
	/*	PA/PAD controlled by 0x0*/
	odm_set_rf_reg(p_dm, RF_PATH_A, 0xdf, BIT11, 0x0);
	/*odm_set_rf_reg(p_dm, RF_PATH_A, 0x1, BIT(0), 0x0); //?*/
	/*odm_set_rf_reg(p_dm, RF_PATH_A, 0x2, BIT(0), 0x0); //?*/

	/*reload LOK value*/
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x8, RFREGOFFSETMASK, p_dm->rf_calibrate_info.lok_result );
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[LOK] reg rf 0x8 = 0x%x\n", odm_get_rf_reg(p_dm, RF_PATH_A, 0x8, RFREGOFFSETMASK )));

	/*check failed*/
	reg_eac = odm_get_bb_reg(p_dm, REG_RX_POWER_AFTER_IQK_A_2, MASKDWORD);
	reg_ea4 = odm_get_bb_reg(p_dm, REG_RX_POWER_BEFORE_IQK_A_2, MASKDWORD);
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xeac = 0x%x\n", reg_eac));
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xea4 = 0x%x, 0xeac = 0x%x\n", reg_ea4, reg_eac));

	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xea0(before IQK)= 0x%x, 0xea8(afer IQK) = 0x%x\n",
		odm_get_bb_reg(p_dm, 0xea0, MASKDWORD), odm_get_bb_reg(p_dm, 0xea8, MASKDWORD)));


	tmp = (reg_eac & 0x03FF0000) >> 16;
	if ((tmp & 0x200) > 0)
		tmp = 0x400 - tmp;

	if (!(reg_eac & BIT(27)) &&		/*if Tx is OK, check whether Rx is OK*/
	    (((reg_ea4 & 0x03FF0000) >> 16) != 0x132) &&
	    (((reg_eac & 0x03FF0000) >> 16) != 0x36) &&
	    (((reg_ea4 & 0x03FF0000) >> 16) < 0x11a) &&
	    (((reg_ea4 & 0x03FF0000) >> 16) > 0xe6) &&
	    (tmp < 0x1a))
		result |= 0x02;
	else
		ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("S1 RXIQK STEP2 FAIL\n"));


	return result;
}


u8
phy_path_s0_iqk_8710b(
	struct PHY_DM_STRUCT		*p_dm
)
{
	u32 reg_eac, reg_e94, reg_e9c, reg_e94_s0, reg_e9c_s0, reg_ea4_s0, reg_eac_s0, tmp, path_sel_bb;
	u8	result = 0x00, ktime;
	u32 original_path, original_gnt;

	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path S0 TXIQK!\n"));
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0x67 @S0 TXIQK = 0x%x\n", odm_get_mac_reg(p_dm, 0x64, MASKBYTE3)));
	path_sel_bb = odm_get_bb_reg(p_dm, 0x948, MASKDWORD);

	odm_set_bb_reg(p_dm, 0x948, MASKDWORD, 0x99000280); /*10 od 0x948 0x1 [7] ; WL:S1 to S0;BT:S0 to S1;*/
	/*ODM_RT_TRACE(p_dm,ODM_COMP_INIT, ODM_DBG_LOUD, ("[IQK]0x1e6@S0 TXIQK = 0x%x\n", platform_efio_read_1byte(p_adapter, 0x1e6)));*/

	odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x000000);
	/*modify TXIQK mode table*/
	odm_set_rf_reg(p_dm, RF_PATH_A, 0xee, RFREGOFFSETMASK, 0x80000);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x33, RFREGOFFSETMASK, 0x00004);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x3e, RFREGOFFSETMASK, 0x0005d);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x3f, RFREGOFFSETMASK, 0xBFFE0);

	/*path-A IQK setting*/
	odm_set_bb_reg(p_dm, 0xe30, MASKDWORD, 0x08008c0c);
	odm_set_bb_reg(p_dm, 0xe34, MASKDWORD, 0x38008c1c);
	odm_set_bb_reg(p_dm, 0xe38, MASKDWORD, 0x8214018a);
	odm_set_bb_reg(p_dm, 0xe3c, MASKDWORD, 0x28160200);
	odm_set_bb_reg(p_dm, REG_TX_IQK, MASKDWORD, 0x01007c00);
	odm_set_bb_reg(p_dm, REG_RX_IQK, MASKDWORD, 0x01004800);

	/*LO calibration setting*/
	odm_set_bb_reg(p_dm, REG_IQK_AGC_RSP, MASKDWORD, 0x00462911);

	/*PA, PAD setting*/
	odm_set_rf_reg(p_dm, RF_PATH_A, 0xde, 0x800, 0x1);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x66, 0x600, 0x0);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x66, 0x1E0, 0x3);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x8d, 0x1F, 0xf);

	/*LOK setting	added for 8710B*/
	odm_set_rf_reg(p_dm, RF_PATH_A, 0xee, 0x10, 0x1);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x64, 0x1, 0x1);

#if 1
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x1, RFREGOFFSETMASK, 0xe6d);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x2, RFREGOFFSETMASK, 0x66d);
#endif

	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("RF0x1 @S0 TXIQK = 0x%x\n", odm_get_rf_reg(p_dm, RF_PATH_A, 0x1, RFREGOFFSETMASK)));
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("RF0x2 @S0 TXIQK = 0x%x\n", odm_get_rf_reg(p_dm, RF_PATH_A, 0x2, RFREGOFFSETMASK)));


	/*enter IQK mode*/
	odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x808000);

#if 1
	/*backup path & GNT value */
	original_path = odm_get_mac_reg(p_dm, REG_LTECOEX_PATH_CONTROL, MASKDWORD);  /*save 0x70*/
	odm_set_bb_reg(p_dm, REG_LTECOEX_CTRL, MASKDWORD, 0x800f0038);
	ODM_delay_ms(1);
	original_gnt = odm_get_bb_reg(p_dm, REG_LTECOEX_READ_DATA, MASKDWORD);  /*save 0x38*/
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]OriginalGNT = 0x%x\n", original_gnt));

	/*set GNT_WL=1/GNT_BT=1  and path owner to WiFi for pause BT traffic*/
	odm_set_bb_reg(p_dm, REG_LTECOEX_WRITE_DATA, MASKDWORD, 0x0000ff00);
	odm_set_bb_reg(p_dm, REG_LTECOEX_CTRL, MASKDWORD, 0xc0020038);	/*0x38[15:8] = 0x77*/
	odm_set_mac_reg(p_dm, REG_LTECOEX_PATH_CONTROL, BIT(26), 0x1);
#endif

	odm_set_bb_reg(p_dm, REG_LTECOEX_CTRL, MASKDWORD, 0x800f0054);
	ODM_delay_ms(1);
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]GNT_BT @S0 TXIQK = 0x%x\n", odm_get_bb_reg(p_dm, REG_LTECOEX_READ_DATA, MASKDWORD)));
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0x948 @S0 TXIQK = 0x%x\n", odm_get_bb_reg(p_dm, 0x948, MASKDWORD)));

	/*One shot, path S1 LOK & IQK*/
	odm_set_bb_reg(p_dm, REG_IQK_AGC_PTS, MASKDWORD, 0xf9000000);
	odm_set_bb_reg(p_dm, REG_IQK_AGC_PTS, MASKDWORD, 0xf8000000);

	/*delay x ms*/
	/*ODM_delay_ms(IQK_DELAY_TIME_8710B);*/

	ktime = 0;
	while ((!odm_get_bb_reg(p_dm, 0xeac, BIT(26))) && ktime < 10) {
		ODM_delay_ms(1);
		ktime++;
	}

#if 1
	/*Restore GNT_WL/GNT_BT  and path owner*/
	odm_set_bb_reg(p_dm, REG_LTECOEX_WRITE_DATA, MASKDWORD, original_gnt);
	odm_set_bb_reg(p_dm, REG_LTECOEX_CTRL, MASKDWORD, 0xc00f0038);
	odm_set_mac_reg(p_dm, REG_LTECOEX_PATH_CONTROL, 0xffffffff, original_path);
#endif


	/*reload RF path*/
	odm_set_bb_reg(p_dm, 0x948, MASKDWORD, path_sel_bb);

	/*leave IQK mode*/
	odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x000000);
	/*PA/PAD controlled by 0x0*/
	odm_set_rf_reg(p_dm, RF_PATH_A, 0xde, 0x800, 0x0);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x2, BIT(0), 0x0);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x1, BIT(0), 0x0);

	/* Check failed*/
	reg_eac_s0 = odm_get_bb_reg(p_dm, REG_RX_POWER_AFTER_IQK_A_2, MASKDWORD);
	reg_e94_s0 = odm_get_bb_reg(p_dm, REG_TX_POWER_BEFORE_IQK_A, MASKDWORD);
	reg_e9c_s0 = odm_get_bb_reg(p_dm, REG_TX_POWER_AFTER_IQK_A, MASKDWORD);
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xeac_s0 = 0x%x\n", reg_eac_s0));
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xe94_s0 = 0x%x, 0xe9c_s0 = 0x%x\n", reg_e94_s0, reg_e9c_s0));
	/*monitor image power before & after IQK*/
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xe90_s0(before IQK)= 0x%x, 0xe98_s0(afer IQK) = 0x%x\n",
		odm_get_bb_reg(p_dm, 0xe90, MASKDWORD), odm_get_bb_reg(p_dm, 0xe98, MASKDWORD)));

	if (!(reg_eac_s0 & BIT(28)) &&
	    (((reg_e94_s0 & 0x03FF0000) >> 16) != 0x142) &&
	    (((reg_e9c_s0 & 0x03FF0000) >> 16) != 0x42))

		result |= 0x01;
	else
		ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("S0 TXIQK FAIL\n"));

	return result;
}



u8
phy_path_s0_rx_iqk_8710b(
	struct PHY_DM_STRUCT		*p_dm,
	boolean	config_path_s0
)
{
	u32 reg_e94, reg_e9c, reg_ea4, reg_eac, reg_e94_s0, reg_e9c_s0, reg_ea4_s0, reg_eac_s0, tmp, u4tmp, path_sel_bb;
	u8 result = 0x00, ktime;
	u32 original_path, original_gnt;

	path_sel_bb = odm_get_bb_reg(p_dm, 0x948, MASKDWORD);

	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path S0 RxIQK Step1!!\n"));
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0x67 @S0 RXIQK1 = 0x%x\n", odm_get_mac_reg(p_dm, 0x64, MASKBYTE3)));
	odm_set_bb_reg(p_dm, 0x948, MASKDWORD, 0x99000280);
	/*ODM_RT_TRACE(p_dm,ODM_COMP_INIT, ODM_DBG_LOUD, ("[IQK]0x1e6@S0 RXIQK1 = 0x%x\n", platform_efio_read_1byte(p_adapter, 0x1e6)));*/

	odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x000000);

	odm_set_bb_reg(p_dm, REG_TX_IQK, MASKDWORD, 0x01007c00);
	odm_set_bb_reg(p_dm, REG_RX_IQK, MASKDWORD, 0x01004800);

	odm_set_bb_reg(p_dm, REG_TX_IQK_TONE_A, MASKDWORD, 0x18008c1c);
	odm_set_bb_reg(p_dm, REG_RX_IQK_TONE_A, MASKDWORD, 0x38008c1c);
	odm_set_bb_reg(p_dm, REG_TX_IQK_TONE_B, MASKDWORD, 0x38008c1c);
	odm_set_bb_reg(p_dm, REG_RX_IQK_TONE_B, MASKDWORD, 0x38008c1c);

	odm_set_bb_reg(p_dm, REG_TX_IQK_PI_A, MASKDWORD, 0x82160000);
	odm_set_bb_reg(p_dm, REG_RX_IQK_PI_A, MASKDWORD, 0x28160000);


	odm_set_bb_reg(p_dm, REG_IQK_AGC_RSP, MASKDWORD, 0x0046a911);


	odm_set_rf_reg(p_dm, RF_PATH_A, 0xee, RFREGOFFSETMASK, 0x80000);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x33, RFREGOFFSETMASK, 0x00006);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x3e, RFREGOFFSETMASK, 0x0005f);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x3f, RFREGOFFSETMASK, 0xa7ffb);


	odm_set_rf_reg(p_dm, RF_PATH_A, 0xde, 0x800, 0x1);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x66, 0x600, 0x0);

#if 1
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x1, RFREGOFFSETMASK, 0xe6d);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x2, RFREGOFFSETMASK, 0x66d);
#endif

	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("RF0x1 @S0 RXIQK1 = 0x%x\n", odm_get_rf_reg(p_dm, RF_PATH_A, 0x1, RFREGOFFSETMASK)));
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("RF0x2 @S0 RXIQK1 = 0x%x\n", odm_get_rf_reg(p_dm, RF_PATH_A, 0x2, RFREGOFFSETMASK)));

	odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x808000);

#if 1
	/*backup path & GNT value */
	original_path = odm_get_mac_reg(p_dm, REG_LTECOEX_PATH_CONTROL, MASKDWORD);  /*save 0x70*/
	odm_set_bb_reg(p_dm, REG_LTECOEX_CTRL, MASKDWORD, 0x800f0038);
	ODM_delay_ms(1);
	original_gnt = odm_get_bb_reg(p_dm, REG_LTECOEX_READ_DATA, MASKDWORD);  /*save 0x38*/
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]OriginalGNT = 0x%x\n", original_gnt));

	/*set GNT_WL=1/GNT_BT=1  and path owner to WiFi for pause BT traffic*/
	odm_set_bb_reg(p_dm, REG_LTECOEX_WRITE_DATA, MASKDWORD, 0x0000ff00);
	odm_set_bb_reg(p_dm, REG_LTECOEX_CTRL, MASKDWORD, 0xc0020038);	/*0x38[15:8] = 0x77*/
	odm_set_mac_reg(p_dm, REG_LTECOEX_PATH_CONTROL, BIT(26), 0x1);
#endif


	odm_set_bb_reg(p_dm, REG_LTECOEX_CTRL, MASKDWORD, 0x800f0054);
	ODM_delay_ms(1);
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]GNT_BT @S0 RXIQK1 = 0x%x\n", odm_get_bb_reg(p_dm, REG_LTECOEX_READ_DATA, MASKDWORD)));
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0x948 @S0 RXIQK1 = 0x%x\n", odm_get_bb_reg(p_dm, 0x948, MASKDWORD)));

	odm_set_bb_reg(p_dm, REG_IQK_AGC_PTS, MASKDWORD, 0xf9000000);
	odm_set_bb_reg(p_dm, REG_IQK_AGC_PTS, MASKDWORD, 0xf8000000);


	/*ODM_delay_ms(IQK_DELAY_TIME_8710B);*/

	ktime = 0;
	while ((!odm_get_bb_reg(p_dm, 0xeac, BIT(26))) && ktime < 10) {
		ODM_delay_ms(1);
		ktime++;
	}

	reg_eac_s0 = odm_get_bb_reg(p_dm, REG_RX_POWER_AFTER_IQK_A_2, MASKDWORD);
	reg_e94_s0 = odm_get_bb_reg(p_dm, REG_TX_POWER_BEFORE_IQK_A, MASKDWORD);
	reg_e9c_s0 = odm_get_bb_reg(p_dm, REG_TX_POWER_AFTER_IQK_A, MASKDWORD);
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xeac_s0 = 0x%x\n", reg_eac_s0));
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xe94_s0 = 0x%x, 0xe9c_s0 = 0x%x\n", reg_e94_s0, reg_e9c_s0));
	/*monitor image power before & after IQK*/
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xe90_s0(before IQK)= 0x%x, 0xe98_s0(afer IQK) = 0x%x\n",
		odm_get_bb_reg(p_dm, 0xe90, MASKDWORD), odm_get_bb_reg(p_dm, 0xe98, MASKDWORD)));


	tmp = (reg_e9c_s0 & 0x03FF0000) >> 16;
	if ((tmp & 0x200) > 0)
		tmp = 0x400 - tmp;

	if (!(reg_eac_s0 & BIT(28)) &&
	    (((reg_e94_s0 & 0x03FF0000) >> 16) != 0x142) &&
	    (((reg_e9c_s0 & 0x03FF0000) >> 16) != 0x42))

		result |= 0x01;
	else {
		ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("S0 RXIQK STEP1 FAIL\n"));
#if 1
		/*Restore GNT_WL/GNT_BT  and path owner*/
		odm_set_bb_reg(p_dm, REG_LTECOEX_WRITE_DATA, MASKDWORD, original_gnt);
		odm_set_bb_reg(p_dm, REG_LTECOEX_CTRL, MASKDWORD, 0xc00f0038);
		odm_set_mac_reg(p_dm, REG_LTECOEX_PATH_CONTROL, 0xffffffff, original_path);
#endif

		/*reload RF path*/
		odm_set_bb_reg(p_dm, 0x948, MASKDWORD, path_sel_bb);
		odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x000000);
		odm_set_rf_reg(p_dm, RF_PATH_A, 0xde, 0x800, 0x0);
		odm_set_rf_reg(p_dm, RF_PATH_A, 0x2, BIT(0), 0x0);
		odm_set_rf_reg(p_dm, RF_PATH_A, 0x1, BIT(0), 0x0);
		return result;
	}

	u4tmp = 0x80007C00 | (reg_e94_s0 & 0x3FF0000)  | ((reg_e9c_s0 & 0x3FF0000) >> 16);
	odm_set_bb_reg(p_dm, REG_TX_IQK, MASKDWORD, u4tmp);
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xe40_s0 = 0x%x u4tmp = 0x%x\n", odm_get_bb_reg(p_dm, REG_TX_IQK, MASKDWORD), u4tmp));


	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]path S0 RXIQK STEP2!!\n\n"));
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0x67 @S0 RXIQK2 = 0x%x\n", odm_get_mac_reg(p_dm, 0x64, MASKBYTE3)));
	/*ODM_RT_TRACE(p_dm,ODM_COMP_INIT, ODM_DBG_LOUD, ("[IQK]0x1e6@S0 RXIQK2 = 0x%x\n", platform_efio_read_1byte(p_adapter, 0x1e6)));*/
	odm_set_bb_reg(p_dm, REG_RX_IQK, MASKDWORD, 0x01004800);

	odm_set_bb_reg(p_dm, REG_TX_IQK_TONE_A, MASKDWORD, 0x38008c1c);
	odm_set_bb_reg(p_dm, REG_RX_IQK_TONE_A, MASKDWORD, 0x18008c1c);
	odm_set_bb_reg(p_dm, REG_TX_IQK_TONE_B, MASKDWORD, 0x38008c1c);
	odm_set_bb_reg(p_dm, REG_RX_IQK_TONE_B, MASKDWORD, 0x38008c1c);

	odm_set_bb_reg(p_dm, REG_TX_IQK_PI_A, MASKDWORD, 0x82170000);
	odm_set_bb_reg(p_dm, REG_RX_IQK_PI_A, MASKDWORD, 0x28171400);

	odm_set_bb_reg(p_dm, REG_IQK_AGC_RSP, MASKDWORD, 0x0046a8d1);

	odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x000000);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0xee, 0x80000, 0x1);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x33, RFREGOFFSETMASK, 0x00007);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x3e, RFREGOFFSETMASK, 0x0005f);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x3f, RFREGOFFSETMASK, 0xb3fdb);

	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("RF0x1 @S0 RXIQK2 = 0x%x\n", odm_get_rf_reg(p_dm, RF_PATH_A, 0x1, RFREGOFFSETMASK)));
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("RF0x2 @S0 RXIQK2 = 0x%x\n", odm_get_rf_reg(p_dm, RF_PATH_A, 0x2, RFREGOFFSETMASK)));
	/*enter IQK mode*/
	odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x808000);

#if 1
	/*backup path & GNT value */
	original_path = odm_get_mac_reg(p_dm, REG_LTECOEX_PATH_CONTROL, MASKDWORD);  /*save 0x70*/
	odm_set_bb_reg(p_dm, REG_LTECOEX_CTRL, MASKDWORD, 0x800f0038);
	ODM_delay_ms(1);
	original_gnt = odm_get_bb_reg(p_dm, REG_LTECOEX_READ_DATA, MASKDWORD);  /*save 0x38*/
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]OriginalGNT = 0x%x\n", original_gnt));

	/*set GNT_WL=1/GNT_BT=1  and path owner to WiFi for pause BT traffic*/
	odm_set_bb_reg(p_dm, REG_LTECOEX_WRITE_DATA, MASKDWORD, 0x0000ff00);
	odm_set_bb_reg(p_dm, REG_LTECOEX_CTRL, MASKDWORD, 0xc0020038);	/*0x38[15:8] = 0x77*/
	odm_set_mac_reg(p_dm, REG_LTECOEX_PATH_CONTROL, BIT(26), 0x1);
#endif


	odm_set_bb_reg(p_dm, REG_LTECOEX_CTRL, MASKDWORD, 0x800f0054);
	ODM_delay_ms(1);
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]GNT_BT @S0 RXIQK2 = 0x%x\n", odm_get_bb_reg(p_dm, REG_LTECOEX_READ_DATA, MASKDWORD)));
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0x948 @S0 RXIQK2 = 0x%x\n", odm_get_bb_reg(p_dm, 0x948, MASKDWORD)));

	odm_set_bb_reg(p_dm, REG_IQK_AGC_PTS, MASKDWORD, 0xf9000000);
	odm_set_bb_reg(p_dm, REG_IQK_AGC_PTS, MASKDWORD, 0xf8000000);


	/*ODM_delay_ms(IQK_DELAY_TIME_8710B);*/
	ktime = 0;
	while ((!odm_get_bb_reg(p_dm, 0xeac, BIT(26))) && ktime < 10) {
		ODM_delay_ms(1);
		ktime++;
	}

#if 1
	/*Restore GNT_WL/GNT_BT  and path owner*/
	odm_set_bb_reg(p_dm, REG_LTECOEX_WRITE_DATA, MASKDWORD, original_gnt);
	odm_set_bb_reg(p_dm, REG_LTECOEX_CTRL, MASKDWORD, 0xc00f0038);
	odm_set_mac_reg(p_dm, REG_LTECOEX_PATH_CONTROL, 0xffffffff, original_path);
#endif

	/*reload RF path*/
	odm_set_bb_reg(p_dm, 0x948, MASKDWORD, path_sel_bb);

	/*leave IQK mode*/
	odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x000000);

	odm_set_rf_reg(p_dm, RF_PATH_A, 0xde, 0x800, 0x0);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x2, BIT(0), 0x0);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x1, BIT(0), 0x0);

	reg_eac_s0 = odm_get_bb_reg(p_dm, REG_RX_POWER_AFTER_IQK_A_2, MASKDWORD);
	reg_ea4_s0 = odm_get_bb_reg(p_dm, REG_RX_POWER_BEFORE_IQK_A_2, MASKDWORD);
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xeac_s0 = 0x%x\n", reg_eac_s0));
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xea4_s0 = 0x%x, 0xeac_s0 = 0x%x\n", reg_ea4_s0, reg_eac_s0));

	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xea0_s0(before IQK)= 0x%x, 0xea8_s0(afer IQK) = 0x%x\n",
		odm_get_bb_reg(p_dm, 0xea0, MASKDWORD), odm_get_bb_reg(p_dm, 0xea8, MASKDWORD)));


	tmp = (reg_eac_s0 & 0x03FF0000) >> 16;
	if ((tmp & 0x200) > 0)
		tmp = 0x400 - tmp;

	if (!(reg_eac_s0 & BIT(27)) &&		/*if Tx is OK, check whether Rx is OK*/
	    (((reg_ea4_s0 & 0x03FF0000) >> 16) != 0x132) &&
	    (((reg_eac_s0 & 0x03FF0000) >> 16) != 0x36) &&
	    (((reg_ea4_s0 & 0x03FF0000) >> 16) < 0x11a) &&
	    (((reg_ea4_s0 & 0x03FF0000) >> 16) > 0xe6) &&
	    (tmp < 0x1a))
		result |= 0x02;
	else
		ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("S0 RXIQK STEP2 FAIL\n"));


	return result;
}


void
_phy_path_s1_fill_iqk_matrix_8710b(
	struct PHY_DM_STRUCT		*p_dm,
	boolean	is_iqk_ok,
	s32		result[][8],
	u8		final_candidate,
	boolean	is_tx_only
)
{
	u32	oldval_1, X, TX1_A, reg;
	s32	Y, TX1_C;
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path S1 IQ Calibration %s !\n", (is_iqk_ok) ? "Success" : "Failed"));

	if (final_candidate == 0xFF)
		return;

	else if (is_iqk_ok) {
		oldval_1 = (odm_get_bb_reg(p_dm, REG_OFDM_0_XA_TX_IQ_IMBALANCE, MASKDWORD) >> 22) & 0x3FF;

		X = result[final_candidate][0];

		if ((X & 0x00000200) != 0)
			X = X | 0xFFFFFC00;
		TX1_A = (X * oldval_1) >> 8;
		ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("X = 0x%x, TX1_A = 0x%x, oldval_1 0x%x\n", X, TX1_A, oldval_1));
		odm_set_bb_reg(p_dm, REG_OFDM_0_XA_TX_IQ_IMBALANCE, 0x3FF, TX1_A);

		odm_set_bb_reg(p_dm, REG_OFDM_0_ECCA_THRESHOLD, BIT(31), ((X * oldval_1 >> 7) & 0x1));

		Y = result[final_candidate][1];
		if ((Y & 0x00000200) != 0)
			Y = Y | 0xFFFFFC00;

		TX1_C = (Y * oldval_1) >> 8;
		ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Y = 0x%x, TX1_C = 0x%x\n", Y, TX1_C));
		odm_set_bb_reg(p_dm, REG_OFDM_0_XC_TX_AFE, 0xF0000000, ((TX1_C & 0x3C0) >> 6));
		odm_set_bb_reg(p_dm, REG_OFDM_0_XA_TX_IQ_IMBALANCE, 0x003F0000, (TX1_C & 0x3F));

		odm_set_bb_reg(p_dm, REG_OFDM_0_ECCA_THRESHOLD, BIT(29), ((Y * oldval_1 >> 7) & 0x1));

		if (is_tx_only) {
			ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("_phy_path_s1_fill_iqk_matrix_8710b only Tx OK\n"));
			return;
		}
		reg = result[final_candidate][2];
#if (DM_ODM_SUPPORT_TYPE == ODM_AP)
		if (RTL_ABS(reg, 0x100) >= 16)
			reg = 0x100;
#endif
		odm_set_bb_reg(p_dm, REG_OFDM_0_XA_RX_IQ_IMBALANCE, 0x3FF, reg);

		reg = result[final_candidate][3] & 0x3F;
		odm_set_bb_reg(p_dm, REG_OFDM_0_XA_RX_IQ_IMBALANCE, 0xFC00, reg);

		reg = (result[final_candidate][3] >> 6) & 0xF;
		odm_set_bb_reg(p_dm, REG_OFDM_0_RX_IQ_EXT_ANTA, 0xF0000000, reg);
		/*
		10 os 7201 10
		10 id ea4 [25:16] p
		10 os 7202 10
		10 od c14 VarFromTmp [9:0] p

		10 os 7201 11
		10 id eac [25:22] p
		10 os 7202 11
		10 od ca0 VarFromTmp [31:28] p

		10 os 7201 12
		10 id eac [21:16] p
		10 os 7202 12
		10 od c14 VarFromTmp [15:10] p
		*/
	}
}

void
_phy_path_s0_fill_iqk_matrix_8710b(
	struct PHY_DM_STRUCT		*p_dm,
	boolean	is_iqk_ok,
	s32		result[][8],
	u8		final_candidate,
	boolean		is_tx_only
)
{
	u32	oldval_0, X, TX0_A, reg;
	s32	Y, TX0_C;
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path S0 IQ Calibration %s !\n", (is_iqk_ok) ? "Success" : "Failed"));

	if (final_candidate == 0xFF)
		return;

	else if (is_iqk_ok) {
		oldval_0 = (odm_get_bb_reg(p_dm, 0xcd4, MASKDWORD) >> 13) & 0x3FF;

		X = result[final_candidate][4];
		if ((X & 0x00000200) != 0)
			X = X | 0xFFFFFC00;
		TX0_A = (X * oldval_0) >> 8;
		ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("X = 0x%x, TX0_A = 0x%x, oldval_0 0x%x\n", X, TX0_A, oldval_0));
		odm_set_bb_reg(p_dm, 0xcd0, 0x7FE, TX0_A);

		odm_set_bb_reg(p_dm, 0xcd0, BIT(0), ((X * oldval_0 >> 7) & 0x1));

		Y = result[final_candidate][5];
		if ((Y & 0x00000200) != 0)
			Y = Y | 0xFFFFFC00;

		TX0_C = (Y * oldval_0) >> 8;
		ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Y = 0x%x, TX0_C = 0x%x\n", Y, TX0_C));
		odm_set_bb_reg(p_dm, 0xcd4, 0x7FE, (TX0_C & 0x3FF));

		odm_set_bb_reg(p_dm, 0xcd4, BIT(0), ((Y * oldval_0 >> 7) & 0x1));

		if (is_tx_only)
			return;

		reg = result[final_candidate][6];
		odm_set_bb_reg(p_dm, 0xcd8, 0x3FF, reg);

		reg = result[final_candidate][7];
		odm_set_bb_reg(p_dm, 0xcd8, 0x003FF000, reg);
		/*
		10 os 7201 10
		10 id ea4 [25:16] p
		10 os 7202 10
		10 od cd8 VarFromTmp [9:0] p

		10 os 7201 11
		10 id eac [25:16] p
		10 os 7202 11
		10 od cd8 VarFromTmp [21:12] p
				rege94_s1 = result[i][0];
				rege9c_s1 = result[i][1];
				regea4_s1 = result[i][2];
				regeac_s1 = result[i][3];
				rege94_s0 = result[i][4];
				rege9c_s0 = result[i][5];
				regea4_s0 = result[i][6];
				regeac_s0 = result[i][7];
		*/
	}
}

void
_phy_save_adda_registers_8710b(
	struct PHY_DM_STRUCT		*p_dm,
	u32		*adda_reg,
	u32		*adda_backup,
	u32		register_num
)
{
	u32	i;

#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	if (odm_check_power_status(p_dm) == false)
		return;
#endif
	for (i = 0 ; i < register_num ; i++)
		adda_backup[i] = odm_get_bb_reg(p_dm, adda_reg[i], MASKDWORD);
}


void
_phy_save_mac_registers_8710b(
	struct PHY_DM_STRUCT		*p_dm,
	u32		*mac_reg,
	u32		*mac_backup
)
{
	u32	i;

	for (i = 0 ; i < (IQK_MAC_REG_NUM - 1); i++)
		mac_backup[i] = odm_read_1byte(p_dm, mac_reg[i]);
	mac_backup[i] = odm_read_4byte(p_dm, mac_reg[i]);
}


void
_phy_reload_adda_registers_8710b(
	struct PHY_DM_STRUCT		*p_dm,
	u32		*adda_reg,
	u32		*adda_backup,
	u32		regiester_num
)
{
	u32	i;

	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Reload ADDA power saving parameters !\n"));
	for (i = 0 ; i < regiester_num; i++)
		odm_set_bb_reg(p_dm, adda_reg[i], MASKDWORD, adda_backup[i]);
}

void
_phy_reload_mac_registers_8710b(
	struct PHY_DM_STRUCT		*p_dm,
	u32		*mac_reg,
	u32		*mac_backup
)
{
	u32	i;

	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("Reload MAC parameters !\n"));
	for (i = 0 ; i < (IQK_MAC_REG_NUM - 1); i++)
		odm_write_1byte(p_dm, mac_reg[i], (u8)mac_backup[i]);
	odm_write_4byte(p_dm, mac_reg[i], mac_backup[i]);
}


void
_phy_path_adda_on_8710b(
	struct PHY_DM_STRUCT		*p_dm,
	u32		*adda_reg,
	boolean		is_path_a_on,
	boolean		is2T
)
{
	u32	path_on;
	u32	i;

	path_on = is_path_a_on ? 0x03c00016 : 0x03c00016;

	if (false == is2T) {
		path_on = 0x03c00016;
		odm_set_bb_reg(p_dm, adda_reg[0], MASKDWORD, 0x03c00016);
	} else
		odm_set_bb_reg(p_dm, adda_reg[0], MASKDWORD, path_on);

	for (i = 1 ; i < IQK_ADDA_REG_NUM ; i++)
		odm_set_bb_reg(p_dm, adda_reg[i], MASKDWORD, path_on);

}

void
_phy_mac_setting_calibration_8710b(
	struct PHY_DM_STRUCT		*p_dm,
	u32		*mac_reg,
	u32		*mac_backup
)
{
	/*
		odm_write_1byte(p_dm, mac_reg[i], 0x3F);

		for(i = 1 ; i < (IQK_MAC_REG_NUM - 1); i++){
			odm_write_1byte(p_dm, mac_reg[i], (u8)(mac_backup[i]&(~BIT(3))));
		}
		odm_write_1byte(p_dm, mac_reg[i], (u8)(mac_backup[i]&(~BIT(5))));
	*/

	/*odm_set_bb_reg(p_dm, 0x522, MASKBYTE0, 0x7f);*/
	/*odm_set_bb_reg(p_dm, 0x550, MASKBYTE0, 0x15);*/
	/*odm_set_bb_reg(p_dm, 0x551, MASKBYTE0, 0x00);*/

	odm_set_bb_reg(p_dm, 0x520, 0x00ff0000, 0xff);
}

void
_phy_path_a_stand_by_8710b(
	struct PHY_DM_STRUCT		*p_dm
)
{
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path-S1 standby mode!\n"));
	odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x000000);
	/*	odm_set_bb_reg(p_dm, 0x840, MASKDWORD, 0x00010000);*/
	odm_set_rf_reg(p_dm, (enum rf_path)0x0, 0x0, RFREGOFFSETMASK, 0x10000);
	odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x808000);
}

void
_phy_path_b_stand_by_8710b(
	struct PHY_DM_STRUCT		*p_dm
)
{
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path-S0 standby mode!\n"));
	odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x000000);
	odm_set_rf_reg(p_dm, (enum rf_path)0x1, 0x0, RFREGOFFSETMASK, 0x10000);
	odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x808000);
}


void
_phy_pi_mode_switch_8710b(
	struct PHY_DM_STRUCT		*p_dm,
	boolean		pi_mode
)
{
	u32	mode;

	mode = pi_mode ? 0x01000100 : 0x01000000;
	odm_set_bb_reg(p_dm, REG_FPGA0_XA_HSSI_PARAMETER1, MASKDWORD, mode);
	odm_set_bb_reg(p_dm, REG_FPGA0_XB_HSSI_PARAMETER1, MASKDWORD, mode);
}

boolean
phy_simularity_compare_8710b(
	struct PHY_DM_STRUCT		*p_dm,
	s32		result[][8],
	u8		 c1,
	u8		 c2
)
{
	u32		i, j, diff, simularity_bit_map, bound = 0;
	u8		final_candidate[2] = {0xFF, 0xFF};
	boolean		is_result = true;
	/*#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)*/
	/*	bool		is2T = IS_92C_SERIAL( p_hal_data->version_id);*/
	/*#else*/
	boolean		is2T = true;
	/*#endif*/

	s32 tmp1 = 0, tmp2 = 0;

	if (is2T)
		bound = 8;
	else
		bound = 4;

	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("===> IQK:phy_simularity_compare_8710b c1 %d c2 %d!!!\n", c1, c2));


	simularity_bit_map = 0;

	for (i = 0; i < bound; i++) {

		if ((i == 1) || (i == 3) || (i == 5) || (i == 7)) {
			if ((result[c1][i] & 0x00000200) != 0)
				tmp1 = result[c1][i] | 0xFFFFFC00;
			else
				tmp1 = result[c1][i];

			if ((result[c2][i] & 0x00000200) != 0)
				tmp2 = result[c2][i] | 0xFFFFFC00;
			else
				tmp2 = result[c2][i];
		} else {
			tmp1 = result[c1][i];
			tmp2 = result[c2][i];
		}

		diff = (tmp1 > tmp2) ? (tmp1 - tmp2) : (tmp2 - tmp1);

		if (diff > MAX_TOLERANCE) {
			ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("IQK:differnece overflow %d index %d compare1 0x%x compare2 0x%x!!!\n",  diff, i, result[c1][i], result[c2][i]));

			if ((i == 2 || i == 6) && !simularity_bit_map) {
				if (result[c1][i] + result[c1][i + 1] == 0)
					final_candidate[(i / 4)] = c2;
				else if (result[c2][i] + result[c2][i + 1] == 0)
					final_candidate[(i / 4)] = c1;
				else
					simularity_bit_map = simularity_bit_map | (1 << i);
			} else
				simularity_bit_map = simularity_bit_map | (1 << i);
		}
	}

	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("IQK:phy_simularity_compare_8710b simularity_bit_map   %x !!!\n", simularity_bit_map));

	if (simularity_bit_map == 0) {
		for (i = 0; i < (bound / 4); i++) {
			if (final_candidate[i] != 0xFF) {
				for (j = i * 4; j < (i + 1) * 4 - 2; j++)
					result[3][j] = result[final_candidate[i]][j];
				is_result = false;
			}
		}
		return is_result;
	} else {

		if (!(simularity_bit_map & 0x03)) {
			for (i = 0; i < 2; i++)
				result[3][i] = result[c1][i];
		}

		if (!(simularity_bit_map & 0x0c)) {
			for (i = 2; i < 4; i++)
				result[3][i] = result[c1][i];
		}

		if (!(simularity_bit_map & 0x30)) {
			for (i = 4; i < 6; i++)
				result[3][i] = result[c1][i];

		}

		if (!(simularity_bit_map & 0xc0)) {
			for (i = 6; i < 8; i++)
				result[3][i] = result[c1][i];
		}

		return false;
	}



}



void
_phy_iq_calibrate_8710b(
	struct PHY_DM_STRUCT		*p_dm,
	s32		result[][8],
	u8		t,
	boolean		is2T
)
{
	u32			i;
	u8			path_s1_ok;//, path_s0_ok;
	u8			tmp0xc50 = (u8)odm_get_bb_reg(p_dm, 0xC50, MASKBYTE0);
	u8			tmp0xc58 = (u8)odm_get_bb_reg(p_dm, 0xC58, MASKBYTE0);
	u32			ADDA_REG[IQK_ADDA_REG_NUM] = {
		REG_FPGA0_XCD_SWITCH_CONTROL,	REG_BLUE_TOOTH,
		REG_RX_WAIT_CCA,		REG_TX_CCK_RFON,
		REG_TX_CCK_BBON,	REG_TX_OFDM_RFON,
		REG_TX_OFDM_BBON,	REG_TX_TO_RX,
		REG_TX_TO_TX,		REG_RX_CCK,
		REG_RX_OFDM,		REG_RX_WAIT_RIFS,
		REG_RX_TO_RX,		REG_STANDBY,
		REG_SLEEP,			REG_PMPD_ANAEN
	};
	u32			IQK_MAC_REG[IQK_MAC_REG_NUM] = {
		REG_TXPAUSE,		REG_BCN_CTRL,
		REG_BCN_CTRL_1,	REG_GPIO_MUXCFG
	};


	u32	IQK_BB_REG_92C[IQK_BB_REG_NUM] = {
		REG_OFDM_0_TRX_PATH_ENABLE,		REG_OFDM_0_TR_MUX_PAR,
		REG_FPGA0_XCD_RF_INTERFACE_SW,	REG_CONFIG_ANT_A,	REG_CONFIG_ANT_B,
		REG_FPGA0_XAB_RF_INTERFACE_SW,	REG_FPGA0_XA_RF_INTERFACE_OE,
		REG_FPGA0_XB_RF_INTERFACE_OE, REG_CCK_0_AFE_SETTING
	};
	u32	cnt_iqk_fail = 0;
	u32 path_sel_bb, path_sel_rf;

#if (DM_ODM_SUPPORT_TYPE & (ODM_AP))
	u32	retry_count = 2;
#ifdef MP_TEST
	if (*(p_dm->p_mp_mode))
		retry_count = 9;
#endif
#endif

#if (DM_ODM_SUPPORT_TYPE & (ODM_WIN))
#if MP_DRIVER
	const u32	retry_count = 9;
#else
	const u32	retry_count = 2;
#endif
#endif

#if (DM_ODM_SUPPORT_TYPE & (ODM_CE))
	u32	retry_count;
	if (*(p_dm->p_mp_mode))
		retry_count = 9;
	else
		retry_count = 2;
#endif

	if (t == 0) {
		_phy_save_adda_registers_8710b(p_dm, ADDA_REG, p_dm->rf_calibrate_info.ADDA_backup, IQK_ADDA_REG_NUM);
		_phy_save_mac_registers_8710b(p_dm, IQK_MAC_REG, p_dm->rf_calibrate_info.IQK_MAC_backup);
		_phy_save_adda_registers_8710b(p_dm, IQK_BB_REG_92C, p_dm->rf_calibrate_info.IQK_BB_backup, IQK_BB_REG_NUM);
	}
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("IQ Calibration for 1T1R_S0/S1 for %d times\n", t));

	_phy_path_adda_on_8710b(p_dm, ADDA_REG, true, is2T);
#if 1
	if (t == 0)
		p_dm->rf_calibrate_info.is_rf_pi_enable = (u8)odm_get_bb_reg(p_dm, REG_FPGA0_XA_HSSI_PARAMETER1, BIT(8));

	if (!p_dm->rf_calibrate_info.is_rf_pi_enable) {
		/*  Switch BB to PI mode to do IQ Calibration. */
		_phy_pi_mode_switch_8710b(p_dm, true);
	}
#endif

	_phy_mac_setting_calibration_8710b(p_dm, IQK_MAC_REG, p_dm->rf_calibrate_info.IQK_MAC_backup);

	/*?save RF path*/
	path_sel_bb = odm_get_bb_reg(p_dm, 0x948, MASKDWORD); 
	path_sel_rf = odm_get_rf_reg(p_dm, RF_PATH_A, 0xb0, RFREGOFFSETMASK);

	/*BB setting*/
	/*odm_set_bb_reg(p_dm, REG_FPGA0_RFMOD, BIT24, 0x00);*/
	odm_set_bb_reg(p_dm, REG_CCK_0_AFE_SETTING, 0x0f000000, 0xf);  /* ?a04*/
	odm_set_bb_reg(p_dm, REG_RX_WAIT_CCA, MASKDWORD, 0x03c00010);
	odm_set_bb_reg(p_dm, REG_OFDM_0_TRX_PATH_ENABLE, MASKDWORD, 0x03a05601);
	odm_set_bb_reg(p_dm, REG_OFDM_0_TR_MUX_PAR, MASKDWORD, 0x000800e4);
	odm_set_bb_reg(p_dm, REG_FPGA0_XCD_RF_INTERFACE_SW, MASKDWORD, 0x25204000);

	/*IQ calibration setting*/
	ODM_RT_TRACE(p_dm,ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("IQK setting!\n"));
	odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x808000);
	odm_set_bb_reg(p_dm, REG_TX_IQK, MASKDWORD, 0x01007c00);
	odm_set_bb_reg(p_dm, REG_RX_IQK, MASKDWORD, 0x01004800);

	if (is2T) {
		_phy_path_b_stand_by_8710b(p_dm);
		_phy_path_adda_on_8710b(p_dm, ADDA_REG, false, is2T);
	}


#if 1
	for (i = 0 ; i < retry_count ; i++) {
		path_s1_ok = phy_path_s1_iqk_8710b(p_dm, is2T);
		if (path_s1_ok == 0x01) {
			ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path S1 Tx IQK Success!!\n"));
			result[t][0] = (odm_get_bb_reg(p_dm, REG_TX_POWER_BEFORE_IQK_A, MASKDWORD) & 0x3FF0000) >> 16;
			result[t][1] = (odm_get_bb_reg(p_dm, REG_TX_POWER_AFTER_IQK_A, MASKDWORD) & 0x3FF0000) >> 16;
			break;
		} else {
			ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path S1 Tx IQK Fail!!\n"));
			result[t][0] = 0x100;
			result[t][1] = 0x0;
			cnt_iqk_fail++;
		}
#if 0
		else if (i == (retry_count - 1) && path_s1_ok == 0x01) {
			RT_DISP(FINIT, INIT_IQK, ("path S1 IQK Only  Tx Success!!\n"));

			result[t][0] = (odm_get_bb_reg(p_dm, REG_TX_POWER_BEFORE_IQK_A, MASKDWORD) & 0x3FF0000) >> 16;
			result[t][1] = (odm_get_bb_reg(p_dm, REG_TX_POWER_AFTER_IQK_A, MASKDWORD) & 0x3FF0000) >> 16;
		}
#endif
	}
#endif


#if 1
	for (i = 0 ; i < retry_count ; i++) {
		path_s1_ok = phy_path_s1_rx_iqk_8710b(p_dm, is2T);
		if (path_s1_ok == 0x03) {
			ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path S1 Rx IQK Success!!\n"));
			result[t][2] = (odm_get_bb_reg(p_dm, REG_RX_POWER_BEFORE_IQK_A_2, MASKDWORD) & 0x3FF0000) >> 16;
			result[t][3] = (odm_get_bb_reg(p_dm, REG_RX_POWER_AFTER_IQK_A_2, MASKDWORD) & 0x3FF0000) >> 16;
			break;
		} else {
			ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path S1 Rx IQK Fail!!\n"));
			result[t][2] = 0x100;
			result[t][3] = 0x0;
			cnt_iqk_fail++;
		}
	}

	if (0x00 == path_s1_ok)
		ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path S1 IQK failed!!\n"));
#endif

	if (is2T) {
		_phy_path_a_stand_by_8710b(p_dm);
		_phy_path_adda_on_8710b(p_dm, ADDA_REG, false, is2T);

		/*odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x808000);*/
		/*odm_set_bb_reg(p_dm, REG_TX_IQK, MASKDWORD, 0x01007c00);*/
		/*odm_set_bb_reg(p_dm, REG_RX_IQK, MASKDWORD, 0x01004800);*/


#if 0
		for (i = 0 ; i < retry_count ; i++) {
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
			path_s0_ok = phy_path_s0_iqk_8710b(p_adapter);
#else
			path_s0_ok = phy_path_s0_iqk_8710b(p_dm);
#endif

			if (path_s0_ok == 0x01) {
				ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path S0 Tx IQK Success!!\n"));
				result[t][4] = (odm_get_bb_reg(p_dm, 0xe94, MASKDWORD) & 0x3FF0000) >> 16;
				result[t][5] = (odm_get_bb_reg(p_dm, 0xe9c, MASKDWORD) & 0x3FF0000) >> 16;
				break;
			} else {
				ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path S0 Tx IQK Fail!!\n"));
				result[t][4] = 0x100;
				result[t][5] = 0x0;
				cnt_iqk_fail++;
			}
#if 0
			else if (i == (retry_count - 1) && path_s1_ok == 0x01) {
				RT_DISP(FINIT, INIT_IQK, ("path S0 IQK Only  Tx Success!!\n"));

				result[t][0] = (odm_get_bb_reg(p_dm, REG_TX_POWER_BEFORE_IQK_B, MASKDWORD) & 0x3FF0000) >> 16;
				result[t][1] = (odm_get_bb_reg(p_dm, REG_TX_POWER_AFTER_IQK_B, MASKDWORD) & 0x3FF0000) >> 16;
			}
#endif
		}
#endif


#if 0

		for (i = 0 ; i < retry_count ; i++) {
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
			path_s0_ok = phy_path_s0_rx_iqk_8710b(p_adapter, is2T);
#else
			path_s0_ok = phy_path_s0_rx_iqk_8710b(p_dm, is2T);
#endif
			if (path_s0_ok == 0x03) {
				ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path S0 Rx IQK Success!!\n"));
				/*				result[t][0] = (odm_get_bb_reg(p_dm, REG_TX_POWER_BEFORE_IQK_A, MASKDWORD)&0x3FF0000)>>16;*/
				/*				result[t][1] = (odm_get_bb_reg(p_dm, REG_TX_POWER_AFTER_IQK_A, MASKDWORD)&0x3FF0000)>>16;*/
				result[t][6] = (odm_get_bb_reg(p_dm, 0xea4, MASKDWORD) & 0x3FF0000) >> 16;
				result[t][7] = (odm_get_bb_reg(p_dm, 0xeac, MASKDWORD) & 0x3FF0000) >> 16;
				break;
			} else {
				ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path S0 Rx IQK Fail!!\n"));
				result[t][6] = 0x100;
				result[t][7] = 0x0;
				cnt_iqk_fail++;
			}
		}



		if (0x00 == path_s0_ok)
			ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path S0 IQK failed!!\n"));

#endif
	}


	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("IQK:Back to BB mode, load original value!\n"));
	odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x000000);

	if (t != 0) {
		_phy_reload_adda_registers_8710b(p_dm, ADDA_REG, p_dm->rf_calibrate_info.ADDA_backup, IQK_ADDA_REG_NUM);
		/* Reload MAC parameters*/
		_phy_reload_mac_registers_8710b(p_dm, IQK_MAC_REG, p_dm->rf_calibrate_info.IQK_MAC_backup);
		_phy_reload_adda_registers_8710b(p_dm, IQK_BB_REG_92C, p_dm->rf_calibrate_info.IQK_BB_backup, IQK_BB_REG_NUM);

		/*?reload RF path*/
		odm_set_bb_reg(p_dm, 0x948, MASKDWORD, path_sel_bb); 
		odm_set_rf_reg(p_dm, RF_PATH_A, 0xb0, RFREGOFFSETMASK, path_sel_rf);

		odm_set_bb_reg(p_dm, 0xc50, MASKBYTE0, 0x50);
		odm_set_bb_reg(p_dm, 0xc50, MASKBYTE0, tmp0xc50);
		if (is2T) {
			odm_set_bb_reg(p_dm, 0xc58, MASKBYTE0, 0x50);
			odm_set_bb_reg(p_dm, 0xc58, MASKBYTE0, tmp0xc58);
		}
		odm_set_bb_reg(p_dm, REG_TX_IQK_TONE_A, MASKDWORD, 0x01008c00);
		odm_set_bb_reg(p_dm, REG_RX_IQK_TONE_A, MASKDWORD, 0x01008c00);
	}
	p_dm->n_iqk_cnt++;

	if (cnt_iqk_fail == 0)
		p_dm->n_iqk_ok_cnt++;
	else
		p_dm->n_iqk_fail_cnt = p_dm->n_iqk_fail_cnt + cnt_iqk_fail;
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("_phy_iq_calibrate_8710b() <==\n"));
}


void
_phy_lc_calibrate_8710b(
	struct PHY_DM_STRUCT		*p_dm,
	boolean	is2T
)
{
	u8	tmp_reg;
	u32	rf_amode = 0, rf_bmode = 0, lc_cal, cnt;

	tmp_reg = odm_read_1byte(p_dm, 0xd03);

	if ((tmp_reg & 0x70) != 0)
		odm_write_1byte(p_dm, 0xd03, tmp_reg & 0x8F);
	else
		odm_write_1byte(p_dm, REG_TXPAUSE, 0xFF);

	/*backup RF0x18*/

	lc_cal = odm_get_rf_reg(p_dm, RF_PATH_A, RF_CHNLBW, RFREGOFFSETMASK);


	/*Start LCK*/
	odm_set_rf_reg(p_dm, RF_PATH_A, RF_CHNLBW, RFREGOFFSETMASK, lc_cal | 0x08000);

	for (cnt = 0; cnt < 100; cnt++) {
		if (odm_get_rf_reg(p_dm, RF_PATH_A, RF_CHNLBW, 0x8000) != 0x1)
			break;
		ODM_delay_ms(10);
	}

	/* Recover channel number*/
	odm_set_rf_reg(p_dm, RF_PATH_A, RF_CHNLBW, RFREGOFFSETMASK, lc_cal);

	/*Restore original situation*/
	if ((tmp_reg & 0x70) != 0)
		odm_write_1byte(p_dm, 0xd03, tmp_reg);
	else
		odm_write_1byte(p_dm, REG_TXPAUSE, 0x00);
}


#if 0
#define		APK_BB_REG_NUM	8
#define		APK_CURVE_REG_NUM 4
#define		PATH_NUM		2

void
_phy_ap_calibrate_8710b(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	struct PHY_DM_STRUCT		*p_dm,
#else
	struct _ADAPTER	*p_adapter,
#endif
	s8		delta,
	boolean		is2T
)
{
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*p_hal_data = GET_HAL_DATA(p_adapter);
#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	struct PHY_DM_STRUCT		*p_dm = &p_hal_data->odmpriv;
#endif
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	struct PHY_DM_STRUCT		*p_dm = &p_hal_data->DM_OutSrc;
#endif
#endif
	u32			reg_d[PATH_NUM];
	u32			tmp_reg, index, offset,  apkbound;
	u8			path, i, pathbound = PATH_NUM;
	u32			BB_backup[APK_BB_REG_NUM];
	u32			BB_REG[APK_BB_REG_NUM] = {
		REG_FPGA1_TX_BLOCK,	REG_OFDM_0_TRX_PATH_ENABLE,
		REG_FPGA0_RFMOD,	REG_OFDM_0_TR_MUX_PAR,
		REG_FPGA0_XCD_RF_INTERFACE_SW,	REG_FPGA0_XAB_RF_INTERFACE_SW,
		REG_FPGA0_XA_RF_INTERFACE_OE,	REG_FPGA0_XB_RF_INTERFACE_OE
	};
	u32			BB_AP_MODE[APK_BB_REG_NUM] = {
		0x00000020, 0x00a05430, 0x02040000,
		0x000800e4, 0x00204000
	};
	u32			BB_normal_AP_MODE[APK_BB_REG_NUM] = {
		0x00000020, 0x00a05430, 0x02040000,
		0x000800e4, 0x22204000
	};

	u32			AFE_backup[IQK_ADDA_REG_NUM];
	u32			AFE_REG[IQK_ADDA_REG_NUM] = {
		REG_FPGA0_XCD_SWITCH_CONTROL,	REG_BLUE_TOOTH,
		REG_RX_WAIT_CCA,		REG_TX_CCK_RFON,
		REG_TX_CCK_BBON,	REG_TX_OFDM_RFON,
		REG_TX_OFDM_BBON,	REG_TX_TO_RX,
		REG_TX_TO_TX,		REG_RX_CCK,
		REG_RX_OFDM,		REG_RX_WAIT_RIFS,
		REG_RX_TO_RX,		REG_STANDBY,
		REG_SLEEP,			REG_PMPD_ANAEN
	};

	u32			MAC_backup[IQK_MAC_REG_NUM];
	u32			MAC_REG[IQK_MAC_REG_NUM] = {
		REG_TXPAUSE,		REG_BCN_CTRL,
		REG_BCN_CTRL_1,	REG_GPIO_MUXCFG
	};

	u32			APK_RF_init_value[PATH_NUM][APK_BB_REG_NUM] = {
		{0x0852c, 0x1852c, 0x5852c, 0x1852c, 0x5852c},
		{0x2852e, 0x0852e, 0x3852e, 0x0852e, 0x0852e}
	};

	u32			APK_normal_RF_init_value[PATH_NUM][APK_BB_REG_NUM] = {
		{0x0852c, 0x0a52c, 0x3a52c, 0x5a52c, 0x5a52c},
		{0x0852c, 0x0a52c, 0x5a52c, 0x5a52c, 0x5a52c}
	};

	u32			APK_RF_value_0[PATH_NUM][APK_BB_REG_NUM] = {
		{0x52019, 0x52014, 0x52013, 0x5200f, 0x5208d},
		{0x5201a, 0x52019, 0x52016, 0x52033, 0x52050}
	};

	u32			APK_normal_RF_value_0[PATH_NUM][APK_BB_REG_NUM] = {
		{0x52019, 0x52017, 0x52010, 0x5200d, 0x5206a},
		{0x52019, 0x52017, 0x52010, 0x5200d, 0x5206a}
	};

	u32			AFE_on_off[PATH_NUM] = {
		0x04db25a4, 0x0b1b25a4
	};

	u32			APK_offset[PATH_NUM] = {
		REG_CONFIG_ANT_A, REG_CONFIG_ANT_B
	};

	u32			APK_normal_offset[PATH_NUM] = {
		REG_CONFIG_PMPD_ANT_A, REG_CONFIG_PMPD_ANT_B
	};

	u32			APK_value[PATH_NUM] = {
		0x92fc0000, 0x12fc0000
	};

	u32			APK_normal_value[PATH_NUM] = {
		0x92680000, 0x12680000
	};

	s8			APK_delta_mapping[APK_BB_REG_NUM][13] = {
		{-4, -3, -2, -2, -1, -1, 0, 1, 2, 3, 4, 5, 6},
		{-4, -3, -2, -2, -1, -1, 0, 1, 2, 3, 4, 5, 6},
		{-6, -4, -2, -2, -1, -1, 0, 1, 2, 3, 4, 5, 6},
		{-1, -1, -1, -1, -1, -1, 0, 1, 2, 3, 4, 5, 6},
		{-11, -9, -7, -5, -3, -1, 0, 0, 0, 0, 0, 0, 0}
	};

	u32			APK_normal_setting_value_1[13] = {
		0x01017018, 0xf7ed8f84, 0x1b1a1816, 0x2522201e, 0x322e2b28,
		0x433f3a36, 0x5b544e49, 0x7b726a62, 0xa69a8f84, 0xdfcfc0b3,
		0x12680000, 0x00880000, 0x00880000
	};

	u32			APK_normal_setting_value_2[16] = {
		0x01c7021d, 0x01670183, 0x01000123, 0x00bf00e2, 0x008d00a3,
		0x0068007b, 0x004d0059, 0x003a0042, 0x002b0031, 0x001f0025,
		0x0017001b, 0x00110014, 0x000c000f, 0x0009000b, 0x00070008,
		0x00050006
	};

	u32			APK_result[PATH_NUM][APK_BB_REG_NUM];
	/*	u32			AP_curve[PATH_NUM][APK_CURVE_REG_NUM];*/

	s32			BB_offset, delta_V, delta_offset;

#if MP_DRIVER == 1
#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	PMPT_CONTEXT	p_mpt_ctx = &(p_adapter->mppriv.mpt_ctx);
#else
	PMPT_CONTEXT	p_mpt_ctx = &(p_adapter->MptCtx);
#endif
	p_mpt_ctx->APK_bound[0] = 45;
	p_mpt_ctx->APK_bound[1] = 52;

#endif

	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("==>_phy_ap_calibrate_8710b() delta %d\n", delta));
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("AP Calibration for %s\n", (is2T ? "2T2R" : "1T1R")));
	if (!is2T)
		pathbound = 1;



#if 0
	/* Temporarily do not allow normal driver to do the following settings because these offset
	*  and value will cause RF internal PA to be unpredictably disabled by HW, such that RF Tx signal
	*  will disappear after disable/enable card many times on 88CU. RF SD and DD have not find the
	* root cause, so we remove these actions temporarily. Added by tynli and SD3 Allen. 2010.05.31. */
#endif
#if MP_DRIVER != 1
	return;
#endif

	for (index = 0; index < PATH_NUM; index++) {
		APK_offset[index] = APK_normal_offset[index];
		APK_value[index] = APK_normal_value[index];
		AFE_on_off[index] = 0x6fdb25a4;
	}

	for (index = 0; index < APK_BB_REG_NUM; index++) {
		for (path = 0; path < pathbound; path++) {
			APK_RF_init_value[path][index] = APK_normal_RF_init_value[path][index];
			APK_RF_value_0[path][index] = APK_normal_RF_value_0[path][index];
		}
		BB_AP_MODE[index] = BB_normal_AP_MODE[index];
	}

	apkbound = 6;


	for (index = 0; index < APK_BB_REG_NUM ; index++) {
		if (index == 0)
			continue;
		BB_backup[index] = odm_get_bb_reg(p_dm, BB_REG[index], MASKDWORD);
	}


#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	_phy_save_mac_registers_8710b(p_adapter, MAC_REG, MAC_backup);


	_phy_save_adda_registers_8710b(p_adapter, AFE_REG, AFE_backup, IQK_ADDA_REG_NUM);
#else
	_phy_save_mac_registers_8710b(p_dm, MAC_REG, MAC_backup);


	_phy_save_adda_registers_8710b(p_dm, AFE_REG, AFE_backup, IQK_ADDA_REG_NUM);
#endif

	for (path = 0; path < pathbound; path++) {


		if (path == RF_PATH_A) {

			offset = REG_PDP_ANT_A;
			for (index = 0; index < 11; index++) {
				odm_set_bb_reg(p_dm, offset, MASKDWORD, APK_normal_setting_value_1[index]);
				ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("_phy_ap_calibrate_8710b() offset 0x%x value 0x%x\n", offset, odm_get_bb_reg(p_dm, offset, MASKDWORD)));

				offset += 0x04;
			}

			odm_set_bb_reg(p_dm, REG_CONFIG_PMPD_ANT_B, MASKDWORD, 0x12680000);

			offset = REG_CONFIG_ANT_A;
			for (; index < 13; index++) {
				odm_set_bb_reg(p_dm, offset, MASKDWORD, APK_normal_setting_value_1[index]);
				ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("_phy_ap_calibrate_8710b() offset 0x%x value 0x%x\n", offset, odm_get_bb_reg(p_dm, offset, MASKDWORD)));

				offset += 0x04;
			}


			odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x400000);


			offset = REG_PDP_ANT_A;
			for (index = 0; index < 16; index++) {
				odm_set_bb_reg(p_dm, offset, MASKDWORD, APK_normal_setting_value_2[index]);
				ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("_phy_ap_calibrate_8710b() offset 0x%x value 0x%x\n", offset, odm_get_bb_reg(p_dm, offset, MASKDWORD)));

				offset += 0x04;
			}
			odm_set_bb_reg(p_dm,  REG_FPGA0_IQK, 0xffffff00, 0x000000);
		} else if (path == RF_PATH_B) {

			offset = REG_PDP_ANT_B;
			for (index = 0; index < 10; index++) {
				odm_set_bb_reg(p_dm, offset, MASKDWORD, APK_normal_setting_value_1[index]);
				ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("_phy_ap_calibrate_8710b() offset 0x%x value 0x%x\n", offset, odm_get_bb_reg(p_dm, offset, MASKDWORD)));

				offset += 0x04;
			}
			odm_set_bb_reg(p_dm, REG_CONFIG_PMPD_ANT_A, MASKDWORD, 0x12680000);
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
			odm_set_bb_reg(p_dm, REG_CONFIG_PMPD_ANT_B, MASKDWORD, 0x12680000);
#else
			odm_set_bb_reg(p_dm, REG_CONFIG_PMPD_ANT_B, MASKDWORD, 0x12680000);
#endif

			offset = REG_CONFIG_ANT_A;
			index = 11;
			for (; index < 13; index++) {
				odm_set_bb_reg(p_dm, offset, MASKDWORD, APK_normal_setting_value_1[index]);
				ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("_phy_ap_calibrate_8710b() offset 0x%x value 0x%x\n", offset, odm_get_bb_reg(p_dm, offset, MASKDWORD)));

				offset += 0x04;
			}


			odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x400000);


			offset = 0xb60;
			for (index = 0; index < 16; index++) {
				odm_set_bb_reg(p_dm, offset, MASKDWORD, APK_normal_setting_value_2[index]);
				ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("_phy_ap_calibrate_8710b() offset 0x%x value 0x%x\n", offset, odm_get_bb_reg(p_dm, offset, MASKDWORD)));

				offset += 0x04;
			}
			odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x000000);
		}


#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
		reg_d[path] = odm_get_rf_reg(p_dm, path, RF_TXBIAS_A, MASKDWORD);
#else
		reg_d[path] = odm_get_rf_reg(p_dm, path, RF_TXBIAS_A, MASKDWORD);
#endif


		for (index = 0; index < IQK_ADDA_REG_NUM ; index++)
			odm_set_bb_reg(p_dm, AFE_REG[index], MASKDWORD, AFE_on_off[path]);
		ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("_phy_ap_calibrate_8710b() offset 0xe70 %x\n", odm_get_bb_reg(p_dm, REG_RX_WAIT_CCA, MASKDWORD)));


		if (path == 0) {
			for (index = 0; index < APK_BB_REG_NUM ; index++) {

				if (index == 0)
					continue;
				else if (index < 5)
					odm_set_bb_reg(p_dm, BB_REG[index], MASKDWORD, BB_AP_MODE[index]);
				else if (BB_REG[index] == 0x870)
					odm_set_bb_reg(p_dm, BB_REG[index], MASKDWORD, BB_backup[index] | BIT(10) | BIT(26));
				else
					odm_set_bb_reg(p_dm, BB_REG[index], BIT(10), 0x0);
			}

			odm_set_bb_reg(p_dm, REG_TX_IQK_TONE_A, MASKDWORD, 0x01008c00);
			odm_set_bb_reg(p_dm, REG_RX_IQK_TONE_A, MASKDWORD, 0x01008c00);
		} else {
			odm_set_bb_reg(p_dm, REG_TX_IQK_TONE_B, MASKDWORD, 0x01008c00);
			odm_set_bb_reg(p_dm, REG_RX_IQK_TONE_B, MASKDWORD, 0x01008c00);

		}

		ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("_phy_ap_calibrate_8710b() offset 0x800 %x\n", odm_get_bb_reg(p_dm, 0x800, MASKDWORD)));


#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
		_phy_mac_setting_calibration_8710b(p_adapter, MAC_REG, MAC_backup);
#else
		_phy_mac_setting_calibration_8710b(p_dm, MAC_REG, MAC_backup);
#endif

		if (path == RF_PATH_A)
			odm_set_rf_reg(p_dm, RF_PATH_B, RF_AC, MASKDWORD, 0x10000);
		else {
			odm_set_rf_reg(p_dm, RF_PATH_A, RF_AC, MASKDWORD, 0x10000);
			odm_set_rf_reg(p_dm, RF_PATH_A, RF_MODE1, MASKDWORD, 0x1000f);
			odm_set_rf_reg(p_dm, RF_PATH_A, RF_MODE2, MASKDWORD, 0x20103);
		}

		delta_offset = ((delta + 14) / 2);
		if (delta_offset < 0)
			delta_offset = 0;
		else if (delta_offset > 12)
			delta_offset = 12;


		for (index = 0; index < APK_BB_REG_NUM; index++) {
			if (index != 1)
				continue;

			tmp_reg = APK_RF_init_value[path][index];
#if 1
			if (!p_dm->rf_calibrate_info.is_apk_thermal_meter_ignore) {
				BB_offset = (tmp_reg & 0xF0000) >> 16;

				if (!(tmp_reg & BIT(15)))
					BB_offset = -BB_offset;

				delta_V = APK_delta_mapping[index][delta_offset];

				BB_offset += delta_V;

				ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("_phy_ap_calibrate_8710b() APK index %d tmp_reg 0x%x delta_V %d delta_offset %d\n", index, tmp_reg, delta_V, delta_offset));

				if (BB_offset < 0) {
					tmp_reg = tmp_reg & (~BIT(15));
					BB_offset = -BB_offset;
				} else
					tmp_reg = tmp_reg | BIT(15);
				tmp_reg = (tmp_reg & 0xFFF0FFFF) | (BB_offset << 16);
			}
#endif

			odm_set_rf_reg(p_dm, (enum rf_path)path, RF_IPA_A, MASKDWORD, 0x8992e);
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
			ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("_phy_ap_calibrate_8710b() offset 0xc %x\n", odm_get_rf_reg(p_dm, path, RF_IPA_A, MASKDWORD)));
			odm_set_rf_reg(p_dm, (enum rf_path)path, RF_AC, MASKDWORD, APK_RF_value_0[path][index]);
			ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("_phy_ap_calibrate_8710b() offset 0x0 %x\n", odm_get_rf_reg(p_dm, path, RF_AC, MASKDWORD)));
			odm_set_rf_reg(p_dm, (enum rf_path)path, RF_TXBIAS_A, MASKDWORD, tmp_reg);
			ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("_phy_ap_calibrate_8710b() offset 0xd %x\n", odm_get_rf_reg(p_dm, path, RF_TXBIAS_A, MASKDWORD)));
#else
			ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("_phy_ap_calibrate_8710b() offset 0xc %x\n", odm_get_rf_reg(p_dm, path, RF_IPA_A, MASKDWORD)));
			odm_set_rf_reg(p_dm, path, RF_AC, MASKDWORD, APK_RF_value_0[path][index]);
			ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("_phy_ap_calibrate_8710b() offset 0x0 %x\n", odm_get_rf_reg(p_dm, path, RF_AC, MASKDWORD)));
			odm_set_rf_reg(p_dm, path, RF_TXBIAS_A, MASKDWORD, tmp_reg);
			ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("_phy_ap_calibrate_8710b() offset 0xd %x\n", odm_get_rf_reg(p_dm, path, RF_TXBIAS_A, MASKDWORD)));
#endif


			i = 0;
			do {
				odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x800000);
				{
					odm_set_bb_reg(p_dm, APK_offset[path], MASKDWORD, APK_value[0]);
					ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("_phy_ap_calibrate_8710b() offset 0x%x value 0x%x\n", APK_offset[path], odm_get_bb_reg(p_dm, APK_offset[path], MASKDWORD)));
					ODM_delay_ms(3);
					odm_set_bb_reg(p_dm, APK_offset[path], MASKDWORD, APK_value[1]);
					ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("_phy_ap_calibrate_8710b() offset 0x%x value 0x%x\n", APK_offset[path], odm_get_bb_reg(p_dm, APK_offset[path], MASKDWORD)));

					ODM_delay_ms(20);
				}
				odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x000000);

				if (path == RF_PATH_A)
					tmp_reg = odm_get_bb_reg(p_dm, REG_APK, 0x03E00000);
				else
					tmp_reg = odm_get_bb_reg(p_dm, REG_APK, 0xF8000000);
				ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("_phy_ap_calibrate_8710b() offset 0xbd8[25:21] %x\n", tmp_reg));


				i++;
			} while (tmp_reg > apkbound && i < 4);

			APK_result[path][index] = tmp_reg;
		}
	}


#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	_phy_reload_mac_registers_8710b(p_adapter, MAC_REG, MAC_backup);
#else
	_phy_reload_mac_registers_8710b(p_dm, MAC_REG, MAC_backup);
#endif


	for (index = 0; index < APK_BB_REG_NUM ; index++) {

		if (index == 0)
			continue;
		odm_set_bb_reg(p_dm, BB_REG[index], MASKDWORD, BB_backup[index]);
	}


#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	_phy_reload_adda_registers_8710b(p_adapter, AFE_REG, AFE_backup, IQK_ADDA_REG_NUM);
#else
	_phy_reload_adda_registers_8710b(p_dm, AFE_REG, AFE_backup, IQK_ADDA_REG_NUM);
#endif


	for (path = 0; path < pathbound; path++) {
		odm_set_rf_reg(p_dm, (enum rf_path)path, 0xd, MASKDWORD, reg_d[path]);
		if (path == RF_PATH_B) {
			odm_set_rf_reg(p_dm, RF_PATH_A, RF_MODE1, MASKDWORD, 0x1000f);
			odm_set_rf_reg(p_dm, RF_PATH_A, RF_MODE2, MASKDWORD, 0x20101);
		}


		if (APK_result[path][1] > 6)
			APK_result[path][1] = 6;
		ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("apk path %d result %d 0x%x \t", path, 1, APK_result[path][1]));
	}

	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("\n"));


	for (path = 0; path < pathbound; path++) {
		odm_set_rf_reg(p_dm, (enum rf_path)path, 0x3, MASKDWORD,
			((APK_result[path][1] << 15) | (APK_result[path][1] << 10) | (APK_result[path][1] << 5) | APK_result[path][1]));
		if (path == RF_PATH_A)
			odm_set_rf_reg(p_dm, (enum rf_path)path, 0x4, MASKDWORD,
				((APK_result[path][1] << 15) | (APK_result[path][1] << 10) | (0x00 << 5) | 0x05));
		else
			odm_set_rf_reg(p_dm, (enum rf_path)path, 0x4, MASKDWORD,
				((APK_result[path][1] << 15) | (APK_result[path][1] << 10) | (0x02 << 5) | 0x05));
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
		if (!IS_HARDWARE_TYPE_8723A(p_adapter))
			odm_set_rf_reg(p_dm, (enum rf_path)path, RF_BS_PA_APSET_G9_G11, MASKDWORD,
				((0x08 << 15) | (0x08 << 10) | (0x08 << 5) | 0x08));
#endif
	}

	p_dm->rf_calibrate_info.is_ap_kdone = true;

	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("<==_phy_ap_calibrate_8710b()\n"));
}
#endif

void
phy_iq_calibrate_8710b(
	void		*p_dm_void,
	boolean	is_recovery
)
{
	struct PHY_DM_STRUCT	*p_dm = (struct PHY_DM_STRUCT *)p_dm_void;
	struct odm_rf_calibration_structure	*p_rf_calibrate_info = &(p_dm->rf_calibrate_info);
	//u8			u1b_tmp;
	u16			count = 0;
	s32			result[4][8];
	u8			i, final_candidate, indexforchannel;
	boolean			is_path_s1_ok, is_path_s0_ok;
	s32			rege94_s1, rege9c_s1, regea4_s1, regeac_s1, rege94_s0, rege9c_s0, regea4_s0, regeac_s0, reg_tmp = 0;
	s32			regc80, regc94, regc14, regca0, regcd0, regcd4, regcd8;
	boolean			is12simular, is13simular, is23simular;
	u32			IQK_BB_REG_92C[IQK_BB_REG_NUM] = {
		REG_OFDM_0_XA_RX_IQ_IMBALANCE,	REG_OFDM_0_XB_RX_IQ_IMBALANCE,
		REG_OFDM_0_ECCA_THRESHOLD,	REG_OFDM_0_AGC_RSSI_TABLE,
		REG_OFDM_0_XA_TX_IQ_IMBALANCE,	REG_OFDM_0_XB_TX_IQ_IMBALANCE,
		REG_OFDM_0_XC_TX_AFE,			REG_OFDM_0_XD_TX_AFE,
		REG_OFDM_0_RX_IQ_EXT_ANTA
	};
	u32			path_sel_bb_phy_iqk;
	u32			original_path, original_gnt, ori_path_ctrl;
	u32			iqk_fail_b, iqk_fail_a;

	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("================ IQK Start ===================\n"));
	iqk_fail_b = p_dm->n_iqk_fail_cnt; 

	ODM_RT_TRACE(p_dm, ODM_COMP_INIT, ODM_DBG_LOUD, ("=====>phy_iq_calibrate_8710b\n"));


	path_sel_bb_phy_iqk = odm_get_bb_reg(p_dm, 0x948, MASKDWORD);

#if (DM_ODM_SUPPORT_TYPE & (ODM_CE | ODM_AP))
	if (is_recovery)
#else
	if (is_recovery && (!p_dm->is_in_hct_test))
#endif
	{
		ODM_RT_TRACE(p_dm, ODM_COMP_INIT, ODM_DBG_LOUD, ("phy_iq_calibrate_8710b: Return due to is_recovery!\n"));
		_phy_reload_adda_registers_8710b(p_dm, IQK_BB_REG_92C, p_dm->rf_calibrate_info.IQK_BB_backup_recover, 9);
		return;
	}

	/*Check & wait if BT is doing IQK*/
#if 0

	if (*(p_dm->p_mp_mode) == false) {
#if MP_DRIVER != 1
		SetFwWiFiCalibrationCmd(p_adapter, 1);


		count = 0;
		u1b_tmp = odm_read_1byte(p_dm, 0x1e6);
		while (u1b_tmp != 0x1 && count < 1000) {
			ODM_delay_us(10);
			u1b_tmp = odm_read_1byte(p_dm, 0x1e6);
			count++;
		}
		if (count >= 1000)
			ODM_RT_TRACE(p_dm, ODM_COMP_INIT, ODM_DBG_LOUD, ("[IQK]Polling 0x1e6 to 1 for WiFi calibration H2C cmd FAIL! count(%d)", count));


		u1b_tmp = odm_read_1byte(p_dm, 0x1e7);
		while ((!(u1b_tmp & BIT(0))) && count < 6000) {
			ODM_delay_us(50);
			u1b_tmp = odm_read_1byte(p_dm, 0x1e7);
			count++;
		}
#endif
	}

#endif

	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("IQK:Start!!!\n"));
	for (i = 0; i < 8; i++) {
		result[0][i] = 0;
		result[1][i] = 0;
		result[2][i] = 0;
		result[3][i] = 0;
	}

	final_candidate = 0xff;
	is_path_s1_ok = false;
	is_path_s0_ok = false;
	is12simular = false;
	is23simular = false;
	is13simular = false;

	for (i = 0; i < 3; i++) {

#if 0
		/*set path control to WL*/
		ori_path_ctrl = odm_get_mac_reg(p_dm, 0x64, MASKBYTE3);  /*save 0x67*/
		ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]original 0x67 = 0x%x\n", ori_path_ctrl));
		odm_set_mac_reg(p_dm, 0x64, BIT(31), 0x1);
		ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]set 0x67 = 0x%x\n", odm_get_mac_reg(p_dm, 0x64, MASKBYTE3)));

		/*backup path & GNT value */
		original_path = odm_get_mac_reg(p_dm, REG_LTECOEX_PATH_CONTROL, MASKDWORD);  /*save 0x70*/
		odm_set_bb_reg(p_dm, REG_LTECOEX_CTRL, MASKDWORD, 0x800f0038);
		ODM_delay_ms(1);
		original_gnt = odm_get_bb_reg(p_dm, REG_LTECOEX_READ_DATA, MASKDWORD);  /*save 0x38*/
		ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]OriginalGNT = 0x%x\n", original_gnt));

		/*set GNT_WL=1/GNT_BT=1  and path owner to WiFi for pause BT traffic*/
		odm_set_bb_reg(p_dm, REG_LTECOEX_WRITE_DATA, MASKDWORD, 0x0000ff00);
		odm_set_bb_reg(p_dm, REG_LTECOEX_CTRL, MASKDWORD, 0xc0020038);	/*0x38[15:8] = 0x77*/
		odm_set_mac_reg(p_dm, REG_LTECOEX_PATH_CONTROL, BIT(26), 0x1);
#endif
		_phy_iq_calibrate_8710b(p_dm, result, i, false);

#if 0
		/*Restore GNT_WL/GNT_BT  and path owner*/
		odm_set_bb_reg(p_dm, REG_LTECOEX_WRITE_DATA, MASKDWORD, original_gnt);
		odm_set_bb_reg(p_dm, REG_LTECOEX_CTRL, MASKDWORD, 0xc00f0038);
		odm_set_mac_reg(p_dm, REG_LTECOEX_PATH_CONTROL, 0xffffffff, original_path);

		/*Restore path control owner*/
		odm_set_mac_reg(p_dm, 0x64, MASKBYTE3, ori_path_ctrl);
		ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]restore 0x67 = 0x%x\n", odm_get_mac_reg(p_dm, 0x64, MASKBYTE3)));
#endif

		if (i == 1) {
			is12simular = phy_simularity_compare_8710b(p_dm, result, 0, 1);
			if (is12simular) {
				final_candidate = 0;
				ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("IQK: is12simular final_candidate is %x\n", final_candidate));
				break;
			}
		}

		if (i == 2) {
			is13simular = phy_simularity_compare_8710b(p_dm, result, 0, 2);
			if (is13simular) {
				final_candidate = 0;
				ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("IQK: is13simular final_candidate is %x\n", final_candidate));

				break;
			}
			is23simular = phy_simularity_compare_8710b(p_dm, result, 1, 2);
			if (is23simular) {
				final_candidate = 1;
				ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("IQK: is23simular final_candidate is %x\n", final_candidate));
			} else {
				for (i = 0; i < 8; i++)
					reg_tmp += result[3][i];

				if (reg_tmp != 0)
					final_candidate = 3;
				else
					final_candidate = 0xFF;

			}
		}
	}



	for (i = 0; i < 4; i++) {
		rege94_s1 = result[i][0];
		rege9c_s1 = result[i][1];
		regea4_s1 = result[i][2];
		regeac_s1 = result[i][3];
		rege94_s0 = result[i][4];
		rege9c_s0 = result[i][5];
		regea4_s0 = result[i][6];
		regeac_s0 = result[i][7];
		ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK] rege94_s1=%x rege9c_s1=%x regea4_s1=%x regeac_s1=%x rege94_s0=%x rege9c_s0=%x regea4_s0=%x regeac_s0=%x\n ", rege94_s1, rege9c_s1, regea4_s1, regeac_s1, rege94_s0, rege9c_s0, regea4_s0, regeac_s0));
	}

	if (final_candidate != 0xff) {
		p_dm->rf_calibrate_info.rege94 = rege94_s1 = result[final_candidate][0];
		p_dm->rf_calibrate_info.rege9c = rege9c_s1 = result[final_candidate][1];
		regea4_s1 = result[final_candidate][2];
		regeac_s1 = result[final_candidate][3];
		p_dm->rf_calibrate_info.regeb4 = rege94_s0 = result[final_candidate][4];
		p_dm->rf_calibrate_info.regebc = rege9c_s0 = result[final_candidate][5];
		regea4_s0 = result[final_candidate][6];
		regeac_s0 = result[final_candidate][7];
		ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK] final_candidate is %x\n", final_candidate));
		ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK] TX1_X=%x TX1_Y=%x RX1_X=%x RX1_Y=%x TX0_X=%x TX0_Y=%x RX0_X=%x RX0_Y=%x\n ", rege94_s1, rege9c_s1, regea4_s1, regeac_s1, rege94_s0, rege9c_s0, regea4_s0, regeac_s0));
		is_path_s1_ok = is_path_s0_ok = true;
	} else {
		ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK] FAIL use default value\n"));

		p_dm->rf_calibrate_info.rege94 = p_dm->rf_calibrate_info.regeb4 = 0x100;
		p_dm->rf_calibrate_info.rege9c = p_dm->rf_calibrate_info.regebc = 0x0;
	}


	if (rege94_s1 != 0)
		_phy_path_s1_fill_iqk_matrix_8710b(p_dm, is_path_s1_ok, result, final_candidate, (regea4_s1 == 0));
	if (rege94_s0 != 0)
		_phy_path_s0_fill_iqk_matrix_8710b(p_dm, is_path_s0_ok, result, final_candidate, (regea4_s0 == 0));

	iqk_fail_a= p_dm->n_iqk_fail_cnt;
	if( iqk_fail_a - iqk_fail_b > 0 )
		RT_TRACE(COMP_INIT, DBG_LOUD, ("[8710bIQK]n_iqk_fail_cnt+,IQK restore to default value !\n"));



	regc80 = odm_get_bb_reg(p_dm, 0xc80, MASKDWORD);
	regc94 = odm_get_bb_reg(p_dm, 0xc94, MASKDWORD);
	regc14 = odm_get_bb_reg(p_dm, 0xc14, MASKDWORD);
	regca0 = odm_get_bb_reg(p_dm, 0xca0, MASKDWORD);
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xc80 = 0x%x 0xc94 = 0x%x 0xc14 = 0x%x 0xca0 = 0x%x\n", regc80, regc94, regc14, regca0));


	regcd0 = odm_get_bb_reg(p_dm, 0xcd0, MASKDWORD);
	regcd4 = odm_get_bb_reg(p_dm, 0xcd4, MASKDWORD);
	regcd8 = odm_get_bb_reg(p_dm, 0xcd8, MASKDWORD);
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("[IQK]0xcd0 = 0x%x 0xcd4 = 0x%x 0xcd8 = 0x%x\n", regcd0, regcd4, regcd8));

#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	indexforchannel = odm_get_right_chnl_place_for_iqk(*p_dm->p_channel);
#else
	indexforchannel = 0;
#endif


	if (final_candidate < 4) {
		for (i = 0; i < iqk_matrix_reg_num; i++)
			p_dm->rf_calibrate_info.iqk_matrix_reg_setting[indexforchannel].value[0][i] = result[final_candidate][i];
		p_dm->rf_calibrate_info.iqk_matrix_reg_setting[indexforchannel].is_iqk_done = true;
	}

	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("\nIQK OK indexforchannel %d.\n", indexforchannel));
	_phy_save_adda_registers_8710b(p_dm, IQK_BB_REG_92C, p_dm->rf_calibrate_info.IQK_BB_backup_recover, IQK_BB_REG_NUM);

#if 0
	if (*(p_dm->p_mp_mode) == false) {
#if MP_DRIVER != 1
		SetFwWiFiCalibrationCmd(p_adapter, 0);


		count = 0;
		u1b_tmp = odm_read_1byte(p_dm, 0x1e6);
		while (u1b_tmp != 0 && count < 1000) {
			ODM_delay_us(10);
			u1b_tmp = odm_read_1byte(p_dm, 0x1e6);
			count++;
		}

		if (count >= 1000)
			ODM_RT_TRACE(p_dm, ODM_COMP_INIT, ODM_DBG_LOUD, ("[IQK]Polling 0x1e6 to 0 for WiFi calibration H2C cmd FAIL! count(%d)", count));
#endif
	}
#endif
	odm_set_bb_reg(p_dm, 0x948, MASKDWORD, path_sel_bb_phy_iqk);
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("IQK finished\n"));
}


void
phy_lc_calibrate_8710b(
	void		*p_dm_void
)
{
	struct PHY_DM_STRUCT	*p_dm = (struct PHY_DM_STRUCT *)p_dm_void;

	_phy_lc_calibrate_8710b(p_dm, false);
}

#if 0
void
phy_ap_calibrate_8710b(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	struct PHY_DM_STRUCT		*p_dm,
#else
	struct _ADAPTER	*p_adapter,
#endif
	s8		delta
)
{
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*p_hal_data = GET_HAL_DATA(p_adapter);
#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	struct PHY_DM_STRUCT		*p_dm = &p_hal_data->odmpriv;
#endif
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	struct PHY_DM_STRUCT		*p_dm = &p_hal_data->DM_OutSrc;
#endif
#endif
	struct _hal_rf_				*p_rf = &(p_dm->rf_table);
#if DISABLE_BB_RF
	return;
#endif

	return;
#if (DM_ODM_SUPPORT_TYPE == ODM_CE)

	if (!(p_rf->rf_supportability & HAL_RF_IQK))
		return;
#endif

#if FOR_BRAZIL_PRETEST != 1
	if (p_dm->rf_calibrate_info.is_ap_kdone)
#endif
		return;


#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	_phy_ap_calibrate_8710b(p_adapter, delta, false);
#else
	_phy_ap_calibrate_8710b(p_dm, delta, false);
#endif
}
#endif

void _phy_set_rf_path_switch_8710b(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	struct PHY_DM_STRUCT		*p_dm,
#else
	struct _ADAPTER	*p_adapter,
#endif
	boolean		is_main,
	boolean		is2T
)
{
	HAL_DATA_TYPE	*p_hal_data = GET_HAL_DATA(p_adapter);
#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	struct PHY_DM_STRUCT		*p_dm = &p_hal_data->odmpriv;
#endif
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	struct PHY_DM_STRUCT		*p_dm = &p_hal_data->DM_OutSrc;
#endif

	if (is_main)
		odm_set_mac_reg(p_dm, 0x7C4, MASKLWORD, 0x7700);
	else
		odm_set_mac_reg(p_dm, 0x7C4, MASKLWORD, 0xDD00);

	odm_set_mac_reg(p_dm, 0x7C0, MASKDWORD, 0xC00F0038);
	odm_set_mac_reg(p_dm, 0x70, BIT(26), 1);
	odm_set_mac_reg(p_dm, 0x64, BIT(31), 1);
}

void phy_set_rf_path_switch_8710b(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	struct PHY_DM_STRUCT		*p_dm,
#else
	struct _ADAPTER	*p_adapter,
#endif
	boolean		is_main
)
{
	HAL_DATA_TYPE	*p_hal_data = GET_HAL_DATA(p_adapter);
#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	struct PHY_DM_STRUCT		*p_dm = &p_hal_data->odmpriv;
#endif
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	struct PHY_DM_STRUCT		*p_dm = &p_hal_data->DM_OutSrc;
#endif

#if DISABLE_BB_RF
	return;
#endif

#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	_phy_set_rf_path_switch_8710b(p_adapter, is_main, true);
#endif

}

#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)

#if 0
#define		DP_BB_REG_NUM		7
#define		DP_RF_REG_NUM		1
#define		DP_RETRY_LIMIT		10
#define		DP_PATH_NUM		2
#define		DP_DPK_NUM			3
#define		DP_DPK_VALUE_NUM	2

void
_phy_digital_predistortion_8710b(
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	struct _ADAPTER	*p_adapter,
#else
	struct PHY_DM_STRUCT	*p_dm,
#endif
	boolean		is2T
)
{
#if (RT_PLATFORM == PLATFORM_WINDOWS)
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*p_hal_data = GET_HAL_DATA(p_adapter);
#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	struct PHY_DM_STRUCT		*p_dm = &p_hal_data->odmpriv;
#endif
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	struct PHY_DM_STRUCT		*p_dm = &p_hal_data->DM_OutSrc;
#endif
#endif

	u32			tmp_reg, tmp_reg2, index,  i;
	u8			path, pathbound = PATH_NUM;
	u32			AFE_backup[IQK_ADDA_REG_NUM];
	u32			AFE_REG[IQK_ADDA_REG_NUM] = {
		REG_FPGA0_XCD_SWITCH_CONTROL,	REG_BLUE_TOOTH,
		REG_RX_WAIT_CCA,		REG_TX_CCK_RFON,
		REG_TX_CCK_BBON,	REG_TX_OFDM_RFON,
		REG_TX_OFDM_BBON,	REG_TX_TO_RX,
		REG_TX_TO_TX,		REG_RX_CCK,
		REG_RX_OFDM,		REG_RX_WAIT_RIFS,
		REG_RX_TO_RX,		REG_STANDBY,
		REG_SLEEP,			REG_PMPD_ANAEN
	};

	u32			BB_backup[DP_BB_REG_NUM];
	u32			BB_REG[DP_BB_REG_NUM] = {
		REG_OFDM_0_TRX_PATH_ENABLE, REG_FPGA0_RFMOD,
		REG_OFDM_0_TR_MUX_PAR,	REG_FPGA0_XCD_RF_INTERFACE_SW,
		REG_FPGA0_XAB_RF_INTERFACE_SW, REG_FPGA0_XA_RF_INTERFACE_OE,
		REG_FPGA0_XB_RF_INTERFACE_OE
	};
	u32			BB_settings[DP_BB_REG_NUM] = {
		0x00a05430, 0x02040000, 0x000800e4, 0x22208000,
		0x0, 0x0, 0x0
	};

	u32			RF_backup[DP_PATH_NUM][DP_RF_REG_NUM];
	u32			RF_REG[DP_RF_REG_NUM] = {
		RF_TXBIAS_A
	};

	u32			MAC_backup[IQK_MAC_REG_NUM];
	u32			MAC_REG[IQK_MAC_REG_NUM] = {
		REG_TXPAUSE,		REG_BCN_CTRL,
		REG_BCN_CTRL_1,	REG_GPIO_MUXCFG
	};

	u32			tx_agc[DP_DPK_NUM][DP_DPK_VALUE_NUM] = {
		{0x1e1e1e1e, 0x03901e1e},
		{0x18181818, 0x03901818},
		{0x0e0e0e0e, 0x03900e0e}
	};

	u32			AFE_on_off[PATH_NUM] = {
		0x04db25a4, 0x0b1b25a4
	};

	u8			retry_count = 0;


	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("==>_phy_digital_predistortion_8710b()\n"));

	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("_phy_digital_predistortion_8710b for %s\n", (is2T ? "2T2R" : "1T1R")));


	for (index = 0; index < DP_BB_REG_NUM; index++)
		BB_backup[index] = odm_get_bb_reg(p_dm, BB_REG[index], MASKDWORD);


#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	_phy_save_mac_registers_8710b(p_adapter, BB_REG, MAC_backup);
#else
	_phy_save_mac_registers_8710b(p_dm, BB_REG, MAC_backup);
#endif


	for (path = 0; path < DP_PATH_NUM; path++) {
		for (index = 0; index < DP_RF_REG_NUM; index++)
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
			RF_backup[path][index] = odm_get_rf_reg(p_dm, path, RF_REG[index], MASKDWORD);
#else
			RF_backup[path][index] = odm_get_rf_reg(p_dm, path, RF_REG[index], MASKDWORD);
#endif
	}


#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	_phy_save_adda_registers_8710b(p_adapter, AFE_REG, AFE_backup, IQK_ADDA_REG_NUM);
#else
	_phy_save_adda_registers_8710b(p_dm, AFE_REG, AFE_backup, IQK_ADDA_REG_NUM);
#endif


	for (index = 0; index < IQK_ADDA_REG_NUM ; index++)
		odm_set_bb_reg(p_dm, AFE_REG[index], MASKDWORD, 0x6fdb25a4);


	for (index = 0; index < DP_BB_REG_NUM; index++) {
		if (index < 4)
			odm_set_bb_reg(p_dm, BB_REG[index], MASKDWORD, BB_settings[index]);
		else if (index == 4)
			odm_set_bb_reg(p_dm, BB_REG[index], MASKDWORD, BB_backup[index] | BIT(10) | BIT(26));
		else
			odm_set_bb_reg(p_dm, BB_REG[index], BIT(10), 0x00);
	}


#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	_phy_mac_setting_calibration_8710b(p_adapter, MAC_REG, MAC_backup);
#else
	_phy_mac_setting_calibration_8710b(p_dm, MAC_REG, MAC_backup);
#endif


	odm_set_bb_reg(p_dm, REG_TX_IQK_TONE_A, MASKDWORD, 0x01008c00);
	odm_set_bb_reg(p_dm, REG_RX_IQK_TONE_A, MASKDWORD, 0x01008c00);
	odm_set_bb_reg(p_dm, REG_TX_IQK_TONE_B, MASKDWORD, 0x01008c00);
	odm_set_bb_reg(p_dm, REG_RX_IQK_TONE_B, MASKDWORD, 0x01008c00);


	odm_set_rf_reg(p_dm, RF_PATH_B, RF_AC, MASKDWORD, 0x10000);


	odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x400000);
	odm_set_bb_reg(p_dm, 0xbc0, MASKDWORD, 0x0005361f);
	odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x000000);


	for (i = 0; i < 3; i++) {

		for (index = 0; index < 3; index++)
			odm_set_bb_reg(p_dm, 0xe00 + index * 4, MASKDWORD, tx_agc[i][0]);
		odm_set_bb_reg(p_dm, 0xe00 + index * 4, MASKDWORD, tx_agc[i][1]);
		for (index = 0; index < 4; index++)
			odm_set_bb_reg(p_dm, 0xe10 + index * 4, MASKDWORD, tx_agc[i][0]);


		odm_set_bb_reg(p_dm, REG_PDP_ANT_A, MASKDWORD, 0x02097098);
		odm_set_bb_reg(p_dm, REG_PDP_ANT_A_4, MASKDWORD, 0xf76d9f84);
		odm_set_bb_reg(p_dm, REG_CONFIG_PMPD_ANT_A, MASKDWORD, 0x0004ab87);
		odm_set_bb_reg(p_dm, REG_CONFIG_ANT_A, MASKDWORD, 0x00880000);


		odm_set_bb_reg(p_dm, REG_CONFIG_PMPD_ANT_A, MASKDWORD, 0x80047788);
		ODM_delay_ms(1);
		odm_set_bb_reg(p_dm, REG_CONFIG_PMPD_ANT_A, MASKDWORD, 0x00047788);
		ODM_delay_ms(50);
	}


	for (index = 0; index < 3; index++)
		odm_set_bb_reg(p_dm, 0xe00 + index * 4, MASKDWORD, 0x34343434);
	odm_set_bb_reg(p_dm, 0xe08 + index * 4, MASKDWORD, 0x03903434);
	for (index = 0; index < 4; index++)
		odm_set_bb_reg(p_dm, 0xe10 + index * 4, MASKDWORD, 0x34343434);


	odm_set_bb_reg(p_dm, REG_PDP_ANT_A, MASKDWORD, 0x02017098);
	odm_set_bb_reg(p_dm, REG_PDP_ANT_A_4, MASKDWORD, 0xf76d9f84);
	odm_set_bb_reg(p_dm, REG_CONFIG_PMPD_ANT_A, MASKDWORD, 0x0004ab87);
	odm_set_bb_reg(p_dm, REG_CONFIG_ANT_A, MASKDWORD, 0x00880000);


	odm_set_rf_reg(p_dm, RF_PATH_A, 0x0c, MASKDWORD, 0x8992b);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x0d, MASKDWORD, 0x0e52c);
	odm_set_rf_reg(p_dm, RF_PATH_A, 0x00, MASKDWORD, 0x5205a);


	odm_set_bb_reg(p_dm, REG_CONFIG_PMPD_ANT_A, MASKDWORD, 0x800477c0);
	ODM_delay_ms(1);
	odm_set_bb_reg(p_dm, REG_CONFIG_PMPD_ANT_A, MASKDWORD, 0x000477c0);
	ODM_delay_ms(50);

	while (retry_count < DP_RETRY_LIMIT && !p_dm->rf_calibrate_info.is_dp_path_aok) {

		odm_set_bb_reg(p_dm, REG_PDP_ANT_A, MASKDWORD, 0x0c297018);
		tmp_reg = odm_get_bb_reg(p_dm, 0xbe0, MASKDWORD);
		ODM_delay_ms(10);
		odm_set_bb_reg(p_dm, REG_PDP_ANT_A, MASKDWORD, 0x0c29701f);
		tmp_reg2 = odm_get_bb_reg(p_dm, 0xbe8, MASKDWORD);
		ODM_delay_ms(10);

		tmp_reg = (tmp_reg & MASKHWORD) >> 16;
		tmp_reg2 = (tmp_reg2 & MASKHWORD) >> 16;
		if (tmp_reg < 0xf0 || tmp_reg > 0x105 || tmp_reg2 > 0xff) {
			odm_set_bb_reg(p_dm, REG_PDP_ANT_A, MASKDWORD, 0x02017098);

			odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x800000);
			odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x000000);
			ODM_delay_ms(1);
			odm_set_bb_reg(p_dm, REG_CONFIG_PMPD_ANT_A, MASKDWORD, 0x800477c0);
			ODM_delay_ms(1);
			odm_set_bb_reg(p_dm, REG_CONFIG_PMPD_ANT_A, MASKDWORD, 0x000477c0);
			ODM_delay_ms(50);
			retry_count++;
			ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path S1 DPK retry_count %d 0xbe0[31:16] %x 0xbe8[31:16] %x\n", retry_count, tmp_reg, tmp_reg2));
		} else {
			ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path S1 DPK Success\n"));
			p_dm->rf_calibrate_info.is_dp_path_aok = true;
			break;
		}
	}
	retry_count = 0;


	if (p_dm->rf_calibrate_info.is_dp_path_aok) {

		odm_set_bb_reg(p_dm, REG_PDP_ANT_A, MASKDWORD, 0x01017098);
		odm_set_bb_reg(p_dm, REG_PDP_ANT_A_4, MASKDWORD, 0x776d9f84);
		odm_set_bb_reg(p_dm, REG_CONFIG_PMPD_ANT_A, MASKDWORD, 0x0004ab87);
		odm_set_bb_reg(p_dm, REG_CONFIG_ANT_A, MASKDWORD, 0x00880000);
		odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x400000);

		for (i = REG_PDP_ANT_A; i <= 0xb3c; i += 4) {
			odm_set_bb_reg(p_dm, i, MASKDWORD, 0x40004000);
			ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path S1 ofsset = 0x%x\n", i));
		}


		odm_set_bb_reg(p_dm, 0xb40, MASKDWORD, 0x40404040);
		odm_set_bb_reg(p_dm, 0xb44, MASKDWORD, 0x28324040);
		odm_set_bb_reg(p_dm, 0xb48, MASKDWORD, 0x10141920);

		for (i = 0xb4c; i <= 0xb5c; i += 4)
			odm_set_bb_reg(p_dm, i, MASKDWORD, 0x0c0c0c0c);


		odm_set_bb_reg(p_dm, 0xbc0, MASKDWORD, 0x0005361f);
		odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x000000);
	} else {
		odm_set_bb_reg(p_dm, REG_PDP_ANT_A, MASKDWORD, 0x00000000);
		odm_set_bb_reg(p_dm, REG_PDP_ANT_A_4, MASKDWORD, 0x00000000);
	}


	if (is2T) {

		odm_set_rf_reg(p_dm, RF_PATH_A, RF_AC, MASKDWORD, 0x10000);


		odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x400000);
		odm_set_bb_reg(p_dm, 0xbc4, MASKDWORD, 0x0005361f);
		odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x000000);


		for (i = 0; i < 3; i++) {

			for (index = 0; index < 4; index++)
				odm_set_bb_reg(p_dm, 0x830 + index * 4, MASKDWORD, tx_agc[i][0]);
			for (index = 0; index < 2; index++)
				odm_set_bb_reg(p_dm, 0x848 + index * 4, MASKDWORD, tx_agc[i][0]);
			for (index = 0; index < 2; index++)
				odm_set_bb_reg(p_dm, 0x868 + index * 4, MASKDWORD, tx_agc[i][0]);


			odm_set_bb_reg(p_dm, REG_PDP_ANT_B, MASKDWORD, 0x02097098);
			odm_set_bb_reg(p_dm, REG_PDP_ANT_B_4, MASKDWORD, 0xf76d9f84);
			odm_set_bb_reg(p_dm, REG_CONFIG_PMPD_ANT_B, MASKDWORD, 0x0004ab87);
			odm_set_bb_reg(p_dm, REG_CONFIG_ANT_B, MASKDWORD, 0x00880000);


			odm_set_bb_reg(p_dm, REG_CONFIG_PMPD_ANT_B, MASKDWORD, 0x80047788);
			ODM_delay_ms(1);
			odm_set_bb_reg(p_dm, REG_CONFIG_PMPD_ANT_B, MASKDWORD, 0x00047788);
			ODM_delay_ms(50);
		}


		for (index = 0; index < 4; index++)
			odm_set_bb_reg(p_dm, 0x830 + index * 4, MASKDWORD, 0x34343434);
		for (index = 0; index < 2; index++)
			odm_set_bb_reg(p_dm, 0x848 + index * 4, MASKDWORD, 0x34343434);
		for (index = 0; index < 2; index++)
			odm_set_bb_reg(p_dm, 0x868 + index * 4, MASKDWORD, 0x34343434);


		odm_set_bb_reg(p_dm, REG_PDP_ANT_B, MASKDWORD, 0x02017098);
		odm_set_bb_reg(p_dm, REG_PDP_ANT_B_4, MASKDWORD, 0xf76d9f84);
		odm_set_bb_reg(p_dm, REG_CONFIG_PMPD_ANT_B, MASKDWORD, 0x0004ab87);
		odm_set_bb_reg(p_dm, REG_CONFIG_ANT_B, MASKDWORD, 0x00880000);


		odm_set_bb_reg(p_dm, 0x840, MASKDWORD, 0x0101000f);
		odm_set_bb_reg(p_dm, 0x840, MASKDWORD, 0x01120103);


		odm_set_rf_reg(p_dm, RF_PATH_B, 0x0c, MASKDWORD, 0x8992b);
		odm_set_rf_reg(p_dm, RF_PATH_B, 0x0d, MASKDWORD, 0x0e52c);
		odm_set_rf_reg(p_dm, RF_PATH_B, RF_AC, MASKDWORD, 0x5205a);


		odm_set_bb_reg(p_dm, REG_CONFIG_PMPD_ANT_B, MASKDWORD, 0x800477c0);
		ODM_delay_ms(1);
		odm_set_bb_reg(p_dm, REG_CONFIG_PMPD_ANT_B, MASKDWORD, 0x000477c0);
		ODM_delay_ms(50);

		while (retry_count < DP_RETRY_LIMIT && !p_dm->rf_calibrate_info.is_dp_path_bok) {

			odm_set_bb_reg(p_dm, REG_PDP_ANT_B, MASKDWORD, 0x0c297018);
			tmp_reg = odm_get_bb_reg(p_dm, 0xbf0, MASKDWORD);
			odm_set_bb_reg(p_dm, REG_PDP_ANT_B, MASKDWORD, 0x0c29701f);
			tmp_reg2 = odm_get_bb_reg(p_dm, 0xbf8, MASKDWORD);

			tmp_reg = (tmp_reg & MASKHWORD) >> 16;
			tmp_reg2 = (tmp_reg2 & MASKHWORD) >> 16;

			if (tmp_reg < 0xf0 || tmp_reg > 0x105 || tmp_reg2 > 0xff) {
				odm_set_bb_reg(p_dm, REG_PDP_ANT_B, MASKDWORD, 0x02017098);

				odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x800000);
				odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x000000);
				ODM_delay_ms(1);
				odm_set_bb_reg(p_dm, REG_CONFIG_PMPD_ANT_B, MASKDWORD, 0x800477c0);
				ODM_delay_ms(1);
				odm_set_bb_reg(p_dm, REG_CONFIG_PMPD_ANT_B, MASKDWORD, 0x000477c0);
				ODM_delay_ms(50);
				retry_count++;
				ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path B DPK retry_count %d 0xbf0[31:16] %x, 0xbf8[31:16] %x\n", retry_count, tmp_reg, tmp_reg2));
			} else {
				ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path B DPK Success\n"));
				p_dm->rf_calibrate_info.is_dp_path_bok = true;
				break;
			}
		}


		if (p_dm->rf_calibrate_info.is_dp_path_bok) {

			odm_set_bb_reg(p_dm, REG_PDP_ANT_B, MASKDWORD, 0x01017098);
			odm_set_bb_reg(p_dm, REG_PDP_ANT_B_4, MASKDWORD, 0x776d9f84);
			odm_set_bb_reg(p_dm, REG_CONFIG_PMPD_ANT_B, MASKDWORD, 0x0004ab87);
			odm_set_bb_reg(p_dm, REG_CONFIG_ANT_B, MASKDWORD, 0x00880000);

			odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x400000);
			for (i = 0xb60; i <= 0xb9c; i += 4) {
				odm_set_bb_reg(p_dm, i, MASKDWORD, 0x40004000);
				ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("path B ofsset = 0x%x\n", i));
			}


			odm_set_bb_reg(p_dm, 0xba0, MASKDWORD, 0x40404040);
			odm_set_bb_reg(p_dm, 0xba4, MASKDWORD, 0x28324050);
			odm_set_bb_reg(p_dm, 0xba8, MASKDWORD, 0x0c141920);

			for (i = 0xbac; i <= 0xbbc; i += 4)
				odm_set_bb_reg(p_dm, i, MASKDWORD, 0x0c0c0c0c);


			odm_set_bb_reg(p_dm, 0xbc4, MASKDWORD, 0x0005361f);
			odm_set_bb_reg(p_dm, REG_FPGA0_IQK, 0xffffff00, 0x000000);

		} else {
			odm_set_bb_reg(p_dm, REG_PDP_ANT_B, MASKDWORD, 0x00000000);
			odm_set_bb_reg(p_dm, REG_PDP_ANT_B_4, MASKDWORD, 0x00000000);
		}
	}


	for (index = 0; index < DP_BB_REG_NUM; index++)
		odm_set_bb_reg(p_dm, BB_REG[index], MASKDWORD, BB_backup[index]);


	for (path = 0; path < DP_PATH_NUM; path++) {
		for (i = 0 ; i < DP_RF_REG_NUM ; i++)
			odm_set_rf_reg(p_dm, path, RF_REG[i], MASKDWORD, RF_backup[path][i]);
	}
	odm_set_rf_reg(p_dm, RF_PATH_A, RF_MODE1, MASKDWORD, 0x1000f);
	odm_set_rf_reg(p_dm, RF_PATH_A, RF_MODE2, MASKDWORD, 0x20101);


#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	_phy_reload_adda_registers_8710b(p_adapter, AFE_REG, AFE_backup, IQK_ADDA_REG_NUM);


	_phy_reload_mac_registers_8710b(p_adapter, MAC_REG, MAC_backup);
#else
	_phy_reload_adda_registers_8710b(p_dm, AFE_REG, AFE_backup, IQK_ADDA_REG_NUM);


	_phy_reload_mac_registers_8710b(p_dm, MAC_REG, MAC_backup);
#endif

	p_dm->rf_calibrate_info.is_dp_done = true;
	ODM_RT_TRACE(p_dm, ODM_COMP_CALIBRATION, ODM_DBG_LOUD, ("<==_phy_digital_predistortion_8710b()\n"));
#endif
}

void
phy_digital_predistortion_8710b(
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	struct _ADAPTER	*p_adapter
#else
	struct PHY_DM_STRUCT	*p_dm
#endif
)
{
	return;

	
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*p_hal_data = GET_HAL_DATA(p_adapter);
#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	struct PHY_DM_STRUCT		*p_dm = &p_hal_data->odmpriv;
#endif
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	struct PHY_DM_STRUCT		*p_dm = &p_hal_data->DM_OutSrc;
#endif
#endif
#if DISABLE_BB_RF
	return;
#endif

	return;

	if (p_dm->rf_calibrate_info.is_dp_done)
		return;
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)

	if (IS_92C_SERIAL(p_hal_data->version_id))
		_phy_digital_predistortion_8710b(p_adapter, true);
	else
#endif
	{

		_phy_digital_predistortion_8710b(p_adapter, false);
	}

}
#endif


boolean _phy_query_rf_path_switch_8710b(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	struct PHY_DM_STRUCT		*p_dm,
#else
	struct _ADAPTER	*p_adapter,
#endif
	boolean		is2T
)
{
#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	HAL_DATA_TYPE	*p_hal_data = GET_HAL_DATA(p_adapter);
#if (DM_ODM_SUPPORT_TYPE == ODM_CE)
	struct PHY_DM_STRUCT		*p_dm = &p_hal_data->odmpriv;
#endif
#if (DM_ODM_SUPPORT_TYPE == ODM_WIN)
	struct PHY_DM_STRUCT		*p_dm = &p_hal_data->DM_OutSrc;
#endif
#endif


	if (odm_get_bb_reg(p_dm, 0x7C4, MASKLWORD) == 0x7700)
		return true;
	else
		return false;

}




boolean phy_query_rf_path_switch_8710b(
#if (DM_ODM_SUPPORT_TYPE & ODM_AP)
	struct PHY_DM_STRUCT		*p_dm
#else
	struct _ADAPTER	*p_adapter
#endif
)
{

#if DISABLE_BB_RF
	return true;
#endif

#if !(DM_ODM_SUPPORT_TYPE & ODM_AP)
	return _phy_query_rf_path_switch_8710b(p_adapter, false);
#else
	return _phy_query_rf_path_switch_8710b(p_dm, false);
#endif

}
#endif



#else

void
phy_iq_calibrate_8710b(
	void		*p_dm_void,
	boolean	is_recovery
) {}

void
phy_lc_calibrate_8710b(
	void		*p_dm_void
) {}

void
odm_tx_pwr_track_set_pwr_8710b(
	struct PHY_DM_STRUCT			*p_dm,
	enum pwrtrack_method	method,
	u8				rf_path,
	u8				channel_mapped_index
) {}

#endif
