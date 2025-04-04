/**
 ******************************************************************************
 *
 *  @file bl_strs.c
 *
 *  Copyright (C) BouffaloLab 2017-2021
 *
 *  Licensed under the Apache License, Version 2.0 (the License);
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an ASIS BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************
 */


#include "bl_utils.h"
#include "lmac_msg.h"

static const char *const bl_mmid2str[MSG_I(MM_MAX)] = {
    [MSG_I(MM_RESET_REQ)]                 = "MM_RESET_REQ",
    [MSG_I(MM_RESET_CFM)]                 = "MM_RESET_CFM",
    [MSG_I(MM_START_REQ)]                 = "MM_START_REQ",
    [MSG_I(MM_START_CFM)]                 = "MM_START_CFM",
    [MSG_I(MM_VERSION_REQ)]               = "MM_VERSION_REQ",
    [MSG_I(MM_VERSION_CFM)]               = "MM_VERSION_CFM",
    [MSG_I(MM_ADD_IF_REQ)]                = "MM_ADD_IF_REQ",
    [MSG_I(MM_ADD_IF_CFM)]                = "MM_ADD_IF_CFM",
    [MSG_I(MM_REMOVE_IF_REQ)]             = "MM_REMOVE_IF_REQ",
    [MSG_I(MM_REMOVE_IF_CFM)]             = "MM_REMOVE_IF_CFM",
    [MSG_I(MM_STA_ADD_REQ)]               = "MM_STA_ADD_REQ",
    [MSG_I(MM_STA_ADD_CFM)]               = "MM_STA_ADD_CFM",
    [MSG_I(MM_STA_DEL_REQ)]               = "MM_STA_DEL_REQ",
    [MSG_I(MM_STA_DEL_CFM)]               = "MM_STA_DEL_CFM",
    [MSG_I(MM_SET_FILTER_REQ)]            = "MM_SET_FILTER_REQ",
    [MSG_I(MM_SET_FILTER_CFM)]            = "MM_SET_FILTER_CFM",
    [MSG_I(MM_SET_CHANNEL_REQ)]           = "MM_SET_CHANNEL_REQ",
    [MSG_I(MM_SET_CHANNEL_CFM)]           = "MM_SET_CHANNEL_CFM",
    [MSG_I(MM_SET_DTIM_REQ)]              = "MM_SET_DTIM_REQ",
    [MSG_I(MM_SET_DTIM_CFM)]              = "MM_SET_DTIM_CFM",
    [MSG_I(MM_SET_BEACON_INT_REQ)]        = "MM_SET_BEACON_INT_REQ",
    [MSG_I(MM_SET_BEACON_INT_CFM)]        = "MM_SET_BEACON_INT_CFM",
    [MSG_I(MM_SET_BASIC_RATES_REQ)]       = "MM_SET_BASIC_RATES_REQ",
    [MSG_I(MM_SET_BASIC_RATES_CFM)]       = "MM_SET_BASIC_RATES_CFM",
    [MSG_I(MM_SET_BSSID_REQ)]             = "MM_SET_BSSID_REQ",
    [MSG_I(MM_SET_BSSID_CFM)]             = "MM_SET_BSSID_CFM",
    [MSG_I(MM_SET_EDCA_REQ)]              = "MM_SET_EDCA_REQ",
    [MSG_I(MM_SET_EDCA_CFM)]              = "MM_SET_EDCA_CFM",
    [MSG_I(MM_SET_MODE_REQ)]              = "MM_SET_MODE_REQ",
    [MSG_I(MM_SET_MODE_CFM)]              = "MM_SET_MODE_CFM",
    [MSG_I(MM_SET_VIF_STATE_REQ)]         = "MM_SET_VIF_STATE_REQ",
    [MSG_I(MM_SET_VIF_STATE_CFM)]         = "MM_SET_VIF_STATE_CFM",
    [MSG_I(MM_SET_SLOTTIME_REQ)]          = "MM_SET_SLOTTIME_REQ",
    [MSG_I(MM_SET_SLOTTIME_CFM)]          = "MM_SET_SLOTTIME_CFM",
    [MSG_I(MM_SET_IDLE_REQ)]              = "MM_SET_IDLE_REQ",
    [MSG_I(MM_SET_IDLE_CFM)]              = "MM_SET_IDLE_CFM",
    [MSG_I(MM_KEY_ADD_REQ)]               = "MM_KEY_ADD_REQ",
    [MSG_I(MM_KEY_ADD_CFM)]               = "MM_KEY_ADD_CFM",
    [MSG_I(MM_KEY_DEL_REQ)]               = "MM_KEY_DEL_REQ",
    [MSG_I(MM_KEY_DEL_CFM)]               = "MM_KEY_DEL_CFM",
    [MSG_I(MM_BA_ADD_REQ)]                = "MM_BA_ADD_REQ",
    [MSG_I(MM_BA_ADD_CFM)]                = "MM_BA_ADD_CFM",
    [MSG_I(MM_BA_DEL_REQ)]                = "MM_BA_DEL_REQ",
    [MSG_I(MM_BA_DEL_CFM)]                = "MM_BA_DEL_CFM",
    [MSG_I(MM_PRIMARY_TBTT_IND)]          = "MM_PRIMARY_TBTT_IND",
    [MSG_I(MM_SECONDARY_TBTT_IND)]        = "MM_SECONDARY_TBTT_IND",
    [MSG_I(MM_SET_POWER_REQ)]             = "MM_SET_POWER_REQ",
    [MSG_I(MM_SET_POWER_CFM)]             = "MM_SET_POWER_CFM",
    [MSG_I(MM_DBG_TRIGGER_REQ)]           = "MM_DBG_TRIGGER_REQ",
    [MSG_I(MM_SET_PS_MODE_REQ)]           = "MM_SET_PS_MODE_REQ",
    [MSG_I(MM_SET_PS_MODE_CFM)]           = "MM_SET_PS_MODE_CFM",
    [MSG_I(MM_CHAN_CTXT_ADD_REQ)]         = "MM_CHAN_CTXT_ADD_REQ",
    [MSG_I(MM_CHAN_CTXT_ADD_CFM)]         = "MM_CHAN_CTXT_ADD_CFM",
    [MSG_I(MM_CHAN_CTXT_DEL_REQ)]         = "MM_CHAN_CTXT_DEL_REQ",
    [MSG_I(MM_CHAN_CTXT_DEL_CFM)]         = "MM_CHAN_CTXT_DEL_CFM",
    [MSG_I(MM_CHAN_CTXT_LINK_REQ)]        = "MM_CHAN_CTXT_LINK_REQ",
    [MSG_I(MM_CHAN_CTXT_LINK_CFM)]        = "MM_CHAN_CTXT_LINK_CFM",
    [MSG_I(MM_CHAN_CTXT_UNLINK_REQ)]      = "MM_CHAN_CTXT_UNLINK_REQ",
    [MSG_I(MM_CHAN_CTXT_UNLINK_CFM)]      = "MM_CHAN_CTXT_UNLINK_CFM",
    [MSG_I(MM_CHAN_CTXT_UPDATE_REQ)]      = "MM_CHAN_CTXT_UPDATE_REQ",
    [MSG_I(MM_CHAN_CTXT_UPDATE_CFM)]      = "MM_CHAN_CTXT_UPDATE_CFM",
    [MSG_I(MM_CHAN_CTXT_SCHED_REQ)]       = "MM_CHAN_CTXT_SCHED_REQ",
    [MSG_I(MM_CHAN_CTXT_SCHED_CFM)]       = "MM_CHAN_CTXT_SCHED_CFM",
    [MSG_I(MM_BCN_CHANGE_REQ)]            = "MM_BCN_CHANGE_REQ",
    [MSG_I(MM_BCN_CHANGE_CFM)]            = "MM_BCN_CHANGE_CFM",
    [MSG_I(MM_TIM_UPDATE_REQ)]            = "MM_TIM_UPDATE_REQ",
    [MSG_I(MM_TIM_UPDATE_CFM)]            = "MM_TIM_UPDATE_CFM",
    [MSG_I(MM_CONNECTION_LOSS_IND)]       = "MM_CONNECTION_LOSS_IND",
    [MSG_I(MM_CHANNEL_SWITCH_IND)]        = "MM_CHANNEL_SWITCH_IND",
    [MSG_I(MM_CHANNEL_PRE_SWITCH_IND)]    = "MM_CHANNEL_PRE_SWITCH_IND",
    [MSG_I(MM_REMAIN_ON_CHANNEL_REQ)]     = "MM_REMAIN_ON_CHANNEL_REQ",
    [MSG_I(MM_REMAIN_ON_CHANNEL_CFM)]     = "MM_REMAIN_ON_CHANNEL_CFM",
    [MSG_I(MM_REMAIN_ON_CHANNEL_EXP_IND)] = "MM_REMAIN_ON_CHANNEL_EXP_IND",
    [MSG_I(MM_PS_CHANGE_IND)]             = "MM_PS_CHANGE_IND",
    [MSG_I(MM_TRAFFIC_REQ_IND)]           = "MM_TRAFFIC_REQ_IND",
    [MSG_I(MM_SET_PS_OPTIONS_REQ)]        = "MM_SET_PS_OPTIONS_REQ",
    [MSG_I(MM_SET_PS_OPTIONS_CFM)]        = "MM_SET_PS_OPTIONS_CFM",
    [MSG_I(MM_P2P_VIF_PS_CHANGE_IND)]     = "MM_P2P_VIF_PS_CHANGE_IND",
    [MSG_I(MM_CSA_COUNTER_IND)]           = "MM_CSA_COUNTER_IND",
    [MSG_I(MM_CHANNEL_SURVEY_IND)]        = "MM_CHANNEL_SURVEY_IND",
    [MSG_I(MM_CFG_RSSI_REQ)]              = "MM_CFG_RSSI_REQ",
    [MSG_I(MM_RSSI_STATUS_IND)]           = "MM_RSSI_STATUS_IND",
    [MSG_I(MM_CSA_FINISH_IND)]            = "MM_CSA_FINISH_IND",
    [MSG_I(MM_CSA_TRAFFIC_IND)]           = "MM_CSA_TRAFFIC_IND",
    [MSG_I(MM_SET_RFTLV_REQ)]             = "MM_SET_RFTLV_REQ",
    [MSG_I(MM_SET_RFTLV_CFM)]             = "MM_SET_RFTLV_CFM",
    [MSG_I(MM_WMM_CFG_REQ)]               = "MM_WMM_CFG_REQ",
    [MSG_I(MM_WMM_CFG_CFM)]               = "MM_WMM_CFG_CFM",
    [MSG_I(MM_CAL_CFG_REQ)]               = "MM_CAL_CFG_REQ",
    [MSG_I(MM_CAL_CFG_CFM)]               = "MM_CAL_CFG_CFM",
    [MSG_I(MM_TEMP_READ_REQ)]             = "MM_TEMP_READ_REQ",
    [MSG_I(MM_TEMP_READ_CFM)]             = "MM_TEMP_READ_CFM",
};

static const char *const bl_dbgid2str[MSG_I(DBG_MAX)] = {
    [MSG_I(DBG_MEM_READ_REQ)]        = "DBG_MEM_READ_REQ",
    [MSG_I(DBG_MEM_READ_CFM)]        = "DBG_MEM_READ_CFM",
    [MSG_I(DBG_MEM_WRITE_REQ)]       = "DBG_MEM_WRITE_REQ",
    [MSG_I(DBG_MEM_WRITE_CFM)]       = "DBG_MEM_WRITE_CFM",
    [MSG_I(DBG_SET_MOD_FILTER_REQ)]  = "DBG_SET_MOD_FILTER_REQ",
    [MSG_I(DBG_SET_MOD_FILTER_CFM)]  = "DBG_SET_MOD_FILTER_CFM",
    [MSG_I(DBG_SET_SEV_FILTER_REQ)]  = "DBG_SET_SEV_FILTER_REQ",
    [MSG_I(DBG_SET_SEV_FILTER_CFM)]  = "DBG_SET_SEV_FILTER_CFM",
    [MSG_I(DBG_ERROR_IND)]           = "DBG_ERROR_IND",
    [MSG_I(DBG_GET_SYS_STAT_REQ)]    = "DBG_GET_SYS_STAT_REQ",
    [MSG_I(DBG_GET_SYS_STAT_CFM)]    = "DBG_GET_SYS_STAT_CFM",
};

static const char *const bl_scanid2str[MSG_I(SCAN_MAX)] = {
    [MSG_I(SCAN_START_REQ)]          = "SCAN_START_REQ",
    [MSG_I(SCAN_START_CFM)]          = "SCAN_START_CFM",
    [MSG_I(SCAN_DONE_IND)]           = "SCAN_DONE_IND",
};

static const char *const bl_scanuid2str[MSG_I(SCANU_MAX)] = {
    [MSG_I(SCANU_START_REQ)]  = "SCANU_START_REQ",
    [MSG_I(SCANU_START_CFM)]  = "SCANU_START_CFM",
    [MSG_I(SCANU_JOIN_REQ)]   = "SCANU_JOIN_REQ",
    [MSG_I(SCANU_JOIN_CFM)]   = "SCANU_JOIN_CFM",
    [MSG_I(SCANU_RESULT_IND)] = "SCANU_RESULT_IND",
    [MSG_I(SCANU_FAST_REQ)]   = "SCANU_FAST_REQ",
    [MSG_I(SCANU_FAST_CFM)]   = "SCANU_FAST_CFM",
};

static const char *const bl_meid2str[MSG_I(ME_MAX)] = {
    [MSG_I(ME_CONFIG_REQ)]           = "ME_CONFIG_REQ",
    [MSG_I(ME_CONFIG_CFM)]           = "ME_CONFIG_CFM",
    [MSG_I(ME_CHAN_CONFIG_REQ)]      = "ME_CHAN_CONFIG_REQ",
    [MSG_I(ME_CHAN_CONFIG_CFM)]      = "ME_CHAN_CONFIG_CFM",
    [MSG_I(ME_SET_CONTROL_PORT_REQ)] = "ME_SET_CONTROL_PORT_REQ",
    [MSG_I(ME_SET_CONTROL_PORT_CFM)] = "ME_SET_CONTROL_PORT_CFM",
    [MSG_I(ME_TKIP_MIC_FAILURE_IND)] = "ME_TKIP_MIC_FAILURE_IND",
    [MSG_I(ME_STA_ADD_REQ)]          = "ME_STA_ADD_REQ",
    [MSG_I(ME_STA_ADD_CFM)]          = "ME_STA_ADD_CFM",
    [MSG_I(ME_STA_DEL_REQ)]          = "ME_STA_DEL_REQ",
    [MSG_I(ME_STA_DEL_CFM)]          = "ME_STA_DEL_CFM",
    [MSG_I(ME_TX_CREDITS_UPDATE_IND)]= "ME_TX_CREDITS_UPDATE_IND",
    [MSG_I(ME_RC_STATS_REQ)]         = "ME_RC_STATS_REQ",
    [MSG_I(ME_RC_STATS_CFM)]         = "ME_RC_STATS_CFM",
    [MSG_I(ME_RC_SET_RATE_REQ)]      = "ME_RC_SET_RATE_REQ",
    [MSG_I(ME_TRAFFIC_IND_REQ)]      = "ME_TRAFFIC_IND_REQ",
    [MSG_I(ME_TRAFFIC_IND_CFM)]      = "ME_TRAFFIC_IND_CFM",
};

static const char *const bl_smid2str[MSG_I(SM_MAX)] = {
    [MSG_I(SM_CONNECT_REQ)]       = "SM_CONNECT_REQ",
    [MSG_I(SM_CONNECT_CFM)]       = "SM_CONNECT_CFM",
    [MSG_I(SM_CONNECT_IND)]       = "SM_CONNECT_IND",
    [MSG_I(SM_DISCONNECT_REQ)]    = "SM_DISCONNECT_REQ",
    [MSG_I(SM_DISCONNECT_CFM)]    = "SM_DISCONNECT_CFM",
    [MSG_I(SM_DISCONNECT_IND)]    = "SM_DISCONNECT_IND",
    [MSG_I(SM_EXTERNAL_AUTH_REQUIRED_IND)] = "SM_EXTERNAL_AUTH_REQUIRED_IND",
    [MSG_I(SM_EXTERNAL_AUTH_REQUIRED_RSP)] = "SM_EXTERNAL_AUTH_REQUIRED_RSP",
    [MSG_I(SM_RSP_TIMEOUT_IND)]   = "SM_RSP_TIMEOUT_IND",
};

static const char *const bl_apmid2str[MSG_I(APM_MAX)] = {
    [MSG_I(APM_START_REQ)]     = "APM_START_REQ",
    [MSG_I(APM_START_CFM)]     = "APM_START_CFM",
    [MSG_I(APM_STOP_REQ)]      = "APM_STOP_REQ",
    [MSG_I(APM_STOP_CFM)]      = "APM_STOP_CFM",
    [MSG_I(APM_START_CAC_REQ)] = "APM_START_CAC_REQ",
    [MSG_I(APM_START_CAC_CFM)] = "APM_START_CAC_CFM",
    [MSG_I(APM_STOP_CAC_REQ)]  = "APM_STOP_CAC_REQ",
    [MSG_I(APM_STOP_CAC_CFM)]  = "APM_STOP_CAC_CFM",
};

static const char *const bl_mpid2str[MSG_I(MP_MAX)] = {
    [MSG_I(MP_TEST_REQ)]     = "MP_TEST_REQ",
    [MSG_I(MP_TEST_CFM)]     = "MP_TEST_CFM",
};

const char *const *bl_id2str[TASK_MAX+1] = {
    [TASK_MM]    = bl_mmid2str,
    [TASK_DBG]   = bl_dbgid2str,
    [TASK_SCAN]  = bl_scanid2str,
    [TASK_SCANU] = bl_scanuid2str,
    [TASK_ME]    = bl_meid2str,
    [TASK_SM]    = bl_smid2str,
    [TASK_APM]   = bl_apmid2str,
    [TASK_MP]    = bl_mpid2str,
};
