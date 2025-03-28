/**
 ******************************************************************************
 *
 *  @file bl_compat.h
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

#ifndef _bl_COMPAT_H_
#define _bl_COMPAT_H_
#include <linux/version.h>
#include <linux/completion.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 0, 0)
#error "Minimum kernel version supported is 3.0.0"
#endif

/* Generic */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
#define __bf_shf(x) (__builtin_ffsll(x) - 1)
#define FIELD_PREP(_mask, _val) \
    (((typeof(_mask))(_val) << __bf_shf(_mask)) & (_mask))
#else
#include <linux/bitfield.h>
#endif // 4.9

//#define BL_WPA3_COMPAT  

#if 0
/* CFG80211 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 1, 0)
#define WLAN_EXT_CAPA3_MULTI_BSSID_SUPPORT	0

#define IEEE80211_HE_PHY_CAP9_NOMIMAL_PKT_PADDING_0US           0x00
#define IEEE80211_HE_PHY_CAP9_NOMIMAL_PKT_PADDING_8US           0x40
#define IEEE80211_HE_PHY_CAP9_NOMIMAL_PKT_PADDING_16US          0x80
#define IEEE80211_HE_PHY_CAP9_NOMIMAL_PKT_PADDING_RESERVED      0xc0
#define IEEE80211_HE_PHY_CAP9_NOMIMAL_PKT_PADDING_MASK          0xc0

struct element {
	u8 id;
	u8 datalen;
	u8 data[];
} __packed;

#define for_each_element(_elem, _data, _datalen)			\
	for (_elem = (const struct element *)(_data);			\
	     (const u8 *)(_data) + (_datalen) - (const u8 *)_elem >=	\
		(int)sizeof(*_elem) &&					\
	     (const u8 *)(_data) + (_datalen) - (const u8 *)_elem >=	\
		(int)sizeof(*_elem) + _elem->datalen;			\
	     _elem = (const struct element *)(_elem->data + _elem->datalen))

static inline const struct element *
cfg80211_find_elem(u8 eid, const u8 *ies, int len)
{
    const struct element *elem;
    for_each_element(elem, ies, len) {
        if (elem->id == (eid))
            return elem;
    }
    return NULL;
}

#endif // 5.1
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 11, 0)
#define cfg80211_ch_switch_started_notify(dev, chandef, count, quiet)   \
    cfg80211_ch_switch_started_notify(dev, chandef, count)
#endif // 5.11

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 0, 0)
#define cfg80211_notify_new_peer_candidate(dev, addr, ie, ie_len, sig_dbm, gfp) \
    cfg80211_notify_new_peer_candidate(dev, addr, ie, ie_len, gfp)

#define WLAN_EXT_CAPA10_TWT_REQUESTER_SUPPORT    BIT(5)
#define WLAN_EXT_CAPA10_TWT_RESPONDER_SUPPORT    BIT(6)

#endif // 5.0

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 20, 0)
#define IEEE80211_HE_MAC_CAP3_MAX_AMPDU_LEN_EXP_MASK IEEE80211_HE_MAC_CAP3_MAX_A_AMPDU_LEN_EXP_MASK
#endif // 4.20

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 0)
#define IEEE80211_RADIOTAP_HE 23
#define IEEE80211_RADIOTAP_HE_MU 24

struct ieee80211_radiotap_he {
	__le16 data1, data2, data3, data4, data5, data6;
};

enum ieee80211_radiotap_he_bits {
	IEEE80211_RADIOTAP_HE_DATA1_FORMAT_MASK		= 3,
	IEEE80211_RADIOTAP_HE_DATA1_FORMAT_SU		= 0,
	IEEE80211_RADIOTAP_HE_DATA1_FORMAT_EXT_SU	= 1,
	IEEE80211_RADIOTAP_HE_DATA1_FORMAT_MU		= 2,
	IEEE80211_RADIOTAP_HE_DATA1_FORMAT_TRIG		= 3,

	IEEE80211_RADIOTAP_HE_DATA1_BSS_COLOR_KNOWN	= 0x0004,
	IEEE80211_RADIOTAP_HE_DATA1_BEAM_CHANGE_KNOWN	= 0x0008,
	IEEE80211_RADIOTAP_HE_DATA1_UL_DL_KNOWN		= 0x0010,
	IEEE80211_RADIOTAP_HE_DATA1_DATA_MCS_KNOWN	= 0x0020,
	IEEE80211_RADIOTAP_HE_DATA1_DATA_DCM_KNOWN	= 0x0040,
	IEEE80211_RADIOTAP_HE_DATA1_CODING_KNOWN	= 0x0080,
	IEEE80211_RADIOTAP_HE_DATA1_LDPC_XSYMSEG_KNOWN	= 0x0100,
	IEEE80211_RADIOTAP_HE_DATA1_STBC_KNOWN		= 0x0200,
	IEEE80211_RADIOTAP_HE_DATA1_SPTL_REUSE_KNOWN	= 0x0400,
	IEEE80211_RADIOTAP_HE_DATA1_SPTL_REUSE2_KNOWN	= 0x0800,
	IEEE80211_RADIOTAP_HE_DATA1_SPTL_REUSE3_KNOWN	= 0x1000,
	IEEE80211_RADIOTAP_HE_DATA1_SPTL_REUSE4_KNOWN	= 0x2000,
	IEEE80211_RADIOTAP_HE_DATA1_BW_RU_ALLOC_KNOWN	= 0x4000,
	IEEE80211_RADIOTAP_HE_DATA1_DOPPLER_KNOWN	= 0x8000,

	IEEE80211_RADIOTAP_HE_DATA2_PRISEC_80_KNOWN	= 0x0001,
	IEEE80211_RADIOTAP_HE_DATA2_GI_KNOWN		= 0x0002,
	IEEE80211_RADIOTAP_HE_DATA2_NUM_LTF_SYMS_KNOWN	= 0x0004,
	IEEE80211_RADIOTAP_HE_DATA2_PRE_FEC_PAD_KNOWN	= 0x0008,
	IEEE80211_RADIOTAP_HE_DATA2_TXBF_KNOWN		= 0x0010,
	IEEE80211_RADIOTAP_HE_DATA2_PE_DISAMBIG_KNOWN	= 0x0020,
	IEEE80211_RADIOTAP_HE_DATA2_TXOP_KNOWN		= 0x0040,
	IEEE80211_RADIOTAP_HE_DATA2_MIDAMBLE_KNOWN	= 0x0080,
	IEEE80211_RADIOTAP_HE_DATA2_RU_OFFSET		= 0x3f00,
	IEEE80211_RADIOTAP_HE_DATA2_RU_OFFSET_KNOWN	= 0x4000,
	IEEE80211_RADIOTAP_HE_DATA2_PRISEC_80_SEC	= 0x8000,

	IEEE80211_RADIOTAP_HE_DATA3_BSS_COLOR		= 0x003f,
	IEEE80211_RADIOTAP_HE_DATA3_BEAM_CHANGE		= 0x0040,
	IEEE80211_RADIOTAP_HE_DATA3_UL_DL		= 0x0080,
	IEEE80211_RADIOTAP_HE_DATA3_DATA_MCS		= 0x0f00,
	IEEE80211_RADIOTAP_HE_DATA3_DATA_DCM		= 0x1000,
	IEEE80211_RADIOTAP_HE_DATA3_CODING		= 0x2000,
	IEEE80211_RADIOTAP_HE_DATA3_LDPC_XSYMSEG	= 0x4000,
	IEEE80211_RADIOTAP_HE_DATA3_STBC		= 0x8000,

	IEEE80211_RADIOTAP_HE_DATA4_SU_MU_SPTL_REUSE	= 0x000f,
	IEEE80211_RADIOTAP_HE_DATA4_MU_STA_ID		= 0x7ff0,
	IEEE80211_RADIOTAP_HE_DATA4_TB_SPTL_REUSE1	= 0x000f,
	IEEE80211_RADIOTAP_HE_DATA4_TB_SPTL_REUSE2	= 0x00f0,
	IEEE80211_RADIOTAP_HE_DATA4_TB_SPTL_REUSE3	= 0x0f00,
	IEEE80211_RADIOTAP_HE_DATA4_TB_SPTL_REUSE4	= 0xf000,

	IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC	= 0x000f,
		IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC_20MHZ	= 0,
		IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC_40MHZ	= 1,
		IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC_80MHZ	= 2,
		IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC_160MHZ	= 3,
		IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC_26T	= 4,
		IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC_52T	= 5,
		IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC_106T	= 6,
		IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC_242T	= 7,
		IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC_484T	= 8,
		IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC_996T	= 9,
		IEEE80211_RADIOTAP_HE_DATA5_DATA_BW_RU_ALLOC_2x996T	= 10,

	IEEE80211_RADIOTAP_HE_DATA5_GI			= 0x0030,
		IEEE80211_RADIOTAP_HE_DATA5_GI_0_8			= 0,
		IEEE80211_RADIOTAP_HE_DATA5_GI_1_6			= 1,
		IEEE80211_RADIOTAP_HE_DATA5_GI_3_2			= 2,

	IEEE80211_RADIOTAP_HE_DATA5_LTF_SIZE		= 0x00c0,
		IEEE80211_RADIOTAP_HE_DATA5_LTF_SIZE_UNKNOWN		= 0,
		IEEE80211_RADIOTAP_HE_DATA5_LTF_SIZE_1X			= 1,
		IEEE80211_RADIOTAP_HE_DATA5_LTF_SIZE_2X			= 2,
		IEEE80211_RADIOTAP_HE_DATA5_LTF_SIZE_4X			= 3,
	IEEE80211_RADIOTAP_HE_DATA5_NUM_LTF_SYMS	= 0x0700,
	IEEE80211_RADIOTAP_HE_DATA5_PRE_FEC_PAD		= 0x3000,
	IEEE80211_RADIOTAP_HE_DATA5_TXBF		= 0x4000,
	IEEE80211_RADIOTAP_HE_DATA5_PE_DISAMBIG		= 0x8000,

	IEEE80211_RADIOTAP_HE_DATA6_NSTS		= 0x000f,
	IEEE80211_RADIOTAP_HE_DATA6_DOPPLER		= 0x0010,
	IEEE80211_RADIOTAP_HE_DATA6_TXOP		= 0x7f00,
	IEEE80211_RADIOTAP_HE_DATA6_MIDAMBLE_PDCTY	= 0x8000,
};

struct ieee80211_radiotap_he_mu {
	__le16 flags1, flags2;
	u8 ru_ch1[4];
	u8 ru_ch2[4];
};

enum ieee80211_radiotap_he_mu_bits {
	IEEE80211_RADIOTAP_HE_MU_FLAGS1_SIG_B_MCS		= 0x000f,
	IEEE80211_RADIOTAP_HE_MU_FLAGS1_SIG_B_MCS_KNOWN		= 0x0010,
	IEEE80211_RADIOTAP_HE_MU_FLAGS1_SIG_B_DCM		= 0x0020,
	IEEE80211_RADIOTAP_HE_MU_FLAGS1_SIG_B_DCM_KNOWN		= 0x0040,
	IEEE80211_RADIOTAP_HE_MU_FLAGS1_CH2_CTR_26T_RU_KNOWN	= 0x0080,
	IEEE80211_RADIOTAP_HE_MU_FLAGS1_CH1_RU_KNOWN		= 0x0100,
	IEEE80211_RADIOTAP_HE_MU_FLAGS1_CH2_RU_KNOWN		= 0x0200,
	IEEE80211_RADIOTAP_HE_MU_FLAGS1_CH1_CTR_26T_RU_KNOWN	= 0x1000,
	IEEE80211_RADIOTAP_HE_MU_FLAGS1_CH1_CTR_26T_RU		= 0x2000,
	IEEE80211_RADIOTAP_HE_MU_FLAGS1_SIG_B_COMP_KNOWN	= 0x4000,
	IEEE80211_RADIOTAP_HE_MU_FLAGS1_SIG_B_SYMS_USERS_KNOWN	= 0x8000,

	IEEE80211_RADIOTAP_HE_MU_FLAGS2_BW_FROM_SIG_A_BW	= 0x0003,
		IEEE80211_RADIOTAP_HE_MU_FLAGS2_BW_FROM_SIG_A_BW_20MHZ	= 0x0000,
		IEEE80211_RADIOTAP_HE_MU_FLAGS2_BW_FROM_SIG_A_BW_40MHZ	= 0x0001,
		IEEE80211_RADIOTAP_HE_MU_FLAGS2_BW_FROM_SIG_A_BW_80MHZ	= 0x0002,
		IEEE80211_RADIOTAP_HE_MU_FLAGS2_BW_FROM_SIG_A_BW_160MHZ	= 0x0003,
	IEEE80211_RADIOTAP_HE_MU_FLAGS2_BW_FROM_SIG_A_BW_KNOWN	= 0x0004,
	IEEE80211_RADIOTAP_HE_MU_FLAGS2_SIG_B_COMP		= 0x0008,
	IEEE80211_RADIOTAP_HE_MU_FLAGS2_SIG_B_SYMS_USERS	= 0x00f0,
	IEEE80211_RADIOTAP_HE_MU_FLAGS2_PUNC_FROM_SIG_A_BW	= 0x0300,
	IEEE80211_RADIOTAP_HE_MU_FLAGS2_PUNC_FROM_SIG_A_BW_KNOWN= 0x0400,
	IEEE80211_RADIOTAP_HE_MU_FLAGS2_CH2_CTR_26T_RU		= 0x0800,
};

enum {
    IEEE80211_HE_MCS_SUPPORT_0_7    = 0,
    IEEE80211_HE_MCS_SUPPORT_0_9    = 1,
    IEEE80211_HE_MCS_SUPPORT_0_11   = 2,
    IEEE80211_HE_MCS_NOT_SUPPORTED  = 3,
};
#endif // 4.19

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 17, 0)
#define cfg80211_probe_status(ndev, addr, cookie, ack, ack_pwr, pwr_valid, gfp) \
    cfg80211_probe_status(ndev, addr, cookie, ack, gfp)
#endif // 4.17

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 12, 0)
#define bl_cfg80211_change_iface(wiphy, dev, type, params) \
    bl_cfg80211_change_iface(wiphy, dev, type, u32 *flags, params)

#define CCFS0(vht) vht->center_freq_seg1_idx
#define CCFS1(vht) vht->center_freq_seg2_idx

#define nla_parse(tb, maxtype, head, len, policy, extack)       \
    nla_parse(tb, maxtype, head, len, policy)

struct cfg80211_roam_info {
	struct ieee80211_channel *channel;
	struct cfg80211_bss *bss;
	const u8 *bssid;
	const u8 *req_ie;
	size_t req_ie_len;
	const u8 *resp_ie;
	size_t resp_ie_len;
};

#define cfg80211_roamed(_dev, _info, _gfp) \
    cfg80211_roamed(_dev, (_info)->channel, (_info)->bssid, (_info)->req_ie, \
                    (_info)->req_ie_len, (_info)->resp_ie, (_info)->resp_ie_len, _gfp)

#else

#define CCFS0(vht) vht->center_freq_seg0_idx
#define CCFS1(vht) vht->center_freq_seg1_idx
#endif // 4.12

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 11, 0)
#define cfg80211_cqm_rssi_notify(dev, event, level, gfp) \
    cfg80211_cqm_rssi_notify(dev, event, gfp)
#endif // 4.11

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
#define ieee80211_amsdu_to_8023s(skb, list, addr, iftype, extra_headroom, check_da, check_sa) \
    ieee80211_amsdu_to_8023s(skb, list, addr, iftype, extra_headroom, false)
#endif // 4.9

#if LINUX_VERSION_CODE  < KERNEL_VERSION(4, 7, 0)
#define NUM_NL80211_BANDS IEEE80211_NUM_BANDS
#endif // 4.7

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0)
#define cfg80211_disconnected(dev, reason, ie, len, local, gfp) \
    cfg80211_disconnected(dev, reason, ie, len, gfp)
#endif // 4.2

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 1, 0)) && !(defined CONFIG_VENDOR_bl)
#define ieee80211_chandef_to_operating_class(chan_def, op_class) 0
#endif // 4.1

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 0, 0)
#define WLAN_CIPHER_SUITE_GCMP_256	0x000FAC09
#define WLAN_CIPHER_SUITE_CCMP_256	0x000FAC0A

#define IEEE80211_CCMP_256_MIC_LEN	16
#define IEEE80211_GCMP_MIC_LEN		16

#define SURVEY_INFO_TIME          SURVEY_INFO_CHANNEL_TIME
#define SURVEY_INFO_TIME_BUSY     SURVEY_INFO_CHANNEL_TIME_BUSY
#define SURVEY_INFO_TIME_EXT_BUSY SURVEY_INFO_CHANNEL_TIME_EXT_BUSY
#define SURVEY_INFO_TIME_RX       SURVEY_INFO_CHANNEL_TIME_RX
#define SURVEY_INFO_TIME_TX       SURVEY_INFO_CHANNEL_TIME_TX

#define SURVEY_TIME(s) s->channel_time
#define SURVEY_TIME_BUSY(s) s->channel_time_busy
#else
#define SURVEY_TIME(s) s->time
#define SURVEY_TIME_BUSY(s) s->time_busy
#endif // 4.0

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 19, 0)
#define cfg80211_ch_switch_started_notify(dev, chandef, count, quiet)

#define WLAN_BSS_COEX_INFORMATION_REQUEST	BIT(0)
#define WLAN_EXT_CAPA1_EXT_CHANNEL_SWITCHING	BIT(2)
#define WLAN_EXT_CAPA4_TDLS_BUFFER_STA		BIT(4)
#define WLAN_EXT_CAPA4_TDLS_PEER_PSM		BIT(5)
#define WLAN_EXT_CAPA4_TDLS_CHAN_SWITCH		BIT(6)
#define WLAN_EXT_CAPA5_TDLS_CH_SW_PROHIBITED	BIT(7)
#define NL80211_FEATURE_TDLS_CHANNEL_SWITCH     0

#define STA_TDLS_INITIATOR(sta) 0

#define REGULATORY_IGNORE_STALE_KICKOFF 0
#else
#define STA_TDLS_INITIATOR(sta) sta->tdls_initiator
#endif // 3.19


#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 17, 0)
#define bl_cfg80211_tdls_mgmt(wiphy, dev, peer, act, tok, status, peer_capability, initiator, buf, len) \
    bl_cfg80211_tdls_mgmt(wiphy, dev, peer, act, tok, status, peer_capability, buf, len)


#include <linux/types.h>

struct ieee80211_wmm_ac_param {
	u8 aci_aifsn; /* AIFSN, ACM, ACI */
	u8 cw; /* ECWmin, ECWmax (CW = 2^ECW - 1) */
	__le16 txop_limit;
} __packed;

struct ieee80211_wmm_param_ie {
	u8 element_id; /* Element ID: 221 (0xdd); */
	u8 len; /* Length: 24 */
	/* required fields for WMM version 1 */
	u8 oui[3]; /* 00:50:f2 */
	u8 oui_type; /* 2 */
	u8 oui_subtype; /* 1 */
	u8 version; /* 1 for WMM version 1.0 */
	u8 qos_info; /* AP/STA specific QoS info */
	u8 reserved; /* 0 */
	/* AC_BE, AC_BK, AC_VI, AC_VO */
	struct ieee80211_wmm_ac_param ac[4];
} __packed;
#endif // 3.17

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 9, 0)
#define cfg80211_put_bss(wiphy, bss) \
    cfg80211_put_bss(bss)
#endif // 3.9

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
#define IEEE80211_NUM_TIDS (16)
enum ieee80211_vht_mcs_support {
    IEEE80211_VHT_MCS_SUPPORT_0_7   = 0,
    IEEE80211_VHT_MCS_SUPPORT_0_8   = 1,
    IEEE80211_VHT_MCS_SUPPORT_0_9   = 2,
    IEEE80211_VHT_MCS_NOT_SUPPORTED = 3,
};
enum nl80211_chan_width {
    NL80211_CHAN_WIDTH_20_NOHT,
    NL80211_CHAN_WIDTH_20,
    NL80211_CHAN_WIDTH_40,
    NL80211_CHAN_WIDTH_80,
    NL80211_CHAN_WIDTH_80P80,
    NL80211_CHAN_WIDTH_160,
};
#else
#define cfg80211_ready_on_channel(dev, cookie, chan, chan_type, duration, gfp) \
    cfg80211_ready_on_channel(dev, cookie, chan, duration, gfp)

#define cfg80211_remain_on_channel_expired(dev, cookie, chan, chan_type, gfp) \
    cfg80211_remain_on_channel_expired(dev, cookie, chan, gfp)

#define cfg80211_report_obss_beacon(wiphy, frame, len, freq, sig_dbm, gfp) \
    cfg80211_report_obss_beacon(wiphy, frame, len, freq, sig_dbm)
#endif // 3.8

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 5, 0)
enum nl80211_ac {
    NL80211_AC_VO,
    NL80211_AC_VI,
    NL80211_AC_BE,
    NL80211_AC_BK,
    NL80211_NUM_ACS
};
#endif // 3.5

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 3, 0)
#define WLAN_CIPHER_SUITE_SMS4          0x00147201
#endif // 3.3


/* MAC80211 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 18, 0)
#define bl_ops_mgd_prepare_tx(hw, vif, duration) \
    bl_ops_mgd_prepare_tx(hw, vif)
#endif // 4.18

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 12, 0)

#define RX_ENC_HT(s) s->flag |= RX_FLAG_HT
#define RX_ENC_HT_GF(s) s->flag |= (RX_FLAG_HT | RX_FLAG_HT_GF)
#define RX_ENC_VHT(s) s->flag |= RX_FLAG_HT
#define RX_ENC_HE(s) s->flag |= RX_FLAG_HT
#define RX_ENC_FLAG_SHORT_GI(s) s->flag |= RX_FLAG_SHORT_GI
#define RX_ENC_FLAG_SHORT_PRE(s) s->flag |= RX_FLAG_SHORTPRE
#define RX_ENC_FLAG_LDPC(s) s->flag |= RX_FLAG_LDPC
#define RX_BW_40MHZ(s) s->flag |= RX_FLAG_40MHZ
#define RX_BW_80MHZ(s) s->vht_flag |= RX_VHT_FLAG_80MHZ
#define RX_BW_160MHZ(s) s->vht_flag |= RX_VHT_FLAG_160MHZ
#define RX_NSS(s) s->vht_nss

#else
#define RX_ENC_HT(s) s->encoding = RX_ENC_HT
#define RX_ENC_HT_GF(s) { s->encoding = RX_ENC_HT;      \
        s->enc_flags |= RX_ENC_FLAG_HT_GF; }
#define RX_ENC_VHT(s) s->encoding = RX_ENC_VHT
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 0)
#define RX_ENC_HE(s) s->encoding = RX_ENC_VHT
#else
#define RX_ENC_HE(s) s->encoding = RX_ENC_HE
#endif
#define RX_ENC_FLAG_SHORT_GI(s) s->enc_flags |= RX_ENC_FLAG_SHORT_GI
#define RX_ENC_FLAG_SHORT_PRE(s) s->enc_flags |= RX_ENC_FLAG_SHORTPRE
#define RX_ENC_FLAG_LDPC(s) s->enc_flags |= RX_ENC_FLAG_LDPC
#define RX_BW_40MHZ(s) s->bw = RATE_INFO_BW_40
#define RX_BW_80MHZ(s) s->bw = RATE_INFO_BW_80
#define RX_BW_160MHZ(s) s->bw = RATE_INFO_BW_160
#define RX_NSS(s) s->nss

#endif // 4.12

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 11, 0)
#define ieee80211_cqm_rssi_notify(vif, event, level, gfp) \
    ieee80211_cqm_rssi_notify(vif, event, gfp)
#endif // 4.11

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 7, 0)
#define RX_FLAG_MIC_STRIPPED 0
#endif // 4.7

#ifndef CONFIG_VENDOR_bl_AMSDUS_TX
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0))
#define bl_ops_ampdu_action(hw, vif, params) \
    bl_ops_ampdu_action(hw, vif, enum ieee80211_ampdu_mlme_action action, \
                          struct ieee80211_sta *sta, u16 tid, u16 *ssn, u8 buf_size)
#elif  (LINUX_VERSION_CODE < KERNEL_VERSION(4, 6, 0))
#define bl_ops_ampdu_action(hw, vif, params) \
    bl_ops_ampdu_action(hw, vif, enum ieee80211_ampdu_mlme_action action, \
                          struct ieee80211_sta *sta, u16 tid, u16 *ssn, u8 buf_size, \
                          bool amsdu)
#endif // 4.4
#endif /* CONFIG_VENDOR_bl_AMSDUS_TX */

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0)
#define IEEE80211_HW_SUPPORT_FAST_XMIT 0
#define ieee80211_hw_check(hw, feat) (hw->flags & IEEE80211_HW_##feat)
#define ieee80211_hw_set(hw, feat) {hw->flags |= IEEE80211_HW_##feat;}
#endif // 4.2

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 19, 0)
#define bl_ops_sw_scan_start(hw, vif, mac_addr) \
    bl_ops_sw_scan_start(hw)
#define bl_ops_sw_scan_complete(hw, vif) \
    bl_ops_sw_scan_complete(hw)
#endif // 3.19

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 17, 0)
#define bl_ops_hw_scan(hw, vif, hw_req) \
    bl_ops_hw_scan(hw, vif, struct cfg80211_scan_request *req)
#endif // 3.17

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 2, 0)
#define IEEE80211_NUM_ACS   4
#endif // 3.2

/* NET */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 13, 0)
#define bl_select_queue(dev, skb, sb_dev) \
    bl_select_queue(dev, skb)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0)
#define bl_select_queue(dev, skb, sb_dev) \
    bl_select_queue(dev, skb, void *accel_priv)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 0)
#define bl_select_queue(dev, skb, sb_dev) \
    bl_select_queue(dev, skb, void *accel_priv, select_queue_fallback_t fallback)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(5, 2, 0)
#define bl_select_queue(dev, skb, sb_dev) \
    bl_select_queue(dev, skb, sb_dev, select_queue_fallback_t fallback)
#elif LINUX_VERSION_CODE <= KERNEL_VERSION(5, 4, 0)
#define bl_select_queue(dev, skb, sb_dev) \
    bl_select_queue(dev, skb, sb_dev)
#endif //3.13 

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 16, 0)) && !(defined CONFIG_VENDOR_bl)
#define sk_pacing_shift_update(sk, shift)
#endif // 4.16

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 17, 0)
#define NET_NAME_UNKNOWN 0
#endif // 3.17

/* TRACE */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 0)
#define trace_print_symbols_seq ftrace_print_symbols_seq
#endif // 4.2

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 17, 0)
#define trace_seq_buffer_ptr(p) p->buffer + p->len
#endif // 3.17

/* TIME */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 8, 0)
#define time64_to_tm(t, o, tm) time_to_tm((time_t)t, o, tm)
#endif // 4.8

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 19, 0)
#define ktime_get_real_seconds get_seconds
#endif // 3.19

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 17, 0)
typedef __s64 time64_t;
#endif // 3.17

/* timer */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)
#define from_timer(var, callback_timer, timer_fieldname) \
	container_of(callback_timer, typeof(*var), timer_fieldname)

#define timer_setup(timer, callback, flags) \
    __setup_timer(timer, (void (*)(unsigned long))callback, (unsigned long)timer, flags)
#endif // 4.14

/* File System */
#if LINUX_VERSION_CODE  < KERNEL_VERSION(3, 4, 0)
static inline int simple_open(struct inode *inode, struct file *file)
{
    if (inode->i_private)
        file->private_data = inode->i_private;
    return 0;
}
#endif // 3.4


/* Misc */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,8,0)
	#define bl_cfg80211_mgmt_frame_register(wiphy, wdev, upd) \
	bl_cfg80211_mgmt_frame_register(wiphy, wdev, u16 frame_type, bool reg)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,13,0)
static inline void reinit_completion(struct completion *x)
{
	x->done = 0;
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0)
static inline bool ether_addr_equal(const u8 *addr1, const u8 *addr2)
{
	const u16 *a = (const u16 *)addr1;
	const u16 *b = (const u16 *)addr2;

	return ((a[0] ^ b[0]) | (a[1] ^ b[1]) | (a[2] ^ b[2])) == 0;
}
#endif

#endif /* _bl_COMPAT_H_ */
