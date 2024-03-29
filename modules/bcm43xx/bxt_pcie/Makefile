# bcmdhd
#####################
# SDIO Basic feature
#####################

DHDCFLAGS += -Wall -Wstrict-prototypes -Dlinux -DLINUX -DBCMDRIVER            \
	-DBCMDONGLEHOST -DUNRELEASEDCHIP -DBCMDMA32 -DBCMFILEIMAGE            \
	-DDHDTHREAD -DSHOW_EVENTS -DBCMDBG -DWLP2P                            \
	-DWIFI_ACT_FRAME -DARP_OFFLOAD_SUPPORT                                \
	-DKEEP_ALIVE -DCSCAN -DPKT_FILTER_SUPPORT                             \
	-DEMBEDDED_PLATFORM -DPNO_SUPPORT                                     \
	-DWL_SUPPORT_AUTO_CHANNEL	                                          \
	-DDHD_DONOT_FORWARD_BCMEVENT_AS_NETWORK_PKT                           \
	-DCUSTOMER_HW2 -DGET_CUSTOM_MAC_ENABLE -DDHD_FW_COREDUMP              \
	-DSEC_ENHANCEMENT
#	-DSHOW_LOGTRACE

#################
# Common feature
#################
DHDCFLAGS += -DWL_VIRTUAL_APSTA

DHDCFLAGS += -DWL_CFG80211
# Print out kernel panic point of file and line info when assertion happened
DHDCFLAGS += -DBCMASSERT_LOG

# CUSTOMER4 flags
DHDCFLAGS += -DDHD_PKTID_AUDIT_ENABLED
DHDCFLAGS += -DSUPPORT_HIDDEN_AP -DDHD_LOSSLESS_ROAMING
DHDCFLAGS += -DPASS_ALL_MCAST_PKTS -DESCAN_BUF_OVERFLOW_MGMT -DPKTPRIO_OVERRIDE
DHDCFLAGS += -DUSE_INITIAL_SHORT_DWELL_TIME -DWL_NEWCFG_PRIVCMD_SUPPORT
DHDCFLAGS += -DENABLE_BCN_LI_BCN_WAKEUP -DSOFTAP_UAPSD_OFF
DHDCFLAGS += -DSUPPORT_LTECX -DSUPPORT_2G_VHT
DHDCFLAGS += -DSUPPORT_WL_TXPOWER -DBLOCK_IPV6_PACKET -DSUPPORT_DEEP_SLEEP
DHDCFLAGS += -DSUPPORT_AMPDU_MPDU_CMD -DSOFTAP_SEND_HANGEVT -DNUM_SCB_MAX_PROBE=3
DHDCFLAGS += -DCUSTOM_AMPDU_MPDU=48 -DCUSTOM_AMPDU_RELEASE=32 -DCUSTOM_AMSDU_AGGSF=2
DHDCFLAGS += -DCUSTOM_IBSS_AMPDU_BA_WSIZE=16
DHDCFLAGS += -DCUSTOM_TCPACK_DELAY_TIME=10
DHDCFLAGS += -DDEBUGFS_CFG80211
DHDCFLAGS += -DSUPPORT_SOFTAP_WPAWPA2_MIXED

ifneq ($(filter y, $(CONFIG_ANDROID)),)
DHDCFLAGS += -DCONFIG_HAS_WAKELOCK
endif

# keepalive
DHDCFLAGS += -DCUSTOM_KEEP_ALIVE_SETTING=28000

DHDCFLAGS += -DVSDB

# For p2p connection issue
DHDCFLAGS += -DWL_SCB_TIMEOUT=10

# TDLS enable
DHDCFLAGS += -DWLTDLS -DWLTDLS_AUTO_ENABLE
# For TDLS tear down inactive time 40 sec
DHDCFLAGS += -DCUSTOM_TDLS_IDLE_MODE_SETTING=10000
# for TDLS RSSI HIGH for establishing TDLS link
DHDCFLAGS += -DCUSTOM_TDLS_RSSI_THRESHOLD_HIGH=-80
# for TDLS RSSI HIGH for tearing down TDLS link
DHDCFLAGS += -DCUSTOM_TDLS_RSSI_THRESHOLD_LOW=-85

# Roaming
DHDCFLAGS += -DROAM_AP_ENV_DETECTION
DHDCFLAGS += -DROAM_ENABLE -DROAM_CHANNEL_CACHE -DROAM_API
DHDCFLAGS += -DENABLE_FW_ROAM_SUSPEND
# Roaming trigger
DHDCFLAGS += -DCUSTOM_ROAM_TRIGGER_SETTING=-75
DHDCFLAGS += -DCUSTOM_ROAM_DELTA_SETTING=10
# Set PM 2 always regardless suspend/resume
DHDCFLAGS += -DSUPPORT_PM2_ONLY

# For special PNO Event keep wake lock for 10sec
DHDCFLAGS += -DCUSTOM_PNO_EVENT_LOCK_xTIME=10
DHDCFLAGS += -DMIRACAST_AMPDU_SIZE=8

# Vendor Extension support for Android
# if the kernel has support for vendor-extension then enable this flag in kernel \
	config.
ifeq ($(CONFIG_BCMDHD_VENDOR_EXT),y)
	DHDCFLAGS += DWL_VENDOR_EXT_SUPPORT
	DHDOFILES += wl_cfgvendor.o
endif

#Gscan
#DHDCFLAGS += -DGSCAN_SUPPORT
#RTT
DHDCFLAGS += -DRTT_SUPPORT -DRTT_DEBUG
#DHDCFLAGS += -DRTT_SUPPORT


#Link Statistics
DHDCFLAGS += -DLINKSTAT_SUPPORT

# Early suspend
DHDCFLAGS += -DDHD_USE_EARLYSUSPEND

# For Scan result patch
DHDCFLAGS += -DESCAN_RESULT_PATCH

# For Static Buffer
ifeq ($(CONFIG_DHD_USE_STATIC_BUF),y)
  DHDCFLAGS += -DENHANCED_STATIC_BUF
  DHDCFLAGS += -DSTATIC_WL_PRIV_STRUCT
endif
ifneq ($(CONFIG_DHD_USE_SCHED_SCAN),)
DHDCFLAGS += -DWL_SCHED_SCAN
endif

# Ioctl timeout 5000ms
DHDCFLAGS += -DIOCTL_RESP_TIMEOUT=5000

# Prevent rx thread monopolize
DHDCFLAGS += -DWAIT_DEQUEUE

# Config PM Control
DHDCFLAGS += -DDHD_PM_CONTROL_FROM_FILE

# idle count
DHDCFLAGS += -DDHD_USE_IDLECOUNT

# SKB TAILPAD to avoid out of boundary memory access
DHDCFLAGS += -DDHDENABLE_TAILPAD

# Wi-Fi Direct
DHDCFLAGS += -DWL_CFG80211_VSDB_PRIORITIZE_SCAN_REQUEST
DHDCFLAGS += -DWL_CFG80211_STA_EVENT
DHDCFLAGS += -DWL_IFACE_COMB_NUM_CHANNELS
DHDCFLAGS += -DWL_ENABLE_P2P_IF
#SCAN time
DHDCFLAGS += -DCUSTOM_SET_SHORT_DWELL_TIME
#DHDCFLAGS += -DCUSTOM_FORCE_NODFS_FLAG


# SoftAP
#DHDCFLAGS += -DDISABLE_11H_SOFTAP
DHDCFLAGS += -DSET_RANDOM_MAC_SOFTAP
DHDCFLAGS += -DWL_CFG80211_ACL
#DHDCFLAGS += -DDISABLE_11H

# Disable 11ac support
#DHDCFLAGS += -DDISABLE_11AC

#Disable FRAMEBURST on VSDB
DHDCFLAGS += -DDISABLE_FRAMEBURST_VSDB

# Uncomment the below line to disable 11K
#DHDCFLAGS += -DDISABLE_11K

# Prevent send hang event even though p2p discovery interface is not removed.
DHDCFLAGS += -DEXPLICIT_DISCIF_CLEANUP

##########################
# driver type
# m: module type driver
# y: built-in type driver
##########################
DRIVER_TYPE ?= m
#DRIVER_TYPE ?= $(CONFIG_BCMDHD)

#########################
# Chip dependent feature
#########################

ifneq ($(filter y m, $(CONFIG_BCM4359)),)
  DHDCFLAGS += -DUSE_WL_TXBF
  DHDCFLAGS += -DUSE_WL_FRAMEBURST
  DHDCFLAGS += -DCUSTOM_DPC_CPUCORE=0
  DHDCFLAGS += -DCUSTOM_PSPRETEND_THR=30
  DHDCFLAGS += -DMAX_AP_CLIENT_CNT=10
  DHDCFLAGS += -DMAX_GO_CLIENT_CNT=5
# New Features
  DHDCFLAGS += -DWL11U
  DHDCFLAGS += -DMFP
# Enable below macro for Dynamic Link Power Control
# DHDCFLAGS += -DDHD_ENABLE_LPC
ifeq ($(filter y, $(CONFIG_ANDROID)),)
  DHDCFLAGS += -DCUSTOM_COUNTRY_CODE
endif
  DHDCFLAGS += -DSAR_SUPPORT
# debug info
  DHDCFLAGS += -DDHD_WAKE_STATUS

ifneq ($(CONFIG_BCMDHD_PCIE),)
  DHDCFLAGS += -DPCIE_FULL_DONGLE -DBCMPCIE -DCUSTOM_DPC_PRIO_SETTING=-1
# tput enhancement
  DHDCFLAGS += -DCUSTOM_AMPDU_BA_WSIZE=64
  DHDCFLAGS += -DCUSTOM_AMPDU_MPDU=48
  DHDCFLAGS += -DCUSTOM_AMPDU_RELEASE=32
  DHDCFLAGS += -DPROP_TXSTATUS_VSDB
# Disable watchdog thread
  DHDCFLAGS += -DCUSTOM_DHD_WATCHDOG_MS=0
  DHDCFLAGS += -DMAX_CNTL_TX_TIMEOUT=1
  DHDCFLAGS += -DSUPPORT_LINKDOWN_RECOVERY
endif
ifeq ($(CONFIG_DHD_USE_STATIC_BUF),y)
  DHDCFLAGS += -DDHD_USE_STATIC_IOCTLBUF
endif
  DHDCFLAGS += -DDONGLE_ENABLE_ISOLATION
# Print 802.1X packets
  DHDCFLAGS += -DDHD_8021X_DUMP
# Print DHCP packets
  DHDCFLAGS += -DDHD_DHCP_DUMP
# Add Load Balance Feature here
#  DHDCFLAGS += -DDHD_LB
#  DHDCFLAGS += -DDHD_LB_RXP
#  DHDCFLAGS += -DDHD_LB_STATS
endif


ifneq ($(CONFIG_BCM4339),)
  DHDCFLAGS += -DBCM4339_CHIP -DHW_OOB

  # tput enhancement
  DHDCFLAGS += -DCUSTOM_GLOM_SETTING=8 -DCUSTOM_RXCHAIN=1
  DHDCFLAGS += -DUSE_DYNAMIC_F2_BLKSIZE -DDYNAMIC_F2_BLKSIZE_FOR_NONLEGACY=256
  DHDCFLAGS += -DBCMSDIOH_TXGLOM -DCUSTOM_TXGLOM=1 -DBCMSDIOH_TXGLOM_HIGHSPEED
  DHDCFLAGS += -DDHDTCPACK_SUPPRESS
  DHDCFLAGS += -DUSE_WL_TXBF
  DHDCFLAGS += -DUSE_WL_FRAMEBURST
  DHDCFLAGS += -DRXFRAME_THREAD
  DHDCFLAGS += -DCUSTOM_AMPDU_BA_WSIZE=64
  DHDCFLAGS += -DCUSTOM_DPC_CPUCORE=0
  DHDCFLAGS += -DPROP_TXSTATUS_VSDB
  DHDCFLAGS += -DCUSTOM_MAX_TXGLOM_SIZE=32

  # New Features
  DHDCFLAGS += -DWL11U
  DHDCFLAGS += -DDHD_ENABLE_LPC
  DHDCFLAGS += -DCUSTOM_PSPRETEND_THR=30
endif


#EXTRA_LDFLAGS += --strip-debug

ifeq ($(DRIVER_TYPE),y)
  DHDCFLAGS += -DENABLE_INSMOD_NO_FW_LOAD
  DHDCFLAGS += -DUSE_LATE_INITCALL_SYNC
endif

EXTRA_CFLAGS += $(DHDCFLAGS) -DDHD_DEBUG -DDHD_VERSION_NO_DATE_TIME
#EXTRA_CFLAGS += $(DHDCFLAGS)
EXTRA_CFLAGS += -DSRCBASE=\"$(src)\"
EXTRA_CFLAGS += -I$(src)/include/ -I$(src)/
KBUILD_CFLAGS += -I$(LINUXDIR)/include -I$(shell pwd)

DHDOFILES := dhd_pno.o dhd_common.o dhd_ip.o dhd_custom_gpio.o \
	dhd_linux.o dhd_linux_sched.o dhd_cfg80211.o dhd_linux_wq.o aiutils.o bcmevent.o \
	bcmutils.o bcmwifi_channels.o hndpmu.o linux_osl.o sbutils.o siutils.o \
	wl_android.o wl_roam.o wl_cfg80211.o wl_cfgp2p.o wl_cfg_btcoex.o wldev_common.o wl_linux_mon.o  \
	dhd_linux_platdev.o dhd_pno.o dhd_rtt.o dhd_linux_wq.o wl_cfg_btcoex.o \
	hnd_pktq.o hnd_pktpool.o bcmxtlv.o bcm_app_utils.o
ifneq ($(CONFIG_DHD_OF_SUPPORT),)
  DHDOFILES += dhd_custom_platdev.o
endif

ifneq ($(CONFIG_BCMDHD_PCIE),)
  DHDOFILES += dhd_pcie.o dhd_pcie_linux.o dhd_msgbuf.o dhd_flowring.o
  DHDOFILES += pcie_core.o
endif

bcmdhd-objs := $(DHDOFILES)
obj-$(DRIVER_TYPE)   += bcmdhd.o

all:
	@echo "$(MAKE) --no-print-directory -C $(KDIR) SUBDIRS=$(CURDIR) modules"
	@$(MAKE) --no-print-directory -C $(KDIR) SUBDIRS=$(CURDIR) modules

clean:
	rm -rf *.o *.ko *.mod.c *~ .*.cmd *.o.cmd .*.o.cmd \
	Module.symvers modules.order .tmp_versions modules.builtin

install:
	@$(MAKE) --no-print-directory -C $(KDIR) \
		SUBDIRS=$(CURDIR) modules_install
