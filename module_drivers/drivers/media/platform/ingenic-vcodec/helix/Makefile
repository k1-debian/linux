ingenic_helix-y :=	helix_drv.o \
			jpge.o \
			jpgd.o \
			helix_ops.o \
			api/helix_x264_enc.o \
			api/helix_jpeg_enc.o \
			api/helix_jpeg_dec.o \
			h264enc/set.o  \
			h264enc/cabac.o \
			h264enc/common.o \
			h264enc/slice.o \
			h264e_rc.o \

ingenic_helix-$(CONFIG_INGENIC_HELIX_H264E_RC_NETLINK) += h264e_rc_nl.o
obj-$(CONFIG_INGENIC_HELIX_H264E_RC_IN_KERNEL) += h264e-rc/

obj-$(CONFIG_INGENIC_HELIX) += ingenic_helix.o

