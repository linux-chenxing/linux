# SPDX-License-Identifier: GPL-2.0
#
# for USB OTG silicon based on Mentor Graphics INVENTRA designs
#

# define_trace.h needs to know how to find our header
CFLAGS_musb_trace.o	:= -I$(src)

obj-$(CONFIG_USB_MUSB_HDRC) += musb_hdrc.o

musb_hdrc-y := musb_core.o musb_trace.o

musb_hdrc-$(CONFIG_USB_MUSB_HOST)$(CONFIG_USB_MUSB_DUAL_ROLE) += musb_virthub.o musb_host.o
musb_hdrc-$(CONFIG_USB_MUSB_GADGET)$(CONFIG_USB_MUSB_DUAL_ROLE) += musb_gadget_ep0.o musb_gadget.o
musb_hdrc-$(CONFIG_DEBUG_FS)			+= musb_debugfs.o

# Hardware Glue Layer
obj-$(CONFIG_USB_MUSB_OMAP2PLUS)		+= omap2430.o
obj-$(CONFIG_USB_MUSB_DSPS)			+= musb_dsps.o
obj-$(CONFIG_USB_MUSB_TUSB6010)			+= tusb6010.o
obj-$(CONFIG_USB_MUSB_DA8XX)			+= da8xx.o
obj-$(CONFIG_USB_MUSB_UX500)			+= ux500.o
obj-$(CONFIG_USB_MUSB_JZ4740)			+= jz4740.o
obj-$(CONFIG_USB_MUSB_SUNXI)			+= sunxi.o
obj-$(CONFIG_USB_MUSB_MEDIATEK)      		+= mediatek.o
obj-$(CONFIG_USB_MUSB_POLARFIRE_SOC)		+= mpfs.o
obj-$(CONFIG_USB_MUSB_MSTAR)                    += mstar.o

# the kconfig must guarantee that only one of the
# possible I/O schemes will be enabled at a time ...
# PIO only, or DMA (several potential schemes).
# though PIO is always there to back up DMA, and for ep0

musb_hdrc-$(CONFIG_USB_INVENTRA_DMA)		+= musbhsdma.o
musb_hdrc-$(CONFIG_USB_TUSB_OMAP_DMA)		+= tusb6010_omap.o
musb_hdrc-$(CONFIG_USB_UX500_DMA)		+= ux500_dma.o
musb_hdrc-$(CONFIG_USB_TI_CPPI41_DMA)		+= musb_cppi41.o
