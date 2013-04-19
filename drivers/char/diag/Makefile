obj-$(CONFIG_DIAG_CHAR) := diagchar.o
obj-$(CONFIG_DIAG_SDIO_PIPE) += diagfwd_sdio.o
obj-$(CONFIG_DIAG_HSIC_PIPE) += diagfwd_hsic.o
diagchar-objs := diagchar_core.o diagchar_hdlc.o diagfwd.o diagmem.o diagfwd_cntl.o

# FUJITSU:2012-04-20 DN DIAG_USB_LOG_en_001 start
#ifeq ($(TARGET_BUILD_VARIANT),eng)
#ccflags-y += -DDIAG_DEBUG
#endif
# FUJITSU:2012-04-20 DN DIAG_USB_LOG_en_001 end
