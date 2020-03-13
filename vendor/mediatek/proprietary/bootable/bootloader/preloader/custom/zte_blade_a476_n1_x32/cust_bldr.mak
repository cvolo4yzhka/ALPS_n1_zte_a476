###################################################################
# Include Project Feautre  (cust_bldr.h)
###################################################################
#ifeq ("$(MTK_EMMC_SUPPORT)","yes")
ifdef MTK_EMMC_SUPPORT
CFG_BOOT_DEV :=BOOTDEV_SDMMC
else
CFG_BOOT_DEV :=BOOTDEV_NAND
endif
CFG_UART_LOG :=UART1
CFG_UART_META :=UART1
CFG_USB_UART_SWITCH := 0
CFG_TEE_SUPPORT = 0
CFG_TRUSTONIC_TEE_SUPPORT = 0
CFG_MICROTRUST_TEE_SUPPORT = 0
CFG_GOOGLE_TRUSTY_SUPPORT=0
