include $(srctree)/drivers/misc/mediatek/Makefile.custom

#Top level common code
obj-$(CONFIG_MTK_RTC) += mtk_rtc_common.o mtk_rtc_hal_common.o

#IP dependent part
obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/
