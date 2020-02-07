include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifneq ($(CONFIG_ARCH_MTK_PROJECT),)
ARCH_MTK_PROJECT := $(shell echo $(CONFIG_ARCH_MTK_PROJECT))
obj-y   += core/
obj-y   += dct/dct/
endif

ifeq ($(CONFIG_MTK_SENSOR_SUPPORT),y)
obj-$(CONFIG_MTK_BMI160_ACC) += accelerometer/bmi160_acc/
obj-$(CONFIG_MTK_LTR559) += alsps/ltr559/
obj-$(CONFIG_MTK_STK3X1X) += alsps/stk3x1x/
obj-$(CONFIG_MTK_BMI160_GYRO) += gyroscope/bmi160_gyro/
ifeq (,$(filter y,$(CONFIG_MTK_AKM09911)$(CONFIG_MTK_YAS537)))
obj-y += magnetometer/common/
endif
endif

obj-y   +=  accdet/
obj-y   +=  camera/camera/
obj-y   +=  keypad/
obj-y   +=  leds/
obj-y   +=  sound/
obj-y   +=  vibrator/
