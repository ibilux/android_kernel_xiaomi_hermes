SUPPORT_MT6630 := no

SUPPORT_MT6630 := $(if $(filter "MT6630", $(CONFIG_MTK_COMBO_CHIP)),yes,no)

ifeq ($(SUPPORT_MT6630), yes)
    obj-$(CONFIG_MTK_COMBO_WIFI) += mt6630/
    $(warning include mt6630)
endif
