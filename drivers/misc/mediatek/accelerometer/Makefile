include $(srctree)/drivers/misc/mediatek/Makefile.custom

# In case the platform does NOT support this type of sensors

obj-y   +=  accel.o accel_factory.o

obj-$(CONFIG_MTK_BMI160_ACC) += bmi160_acc/

obj-y   +=  bmi160_stc/

obj-$(CONFIG_MTK_LSM6DS3_ACC)   +=  lsm6ds3/
