PROJECT_DIR = $(CURDIR)
FREERTOS_SDK_DIR = $(CURDIR)/../../../..

USER_CSRC := main.c
USER_CSRC += $(wildcard  src/*.c)
USER_CSRC += $(wildcard  ../common/*.c)
			
USER_ASRC := 
USER_CXXSRC := 

USER_INCLUDE := $(PROJECT_DIR)	\
				$(PROJECT_DIR)/inc \


include $(FREERTOS_SDK_DIR)/tools/makeall.mk

# 用户定义的编译目标文件上传路径 
ifeq ($(OS),Windows_NT)
	USR_BOOT_DIR		?= $(subst \,/, $(PHYTIUM_DEV_PATH))/tftp
else
	USR_BOOT_DIR		?= /home/wang
endif

# 设置启动镜像名
USER_BOOT_IMAGE    ?= openamp_core0

# 完成编译
image:
	$(MAKE) clean
	$(MAKE) all -j
	@cp ./$(IMAGE_OUT_NAME).elf $(USR_BOOT_DIR)/$(USER_BOOT_IMAGE).elf
ifdef CONFIG_OUTPUT_BINARY
	@cp ./$(IMAGE_OUT_NAME).bin $(USR_BOOT_DIR)/$(USER_BOOT_IMAGE).bin
endif
	@ls $(USR_BOOT_DIR)/$(USER_BOOT_IMAGE).* -l


#	$(MAKE) menuconfig
