# Makefile by giann
ARCH 	:=arm
COMPILER = /usr/local/arm-linux-gm/bin/arm-linux-
KDIR = /home/giann/Data_Ubuntu/hubble_working/camsdk-gm/hubble-fw-sdk-v2/PlatformBSP/camsdk_gm/kernel/normal_build
PWD := $(shell pwd)

all:
	make -C ${KDIR} M=${PWD} ARCH=${ARCH} CROSS_COMPILE=${COMPILER} modules
clean:
	make -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(COMPILER) clean
