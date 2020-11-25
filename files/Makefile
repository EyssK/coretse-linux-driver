obj-m := coretse.o
coretse-objs :=	altera_msgdma.o altera_sgdma.o altera_tse_ethtool.o altera_tse_main.o altera_utils.o coretse_dma.o

MY_CFLAGS += -g -DDEBUG
ccflags-y += ${MY_CFLAGS}
CC += ${MY_CFLAGS}

SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC)
	EXTRA_CFLAGS="$(MY_CFLAGS)"
	
modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
