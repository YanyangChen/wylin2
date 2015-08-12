#Dual Purpose Makefile to support kernel module builds
ifneq ($(KERNELRELEASE),)
obj-m       := galilpci.o

DRIVER_VER="1.6.3-540"
OS_FLAG=__OS_UBUNTU
CFLAGS_MODULE := -D $(OS_FLAG) -D __DRIVER_VER=\"$(DRIVER_VER)\"

else
KDIR        := /lib/modules/$(shell uname -r)/build
KDEST       := /lib/modules/$(shell uname -r)
PWD         := $(shell pwd)

default:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) clean

install:
	install -d $(DESTDIR)/$(KDEST)
	install galilpci.ko $(DESTDIR)/$(KDEST)

endif
