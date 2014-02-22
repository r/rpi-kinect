obj-m += rpi-kinect.o

all: 
	make ARCH=arm CROSS_COMPILE=${CCPREFIX} -C ${KERNEL_SRC} M=$(PWD) modules
	${CCPREFIX}gcc -o rpi-user rpi-user.c

clean: 
	make -C ${KERNEL_SRC} M=$(PWD) clean

