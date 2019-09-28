CP_PATH=$(abspath ../../..)/CrossCompiler
CROSS_COMPILE=$(CP_PATH)/gcc-linaro-arm-linux-gnueabihf/bin/arm-linux-gnueabihf-

ARCH=arm
CC=$(CROSS_COMPILE)gcc
OBJ = avoid.c car_lib.c

CFLAG := -W -Wall

OBJNAME = avoid 
$(OBJNAME): $(OBJ)
	$(CC) -o $(OBJNAME) $(OBJ)

clean:
	rm -rf *.o $(OBJNAME)




