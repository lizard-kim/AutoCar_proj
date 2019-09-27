CP_PATH=$(abspath ../..)/CrossCompiler
CROSS_COMPILE=$(CP_PATH)/gcc-linaro-arm-linux-gnueabihf/bin/arm-linux-gnueabihf-

ARCH=arm
CC=$(CROSS_COMPILE)gcc
OBJ = CameraServo.o car_lib.o

OBJNAME = CameraServo
$(OBJNAME): $(OBJ)
	$(CC) -lm -o $(OBJNAME) $(OBJ)

CameraServo.o: CameraServo.c
	$(CC) -c CameraServo.c

car_lib.o: car_lib.c
	$(CC) -c car_lib.c

clean:
	rm -rf *.o $(OBJNAME)

