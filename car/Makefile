CP_PATH=$(abspath ../..)/CrossCompiler
CROSS_COMPILE=$(CP_PATH)/gcc-linaro-arm-linux-gnueabihf/bin/arm-linux-gnueabihf-
ROOTFS=$(CP_PATH)/sysroots/cortexa15t2hf-vfp-neon-linux-gnueabi


CC=$(CROSS_COMPILE)gcc
CXX=$(CROSS_COMPILE)g++

INC += -I protocol
INC += -I$(ROOTFS)/include
INC += -I$(ROOTFS)/usr/include
INC += -I$(ROOTFS)/usr/include/omap
INC += -I$(ROOTFS)/usr/include/libdrm
INC += -I$(ROOTFS)/usr/include/gbm
LIBDIR := $(ROOTFS)/usr/lib

CFLAGS := -O1 -g -Wall -fPIC -mfloat-abi=hard -mfpu=neon -Wl,-rpath,$(ROOTFS)/lib -Wl,-rpath,$(ROOTFS)/usr/lib $(INC)
CXXFLAGS = -Wall -ansi -g -fPIC -mfloat-abi=hard -mfpu=neon $(INC) -I$(ROOTFS)/include/c++/4.7.3/
LDFLAGS = -lm -lpthread -L$(LIBDIR) -lrt -ljpeg -ldrm -lmtdev -ldrm_omap -lstdc++ -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_objdetect

all : main
	
main : stop_when_accident.o car_lib.o
	$(CC) $(CFLAGS) -o cv car_lib.o stop_when_accident.o $(LDFLAGS) 
	#`pkg-config opencv --libs`

stop_when_accident.o: stop_when_accident.cpp
	$(CXX) -c $(CXXFLAGS) stop_when_accident.cpp 
	#`pkg-config opencv --cflags --libs`

car_lib.o: car_lib.c
	$(CC) -c $(CFLAGS) car_lib.c 


clean:
	rm cv
	rm *.o
