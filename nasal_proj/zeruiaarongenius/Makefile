CC = gcc
XX = g++
CFLAGS = -Wall -O -g
SHM = msg
# SENSOR = sensor
TARGET1 = ./main/rcore
TARGET2 = ./hmi/user

%.o:%.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o:%.cpp
	$(XX) $(CFLAGS) -c $< -o $@

SOURCES1 = $(wildcard $(SHM)/*.c $(SHM)/*.cpp main/*.cpp)

SOURCES2 = $(wildcard $(SHM)/*.c $(SHM)/*.cpp hmi/*.cpp)

OBJS1 = $(patsubst %.c,%.o,$(patsubst %.cpp,%.o,$(SOURCES1)))

OBJS2 = $(patsubst %.c,%.o,$(patsubst %.cpp,%.o,$(SOURCES2)))

$(TARGET1) $(TARGER2):$(OBJS1) $(OBJS2)
	$(XX) $(OBJS1) -lGalil -o $(TARGET1)
	chmod a+x $(TARGET1)
	$(XX) $(OBJS2) -lpthread -o $(TARGET2)
	chmod a+x $(TARGET2)
	@echo ---------------- success -------------------

clean:
	rm -rf $(SHM)/*.o main/*.o hmi/*.o $(TARGET1) $(TARGET2)
	rm -rf $(SHM)/*~  main/*~ hmi/*~ ./*~
	rm -rf $(SHM)/._*  main/._* hmi/._* ./._*
	rm -rf $(SHM)/.DS_* main/.DS_* hmi/.DS_* ./.DS_*
