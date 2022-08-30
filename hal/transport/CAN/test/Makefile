
SHELL = /bin/sh

CC = gcc
OBJS = test.o
# FAST_FLAG = -DMY_CAN_FAST_SLOT_ACCESS
CFLAGS = -Wall -g ${FAST_FLAG}
INCLUDES = -I.
LIBS = -lm

run: test
	@echo "\n"
	@./test
	@echo "\n"

test: cp ${OBJS}
	${CC} ${CFLAGS} ${INCLUDES} -o $@ ${OBJS} ${LIBS}

clean:
	-rm -f MyTransportCAN.h
	-rm -f MyTransportCAN.cpp
	-rm -f *.o test

.cpp.o:
	${CC} ${CFLAGS} ${INCLUDES} -c $<

cp:
	-cp ../MyTransportCAN.h .
	-cp ../MyTransportCAN.cpp .
	-sync -f ./MyTransportCAN.h ./MyTransportCAN.cpp