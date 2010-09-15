#! /bin/make

MKDIR= mkdir -p

.PHONY: all clean

all: liquid data images

liquid: liquid.c
	$(CC) -g -O2 -o liquid liquid.c libode.a libdrawstuff.a -lGLU -lGL -lSM -lICE -lm -lpthread -I.

data:
	$(MKDIR) data

images:
	$(MKDIR) images

clean:
	$(RM) liquid.o frame.pov

