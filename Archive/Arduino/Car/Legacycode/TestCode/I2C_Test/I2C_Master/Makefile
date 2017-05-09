SRCFILES:=$(shell find ./src -name "*.c")
OBJFILES:=$(SRCFILES:.c=.o)

CFLAGS:=-Wall -std=c99 -I../I2C_Slave
LDFLAGS:=-lwiringPi

PROJECT:=I2C_Master

.PHONY: all clean

all: $(PROJECT)

clean:
	rm -f $(PROJECT) $(OBJFILES)

$(PROJECT): $(OBJFILES)
	gcc -o $(PROJECT) $(OBJFILES) $(LDFLAGS)
