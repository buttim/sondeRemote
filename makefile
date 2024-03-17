SDCCOPTS ?= --iram-size 128 --xram-size 768

SRC=main.c delay.c common.c spi.c iap.c si443x.c
#SRC = $(wildcard *.c)
OBJ=$(patsubst %.c,build/%.rel, $(SRC))

build/%.rel: %.c
	mkdir -p $(dir $@)
	sdcc $(SDCCOPTS) -o build/ -c $<

all: main

main: $(OBJ)
	sdcc -o build/ $(SDCCOPTS) $^
	cp build/$@.ihx build/$@.hex
	hex2bin build/$@.hex > /dev/nul

clean:
	rm -f build/*
