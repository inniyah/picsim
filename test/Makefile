CC= gcc

FLAGS= -Wall -ggdb

all: picsim_simple.c
	$(CC) $(FLAGS) picsim_simple.c -o picsim_simple -I../include/ -L../src/ -lpicsim

clean:
	rm -rf picsim_simple

run:
	LD_LIBRARY_PATH='../src/' ./picsim_simple P16F628A ../examples/p16f648a/shift.hex
