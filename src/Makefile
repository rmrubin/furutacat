
all: main.c furutacat.c furutacat.h text_defines.h
	gcc -o ../bin/furutacat main.c furutacat.c -I. -lwiringPi -lm

debug: main.c furutacat.c furutacat.h text_defines.h
	gcc -g -o ../bin/furutacat main.c furutacat.c -I. -lwiringPi -lm

install:
	cp -f ../bin/furutacat ~/bin
	cp -f ../sh/* ~/bin
	cp -f ../gnuplot/.furutacat_gnuplot ~/

