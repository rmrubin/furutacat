
all: main.c furutacat.c furutacat.h text_defines.h
	gcc -o ../bin/furutacat main.c furutacat.c -I. -lwiringPi

debug: main.c furutacat.c furutacat.h text_defines.h
	gcc -g -o ../bin/furutacat main.c furutacat.c -I. -lwiringPi

install:
	cp -f ../bin/furutacat ~/bin
	cp -f ../sh/* ~/bin

