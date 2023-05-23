main:
	gcc magnet.cpp -lwiringPi -lstdc++ -pthread -lm -o exec
	./exec

magnet:
	gcc mag.cpp -lwiringPi -lstdc++ -pthread -lm -o exec
	./exec