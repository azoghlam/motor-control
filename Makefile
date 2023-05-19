main:
	gcc magnet.cpp -lwiringPi -lstdc++ -pthread -lm -o exec
	./exec
