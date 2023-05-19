main:
	gcc magnet.cpp -lwiringPi -lstdc++ -pthread -o exec
	./exec
