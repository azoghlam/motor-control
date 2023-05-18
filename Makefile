main:
	gcc motor-control.cpp -lwiringPi -lstdc++ -pthread -o exec
	./exec
