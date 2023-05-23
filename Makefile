main:
	gcc main.cpp -lwiringPi -lstdc++ -pthread -lm -o exec
	./exec
