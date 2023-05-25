main:
	gcc main.cpp -lwiringPi -lstdc++ -pthread -lm -o exec
	./exec

felipe:
	gcc felipe.cpp -lwiringPi -lstdc++ -pthread -lm -o exec
	./exec