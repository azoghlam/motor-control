main:
	gcc main.cpp -lwiringPi -lstdc++ -pthread -lm  -o exec
	./exec

t1:
	gcc felipe.cpp -lwiringPi -lstdc++ -pthread -lm -o exec
	./exec