main:
	gcc main.cpp -lwiringPi -lstdc++ -pthread -lm -y i2c-tools -o exec
	./exec

t1:
	gcc felipe.cpp -lwiringPi -lstdc++ -pthread -lm -o exec
	./exec