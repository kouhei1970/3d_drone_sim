dronesim: main.o motor.o propeller.o quadcopter.o rigidbody.o solver.o
	g++ -o dronsim main.o motor.o propeller.o quadcopter.o rigidbody.o solver.o
main.o:    main.cpp
	g++ -c main.cpp
motor.o:     motor.cpp
	g++ -c motor.cpp
propeller.o: propeller.cpp
	g++ -c propeller.cpp
quadcopter.o: quadcopter.cpp
	g++ -c quadcopter.cpp
rigidbody.o: rigidbody.cpp
	g++ -c rigidbody.cpp
solver.o: solver.cpp
	g++	-c solver.cpp

clean:
	rm -f *.o dronesim