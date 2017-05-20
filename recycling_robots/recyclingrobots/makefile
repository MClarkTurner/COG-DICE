GCC=g++
FLAGS=-std=c++11

all: dpomdp.o recyclingrobots.o

dpomdp.o: dpomdp.cpp dpomdp.h
	$(GCC) $(FLAGS) -c dpomdp.cpp

recyclingrobots.o: recyclingrobots.cpp recyclingrobots.h dpomdp.h
	$(GCC) $(FLAGS) -c recyclingrobots.cpp

recyclingrobotsdiscrete.o: recyclingrobotsdiscrete.h recyclingrobots.h dpomdp.h dpomdp.o
	$(GCC) $(FLAGS) -c recyclingrobotsdiscrete.cpp

clean:
	rm *.o
