FLAGS=-std=c++11
TAGSIM_O=../tagsim/tagsim.o

all: runcgdice

betadist.o: betadist.cpp betadist.h
	g++ $(FLAGS) -c betadist.cpp

cgdice.o: cgdice.cpp cgdice.h jointpolicygraphcont.h
	g++ $(FLAGS) -c cgdice.cpp

runcgdice: runcgdice.cpp cgdice.o betadist.o
	g++ $(FLAGS) -o $@ runcgdice.cpp cgdice.o betadist.o $(TAGSIM_O)

clean:
	rm runcgdice *.o
