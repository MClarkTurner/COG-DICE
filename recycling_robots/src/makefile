FLAGS=-std=c++11
TAGSIM_O=../recyclingrobots/recyclingrobotsdiscrete.o
DPOMDP_O=../recyclingrobots/dpomdp.o

all: runcog

betadist.o: betadist.cpp betadist.h
	g++ $(FLAGS) -c betadist.cpp

cgdice.o: cgdice.cpp cgdice.h jointpolicygraphcont.h
	g++ $(FLAGS) -c cgdice.cpp

runcog: runcgdice.cpp cgdice.o betadist.o
	g++ $(FLAGS) -o $@ runcgdice.cpp cgdice.o betadist.o $(TAGSIM_O) $(DPOMDP_O)

testpolicy: testpolicy.cpp cgdice.o betadist.o
	g++ $(FLAGS) -o $@ testpolicy.cpp cgdice.o betadist.o $(TAGSIM_O) $(DPOMDP_O)

clean:
	rm runcog *.o
