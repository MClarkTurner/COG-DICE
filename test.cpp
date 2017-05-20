#include <iostream>
#include "../recyclingrobots/recyclingrobots.h"
using namespace std;

int main () {
  RecyclingRobots* rr = new RecyclingRobots("../recyclingrobots/simRR.txt");
  rr->reset();
}
