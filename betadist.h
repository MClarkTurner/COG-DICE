#ifndef DISTRIBUTION_H
#define DISTRIBUTION_H

#include <boost/math/distributions/beta.hpp>
#include <vector>
#include <iostream>

using namespace boost::math;
using namespace std;

typedef struct{
	double alpha;
	double beta;
} DivisionData;

DivisionData MLE(vector<double> v, unsigned noise);
double sampleBeta(double alpha, double beta);
double genValue(DivisionData dd);


#endif