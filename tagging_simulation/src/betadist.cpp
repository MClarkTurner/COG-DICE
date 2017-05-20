#include "betadist.h"
#include <algorithm>
using namespace std;

DivisionData MLE(vector<double> v, unsigned noise){
	//Returns the alpha-beta pair that matches the distribution
	/*for(int i = 0; i < noise; i++){
		v.push_back(sampleBeta(1,1));
	}*/

	double v1 = 0;
	double v2 = 0;
	for(int i = 0; i < v.size(); i++){
		v1 += log(v[i]);
		v2 += log(1-v[i]);
	}
	v1/=v.size();
	v2/=v.size();
	v1 = 1 - exp(v1);
	v2 = 1 - exp(v2);
	double sumv = v1 + v2;
	double v1hat = v1 * (0.5/(sumv-1));
	double v2hat = v2 * (0.5/(sumv-1));

	DivisionData dd;
	dd.alpha = v2hat;
	dd.beta = v1hat;
	/*double lowVal = 2;

	if(dd.alpha < lowVal)
		dd.alpha = lowVal;
	if(dd.beta < lowVal)
		dd.beta = lowVal;*/
	return dd;
}
/*
DivisionData MLE2(vector<double> v){
	//Returns the alpha-beta pair that matches the distribution
	double v1 = 1;
	double v2 = 1;
	for(int i = 0; i < v.size(); i++){
		//cout << "T: " <<  1-v[i] << ' ' << 1.0/v.size() << ' '<< pow(1-v[i],1.0/v.size()) << endl;
		v1 *= pow(1-v[i],1.0/v.size());
		v2 *= pow(v[i],1.0/v.size());
	}
	cout << "v1: " << v1 << " v2: " << v2 << endl;

	double sumv = 1 - v1 - v2;
	double v1hat = 0.5 * (1-v1) / sumv;
	double v2hat = 0.5 * (1-v2) / sumv;

	DivisionData dd;
	dd.alpha = v1hat;
	dd.beta = v2hat;
	return dd;
}

DivisionData MLE3(vector<double> v){
	//Returns the alpha-beta pair that matches the distribution
	double s1 = 0;
	double s2 = 0;
	double xbar = 0;
	for(int i = 0; i < v.size(); i++){
		s1 += log(v[i]);
		s2 += log(1-v[i]);
		xbar += v[i];
	}

	xbar /= v.size();
	double xvar = 0;
	for(int i = 0; i < v.size(); i++){
		xvar += pow(v[i]-xbar, 2);
	}
	xvar /= (v.size()-1);

	beta_distribution<> dist(1, 1);
	double alpha = dist.find_alpha(xbar, xvar);
	double beta = dist.find_beta(xbar, xvar);

	cout << "a: " << alpha << " b: " << beta << endl;
	DivisionData dd;
	return dd;
}

DivisionData MLE(vector<double> v){
	//Returns the alpha-beta pair that matches the distribution
	double xbar = 0;
	for(int i = 0; i < v.size(); i++){
		xbar += v[i];
	}

	xbar /= v.size();
	double xvar = 0;
	for(int i = 0; i < v.size(); i++){
		xvar += pow(v[i]-xbar, 2);
	}
	xvar /= (v.size()-1);

	beta_distribution<> dist(1, 1);

	DivisionData dd;
	dd.alpha = dist.find_alpha(xbar, xvar);
	if(dd.alpha < 1.1)
		dd.alpha = 1.1;
	dd.beta = dist.find_beta(xbar, xvar);
	if(dd.beta < 1.1)
		dd.beta = 1.1;
	return dd;
}*/

double sampleBeta(double alpha, double beta){
    double randFromUnif = ((double) rand() / (RAND_MAX));
    beta_distribution<> dist(alpha, beta);
   	return quantile(dist, randFromUnif);
}

double genValue(DivisionData dd){
	return sampleBeta(dd.alpha, dd.beta);
}

struct item{
	double val;
	double dist;
};

bool myfunc(item a , item b){
	return (a.dist < b.dist);
}
/*
int main(int argc, char** argv){
	srand(time(0));
	
	vector<item> v;
	for(int k = 0; k < 100; k++){
		item q;
		q.val = sampleBeta(atoi(argv[1]), atoi(argv[2]));
		q.dist = abs(0.45-q.val);
		v.push_back(q);
	}
	sort(v.begin(), v.end(), myfunc);

	vector<double> v2;
	for(int i =0; i < 5; i++){
		v2.push_back(v[i].val);
		cout << v[i].val << ":" << v[i].dist << ", ";
	}
	cout << endl;
	DivisionData dd = MLE(v2);
	cout << dd.alpha << ' ' << dd.beta << endl;
	//dd = MLE3(v);*/
	/*srand(time(0));
	
	vector<double> v;
	for(int k = 0; k < 1000; k++){
		v.push_back(sampleBeta(atoi(argv[1]), atoi(argv[2])));
	}
	DivisionData dd = MLE(v);
	cout << dd.alpha << ' ' << dd.beta << endl;
	dd = MLE3(v);*/
//}
