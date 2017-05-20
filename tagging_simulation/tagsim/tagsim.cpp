/*
 Madison Clark-Turner
 tagsim.cpp
 1/24/2016
 */
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <ctime>
#include <cmath>
#include <random>
#include <cstdlib>
#include <cfloat>

#include "tagsim.h"

using namespace std;

#define DEBUG 0

TagSim::TagSim(size_t numTaggers, double minSize, double maxSize, double* tagRange,
               double* obsError, double* transitionError, double* agentSpeed,
               vector<string>* actions, double reward, double penalty, double waitPenalty,
               bool contStates, bool contObs, double divisionSize = 0.1)
{
	rand();//eat the first rand to ensure diversity in initial random sampling
	_minSize = minSize;
	_maxSize = maxSize;
    
	_numTaggers = numTaggers;
	_agentPositions = new double[_numTaggers + 1];
	
	_continuousStates = contStates;
	_continuousObservations = contObs;
    
	_divisionSize = divisionSize;
	_obsError = obsError;
	_tagRange = tagRange;
	_transitionError = transitionError;
	_agentSpeed = agentSpeed;
	_actions = actions;
	_reward = reward;
	_penalty = penalty;
	_waitPenalty = waitPenalty;
	
	_macroAction = new string[_numTaggers];
    _lastAction = new string[_numTaggers];
	_macroDuration = new size_t[_numTaggers];
    
	restart();
}

TagSim::~TagSim()
{
	if(_agentPositions)
		delete[] _agentPositions;
	if(_tagRange)
		delete[] _tagRange;
	if(_obsError)
		delete[] _obsError;
	if(_transitionError)
		delete[] _transitionError;
	if(_agentSpeed)
		delete[] _agentSpeed;
	if(_actions)
		delete[] _actions;
	if(_macroAction)
		delete[] _macroAction;
    if(_lastAction)
		delete[] _lastAction;
	if(_macroDuration)
		delete[] _macroDuration;
}

void TagSim::restart()
{
	if(DEBUG)
		cerr << endl << "restart: " << endl;
	for(int i = 0; i < _numTaggers+1; i++){
		if(i < _numTaggers){
			_macroAction[i] = "";
            _lastAction[i] = "";
			_macroDuration[i] = 0;
		}
	    double f = (double)rand() / RAND_MAX;
	    _agentPositions[i] = _minSize + f*(_maxSize - _minSize);
        
		if(!_continuousStates){
			_agentPositions[i] = makeDiscrete(_agentPositions[i]);
			
		}
		if(DEBUG)
			cerr << "agent " << i << " starts at: " << _agentPositions[i] << endl;
	}
}

TagSim::TagSim(string filename)
{
	readValuesFromFile(filename);
    
	// initialize agent positions
	_agentPositions = new double[_numTaggers + 1];
    
	restart();
}

void TagSim::readValuesFromFile(string filename)
{
	ifstream ifile(filename);
    if(!ifile){
        cout << "file: " << filename << " not found." << endl;
        exit(-1);
    }
	
	string s;
	getline(ifile, s);
	while(!ifile.eof()){
		if(s.size() > 0 && s[0] != '#'){
			stringstream ss (s);
			ss >> s;
            
			if(s == "minSize"){
				ss >> _minSize;
			}
			else if(s == "maxSize"){
				ss >> _maxSize;
			}
			else if(s == "contStates"){
				ss >> _continuousStates;
			}
			else if(s == "contObs"){
				ss >> _continuousObservations;
			}
			else if(s == "divisionSize"){
				ss >> _divisionSize;
			}
			else if(s == "numTaggers"){
				ss >> _numTaggers;
				_macroAction = new string[_numTaggers];
                _lastAction = new string[_numTaggers];
				_macroDuration = new size_t[_numTaggers];
			}
			else if(s == "tagRange"){
				if(_numTaggers == 0){
					cout << "Input File format Err: numTaggers must be declared before tagRange" << endl;
					exit(-1);
				}
				_tagRange = new double[_numTaggers];
				for(int i = 0; i < _numTaggers; i++){
					ss >> _tagRange[i];
				}
			}
			else if(s == "obsError"){
				if(_numTaggers == 0){
					cout << "Input File format Err: numTaggers must be declared before obsError" << endl;
					exit(-1);
				}
				_obsError = new double[_numTaggers];
				for(int i = 0; i < _numTaggers; i++){
					ss >> _obsError[i];
				}
			}
			else if(s == "transitionError"){
				if(_numTaggers == 0){
					cout << "Input File format Err: numTaggers must be declared before transitionError" << endl;
					exit(-1);
				}
				_transitionError = new double[_numTaggers];
				for(int i = 0; i < _numTaggers; i++){
					ss >> _transitionError[i];
				}
			}
			else if(s == "agentSpeed"){
				if(_numTaggers == 0){
					cout << "Input File format Err: numTaggers must be declared before agentSpeed" << endl;
					exit(-1);
				}
				_agentSpeed = new double[_numTaggers + 1];
				for(int i = 0; i < _numTaggers + 1; i++){
					ss >> _agentSpeed[i];
				}
			}
			else if(s == "actions"){
				if(_numTaggers == 0){
					cout << "Input File format Err: numTaggers must be declared before actions" << endl;
					exit(-1);
				}
				string q;
				_actions = new vector<string>[_numTaggers];
				for(int i = 0; i < _numTaggers; i++){
					getline(ifile, s);
					stringstream ss_acts(s);
					while(!ss_acts.fail() && !ss_acts.eof()){
						ss_acts >> q;
						_actions[i].push_back(q);
					}
				}
			}
			else if(s == "reward"){
				ss >> _reward;
			}
			else if(s == "penalty"){
				ss >> _penalty;
			}
			else if(s == "wait"){
				ss >> _waitPenalty;
			}
		}
		getline(ifile, s);
	}
	//cout << "contObs: " << _continuousObservations << endl;
	ifile.close();
}

double TagSim::makeDiscrete(double val)
{
	int m = (val/_divisionSize);
	return m * _divisionSize;
}

void TagSim::performActions()
// increments the simulation a single time step
{
	bool restartSim = false;
	double dist = 0;
    
	if(DEBUG)
		cerr << "step " << _timeStep << ": " ;
	int numTags = 0;
	for(int i = 0; i < _numTaggers; i++){
        _lastAction[i] = _macroAction[i];
		if(DEBUG)
			cerr << _macroAction[i] << ", ";
		int agentPos = i+1;
		if(_macroAction[i] == "tag" && abs(_agentPositions[agentPos] - _agentPositions[0]) <= _tagRange[i]){
			numTags++;
			restartSim = true;
		}
		else if(_macroAction[i] == "mvL1" || _macroAction[i] == "mvLHit" ){
			_agentPositions[agentPos] -= _agentSpeed[agentPos];
		}
		else if(_macroAction[i] == "mvR1" || _macroAction[i] == "mvRHit" ){
			_agentPositions[agentPos] += _agentSpeed[agentPos];
		}
        
        //introduce action error
		normal_distribution<double> distribution(_agentPositions[agentPos], _transitionError[i]);
        _agentPositions[agentPos] = distribution(_generator);
        
		if(_agentPositions[agentPos] < _minSize)
			_agentPositions[agentPos] = _minSize;
		else if(_agentPositions[agentPos] > _maxSize)
			_agentPositions[agentPos] = _maxSize;
		else if(!_continuousStates)
			_agentPositions[agentPos] = makeDiscrete(_agentPositions[agentPos]);
        
		if(_macroAction[i] == "mvLHit" || _macroAction[i] == "mvRHit"){
			if(abs(_agentPositions[agentPos] - _agentPositions[0]) <= _tagRange[i]
               || _agentPositions[agentPos] == _minSize
               || _agentPositions[agentPos] == _maxSize
               ){
				_macroAction[i] = "";
			}
			else{
				_macroDuration[i]++;
			}
		}
		else{
			_macroAction[i] = "";
		}
        
		dist += _agentPositions[agentPos] - _agentPositions[0];
	}
	if(DEBUG)
		cerr << endl;
	// If Taggers are on top of agent we move randomly to the left or right
    /*if(dist == 0){
        dist = -1;
        if(rand()%2){
            dist = 1;
        }
    }*/
    if(numTags == 1)
    	_singletag++;
    else if (numTags == 2)
    	_doubletag++;
    
	// Set evader's new position
	if(restartSim){
		restart();
	}
	else{
		if( dist < 0){
			_agentPositions[0] += _agentSpeed[0];
			if(_agentPositions[0] > _maxSize){
				_agentPositions[0] = makeDiscrete(_maxSize);
			}
		}
		if (dist > 0){
			_agentPositions[0] -= _agentSpeed[0];
			if(_agentPositions[0] < _minSize){
				_agentPositions[0] = makeDiscrete(_minSize);
			}
		}
	}
    
	_timeStep++;
}

double* TagSim::Step(string* jointaction)
// pass in a jointaction and returns a joint observation in addition to advancing the state
{
	// Set Macro Actions
	for(int i = 0; i < _numTaggers; i++){
		int agentPos = i+1;
        
		if(_macroAction[i] == ""){
			_macroAction[i] = jointaction[i];
			_macroDuration[i] = 0;
		}
	}
	performActions();
	
	// Generate joint observation
	double* jointobs = new double[_numTaggers];
	for(int i = 0; i < _numTaggers; i++){
		if(_macroAction[i] == ""){
			//generate observation error
			jointobs[i] = (_agentPositions[0] - _agentPositions[i+1]);
			std::normal_distribution<double> distribution(jointobs[i], _obsError[i]);
			jointobs[i] = distribution(_generator);
			
			if(!_continuousObservations){
				if(_divisionSize >= 1)
					jointobs[i] = makeDiscrete(round(jointobs[i]));
				else
					jointobs[i] = makeDiscrete(round(jointobs[i]*2)*0.5);//Only works for division of 0.5
			}
		}
		else{
			jointobs[i] = DBL_MAX;
		}
	}

	if(DEBUG){
			cerr << "evader now at: " << _agentPositions[0] << endl;
			for(int i = 0; i < _numTaggers; i++)
				cerr << "agent " << i << " now at: " << _agentPositions[i+1] << " observes: " << jointobs[i] << endl;
		}
    
	return jointobs;
}

double TagSim::Reward(string* jointaction)
// pass in a joint action and returns an integer reward
{
	double reward = 0;
	unsigned tag = 0;

	for(int i = 0; i < _numTaggers; i++){
		if(jointaction[i] == "tag"){
			if(abs(_agentPositions[i+1] - _agentPositions[0]) <= _tagRange[i])
				reward += _reward;
			else
				reward += _penalty;
		}
		else{
			reward += _waitPenalty;
		}
	}
	if(DEBUG)
		cerr << "reward: " << reward << endl;
	
	return reward;
}

size_t TagSim::getNumObservations(size_t a)
{
	if(!_continuousObservations)
		return ((int)((abs(_maxSize)+abs(_minSize))/_divisionSize))*2+1;
	cerr << "Cannot call getNumObservations when _continuousObservations is true" << endl;
	return 0;
};

double TagSim::getObservation(size_t a, size_t i)
{
	if(!_continuousObservations){
		size_t nrO = getNumObservations(a);
		double val = 0;
		if(i < nrO/2){
			val = (nrO/2) - i;
			val *= -_divisionSize;
		}
		else if(i > nrO/2){
			val = i - (nrO/2);
			val *= _divisionSize;
		}
        
		return val;
	}
	cerr << "Cannot call getNumObservations when _continuousObservations is true" << endl;
	return 0;
};

size_t TagSim::getObservationIndex(size_t a, string v)
{
	if(!_continuousObservations){
		size_t half = (_maxSize-_minSize)/_divisionSize;
		double i = stod(v);
		if(i >= 0){
			i = (i/_divisionSize) + half;
		}
		else if (i < 0){
			i = half-(-i/_divisionSize);
		}
		return i;
	}
	cerr << "Cannot call getNumObservations when _continuousObservations is true" << endl;
	return 0;
};

size_t TagSim::getObservationIndex(size_t a, double v)
{
	if(!_continuousObservations){
		size_t half = (_maxSize-_minSize)/_divisionSize;
		double i = v;
		if(i >= 0){
			i = (i/_divisionSize) + half;
		}
		else if (i < 0){
			i = half-(-i/_divisionSize);
		}
		return i;
	}
	cerr << "Cannot call getNumObservations when _continuousObservations is true" << endl;
	return 0;
};
/*
 int main(){
 srand(time(0));
 
 TagSim ts("simDD.txt");
 
 if(DEBUG){
 string inp;
 cout << "what to do: ";
 getline(cin, inp);
 while (inp[0] != 'q'){
 stringstream ss_inp (inp);
 string* acts = new string[ts.getNumTaggers()];
 string func;
 ss_inp >> func;
 for(int i = 0; i < ts.getNumTaggers(); i++)
 ss_inp >> acts[i];
 if(inp[0] == 's'){
 double* obs = ts.Step(acts);
 for(int i = 0; i < ts.getNumTaggers(); i++){
 cout << "agent " << i+1 << " recieves observation: " << obs[i] << endl;
 }
 delete[] obs;
 }
 else if(inp[0] == 'r'){
 cout << "reward is " << ts.Reward(acts) << endl;
 }
 delete[] acts;
 for(int i = 0; i < ts.getNumTaggers() + 1; i++)
 cout << "agent " << i << " is at " << ts.getCurrentState()[i] << endl;
 cout << "what to do: ";
 getline(cin, inp);
 }
 }
 
 return 0;
 }*/
