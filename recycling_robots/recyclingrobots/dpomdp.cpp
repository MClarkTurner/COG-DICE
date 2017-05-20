/*
 Madison Clark-Turner
 dpomdp.cpp
 7/25/2016
 */

#include "dpomdp.h"

using namespace std;

DPOMDP::DPOMDP(size_t numAgents, size_t dimension, double* minValues, 
	double* maxValues, double* obsError, double* transitionError, 
	vector<string>* actions)
{
	cerr << "DPOMDP constructor" << endl;
	
	rand();//eat the first rand to ensure diversity in initial random sampling
	_dimension = dimension;

	_minValues = minValues;
	_maxValues = maxValues;
    
	_numAgents = numAgents;
	_curState = new vector<double>[_numAgents];
	
	_obsError = obsError;
	_transitionError = transitionError;
	_actions = actions;
	
	_macroAction = new string[_numAgents];
    _lastAction = new string[_numAgents];
	_macroDuration = new size_t[_numAgents];
    
	//restart();
}

DPOMDP::~DPOMDP()
{
	if(_minValues)
		delete[] _minValues;
	if(_maxValues)
		delete[] _maxValues;
	if(_curState)
		delete[] _curState;
	if(_obsError)
		delete[] _obsError;
	if(_transitionError)
		delete[] _transitionError;
	if(_actions)
		delete[] _actions;
	if(_macroAction)
		delete[] _macroAction;
    if(_lastAction)
		delete[] _lastAction;
	if(_macroDuration)
		delete[] _macroDuration;
}

void DPOMDP::restart()
// The state the simulation should be in at its start
{
	cerr << "DPOMDP restart called" << endl;
}

DPOMDP::DPOMDP(string filename)
{
	cerr << "DPOMDP string constructor called" << endl;
	readValuesFromFile(filename);
    
	// initialize agent positions
	_curState = new vector<double>[_numAgents + 1];
    
	restart();
}

void DPOMDP::readValuesFromFile(string filename)
{
	cerr << "DPOMDP readValuesFromFile called" << endl;
	/*ifstream ifile(filename);
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
            
            if(s == "dimension"){
				ss >> _dimension;
			}
			if(s == "minValue"){
				if(_dimension == 0){
					cout << "Input File format Err: dimension must be declared before tagRange" << endl;
					exit(-1);
				}
				_minValues = new double[_dimension];
				for(int i = 0; i < _dimension; i++){
					ss >> _minValues[i];
				}
			}
			else if(s == "maxValue"){
				if(_dimension == 0){
					cout << "Input File format Err: dimension must be declared before tagRange" << endl;
					exit(-1);
				}
				_maxValues = new double[_dimension];
				for(int i = 0; i < _dimension; i++){
					ss >> _maxValues[i];
				}
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
			else if(s == "numAgents"){
				ss >> _numAgents;
				_macroAction = new string[_numAgents];
                _lastAction = new string[_numAgents];
				_macroDuration = new size_t[_numAgents];
			}
			else if(s == "tagRange"){
				if(_numAgents == 0){
					cout << "Input File format Err: numAgents must be declared before tagRange" << endl;
					exit(-1);
				}
				_tagRange = new double[_numAgents];
				for(int i = 0; i < _numAgents; i++){
					ss >> _tagRange[i];
				}
			}
			else if(s == "obsError"){
				if(_numAgents == 0){
					cout << "Input File format Err: numAgents must be declared before obsError" << endl;
					exit(-1);
				}
				_obsError = new double[_numAgents];
				for(int i = 0; i < _numAgents; i++){
					ss >> _obsError[i];
				}
			}
			else if(s == "transitionError"){
				if(_numAgents == 0){
					cout << "Input File format Err: numAgents must be declared before transitionError" << endl;
					exit(-1);
				}
				_transitionError = new double[_numAgents];
				for(int i = 0; i < _numAgents; i++){
					ss >> _transitionError[i];
				}
			}
			else if(s == "agentSpeed"){
				if(_numAgents == 0){
					cout << "Input File format Err: numAgents must be declared before agentSpeed" << endl;
					exit(-1);
				}
				_agentSpeed = new double[_numAgents + 1];
				for(int i = 0; i < _numAgents + 1; i++){
					ss >> _agentSpeed[i];
				}
			}
			else if(s == "actions"){
				if(_numAgents == 0){
					cout << "Input File format Err: numAgents must be declared before actions" << endl;
					exit(-1);
				}
				string q;
				_actions = new vector<string>[_numAgents];
				for(int i = 0; i < _numAgents; i++){
					getline(ifile, s);
					stringstream ss_acts(s);
					while(!ss_acts.fail() && !ss_acts.eof()){
						ss_acts >> q;
						_actions[i].push_back(q);
					}
				}
			}
			else if(s == "rewards"){
				size_t numRewards;
				ss >> numRewards;

				getline(ifile, s);
				stringstream ss2 (s);
				ss2 >> s;
				
				double val;
				for(int i = 0; i < numRewards; i++){
					ss >> val;
					_rewards.push_back(val);
				}
			}
		}
		getline(ifile, s);
	}
	//cout << "contObs: " << _continuousObservations << endl;
	ifile.close();*/
}

void DPOMDP::performActions()
// increments the simulation a single time step
{
	cerr << "DPOMDP PerformActions called" << endl;
}

vector<double>* DPOMDP::Step(string* jointaction)
// pass in a jointaction and returns a joint observation in addition to advancing the state
{
	cerr << "DPOMDP Step called" << endl;
	// Set Macro Actions
	for(int i = 0; i < _numAgents; i++){
		int agentPos = i+1;
        
		if(_macroAction[i] == ""){
			_macroAction[i] = jointaction[i];
			_macroDuration[i] = 0;
		}
	}
	performActions();
	
	// Generate joint observation
	vector<double>* jointobs = new vector<double>[_numAgents];
	for(int i = 0; i < _numAgents; i++){
		for(int d = 0; d < _dimension; d++){
			jointobs[i].push_back(DBL_MAX);
		}

		if(_macroAction[i] == ""){
			for(int d = 0; d < _dimension; d++){
				jointobs[i][d] = (_curState[0][d] - _curState[i+1][d]);

				//generate observation error
				std::normal_distribution<double> distribution(jointobs[i][d], _obsError[i]);
				jointobs[i][d] = distribution(_generator);
			}
		}
	}
    
	return jointobs;
}

double DPOMDP::Reward(string* jointaction)
// pass in a joint action and returns an integer reward
{
	cerr << "DPOMDP Reward called" << endl;
	return 0;
}