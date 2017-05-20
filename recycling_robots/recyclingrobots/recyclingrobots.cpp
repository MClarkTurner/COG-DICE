#include "recyclingrobots.h"

void RecyclingRobots::readValuesFromFile(string filename)
{
	//cerr << "RR readfromvalues" << endl;
	ifstream ifile(filename);
    if(!ifile){
        cout << "file: " << filename << " not found." << endl;
        exit(-1);
    }

    _dimension = 1;
	
	string s;
	getline(ifile, s);
	while(!ifile.eof()){
		if(s.size() > 0 && s[0] != '#'){
			stringstream ss (s);
			ss >> s;
            
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
			else if(s == "numAgents"){
				ss >> _numAgents;
				_macroAction = new string[_numAgents];
                _lastAction = new string[_numAgents];
				_macroDuration = new size_t[_numAgents];
				_curState = new vector<double>[_numAgents];
				//cerr << "curState :" << &_curState << endl;
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
	ifile.close();
}

void RecyclingRobots::restart()
{
	//cerr << "RR restart called" << endl;
	for(int i = 0; i < _numAgents; i++){
		_curState[i].clear();
		_curState[i].push_back(_maxValues[0]);
	}
}

double smDist(double startingVal){
	return 20 + startingVal/10;
	//return 20 + startingVal/10;//pow(startingVal,3)/100000;
}

double bgDist(double startingVal){
	return 30 + startingVal/5;
	//return 30 + pow(startingVal,10)/(5*pow(10,18));
}

void RecyclingRobots::performActions()
{   
	//cerr << "RR performActions" << endl;
	if(DEBUG)
		cerr << "step " << _timeStep << ": " ;

	//normal_distribution<double> distribution(_curState[i][0], _transitionError[i]);
    //    _curState[i][0] = distribution(_generator);

	for(int i = 0; i < _numAgents; i++){
        _lastAction[i] = _macroAction[i];
		if(DEBUG)
			cerr << _macroAction[i] << ", ";

		double startingVal = _curState[i][0];

		/*
		-at high
			big = 50% chance of decrease
			sml = 30% chance of decrease
		-at low
			big = 30% chance of decrease
			sml = 20% chance of decrease
		*/

		int decreaseBattery = rand()%100;
		double subval;
		if(_macroAction[i] == "recharge"){
			_curState[i][0] = _maxValues[0];
		}
		else if(_macroAction[i] == "smCan"){
			subval = smDist(_curState[i][0]);
		}
		else if(_macroAction[i] == "bgCan"){
			subval = bgDist(_curState[i][0]);
		}

		normal_distribution<double> distribution(subval, _transitionError[i]);
        _curState[i][0] -= distribution(_generator);

		if(_curState[i][0] < _minValues[0]){
			_curState[i][0] = _minValues[0];
		}
	}

	if(DEBUG)
		cerr << endl;
	
	// Set evader's new position
	_timeStep++;
}

vector<double>* RecyclingRobots::Step(string* jointaction)
{
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
		jointobs[i].push_back(_curState[i][0]);
		_macroAction[i] = "";
	}

	if(DEBUG){
		for(int i = 0; i < _numAgents; i++)
			cerr << "agent " << i << " now at: " << _curState[i][0] << " observes: " << jointobs[i][0] << endl;
	}
	return jointobs;
}

double RecyclingRobots::Reward(string* jointaction)
{
	/*double reward = 0;
	pair<double, double> probs[2];
	for(int i = 0; i < _numAgents; i++){
		if(_curState[i][0] == _minValues[0]){
			if(jointaction[i] == "bgCan"){
				probs[i] = pair<double, double>(.7,.3);
			}
			else if(jointaction[i] == "smCan"){
				probs[i] = pair<double, double>(.8,.2);
			}
			else{
				probs[i] = pair<double, double>(1,0);
			}
		}else{
			probs[i] = pair<double, double>(1,0);
		}
	}

	if(jointaction[0] == jointaction[1] && jointaction[0] == "bgCan"){
			reward = _rewards[0] * (probs[0].first * probs[1].first) 
				+ _rewards[3] * probs[0].second + _rewards[3] * probs[1].second;//5.0
	}
	else{
		double posprob = 1;
		double posreward = 0;
		for(int i = 0; i < _numAgents; i++){
			reward += _rewards[3] * probs[i].second;
			posprob *= probs[i].first;
			if(jointaction[i] == "smCan"){
				posreward += _rewards[1]; //2.0
			}
		}
		reward += posprob * posreward;
	}*/
	double reward = 0;
	if(jointaction[0] == jointaction[1] && jointaction[0] == "bgCan"){
		bool pass = true;
		for(int i = 0; i < _numAgents; i++){
			if(_curState[i][0] - bgDist(_curState[i][0]) <= _minValues[0])
				pass = false;
				reward += _rewards[3]*((100 - _curState[i][0])/100);
		}
		if(pass)
			reward += _rewards[0];
	}
	else{
		for(int i = 0; i < _numAgents; i++){
			if(jointaction[i] == "smCan"){
				if(_curState[i][0] - smDist(_curState[i][0]) > _minValues[0])
					reward += _rewards[1];
				else
					reward += _rewards[3]*((100 - _curState[i][0])/100);
			}
			else if(jointaction[i] == "bgCan"){
				if(_curState[i][0] - bgDist(_curState[i][0]) <= _minValues[0])
					reward += _rewards[3]*((100 - _curState[i][0])/100);
			}
		}
	}

	//NEED TO GET THE PROBABILITY THAT THE ACTION GIVEN THE CURRENT STATE WOULD CAUSE THE BATTERY
	//TO BECOME DEPLETED. THIS IS EXPRESSED AS P(s' <= 0 | s, a)

	if(DEBUG)
		cerr << "reward: " << reward << endl;
	return reward;
}