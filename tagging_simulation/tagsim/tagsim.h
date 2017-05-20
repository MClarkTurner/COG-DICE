/*
 Madison Clark-Turner
 tagsim.h
 1/24/2016
 */
#ifndef TAGSIM_H
#define TAGSIM_H

#include <string>
#include <vector>
#include <random>
#include <iostream>

using namespace std;

class TagSim{
private:
	size_t _timeStep = 0;
	default_random_engine _generator;
    
	double _minSize;
	double _maxSize;
    
	bool _continuousStates;
	bool _continuousObservations;
	double _divisionSize;
    
	size_t _numTaggers = 0; // The number of Taggers in the simulation
	double* _agentPositions = NULL; // index 0 is the evader, all others are taggers
	double* _obsError = NULL; // the degree of observation error for each agent
	double* _tagRange = NULL; // the tag range for each agent
	double* _transitionError = NULL; // the degree of error in the agent transtioning from one state to another
	double* _agentSpeed = NULL; // the speed each agent move's in a timestep (1 = 1 distance per timestep)
	string* _macroAction = NULL; // The macro action being performed by each agent
    string* _lastAction = NULL; // The last action that was performed by each agent
	size_t* _macroDuration = NULL; // the time spent performing the current macro action
	vector<string>* _actions = NULL; // the available actions for each agent
    
	double _reward;
	double _penalty;
	double _waitPenalty;
    
	void readValuesFromFile(string);
	void restart();
	double makeDiscrete(double);
	void performActions();


    
public:
	unsigned _singletag = 0;
	unsigned _doubletag = 0;

	TagSim(size_t numTaggers, double minSize, double maxSize, double* tagRange,
           double* obsError, double* transitionError, double* agentSpeed, vector<string>* actions,
           double reward, double penalty, double waitPenalty, bool contStates, bool contObs,
           double divisionSize);
	TagSim(string);
	~TagSim();
    
	double* Step(string* jointaction);// pass in joint action and return joint obs
	double Reward(string* jointaction);// pass in joint action and return reward
    
	size_t getNumTaggers(){return _numTaggers;};
	double* getCurrentState(){return _agentPositions;};
    
	size_t getTimeStep(){return _timeStep;};
	void reset(){
		_timeStep = 0;
		restart();
    };
    
	size_t getNumAgents(){return _numTaggers;};
    double getAgentSpeed(size_t a){return _agentSpeed[a];};
    string getLastAction(size_t a){return _lastAction[a];};
	size_t getNumActions(size_t a){return _actions[a].size();};
	size_t getNumObservations(size_t a);
	string getAction(size_t a, size_t i){return _actions[a][i];};
	double getObservation(size_t a, size_t i);
	size_t getObservationIndex(size_t a, string v);
	size_t getObservationIndex(size_t a, double v);
    
	double getMinSize(){return _minSize;};
	double getMaxSize(){return _maxSize;};
};

#endif

/* File Format
 # Set environment of simulation
 minSize 0
 maxSize 10
 contStates false
 contObs false
 divisionSize 0.1
 evaderSpeed 2
 
 # Set tagger information
 numTaggers 2
 tagRange 1	1
 obsError .8 .8
 transitionError .8 .8
 taggerSpeed 1 1
 
 #declare agent actions
 actions
 tag mvL1 mvR1 mvLHit mvRHit
 tag mvL1 mvR1 mvLHit mvRHit
 
 # Set reward information
 reward 5
 penalty -2
 wait -1
 
 */