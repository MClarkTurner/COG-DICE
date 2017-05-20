// FOR THE DISTRIBUTION CASE OF CGDICE USING MULTIPLE DIMENSIONS
#include "cgdice.h"

#define GET_TIME 0
#define PRINT_DIST_INFO 0
#define PRINT_BEST_INFO 1

CGDICEPlanner::CGDICEPlanner(
		DPOMDP* problem,
        size_t numberOfNodes, 
        size_t numberOfRestarts,
        size_t numberOfIterations, 
        size_t numberOfSamples, 
        size_t numberOfRetainedSamples, 
        size_t numTransitions,
        double learningRate,
        size_t evalRuns,
        size_t evalDepth,
        double discount,
        string outputFileName,
        string writeDist,
        string setDist
	)   
{
    _problem = problem;
	_numberOfNodes = numberOfNodes;
    _numberOfRestarts = numberOfRestarts;
	_numberOfIterations = numberOfIterations;
	_numberOfSamples = numberOfSamples;
	_numberOfRetainedSamples = numberOfRetainedSamples;
    _numTransitions = numTransitions;
	_learningRate = learningRate;
    _evalRuns = evalRuns;
	_evalDepth = evalDepth;
    _discount = discount;
    _outputFile = outputFileName;
    _writeDist = writeDist;
    _setDist = setDist;
}

void CGDICEPlanner::plan()
{
	size_t nrAgents = _problem->getNumAgents();

    // Initialize the distributions
    vector< vector< vector< double > > > _outputFunction;
    vector< vector< vector< size_t > > > _outputCount;

    vector< vector< vector< vector< double > > > > _transitionFunction; 
    vector< vector< vector< vector< size_t > > > > _transitionCount;

    DivFunc _divisionFunction;
    DivCount _divisionCount;

    random_device rd;
    default_random_engine re(time(0));

    // Begin developing JointPolicyGraphs
    for(size_t restart = 0; restart < _numberOfRestarts; restart++)
    {   
        initCEProbDistribution(_outputFunction, _outputCount, _transitionFunction, _transitionCount, _divisionFunction, _divisionCount, re);
        //if(_setDist != "")
        //    readCEProbDistribution(_outputFunction, _outputCount, _transitionFunction, _transitionCount, _setDist);

        cout << "Iter\tBest\tSmplMax\tSmplAvg\tSmplCnt\tTimeInSec" << endl;
        clock_t startTime = clock();

        double worstJointValue = -DBL_MAX;//set to Negative infinity
        for(size_t iter = 0; iter < _numberOfIterations; iter++)
        {
            resetCounts(_outputCount, _transitionCount, _divisionCount);
            ofstream samples;
            if(PRINT_DIST_INFO){
                samples.open("distData/controllersGenerated_r" + to_string(restart) + "_i" + to_string(iter) + ".txt");
            }
            double maxSample = -DBL_MAX;
            double avgSample = 0;
            size_t smplCount = 0;
            list<struct JointPolicyGraph*> policyList;
            if(iter > 0){
                struct JointPolicyGraph* tempbp = new struct JointPolicyGraph(_bestPolicy);
                orderedInsert(tempbp, policyList);
            }
            for(size_t sample = 0; sample < _numberOfSamples; sample++)
            {
                //cerr << "sample: " << sample << endl;
                //generate a JointPolicyGraph sample
            	struct JointPolicyGraph* samplePol = new struct JointPolicyGraph(rd, re, _problem->getDimension(), _outputFunction, _transitionFunction, _divisionFunction);
                //cerr << "sampled: " << sample << endl;
                //evaluate the sample
                //cerr << "evaluate: " << sample << endl;
                double v = ApproximateEvaluate((*samplePol)); 
                samplePol->value = v;
                //cerr << "evaluated: " << sample << endl;

                if(PRINT_DIST_INFO){
                    //samples << "restart: " << restart << " iteration: " << iter << " sample: " << sample << endl;
                    samplePol->print(_problem, samples);
                    //samples << "value:\t" << samplePol->value << endl;// << endl;
                }


                // If sample value is worse than current worstJointValue then prune it.
                // Otherwise add it to the policylist 
                if(v > worstJointValue){
                    orderedInsert(samplePol, policyList);
                }
                else{
                    delete samplePol;
                }

                // if the policy is better than the best policy then set the best policy 
                // to be the current policy.
                if(v > _bestPolicy.value){
                    _bestPolicy = (*samplePol);
                    //cerr << "writePol begin" <<endl;
                    writePolicy();
                    //cerr << "writePol end" <<endl;
                }
            } 

            if(PRINT_DIST_INFO)
                samples.close();
            
            // Set the worst policy value to be the lowest value in the policy list.
            //cerr << "setWorst begin" <<endl;
            if(policyList.size() > 0){
                unsigned worstvalind = min(policyList.size(), _numberOfRetainedSamples); 
                list<struct JointPolicyGraph*>::iterator it = policyList.begin();
                for(int z = 0; z < worstvalind-1; z++){
                    it++;
                }
                worstJointValue = (*it)->value;
                //cout << "WJV: " << worstJointValue << endl;
            }
            //cerr << "setWorst end" <<endl;

            //cerr << "setBestList begin" <<endl;
            list<struct JointPolicyGraph*>::iterator bp = policyList.begin();
            size_t pi = 0;
            while( bp != policyList.end() && pi < _numberOfRetainedSamples){
            // calculate the average, and maximum values for the current iteration of G-DICE
                if((*bp)->value > maxSample)
                    maxSample = (*bp)->value;
                smplCount++;
                avgSample += (*bp)->value;
                pi++;
                bp++;
            }
            //cerr << "setBestList end" <<endl;

            avgSample = avgSample/smplCount;
            //update the probability distribution
            if(PRINT_DIST_INFO){
                cout << "worstJointValue: " << worstJointValue << endl;
            }
            //cerr << "updateDist begin" <<endl;
            updateCEProbDistribution(_outputFunction, _outputCount, _transitionFunction, _transitionCount, _divisionFunction, _divisionCount, restart, iter, policyList);
            //cerr << "updateDist end" <<endl;
            policyList.clear();
            if(_writeDist != ""){
                writeDistToFile(_outputFunction, _outputCount, _transitionFunction, _transitionCount, _divisionFunction, _divisionCount, restart, iter);
            }
            cout << setprecision(4) << iter<< "\t" << _bestPolicy.value << "\t" << maxSample<< "\t" << avgSample << "\t" << smplCount << "\t" << (clock()-startTime)/(double)CLOCKS_PER_SEC << endl;
        }
    } 
}

size_t initialCount = 0;

void CGDICEPlanner::initCEProbDistribution(
    vector< vector< vector< double > > >& outputFunction,
    vector< vector< vector< size_t > > >& outputCount,

    vector< vector< vector< vector< double > > > >& transitionFunction,
    vector< vector< vector< vector< size_t > > > >& transitionCount,

    DivFunc& divisionFunction,
    DivCount& divisionCount,
    default_random_engine& re
    ){
    outputFunction.clear();
    outputCount.clear();
    transitionFunction.clear();
    transitionCount.clear();
    divisionFunction.clear();
    divisionCount.clear();

    for(size_t a = 0; a < _problem->getNumAgents(); a++){
        outputFunction.push_back( vector< vector< double > >() );
        outputCount.push_back( vector< vector< size_t > >() );
        transitionFunction.push_back( vector< vector< vector< double > > >() );
        transitionCount.push_back( vector< vector< vector< size_t > > >() );
        divisionFunction.push_back( vector< vector< DivisionData > >() );
        divisionCount.push_back( vector< vector< vector< double > > >() );

        size_t nrA = _problem->getNumActions(a);

        for(size_t n = 0; n < _numberOfNodes; n++){
            // Set up output Function
            outputFunction[a].push_back(vector<double>());
            outputCount[a].push_back(vector<size_t>());
            for(size_t ma = 0; ma < nrA; ma++){
                outputFunction[a][n].push_back( 1.0/nrA );
                outputCount[a][n].push_back( initialCount );
            }

            // Set up transition Function
            transitionFunction[a].push_back(vector< vector< double > >());
            transitionCount[a].push_back(vector< vector< size_t > >());
            for(size_t t = 0; t < pow(_numTransitions, _problem->getDimension()); t++){ // extended by dimension
                transitionFunction[a][n].push_back(vector< double >());
                transitionCount[a][n].push_back(vector< size_t >());
                for(size_t nn = 0; nn < _numberOfNodes; nn++){
                    transitionFunction[a][n][t].push_back(1.0/_numberOfNodes);
                    transitionCount[a][n][t].push_back( initialCount );
                }
            }

            //Set up division Function
            divisionFunction[a].push_back( vector< DivisionData >() );
            divisionCount[a].push_back( vector< vector< double > >() );
            for(size_t d = 0; d < pow(_numTransitions-1, _problem->getDimension()); d++){ // extended by dimension
                DivisionData dd;
                dd.alpha = 1;
                dd.beta = 1;
                divisionCount[a][n].push_back( vector< double >() );
                divisionFunction[a][n].push_back( dd );
            }
        }
    }   
}

void CGDICEPlanner::resetCounts(
    vector< vector< vector< size_t > > >& _outputCount,
    vector< vector< vector< vector< size_t > > > >& _transitionCount,
    DivCount& _divisionCount
    )
{
    for(size_t a = 0; a < _problem->getNumAgents(); a++){
        size_t nrA = _problem->getNumActions(a);

        for(size_t n = 0; n < _numberOfNodes; n++){
            for(size_t ma = 0; ma < nrA; ma++){
                _outputCount[a][n][ma] = initialCount;
            }

            // Set up transition Function
            for(size_t o = 0; o < pow(_numTransitions, _problem->getDimension()); o++){
                for(size_t nn = 0; nn < _numberOfNodes; nn++){
                    _transitionCount[a][n][o][nn] = initialCount;
                }
            }
            for(size_t d = 0; d < pow(_numTransitions-1, _problem->getDimension()); d++){
                 _divisionCount[a][n][d].clear();
            }
        }
    }   
}
/*
void CGDICEPlanner::readCEProbDistribution(
    vector< vector< vector< double > > >& _outputFunction,
    vector< vector< vector< size_t > > >& _outputCount,

    vector< vector< vector< vector< double > > > >& _transitionFunction,
    vector< vector< vector< vector< size_t > > > >& _transitionCount,
    string inputfile)
{
    
}*/

void CGDICEPlanner::updateCEProbDistribution(
    vector< vector< vector< double > > >& _outputFunction,
    vector< vector< vector< size_t > > >& _outputCount,

    vector< vector< vector< vector< double > > > >& _transitionFunction,
    vector< vector< vector< vector< size_t > > > >& _transitionCount,

    DivFunc& _divisionFunction,
    DivCount& _divisionCount,

    unsigned restart,
    unsigned iter,

    list<struct JointPolicyGraph*> listOfBestPolicies)
{
    list<struct JointPolicyGraph*>::iterator bp = listOfBestPolicies.begin();
    size_t pi = 0;

    ofstream samples;
    if(PRINT_BEST_INFO){
        samples.open("distData/bestGenerated_r" + to_string(restart) + "_i" + to_string(iter) + ".txt");
    }

    while( bp != listOfBestPolicies.end() && pi < _numberOfRetainedSamples){
        if(PRINT_BEST_INFO){
            //samples << "restart: " << restart << " iteration: " << iter << " sample: " << sample << endl;
            (*bp)->print(_problem, samples);
            samples << "value:\t" << (*bp)->value << endl << endl;
        }
        vector<struct FSA> pols = (*bp)->FSAs;
        for(size_t p = 0; p < pols.size(); p++){
            vector<size_t> nodes = pols[p].nodes;
            vector< vector<size_t> > connections = pols[p].connections;
            vector< vector<double> > divisions = pols[p].divisions;
            for(size_t n = 0; n < _numberOfNodes; n++){
                _outputCount[p][n][nodes[n]]++;
                

                for(size_t o = 0; o < pow(_numTransitions, _problem->getDimension()); o++){
                    _transitionCount[p][n][o][connections[n][o]]++;
                }
		
                for(size_t d = 0; d < pow(_numTransitions-1, _problem->getDimension()); d++){
                    //normalize the values between 0 and 1
                    double observationSpaceSize = 100; // needs to be replaced with adaptive value.
                    double normval = (divisions[n][d]/observationSpaceSize);
                    _divisionCount[p][n][d].push_back(normval);
                }
            }
        }
        delete *bp;
        
        pi++;
        bp++;
    }

    if(PRINT_BEST_INFO)
        samples.close();
    for(size_t a = 0; a < _problem->getNumAgents(); a++){
        size_t nrA = _problem->getNumActions(a);

        for(size_t n = 0; n < _numberOfNodes; n++){
            vector<double> oldProbs;
            size_t sum = 0;
            for(size_t act = 0; act < nrA; act++){
                oldProbs.push_back(_outputFunction[a][n][act]);
                sum += _outputCount[a][n][act];
            }
            for(size_t act = 0; act < nrA; act++){
                _outputFunction[a][n][act] = _learningRate*( _outputCount[a][n][act] / double(sum) ) + ( 1 - _learningRate )*oldProbs[act];
                
            }
            
            for(size_t obs = 0; obs < pow(_numTransitions, _problem->getDimension()); obs++){
                oldProbs.clear();
                sum = 0;
                for (size_t nn = 0; nn < _numberOfNodes; nn++)
                {
                    oldProbs.push_back(_transitionFunction[a][n][obs][nn]);
                    sum += _transitionCount[a][n][obs][nn];
                }
                for (size_t nn = 0; nn < _numberOfNodes; nn++)
                {
                    _transitionFunction[a][n][obs][nn] = _learningRate *( _transitionCount[a][n][obs][nn] / double(sum) ) + (1-_learningRate ) * oldProbs[nn];
                }
            }
            if(pi > 1){
                for(size_t d = 0; d < pow(_numTransitions-1, _problem->getDimension()); d++){
                    //_divisionFunction[a][n][d] = MLE(_divisionCount[a][n][d]);
                    DivisionData dd = MLE(_divisionCount[a][n][d], 2*_divisionCount[a][n][d].size());
                    _divisionFunction[a][n][d].alpha = _learningRate * dd.alpha + (1-_learningRate ) * _divisionFunction[a][n][d].alpha;
                    _divisionFunction[a][n][d].beta = _learningRate * dd.beta + (1-_learningRate ) * _divisionFunction[a][n][d].beta;
                } 
            }
    	}
    }
}

void CGDICEPlanner::writeDistToFile(
    vector< vector< vector< double > > >& _outputFunction,
    vector< vector< vector< size_t > > >& _outputCount,

    vector< vector< vector< vector< double > > > >& _transitionFunction,
    vector< vector< vector< vector< size_t > > > >& _transitionCount,

    DivFunc& _divisionFunction,
    DivCount& _divisionCount,

    unsigned restart,
    unsigned iteration
    )
{
    ofstream ofile;
    ofile.open("distData/" + _writeDist + "_r" + to_string(restart) + "_i" + to_string(iteration) + ".txt");

    for(size_t a = 0; a < _problem->getNumAgents(); a++){
        ofile << a << ':' << endl;
        size_t nrA = _problem->getNumActions(a);

        for(size_t n = 0; n < _numberOfNodes; n++){
            ofile << '\t' << n << ":" << endl;
            ofile << "\t\t" << "output: ";
            for(size_t act = 0; act < nrA; act++){
                ofile << "(" << _outputCount[a][n][act] << ", " << _outputFunction[a][n][act] << "), ";
            }
            ofile << endl;
            ofile << "\t\t" << "divisions: " << endl;
            for(size_t d = 0; d < pow(_numTransitions-1, _problem->getDimension()); d++){
                ofile << "\t\t\t" << d << ": ";
                ofile << "(" << _divisionFunction[a][n][d].alpha << ", " << _divisionFunction[a][n][d].beta << "), " << endl;
            }
            ofile << "\t\t" << "transition: " << endl;
            for(size_t obs = 0; obs < pow(_numTransitions, _problem->getDimension()); obs++){
                ofile << "\t\t\t" << obs << ": ";
                for (size_t nn = 0; nn < _numberOfNodes; nn++)
                {
                    ofile << "(" << _transitionCount[a][n][obs][nn] << ", " << _transitionFunction[a][n][obs][nn] << "), ";
                }
                ofile << endl;
            } 

        } 
    }
    ofile.close();
}

void CGDICEPlanner::orderedInsert( struct JointPolicyGraph* pv, 
            list<struct JointPolicyGraph*>& l)
{
// Add a value to a list of definite size ordered from greatest to smallest. If the 
// number of values in the list is smaller than the lists size then the smallest value is evicted.
    double v_pv = pv->value;
    list<struct JointPolicyGraph*>::iterator it = l.begin(); //=front - highest values
    list<struct JointPolicyGraph*>::iterator last = l.end(); //=back - lowest values
    while(it != last)
    {
        struct JointPolicyGraph* temp = *it;
        double val = temp->value;
        if( v_pv < val )
        {
            it++;
        }
        else
        {
            l.insert(it, pv);
            return;
        }
    }
    //this should only happen when size=0
    l.insert(last, pv);
}

double CGDICEPlanner::ApproximateEvaluate(struct JointPolicyGraph& jp)
{
    //cerr << "approx Evaluate Start" << endl;
    //jp.print(_problem, cerr);

    double totalReward = 0;

    for(int i = 0; i < _evalRuns; i++){
        //cerr << "reset Start" << endl;
        _problem->reset();
        //cerr << "reset end" << endl;
        double runReward = 0;

        size_t* currentNode = new size_t[_problem->getNumAgents()];
        for(int a = 0; a < _problem->getNumAgents(); a++){
            currentNode[a] = 0;
        }

        while(_problem->getTimeStep() < _evalDepth){
            string* jointaction = new string[_problem->getNumAgents()];
            for(int a = 0; a < _problem->getNumAgents(); a++){
                size_t node = jp.FSAs[a].nodes[currentNode[a]];
                jointaction[a] = _problem->getAction(a, node);
            }
            
            

            /*for(int i = 0; i < _problem->getNumAgents(); i++){
                cerr << "agent " << i << ": " << jointaction[i] << endl;
                cerr << "\t" << _problem->getCurrentState()[i][0] << endl;
                cerr << "\t" << jointobs[i][0] << endl;
            }*/

            double rew = (_problem->Reward(jointaction));
            runReward += pow(_discount, _problem->getTimeStep()) * rew;

            vector<double>* jointobs = _problem->Step(jointaction);//moved a head of reward function

            /*cerr << "reward: " << rew << endl;
            cerr << endl;*/

            for(int a = 0; a < _problem->getNumAgents(); a++){
                if(jointobs[a][0] != DBL_MAX){
                    size_t t = getTransition(jp.FSAs[a].divisionSort, currentNode[a], jointobs[a]);
                    currentNode[a] = jp.FSAs[a].connections[currentNode[a]][t];
                }
            }
            delete[] jointobs;
            delete[] jointaction;
        }
        delete[] currentNode;

        totalReward += runReward;
    }
    return totalReward/_evalRuns;
}

size_t CGDICEPlanner::getTransition(vector< vector<double> >& divisions, size_t curnode, vector<double>& jointobs){
    size_t retval = 0;
    for(int dim = 0; dim < _problem->getDimension(); dim++){
        size_t s = 0;
        
        while(s < _numTransitions-1 && jointobs[dim] > divisions[curnode][s] ){
            s++;
        }
        retval += s * pow(_numTransitions, dim);
    }
    return retval;    
}
