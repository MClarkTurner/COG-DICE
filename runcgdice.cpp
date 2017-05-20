#include "cgdice.h"
#include "../recyclingrobots/recyclingrobots.h"
#include <iostream>
#include <cstdlib>

using namespace std;

void help(){
    cout<< "runGDICE Usage:" << endl;
    cout<< "\t-f <filename>\tname of tagsim file" << endl;
    cout<< "\t-n <unisgned int>\tcontroller size/number of nodes" << endl;
    cout<< "\t-r <unsigned int>\tnumber of restarts" << endl;
    cout<< "\t-i <unsigned int>\tnumber of iterations" << endl;
    cout<< "\t-s <unsigned int>\tnumber of samples" << endl;
    cout<< "\t-k <unsigned int>\tnumber of retained samples" << endl;
    cout<< "\t-a <float>\tlearning rate" << endl;
    cout << endl;
    cout<< "\t-er <unsigned int>\tnumber of times evaluator will be run" << endl;
    cout<< "\t-ed <unsigned int>\tdepth evaluator will run too" << endl;
    cout<< "\t-d <float>\tdiscount" << endl;
    cout << endl;
    cout<< "\t-o <string>\tfile to output most recent policy to" << endl;
    cout<< "\t-w <string>\tfile to output most recent distribution to" << endl;
    cout<< "\t-set <string>\tfile to seed the distribution with" << endl;
}

int main(int argc, char** argv){
    RecyclingRobots* ts;
    size_t numberOfNodes;
    size_t numberOfRestarts;
    size_t numberOfIterations;
    size_t numberOfSamples;
    size_t numberOfRetainedSamples;
    size_t numberOfTransitions;
    double learningRate;
    size_t evalRuns;
    size_t evalDepth;
    double discount;
    string outputFileName = "";
    string distributionFileName = "";
    string seedDist = "";

    bool* allParts = new bool[10];
    for(int i = 0; i < 10; i++){
        allParts[i] = 0;
    }

    for(int c = 1; c < argc; c++){
        string inp(argv[c]);
        if (inp == "--help"){
            help();
        }
        else if(inp == "-f"){//filename
            ts = new RecyclingRobots(argv[c+1]);
            allParts[0] = 1;
        }
        else if(inp == "-n"){//number of Nodes
            numberOfNodes = atoi(argv[c+1]);
            allParts[1] = 1;
        }
        else if(inp == "-r"){//number of restarts
            numberOfRestarts = atoi(argv[c+1]);
            allParts[2] = 1;
        }
        else if(inp == "-i"){//number of iterations
            numberOfIterations = atoi(argv[c+1]);
            allParts[3] = 1;
        }
        else if(inp == "-s"){//number of Samples
            numberOfSamples = atoi(argv[c+1]);
            allParts[4] = 1;
        }
        else if(inp == "-k"){//number of retainedSamples
            numberOfRetainedSamples = atoi(argv[c+1]);
            allParts[5] = 1;
        }
        else if(inp == "-t"){//number of transitions
            numberOfTransitions = atoi(argv[c+1]);
            allParts[10] = 1;
        }
        else if(inp == "-a"){//learning rate
            learningRate = atof(argv[c+1]);
            allParts[6] = 1;
        }
        else if(inp == "-er"){//number of eval runs
            evalRuns = atoi(argv[c+1]);
            allParts[7] = 1;
        }
        else if(inp == "-ed"){//eval depth
            evalDepth = atoi(argv[c+1]);
            allParts[8] = 1;
        }
        else if(inp == "-d"){//discount
            discount = atof(argv[c+1]);
            allParts[9] = 1;
        }
        else if(inp == "-o"){//output file
            outputFileName = string(argv[c+1]);
        }
        else if(inp == "-w"){//distribution file
            distributionFileName = string(argv[c+1]);
        }
        else if(inp == "-set"){//distribution file
            seedDist = string(argv[c+1]);
        }
    }

    for(int i = 0; i < 11; i++){
        if(!allParts[i]){
            cerr << "type '--help' flag to see usage." << i << endl;
            exit(-1);
        }
    }

	CGDICEPlanner planner(ts, numberOfNodes, numberOfRestarts, numberOfIterations,
		numberOfSamples, numberOfRetainedSamples, numberOfTransitions, learningRate, evalRuns, evalDepth, discount,
        outputFileName, distributionFileName, seedDist);
	planner.plan();

    cout << "================================" << endl;
	planner.printPolicy();
    cout << "================================" << endl;
    delete ts;
	return 0;
}
