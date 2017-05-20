#include <iostream>
#include "cgdice.h"
#include "../recyclingrobots/recyclingrobotsdiscrete.h"
#include "jointpolicygraphcont.h"
#include <string>

using namespace std;
/*
Agent 0
node actions:(0, mvL1), (1, mvR1), (2, tag), 
divisions:
0: 8.26839,9.46934,
1: -5.31743,-7.87131,
2: -8.42584,-5.2188,
connections:
0: (0, 0), (1, 2), (2, 1), 
1: (0, 0), (1, 2), (2, 1), 
2: (0, 0), (1, 2), (2, 1), 
Agent 1
node actions:(0, mvL1), (1, mvR1), (2, tag), 
divisions:
0: 8.95484,9.60366,
1: -2.17808,-8.78627,
2: -6.32493,-8.83731,
connections:
0: (0, 0), (1, 2), (2, 1), 
1: (0, 0), (1, 2), (2, 1), 
2: (0, 0), (1, 2), (2, 1), 
*/

int main(){
    srand(time(0));
	RecyclingRobots* ts = new RecyclingRobots("../recyclingrobots/simRR.txt");
	size_t numberOfNodes = 3;
    size_t numberOfRestarts = 1;
    size_t numberOfIterations = 5;
    size_t numberOfSamples = 5;
    size_t numberOfRetainedSamples = 5;
    size_t numberOfTransitions = 2;
    double learningRate = 0.1;
    size_t evalRuns = 10000;
    size_t evalDepth = 150;
    double discount = 0.9;

	CGDICEPlanner planner(ts, numberOfNodes, numberOfRestarts, numberOfIterations,
		numberOfSamples, numberOfRetainedSamples, numberOfTransitions, learningRate, evalRuns, evalDepth, discount);
	//planner.plan();

	struct JointPolicyGraph jp1;
	for(int i = 0; i < ts->getNumTaggers(); i++){
        struct FSA fsa;
        for(int n = 0; n < 5; n++){
            if(n == 0)
                fsa.nodes.push_back(1);
            else if(n == 1)
                fsa.nodes.push_back(2);
            else if(n == 2)
                fsa.nodes.push_back(3);
            else if(n == 3)
                fsa.nodes.push_back(4);
            else
                fsa.nodes.push_back(0);
            
            fsa.connections.push_back(vector<size_t>());
            for(int o = 0; o < 9; o++){
                if(o == 0){
                    if(n == 2)
                        fsa.connections[n].push_back(0);
                    else
                        fsa.connections[n].push_back(2);
                }
                else if(o == 1)
                    fsa.connections[n].push_back(2);
                else if(o == 2){
                    if(n == 2)
                        fsa.connections[n].push_back(1);
                    else
                        fsa.connections[n].push_back(2);
                }
                else if(o == 3)
                    fsa.connections[n].push_back(0);
                else if(o == 4)
                    fsa.connections[n].push_back(4);
                else if(o == 5)
                    fsa.connections[n].push_back(1);
                else if(o == 6){
                    if(n == 3)
                        fsa.connections[n].push_back(0);
                    else
                        fsa.connections[n].push_back(3);
                }
                else if(o == 7)
                    fsa.connections[n].push_back(3);
                else if(o == 8){
                    if(n == 3)
                        fsa.connections[n].push_back(1);
                    else
                        fsa.connections[n].push_back(3);
                }
            }

            fsa.divisions.push_back(vector<double>());
            fsa.divisionSort.push_back(vector<double>());
            for(int d = 0; d < 4; d++){
                if(d == 0){
                    fsa.divisionSort[n].push_back(-1);
                    fsa.divisions[n].push_back(-1);
                }
                else if(d == 1){
                    fsa.divisionSort[n].push_back(1);
                    fsa.divisions[n].push_back(1);
                }
                else if(d == 2){
                    fsa.divisionSort[n].push_back(-1);
                    fsa.divisions[n].push_back(-1);
                }
                else if(d == 3){
                    fsa.divisionSort[n].push_back(1);
                    fsa.divisions[n].push_back(1);
                }
            }
        }
        
        jp1.FSAs.push_back(fsa);
    }

    struct JointPolicyGraph jprr;
    for(int i = 0; i < ts->getNumTaggers(); i++){
        struct FSA fsa;
        for(int n = 0; n < 4; n++){
            if(n == 0)
                fsa.nodes.push_back(2);
            else if(n == 1)
                fsa.nodes.push_back(0);
            else if(n == 2)
                fsa.nodes.push_back(0);
            else if(n == 3)
                fsa.nodes.push_back(0);
            
            fsa.connections.push_back(vector<size_t>());
            for(int o = 0; o < 2; o++){
                int val = n+1;
                if(val >= 3)
                    val = 0;
                if(o == 0){
                    fsa.connections[n].push_back(2);
                }
                else{
                    fsa.connections[n].push_back(0);
                }
            }

            fsa.divisions.push_back(vector<double>());
            fsa.divisionSort.push_back(vector<double>());
            for(int d = 0; d < 1; d++){
                fsa.divisionSort[n].push_back(50);
                fsa.divisions[n].push_back(50);
            }
        }
        
        jprr.FSAs.push_back(fsa);
    }


    struct JointPolicyGraph jp2;

    struct FSA fsa;
    for(int n = 0; n < 3; n++){
        fsa.divisions.push_back(vector<double>());
        fsa.divisionSort.push_back(vector<double>());
        if(n == 0){
            fsa.nodes.push_back(1);
            fsa.divisionSort[n].push_back(8.26839);
            fsa.divisionSort[n].push_back(9.46934);
            fsa.divisions[n].push_back(8.26839);
            fsa.divisions[n].push_back(9.46934);
        }
        else if(n == 1){
            fsa.nodes.push_back(0);
            fsa.divisionSort[n].push_back(-7.87131);
            fsa.divisionSort[n].push_back(-5.31743);
            fsa.divisions[n].push_back(-7.87131);
            fsa.divisions[n].push_back(-5.31743);
        }
        else{
            fsa.nodes.push_back(2);
            fsa.divisionSort[n].push_back(-8.42584);
            fsa.divisionSort[n].push_back(-5.2188);
            fsa.divisions[n].push_back(-8.42584);
            fsa.divisions[n].push_back(-5.2188);
        }
        
        fsa.connections.push_back(vector<size_t>());
        for(int o = 0; o < 3; o++){
            if(o == 0)
                fsa.connections[n].push_back(0);
            else if(o == 1)
                fsa.connections[n].push_back(2);
            else if(o == 2)
                fsa.connections[n].push_back(1);
        }

        
        for(int d = 0; d < 2; d++){
            if(d == 0){
                fsa.divisionSort[n].push_back(-1);
            }
            else if(d == 1){
                fsa.divisionSort[n].push_back(1);
            }
        }
    }
    
    jp2.FSAs.push_back(fsa);

    struct FSA fsa2;
    for(int n = 0; n < 3; n++){
        fsa2.divisionSort.push_back(vector<double>());
        if(n == 0){
            fsa2.nodes.push_back(1);
            fsa2.divisionSort[n].push_back(8.95484);
            fsa2.divisionSort[n].push_back(9.60366);
        }
        else if(n == 1){
            fsa2.nodes.push_back(0);
            fsa2.divisionSort[n].push_back(-8.78627);
            fsa2.divisionSort[n].push_back(-2.17808);
        }
        else{
            fsa2.nodes.push_back(2);
            fsa2.divisionSort[n].push_back(-8.83731);
            fsa2.divisionSort[n].push_back(-6.32493);
        }
        
        fsa2.connections.push_back(vector<size_t>());
        for(int o = 0; o < 3; o++){
            if(o == 0)
                fsa2.connections[n].push_back(0);
            else if(o == 1)
                fsa2.connections[n].push_back(2);
            else if(o == 2)
                fsa2.connections[n].push_back(1);
        }
    }
    
    jp2.FSAs.push_back(fsa2);
/*
    struct JointPolicyGraph jpmv;
    for(int i = 0; i < ts->getNumTaggers(); i++){
        struct FSA fsa;
       
            fsa.nodes.push_back(1);
            
            fsa.connections.push_back(vector<size_t>());
            for(int o = 0; o < ts->getNumObservations(i); o++){
                fsa.connections[0].push_back(0);
                if(o <= 0)
                    fsa.connections[0].push_back(0);
                else 
                    fsa.connections[0].push_back(1);
            }

        
            fsa.nodes.push_back(2);
                
            fsa.connections.push_back(vector<size_t>());
            for(int o = 0; o < ts->getNumObservations(i); o++){
                fsa.connections[1].push_back(0);
                if(o <= 0)
                    fsa.connections[1].push_back(0);
                else 
                    fsa.connections[1].push_back(1);
            }
    
        jpmv.FSAs.push_back(fsa);
    }
    */
    planner.printPolicy(jprr);
    cout << planner.ApproximateEvaluate(jprr) << endl;
	return 0;
}
