#ifndef pathplanner_h
#define pathplanner_h

#include <iostream>
#include <string>
#include <fstream>
#include <iterator>
#include <sstream>
#include <vector>

#include <sbpl/headers.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>


using namespace std;

struct envState{

float x; //x location of the state in meters
float y; //y location of that state in meters
float th; //theta at that state

};

class d2Exitplanner
{
public:

	d2Exitplanner();

	void createFootprint();
	void initializeEnv();
	//set env start goal
	void initializePlanner(SBPLPlanner*& planner, 
                       int start_id, int goal_id,
                       double initialEpsilon, 
                       bool bsearchuntilfirstsolution);
	int runPlanner(SBPLPlanner* planner, int allocated_time_secs, 
               vector<int>&solution_stateIDs);
	void plan();

	std::vector<int> getSpotCosts();

private:

	char* envCfgFilename; //environment file name
	char* motPrimFilename; //motion primitives
	vector<sbpl_2Dpt_t> perimeter;
	EnvironmentNAVXYTHETALAT env;
	// SBPLPlanner* planner;
	vector<int> solution_stateIDs;
	std::vector<int> spotCosts;

	double initialEpsilon = 5.0;
    bool bsearchuntilfirstsolution = false;
    double allocated_time_secs = 5.0;



};

#endif