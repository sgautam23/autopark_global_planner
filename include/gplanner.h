#ifndef gplanner_h
#define gplanner_h

#include <pathplanner.h>
#include <string>
#include <cstring>

using namespace std;

struct spotstate
{
int x; //x coord of spot
int y;
bool occupied; 
float exitCost;

};

struct tuningParams
{
	float wexit; //weight given to proximity to exit
	float wentry; //weight given to cost from start to exit
	float woccupied; //weight given to whether spot is occupied or not
	float wqueue;
	float wtime;
};


class globalPlanner
{
public:

	globalPlanner();

	virtual void getQuery(std::vector<int> state, int qval);

	void startD2Exitplanner();
	
	// gPlanner(
	// 		//some input for an environment
	// 	int nofspots,
	// 	int qsize

	// 	); //will give an error if not initialised in cpp file


	// virtual void plan();
	

	// virtual void getTimeCosts();
	
	// virtual void getplanrequest();

	
	
	

	//virtual void computeCost();


private:
	
	std::string envName="/home/shivam/sbpl/env_examples/nav3d/env_autopark.cfg"; 
	
	//std::string envName="/home/shivam/sbpl/env_examples/nav3d/willow-25mm-inflated-env.cfg";
	std::string motPrim="/home/shivam/sbpl/matlab/mprim/unicycle_noturninplace.mprim";
	d2Exitplanner* pp =  new d2Exitplanner(envName.c_str(), motPrim.c_str());
	tuningParams params;
	std::vector<int> exitSpotCosts;



};






#endif	
