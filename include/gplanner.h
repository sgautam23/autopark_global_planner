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

	void initCosts();

	void startD2Exitplanner(); 

	virtual void getTimeCosts();
	
	void calculateFinalCosts(std::vector<int> state);

	int returnFinalSpot();

	void useCache();

	std::vector<double> normalize(std::vector<double> v);

private:
	
	std::string envName="/home/shivam/sbpl/env_examples/nav3d/env_autopark_thin.cfg"; 
	std::string motPrim="/home/shivam/sbpl/matlab/mprim/unicycle_noturninplace.mprim";
	
	d2Exitplanner* pp =  new d2Exitplanner(envName.c_str(), motPrim.c_str());
	tuningParams params;
	
	std::vector<double> exitSpotCosts;
	std::vector<double> finalSpotCosts;
	int qSize;

	int nofSpots;



};






#endif	
