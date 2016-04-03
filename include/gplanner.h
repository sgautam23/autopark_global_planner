#ifndef gplanner_h
#define gplanner_h

#include <pathplanner.h>


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

	
	tuningParams params;



};






#endif	
