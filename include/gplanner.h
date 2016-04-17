#ifndef gplanner_h
#define gplanner_h

#include <pathplanner.h>
#include <string>
#include <cstring>
#include <worldtime/timemsg.h>
#include <geometry_msgs/PoseStamped.h>
#include "localplanner/spotsTreadCost.h"
#include <tf/tf.h>
#include <ros/ros.h>
#include <ros/package.h>

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
	float wpath; //weight given to cost from start to exit
	float woccupied; //weight given to whether spot is occupied or not
	float wqueue;
	float wtime;
};


class globalPlanner
{
public:

	globalPlanner();

	virtual void getQuery(int qval);

	void initCosts();

	void startD2Exitplanner(); 

	// virtual void getTimeCosts();
	
	void calculateFinalCosts();

	int returnFinalSpot();

	void useCache();

	void normalize(std::vector<double>& v);

	envState returnConfig(int i);

	void timeUpdate(const worldtime::timemsg::ConstPtr& msg);

	int getBestSpot(int i,localplanner::spotsTreadCost& lplanner);

	void getPathCosts(int i,float pathcost);

	void getBestArea(int a);

	int spotToArea(int pos);




private:
	
	// std::string path = ros::package::getPath("sbpl");
	// std::string envName=path+"/env_examples/nav3d/env_autopark_thin.cfg"; 
	// std::string motPrim=path+"/matlab/mprim/unicycle_noturninplace.mprim";

	std::string envName="/home/shivam/sbpl/env_examples/nav3d/env_autopark_thin.cfg"; 
	std::string motPrim="/home/shivam/sbpl/matlab/mprim/unicycle_noturninplace.mprim";
	
	d2Exitplanner* pp =  new d2Exitplanner(envName.c_str(), motPrim.c_str());
	tuningParams params;
	
	std::vector<double> exitSpotCosts;
	std::vector<double> finalSpotCosts;
	std::vector<int> state;
	std::vector<float> pathcosts;

	int qSize;
	int nofSpots;

	worldtime::timemsg peak;
	int duration;
	int area;



};






#endif	
