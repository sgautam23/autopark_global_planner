#ifndef roshandle_h
#define roshandle_h

#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <utility> 
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

#include <gplanner.h>
#include "gplanner/OptimalSpotGenerator.h" //include service files created
#include "localplanner/spotsTreadCost.h"
#include "gplanner/mab.h"


#include <geometry_msgs/PoseStamped.h>

#define CACHED 1
#define MAB 1

struct Pose
{
float x;
float y;

};

// struct spot_compare
// {
//    bool operator()( const Node *a, const Node *b ) const 
//    {
//     return a->f > b->f;
//    }
// };

class ROShandle
{
public:

	ROShandle(ros::NodeHandle& n);
	
	bool estimation(gplanner::OptimalSpotGenerator::Request &req,gplanner::OptimalSpotGenerator::Response &res);
	void init_ros();


private:
	ros::NodeHandle nh;
	ros::ServiceServer optimalSpot;
	ros::ServiceClient lplanner_client;
	ros::ServiceClient mab_client;
	ros::Subscriber wtimesub;
	ros::Subscriber stateUpdate;

	globalPlanner gp;
	localplanner::spotsTreadCost lplanner_costs;
	gplanner::mab mab_req;
	
	geometry_msgs::PoseStamped start;
	geometry_msgs::PoseStamped goal;
	Pose pose;
};

#endif	