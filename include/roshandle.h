#ifndef roshandle_h
#define roshandle_h

#include <ros/ros.h>
#include <vector>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

#include <gplanner.h>
#include "gplanner/OptimalSpotGenerator.h" //include service files created
#include "gplanner/SpotsTreadCost.h"

struct Pose
{
float x;
float y;

};

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

	globalPlanner gp;
	gplanner::SpotsTreadCost lplanner_costs;
	

	geometry_msgs::PoseStamped posestamped;
	Pose pose;
};

#endif	