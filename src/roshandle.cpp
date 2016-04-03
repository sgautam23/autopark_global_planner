#include <roshandle.h>




ROShandle::ROShandle(ros::NodeHandle& n)
	{
	nh=n;
	init_ros();
	

	}

bool ROShandle::estimation(gplanner::OptimalSpotGenerator::Request &req, gplanner::OptimalSpotGenerator::Response &res)

{	

	if (req.request)
	{	
		ROS_INFO("Request Received from rEngine");

		gp.getQuery(req.state,req.qval);
		//lplanner_costs.request = true;
		/*
		if(lplanner_client.call(lplanner_costs))
		{
			ROS_INFO("Response Received from lplanner");

		}

		else
		{
			ROS_INFO("Failed to get response from lPlanner");
		}
		
		*/
	res.spots[0]=0;
	res.spots[1]=0;
	res.spots[2]=0;
	
//	pose={0,0};
//	posestamped=PosetoPoseStamped(posestamped,pose);
//	res.finalSpots= posestamped;

	ROS_INFO("Response sent to rEngine");
	return true;
	}
}


void ROShandle::init_ros()
{
		optimalSpot = nh.advertiseService("OptimalSpotGenerator",&ROShandle::estimation,this); //initialise the ROS service for the spot query
		lplanner_client = nh.serviceClient<gplanner::SpotsTreadCost>("Local_Planner_Cost_Service");
	
	
}