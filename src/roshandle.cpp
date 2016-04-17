#include <roshandle.h>


#define CACHED 1

ROShandle::ROShandle(ros::NodeHandle& n)
{
	nh=n;
	init_ros();

	if (!CACHED)
	{	
		gp.startD2Exitplanner();
	}

	else
	{
		gp.useCache();
	}
}

bool ROShandle::estimation(gplanner::OptimalSpotGenerator::Request &req, gplanner::OptimalSpotGenerator::Response &res)

{	

	if (req.request)
	{	
		ROS_INFO("Request Received from rEngine");

		gp.getQuery(req.qval);

		for ( int i=0; i<5;i++)
		{
			int pos=gp.getBestSpot(i,lplanner_costs);


			// cout<<"\nlpanner data goal "<<lplanner_costs.request.goal.pose.position.x<< " "
			// 		<<lplanner_costs.request.goal.pose.position.y<<" "<<lplanner_costs.request.goal.pose.orientation.x<<" "<<lplanner_costs.request.goal.pose.orientation.y
			// 		<<" "<<lplanner_costs.request.goal.pose.orientation.z<<" "<<lplanner_costs.request.goal.pose.orientation.w;
			
			// cout<<"\nlpanner data start"<<lplanner_costs.request.start.pose.position.x<< " "
			// 		<<lplanner_costs.request.start.pose.position.y<<" "<<lplanner_costs.request.start.pose.orientation.x<<" "<<lplanner_costs.request.start.pose.orientation.y
			// 		<<" "<<lplanner_costs.request.start.pose.orientation.z<<" "<<lplanner_costs.request.start.pose.orientation.w;
			
			ROS_INFO("Request sent to local planner");
			if(lplanner_client.call(lplanner_costs))
			{
				ROS_INFO("Response Received from local planner");
				gp.getPathCosts(pos,lplanner_costs.response.pathcost);

			}

			else
			{
				ROS_INFO("Failed to get response from local planner");
			}
		}

		int spotNo=gp.returnFinalSpot();
		cout<<endl<<"spot No "<<spotNo;
		struct envState e= gp.returnConfig(spotNo);
		cout<<endl<<"coords"<<e.x<<" "<<e.y<<" "<<e.th<<endl;
		res.spots[0]=e.x;
		res.spots[1]=e.y;
		res.spots[2]=e.th;
		ROS_INFO("Response sent to rEngine");
		return true;
	}
}


void ROShandle::init_ros()
{
		optimalSpot = nh.advertiseService("OptimalSpotGenerator",&ROShandle::estimation,this); //initialise the ROS service for the spot query
		lplanner_client = nh.serviceClient<localplanner::spotsTreadCost>("spotsTreadCost");
		wtimesub =nh.subscribe("worldtime",10,&globalPlanner::timeUpdate,&gp);

	}

