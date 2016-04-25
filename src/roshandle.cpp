#include <roshandle.h>




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
	std::vector<pair<int,double>> top5spots;
	bool call=false;
	int spotNo;

	if (req.request)
	{	
		ROS_INFO("Request Received from rEngine");

		gp.getQuery(req.qval);

		if (MAB)
		{
			ROS_INFO("Request sent to MAB");
			mab_req.request.request=true;
			if (mab_client.call(mab_req))
			{
				ROS_INFO("Response received from MAB");
				ROS_INFO("Area selected is: %d",mab_req.response.area);
				gp.getBestArea(mab_req.response.area);
			
			}

			else
			{
				ROS_INFO("Failed to get response from MAB");
			}
		}

		for ( int i=0; i<5;i++)
		{
			int pos=gp.getBestSpot(i,lplanner_costs);

			if (pos <0 || pos > 104)
		    {
		       continue;
		    }


			// cout<<"\nlpanner data goal "<<lplanner_costs.request.goal.pose.position.x<< " "
			// 		<<lplanner_costs.request.goal.pose.position.y<<" "<<lplanner_costs.request.goal.pose.orientation.x<<" "<<lplanner_costs.request.goal.pose.orientation.y
			// 		<<" "<<lplanner_costs.request.goal.pose.orientation.z<<" "<<lplanner_costs.request.goal.pose.orientation.w;
			
			// cout<<"\nlpanner data start"<<lplanner_costs.request.start.pose.position.x<< " "
			// 		<<lplanner_costs.request.start.pose.position.y<<" "<<lplanner_costs.request.start.pose.orientation.x<<" "<<lplanner_costs.request.start.pose.orientation.y
			// 		<<" "<<lplanner_costs.request.start.pose.orientation.z<<" "<<lplanner_costs.request.start.pose.orientation.w;
			
			ROS_INFO("Request sent to local planner");
			if(lplanner_client.call(lplanner_costs))
			{	
				call=true;
				ROS_INFO("Response Received from local planner");
				gp.getPathCosts(pos,lplanner_costs.response.pathcost);
				top5spots.push_back(std::make_pair(pos,lplanner_costs.response.pathcost));
			}

			else
			{	
				call=false;
				ROS_INFO("Failed to get response from local planner");
			}
		}

		if (call)
		{
			auto cost =std::min_element(std::begin(top5spots),std::end(top5spots),
								[](std::pair<int,double> const& n1 , std::pair<int,double> const& n2)
									{
										return n1.second < n2.second;
									} );
    
    		spotNo= top5spots[std::distance(std::begin(top5spots),cost)].first;

    		if (spotNo <0 || spotNo > 103)
		    {
		       spotNo=gp.returnFinalSpot();
		       ROS_WARN(" Invalid Spot selected in MAB: %d", spotNo);
		    }

    		gp.setState(spotNo);

    
		}
		else
		{
		 spotNo=gp.returnFinalSpot();
		if (spotNo <0 || spotNo > 103)
		    {
		       ROS_WARN(" Invalid Spot selected in GPLANNER: %d", spotNo);
		    }	
		}
		
		if (spotNo>103 || spotNo<0)
		{
			ROS_WARN(" Invalid Spot selected: %d", spotNo);
			ROS_WARN(" Spot came from %s", call ? "MAB" : "GPlanner");

		}
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
		mab_client=nh.serviceClient<gplanner::mab>("segmentRequest");
		wtimesub =nh.subscribe("worldtime",10,&globalPlanner::timeUpdate,&gp);
		stateUpdate=  nh.subscribe("return_update",0,&globalPlanner::unpark,&gp);
	}

