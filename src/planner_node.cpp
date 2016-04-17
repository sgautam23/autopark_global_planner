#include <iostream>
//#include <headers.h>
#include <string>
#include <roshandle.h>
#include <gplanner.h>
#include <pathplanner.h>


int main( int argc, char ** argv)

{
	//globalPlanner planner; //create an object of the global planner
	
	ros::init (argc,argv,"Global_planner"); //initialise the ROS node	
	ros::NodeHandle nh;
	ROShandle ros(nh);
	ros::Rate rate(20.0);
	while( ros::ok() ) {
        ros::Duration(0.02).sleep();
        ros::spinOnce();
    }
	
	// ros::spin();


	return 0;

}