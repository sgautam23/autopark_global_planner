#include <gplanner.h>
#include <iostream>

using namespace std;


globalPlanner::globalPlanner()
{
	//params={0.3,0.3,0.1,0.1,0.2};
    


}


void globalPlanner::getQuery(std::vector<int> state, int qval)
{
	//cout<<qval;
}


void globalPlanner::startD2Exitplanner()
{
    if(pp->plan())
    {
        exitSpotCosts=pp->getSpotCosts();
    }
}
/*
void PoseStampedtoPose(const geometry_msgs::PoseStamped& p1, Pose& p2)
{
    p2.x =  p1.pose.position.x;
    p2.y = p1.pose.position.y;
    tf::Quaternion p1_quat = tf::Quaternion(p1.pose.orientation.x,p1.pose.orientation.y,
                                            p1.pose.orientation.z, p1.pose.orientation.w);
    p2.th = tf::getYaw(p1_quat);

}
void PosetoPoseStamped(geometry_msgs::PoseStamped& p1, const Pose& p2)
{
    
    tf::Quaternion p1_quat = tf::createQuaternionFromYaw(p2.th);
            
    p1.pose.position.x = p2.x;
    p1.pose.position.y = p2.y;
              
    p1.pose.orientation.x = p1_quat.x();
    p1.pose.orientation.y = p1_quat.y();
    p1.pose.orientation.z = p1_quat.z();
    p1.pose.orientation.w = p1_quat.w();

}
*/