#include <gplanner.h>
#include <iostream>
#include <iterator>
#include <fstream>
#include <vector>
#include <iterator>


using namespace std;

#define INF 1000000

globalPlanner::globalPlanner()
{
	params.wexit=1;
    params.wentry=1;
    params.woccupied=1;
    params.wqueue=1;
    params.wtime=1;

    nofSpots=104;
    initCosts();
    


}


void globalPlanner::getQuery(std::vector<int> state, int qval)
{
	//cout<<qval;
    qSize=qval;
    calculateFinalCosts(state);

}


void globalPlanner::startD2Exitplanner()
{
    if(pp->plan())
    {
        exitSpotCosts=pp->getSpotCosts();
    }
}


void globalPlanner::initCosts()
{
    for (int i=0;i<nofSpots;i++)
    {
        exitSpotCosts.push_back(i);
        finalSpotCosts.push_back(0);
    }
}

void globalPlanner::getTimeCosts()
{
}

void globalPlanner::calculateFinalCosts(std::vector<int> state)
{

    std::vector<double> normExitCosts;
    normExitCosts=normalize(exitSpotCosts);
    for (int i=0; i< nofSpots;i++)
    {
        //if(state[i]==1)
        // {
        //     finalSpotCosts[i]=INF;
        // }
        // else
        // {
            finalSpotCosts[i]= params.wexit*normExitCosts[i]+ params.wqueue*qSize*i/1000 +
            params.wtime * i;
        // }
    }

    // write code to find minimum from best five spots and query local planner based on that


}

void globalPlanner::useCache()
{   
    double num;
    std::ifstream costFile("costs.txt", std::ios::in);
    for(int i =0; i<(exitSpotCosts.size()); i++)
    {   
        costFile>>num;
        exitSpotCosts[i]=num;
        cout<<exitSpotCosts[i]<<endl;
        if (costFile.eof())
        {
            while(i<exitSpotCosts.size())
            {
                exitSpotCosts[i]=double(num);
                cout<<i<<" "<<num<<" "<<exitSpotCosts[i]<<endl;
                i++;
            }
            break;
        }

    }

}
int globalPlanner::returnFinalSpot()
{
    auto spot =std::min_element(std::begin(finalSpotCosts),std::end(finalSpotCosts));
    return std::distance(std::begin(finalSpotCosts),spot);

}

std::vector<double> globalPlanner::normalize(std::vector<double> v)
{   
    double sum=0;
    for (std::vector<double>::iterator it =v.begin(); it!=v.end(); ++it)
    {
        sum+=*it;
    }

    for (std::vector<double>::iterator it =v.begin(); it!=v.end(); ++it)
    {
        *it=*it/sum;
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