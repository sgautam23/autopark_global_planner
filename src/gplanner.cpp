#include <gplanner.h>
#include <iostream>
#include <iterator>
#include <fstream>
#include <vector>
#include <iterator>


using namespace std;

#define INF 1000000
#define MAB 1

globalPlanner::globalPlanner()
{
	params.wexit=1.1;
    params.wpath=1.5;
    params.woccupied=1.2;
    params.wqueue=1;
    params.wtime=1;

    nofSpots=104;
    initCosts();

    peak.hr = 20;
    peak.min = 00;


}


void globalPlanner::getQuery(int qval)
{
	//cout<<qval;
    qSize=qval;
    calculateFinalCosts();

}

void globalPlanner::getBestArea(int a)
{
    area=a;
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
        exitSpotCosts.push_back(nofSpots-i);
        finalSpotCosts.push_back(0);
        state.push_back(0);
        pathcosts.push_back(0);
    }
}


void globalPlanner::calculateFinalCosts()

{   
    int sum=0;
    for (std::vector<double>::iterator it =exitSpotCosts.begin(); it!=exitSpotCosts.end(); ++it)
    {
        sum+=*it;
    }

    
    for (int i=0; i< nofSpots;i++)
    {
        
            finalSpotCosts[i]= params.wexit*exitSpotCosts[i]/sum+ 
                                params.wqueue*qSize*i/100 +          
                                params.wtime * (nofSpots-i) + 
                                state[i]*INF + params.wpath* pathcosts[i];
    }


}

int globalPlanner::getBestSpot(int i,localplanner::spotsTreadCost& lplanner) //get the i'th best spot
{
    std::vector<double> costs_temp(finalSpotCosts);
    
    int j=0;
    for (std::vector<double>::iterator it=costs_temp.begin();it!=costs_temp.end();++it,j++)
    {
        int ar= spotToArea(j);
        if (ar!=area)
        {
            *it=INF;
        }
    }

    std::sort(costs_temp.begin(),costs_temp.end());
    int pos= find(finalSpotCosts.begin(),finalSpotCosts.end(),costs_temp[i]) - finalSpotCosts.begin();
    

    struct envState e=pp->spotIDtoCoord(pos);

    lplanner.request.goal.pose.position.x=e.x;
    lplanner.request.goal.pose.position.y=e.y;
    lplanner.request.goal.pose.orientation.x=0;
    lplanner.request.goal.pose.orientation.y=0;
    lplanner.request.goal.pose.orientation.z=0;
    lplanner.request.goal.pose.orientation.w=1;

    lplanner.request.start.pose.position.x=2.5;
    lplanner.request.start.pose.position.y=2;
    lplanner.request.start.pose.orientation.x=0;
    lplanner.request.start.pose.orientation.y=0;
    lplanner.request.start.pose.orientation.z=0;
    lplanner.request.start.pose.orientation.w=1;

    return pos;

}

int globalPlanner::spotToArea(int pos)
{
    struct envState e=pp->spotIDtoCoord(pos);
    struct envState max=pp->spotIDtoCoord(nofSpots-1);
    
    double Xmax=max.x;
    double Ymax=max.y;

    int ar;
    
    if (e.x< Xmax/2)
    {
         ar = e.y< Ymax/2 ? 0 : 1;
    }

    else
    {
         ar= e.y <Ymax/2 ? 2 : 3;
    }

    return ar;
}

void globalPlanner::getPathCosts(int i, float pathcost)
{
    pathcosts[i]=pathcost;
}

void openFile(std::ifstream& file, std::string filename)
{
    while(true)
    {
        file.open(filename.c_str());
        if(file.is_open())
        {
            break;
        }
        else
        {
            std::cerr<<"No such file exists with name "<< filename << std::endl;
            file.clear();
            // ros::Duration(0.1).sleep(); //sleep coz blaaahhhhh
        } 

    }
}

void globalPlanner::useCache()
{   
    double num=0;
   
    std::ifstream costFile;
    std::string s;
    //std::string cpath = ros::package::getPath("gplanner");
    //openFile(costFile,cpath+"/costs.txt");
    
    openFile(costFile,"/home/shivam/catkin_ws/src/gplanner/costs.txt");

    //("costs.txt", std::ios::in);
    //costFile.clear();
    //costFile.seekg(0, ios::beg);
    cout<<"Cached  Costs"<<endl;
    int i=0;
    while(getline(costFile,s))
    {   
        std::stringstream ss(s);
        double num;     
        ss>>num;
        exitSpotCosts[i]=num;
        i++;

        cout<<exitSpotCosts[i]<<endl;
    }


}


int globalPlanner::returnFinalSpot()
{
    auto spot =std::min_element(std::begin(finalSpotCosts),std::end(finalSpotCosts));
    
    int i= std::distance(std::begin(finalSpotCosts),spot);

    state[i]=1;

    if (i!=0 || i!=nofSpots)
        {
            state[i-1]+=0.00001*params.woccupied;
            state[i+1]+=0.00001*params.woccupied;}
    return i;

}

void globalPlanner::normalize(std::vector<double>& v)
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

envState globalPlanner::returnConfig(int i)
{
    struct envState e=pp->spotIDtoCoord(i);
    //cout<<" GOALS SENT "<<e.x<<" "<<e.y<<" "<<e.y;
    return e;
}

void globalPlanner::timeUpdate(const worldtime::timemsg::ConstPtr& msg)
{
    // param.wtime

    int difference = abs( (msg->hr - peak.hr)*60 + (msg->min - peak.min));  
    params.wtime= 60.0/difference;


}
