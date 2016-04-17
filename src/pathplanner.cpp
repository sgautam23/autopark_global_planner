#include <pathplanner.h>
#include <string.h>
#include <fstream>
#include <iostream>

using namespace std;

float round2(float num)
{
    num= float( num*100 + 0.5) /100.00;
}

d2Exitplanner::d2Exitplanner(const char* envName, const char* motPrim)
{
    createFootprint();
    

    envCfgFilename=strdup(envName);
    motPrimFilename=strdup(motPrim);

    initializeEnv();

    start.x= 46.0;
    start.y=40.0;
    start.th=0;
    populateGoals();

}

void d2Exitplanner::createFootprint()
{
    sbpl_2Dpt_t pt_m;
    double halfwidth = 0.01;
    double halflength = 0.01;
    pt_m.x = -halflength;
    pt_m.y = -halfwidth;
    perimeter.push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = -halfwidth;
    perimeter.push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = halfwidth;
    perimeter.push_back(pt_m);
    pt_m.x = -halflength;
    pt_m.y = halfwidth;
    perimeter.push_back(pt_m);
}
 
void d2Exitplanner::initializeEnv()
{   
     
    env = new EnvironmentNAVXYTHETALAT();
    
    if (!env->InitializeEnv(envCfgFilename, perimeter, motPrimFilename)) 
    {
        printf("ERROR: InitializeEnv failed\n");
        throw SBPL_Exception();
    }
}
 
// void d2Exitplanner::initializePlanner(SBPLPlanner*& planner,int start_id, int goal_id,
//                        double initialEpsilon, 
//                        bool bsearchuntilfirstsolution)
// {

//     bool bsearch = false;
//     planner = new ARAPlanner(&env, bsearch);
 
//     // set planner properties
//     if (planner->set_start(start_id) == 0) {
//         printf("ERROR: failed to set start state\n");
//         throw new SBPL_Exception();
//     }
//     if (planner->set_goal(goal_id) == 0) {
//         printf("ERROR: failed to set goal state\n");
//         throw new SBPL_Exception();
//     }
//     planner->set_initialsolution_eps(initialEpsilon);
//     planner->set_search_mode(bsearchuntilfirstsolution);
// }
 


void d2Exitplanner :: initializePlanner(EnvironmentNAVXYTHETALAT* env)
{
  bool bforwardsearch=true;
  planner = new ARAPlanner(env, bforwardsearch);
  //get start and goal from env
  MDPConfig MDPCfg;

  // initialize MDP info
  if (!env->InitializeMDPCfg(&MDPCfg)) {
    printf("ERROR: InitializeMDPCfg failed\n");
    throw new SBPL_Exception();
  }
  // set planner properties
  if (planner->set_start(MDPCfg.startstateid) == 0) {
    printf("ERROR: failed to set start state\n");
    throw new SBPL_Exception();
  }
  if (planner->set_goal(MDPCfg.goalstateid) == 0) {
    printf("ERROR: failed to set goal state\n");
    throw new SBPL_Exception();
  }
  planner->set_initialsolution_eps(initialEpsilon);
  planner->set_search_mode(bsearchuntilfirstsolution);
}


bool d2Exitplanner::plan()
{   
    
    int start_id;
    int goal_id;
    int bRet;
    initializePlanner(env);

    ofstream costFile("costs.txt");

    for (int i =0; i <goal.size(); i++)
    {   
        if( i==91)
        {
            spotCosts.push_back(450);
            continue;
        }

        std::cout<<"Running planner for spot: "<<i+1<<endl;
        std::vector<int> solution_stateIDs;
        

         start_id = env->SetStart(start.x, start.y, start.th); 
         goal_id = env->SetGoal(goal[i].x, goal[i].y, goal[i].th);

         // initializePlanner(ARAplanner, start_id,goal_id,initialEpsilon,bsearchuntilfirstsolution);

         if (planner->set_start(start_id) == 0) {
            printf("ERROR: Unable to set planner start id\n");
            throw new SBPL_Exception();
        }

        if (planner->set_goal(goal_id) == 0) {
            printf("ERROR: Unable to set planner start id\n");
            throw new SBPL_Exception();
        }

         bRet = planner->replan(allocated_time_secs, &solution_stateIDs);

         if (bRet)
         {
            spotCosts.push_back(solution_stateIDs.size());
         }
         else
        {
            spotCosts.push_back(-1);
        }
        
        std::cout<<"Cost for spot "<<i<<" "<<spotCosts[i]<<endl;
        costFile<<spotCosts[i]<<endl;
        reinitEnvironment();
        reinitPlanner();
        

    }
    costFile.close();
    return true;
}


 void d2Exitplanner::populateGoals ()
{
    struct envState pt1;
    pt1.x=6.25;
    pt1.y=1.75;
    pt1.th=0;

    cout<<"Populating Goals";
    for(int i = 0; i <8; i++)
    {   
        pt1.x=6.25;
     
        for (int j=0;j<13;j++)
        {   
            cout<<pt1.x<<" "<<pt1.y<<endl;
            goal.push_back(pt1);
            pt1.x+= 2.50;

        }

    pt1.y += i%2==0 ?8.50 : 3.50;
    //pt1.y=round2(pt1.y);

    }

    cout<<"Goals Populated";
        //env.GetCoordFromState(int stateID, int& x, int& y, int& theta) const;

}

std::vector<double> d2Exitplanner::getSpotCosts()
{
    return spotCosts;
}

void d2Exitplanner::reinitPlanner()
    {   
        printf("Re-Intializing Planner...\n");
        delete planner;
        initializePlanner(env);

    }

void d2Exitplanner::reinitEnvironment()
{
    delete env;
    initializeEnv();
}

envState d2Exitplanner::spotIDtoCoord(int i)
{
    envState e;

    cout<<" GOALS SENT "<<goal[i].x<<" "<<goal[i].y<<" "<<goal[i].y;
    
    e.x=goal[i].x;
    e.y=goal[i].y;
    e.th=goal[i].th;
    cout<<" GOALS SENT "<<e.x<<" "<<e.y<<" "<<e.y;

    return e;
    
}
