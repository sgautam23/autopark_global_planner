#include <pathplanner.h>
#include <string.h>


d2Exitplanner::d2Exitplanner(const char* envName, const char* motPrim)
{
    createFootprint();
    populateGoals();

    envCfgFilename=strdup(envName);
    motPrimFilename=strdup(motPrim);

    initializeEnv();

    start.x= 2;
    start.y=2;
    start.th=0;

}

void d2Exitplanner::createFootprint()
{
    sbpl_2Dpt_t pt_m;
    double halfwidth = 0.5;
    double halflength = 0.5;
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
    if (!env.InitializeEnv(envCfgFilename, perimeter, motPrimFilename)) 
    {
        printf("ERROR: InitializeEnv failed\n");
        throw SBPL_Exception();
    }
}
 
// void d2Exitplanner::setEnvStartGoal(EnvironmentNAVXYTHETALAT& env, 
//                      double start_x, double start_y, double start_theta,
//                      double goal_x, double goal_y, double goal_theta, 
//                      int& start_id, int& goal_id){
 
//     start_id = env.SetStart(start_x, start_y, start_theta);
//     goal_id = env.SetGoal(goal_x, goal_y, goal_theta);
// }
 
void d2Exitplanner::initializePlanner(SBPLPlanner*& planner,int start_id, int goal_id,
                       double initialEpsilon, 
                       bool bsearchuntilfirstsolution){
    // work this out later, what is bforwardsearch?
    bool bsearch = false;
    planner = new ADPlanner(&env, bsearch);
 
    // set planner properties
    if (planner->set_start(start_id) == 0) {
        printf("ERROR: failed to set start state\n");
        throw new SBPL_Exception();
    }
    if (planner->set_goal(goal_id) == 0) {
        printf("ERROR: failed to set goal state\n");
        throw new SBPL_Exception();
    }
    planner->set_initialsolution_eps(initialEpsilon);
    planner->set_search_mode(bsearchuntilfirstsolution);
}
 

bool d2Exitplanner::plan()
{
    int start_id = env.SetStart(start.x, start.y, start.th); 
    int goal_id;
    int bRet;
    for (int i =0; i <goal.size(); i++)
    {
        std::vector<int> solution_stateIDs;
        SBPLPlanner* ARAplanner=NULL ;

         goal_id = env.SetGoal(goal[i].x, goal[i].y, goal[i].th);

         initializePlanner(ARAplanner, start_id,goal_id,initialEpsilon,bsearchuntilfirstsolution);

         bRet = ARAplanner->replan(allocated_time_secs, &solution_stateIDs);

         if (bRet)
         {
            spotCosts.push_back(solution_stateIDs.size());
         }
         else
        {
            spotCosts.push_back(-1);
        }
        
        std::cout<<spotCosts[i];

        delete ARAplanner;

    }
}


 void d2Exitplanner::populateGoals ()
{
    struct envState pt1;
    pt1.x=6.25;
    pt1.y=1.75;
    pt1.th=0;

    for(int i = 0; i <8; i++)
    {   
        pt1.x=6.25;
     
        for (int j=0;j<13;j++)
        {   
            // cout<<pt1.x<<" "<<pt1.y<<endl;
            goal.push_back(pt1);
            pt1.x+=2.5;
        }

    pt1.y+= i%2==0 ?8.5 : 3.5;

    }
        //env.GetCoordFromState(int stateID, int& x, int& y, int& theta) const;

}

std::vector<int> d2Exitplanner::getSpotCosts()
{
    return spotCosts;
}
