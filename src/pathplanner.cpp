#include <pathplanner.h>



d2Exitplanner::d2Exitplanner()
{
    createFootprint();

    populateGoals();

    initializeEnv();
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
    if (!env.InitializeEnv(envCfgFilename, perimeter, motPrimFilename)) {
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
 
int d2Exitplanner::runPlanner(SBPLPlanner* planner, int allocated_time_secs)
{
    int bRet = planner->replan(allocated_time_secs, &solution_stateIDs);
 
    if (bRet) 
        printf("Solution is found\n");
    else 
        printf("Solution does not exist\n");
    return bRet;
}

bool d2Exitplanner::plan()
{

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
