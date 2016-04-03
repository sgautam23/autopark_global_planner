#include <pathplanner.h>



d2Exitplanner::d2Exitplanner()
{
    createFootprint();


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
 
int runPlanner(SBPLPlanner* planner, int allocated_time_secs, 
               vector<int>&solution_stateIDs){
    int bRet = planner->replan(allocated_time_secs, &solution_stateIDs);
 
    if (bRet) 
        printf("Solution is found\n");
    else 
        printf("Solution does not exist\n");
    return bRet;
}