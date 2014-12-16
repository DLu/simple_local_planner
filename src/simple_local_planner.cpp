#include <nav_core/base_local_planner.h>

namespace simple_local_planner {
class SimpleLocalPlanner : public nav_core::BaseLocalPlanner {
    public:
    
    void initialize(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros){
              
    }
    
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
        return true;
    }
    
    bool isGoalReached(){
        return false;
    }
    
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel){ 
        return false;
    }
        
};
};
