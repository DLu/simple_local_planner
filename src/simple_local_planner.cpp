#include <nav_core/base_local_planner.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

namespace simple_local_planner {
class SimpleLocalPlanner : public nav_core::BaseLocalPlanner {
    public:
    
    void initialize(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros){
        ros::NodeHandle private_nh("~/" + name);
        private_nh.param("max_trans_vel", max_trans_vel_, 1.0);
        private_nh.param("max_rot_vel", max_rot_vel_, 1.0);
        private_nh.param("position_window", p_window_, 0.5);
        private_nh.param("orientation_window", o_window_, 0.1);        
        private_nh.param("position_precision", p_precision_, 0.2);
        private_nh.param("orientation_precision", o_precision_, 0.05);
        private_nh.param("base_frame", base_frame_, std::string("/base_footprint"));
        tf_ = tf;
        plan_index_ = 1;
    }
    
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
        global_plan_ = orig_global_plan;
        plan_index_ = 0;
        return true;
    }
    
    bool isGoalReached(){
        return plan_index_ >= global_plan_.size();
    }
    
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
        double gx,gy,gth;
        int n = global_plan_.size();

        // advance to next goal pose
        while( plan_index_ < n ){
            getTransformedPosition(global_plan_[plan_index_], gx, gy, gth);
            double dist = hypot(gx, gy);
            
            if( (plan_index_== n-1 && 
                    dist < p_precision_ && gth < o_precision_)
                || (plan_index_ < n-1 && 
                    dist < p_window_ && gth < o_window_)){
                plan_index_++;    
            }else{
                break;
            }            
        }
        
        if( plan_index_ >= n ){
            return true;
        }

        getTransformedPosition(global_plan_[plan_index_], gx, gy, gth);     
               
        if(fabs(gth) > max_rot_vel_){
            gth = copysign(max_rot_vel_, gth);
        }
       
        cmd_vel.linear.x = gx;
        cmd_vel.linear.y = gy;
        cmd_vel.angular.z = gth;
        //ROS_INFO("%.2f %.2f %.2f       %d", gx, gy, gth, plan_index_);
                
        return true;
    }
    
    protected:
    void getTransformedPosition(geometry_msgs::PoseStamped& pose, double& x, double& y, double& theta)
    {
        geometry_msgs::PoseStamped ps;
        pose.header.stamp = ros::Time(0);
        tf_->transformPose(base_frame_, pose, ps);
        x = ps.pose.position.x;
        y = ps.pose.position.y,
        theta = tf::getYaw(ps.pose.orientation);
    }
    
        std::vector<geometry_msgs::PoseStamped> global_plan_;
        tf::TransformListener* tf_;
        std::string base_frame_;
        int plan_index_;
        double max_trans_vel_, max_rot_vel_;
        double p_window_, o_window_, p_precision_, o_precision_;
     
};
};

PLUGINLIB_EXPORT_CLASS(simple_local_planner::SimpleLocalPlanner, nav_core::BaseLocalPlanner)
