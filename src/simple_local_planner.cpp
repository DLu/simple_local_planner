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
        private_nh.param("forward_angle", forward_angle_, 0.0);
        private_nh.param("base_frame", base_frame_, std::string("/base_footprint"));
        private_nh.param("scale_trans", scale_trans_, 1.0);
        private_nh.param("scale_rot", scale_rot_, 1.0);
        pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("target_pose", 5);
        
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
                    dist < p_precision_ && fabs(gth) < o_precision_)
                || (plan_index_ < n-1 && 
                    dist < p_window_ && fabs(gth) < o_window_)){
                plan_index_++;    
            }else{
                break;
            }            
        }
        
        if( plan_index_ >= n ){
            return true;
        }
        pub_.publish(global_plan_[plan_index_]);

        getTransformedPosition(global_plan_[plan_index_], gx, gy, gth);  
        
        gx *= scale_trans_;
        gy *= scale_trans_;
        gth *= scale_rot_;   
               
        if(fabs(gx) > max_trans_vel_){
            gx = copysign(max_trans_vel_, gx);
        }
        if(fabs(gy) > max_trans_vel_){
            gy = copysign(max_trans_vel_, gy);
        }
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
        theta = tf::getYaw(ps.pose.orientation)-forward_angle_;
    }
    
        std::vector<geometry_msgs::PoseStamped> global_plan_;
        ros::Publisher pub_;
        tf::TransformListener* tf_;
        std::string base_frame_;
        int plan_index_;
        double max_trans_vel_, max_rot_vel_;
        double p_window_, o_window_, p_precision_, o_precision_;
        double forward_angle_;
        double scale_trans_, scale_rot_;
        
     
};
};

PLUGINLIB_EXPORT_CLASS(simple_local_planner::SimpleLocalPlanner, nav_core::BaseLocalPlanner)
