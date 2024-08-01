#ifndef PID_PLANNER_H_
#define PID_PLANNER_H_

#include "ros/forwards.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <tf2/utils.h>

namespace pid_planner
{

    class PIDPlanner
    {
    public:
        PIDPlanner();
        ~PIDPlanner();
        void init(ros::NodeHandle &nh);
        void pointHandler(const geometry_msgs::Point::ConstPtr &recv_point);
        void setGoal(const geometry_msgs::PoseStampedPtr &msg);
        // void subOdom(const geometry_msgs::PoseStampedPtr &msg);
        void subOdom(const nav_msgs::OdometryPtr &msg);        
        void isGoalReached();
        void computeVelocityCommands(double local_target_x_, double local_target_y_);
        void regularizeAngle(double &angle);
        double LinearPIDController(nav_msgs::Odometry &base_odometry, double b_x_d, double b_y_d);
        double AngularPIDController(nav_msgs::Odometry &base_odometry, double theta_d, double theta);
        double getGoalPositionDistance(double x_g, double y_g, double x, double y);
        void runCallback(const ros::TimerEvent &e);
        void debug_error();

    private:
        bool init_run_flag;
        bool initialized_, goal_reached_;

        double x_, y_, theta_;
        double p_window_, o_window_;
        double p_precision_, o_precision_;
        double controller_freqency_, d_t_;
        double max_v_, min_v_, max_v_inc_;
        double max_w_, min_w_, max_w_inc_;
        double k_v_p_, k_v_i_, k_v_d_;
        double k_w_p_, k_w_i_, k_w_d_;
        double k_theta_;

        double e_v_, e_w_;
        double i_v_, i_w_;

        base_local_planner::OdometryHelperRos *odom_helper_;
        ros::Publisher vel_pub_;
        ros::Publisher marker_pub;
        ros::Subscriber target_point_sub_, goal_sub_, odom_sub_;
        ros::Timer runtime_timer_;
        nav_msgs::Odometry current_rb_odom;
        nav_msgs::Odometry twist_odom;

        double goal_x_, goal_y_;
        double cur_target_x_, cur_target_y_;

        bool one_point_move_test;

        double swarm_relative_pts_[10][3];
        int id;
        // debug pid tracking error
        double tracking_error_sum, tracking_error_mean;
        int error_record_num;
        ros::Timer timer_debug_pid;
    };
}

void pid_planner::PIDPlanner::debug_error()
{

    if ((odom_sub_.getNumPublishers() != 0) && (target_point_sub_.getNumPublishers() != 0))
    {
        error_record_num++;
        double dis =
            getGoalPositionDistance(cur_target_x_, cur_target_y_, current_rb_odom.pose.pose.position.x, current_rb_odom.pose.pose.position.y);
        tracking_error_sum += dis;
        tracking_error_mean = tracking_error_sum / error_record_num;
        ///        std::cout<<std::endl<<"====================="<<std::endl;
        //        std::cout<<"[realtime_pid_tracking_error]:"<<dis<<std::endl;
        //        std::cout<<"[mean_pid_tracking_error]:"<<tracking_error_mean<<std::endl;
        //        std::cout<<std::endl<<"====================="<<std::endl;
    }
}

#endif
