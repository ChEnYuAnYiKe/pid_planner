#include "pid_planner.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include <visualization_msgs/Marker.h>

// 代码的作用：给定一个终点，在到达终点前，控制器不会停止，一直输出控制量。终点的订阅话题：/move_base_simple/goal （对应第100行代码）
//	     在到达终点前，可以依次指定需要经过的轨迹点。订阅的话题：point_tracking （对应第95代码）

// test
namespace pid_planner
{
    // init argument
    PIDPlanner::PIDPlanner()
        : init_run_flag(false), initialized_(false), goal_reached_(false),
          goal_x_(0.0), goal_y_(0.0), tracking_error_sum(0.0), tracking_error_mean(0.0),
          error_record_num(0)
    {
        // 线速度和角度的初始化
        twist_odom.twist.twist.linear.x = 0.0;
        twist_odom.twist.twist.angular.z = 0.0;
    }

    PIDPlanner::~PIDPlanner()
    {
    }

    void PIDPlanner::init(ros::NodeHandle &nh)
    {
        // Run Only Once
        if (!initialized_)
        {
            initialized_ = true;

            nh.param("p_window", p_window_, 0.2);
            nh.param("o_window", o_window_, 1.0);

            nh.param("p_precision", p_precision_, 0.2);
            nh.param("o_precision", o_precision_, 0.5);

            // max and min linear velocity
            nh.param("max_v", max_v_, 0.5);
            nh.param("min_v", min_v_, 0.0);
            nh.param("max_v_inc", max_v_inc_, 0.5); // maximum linear increase per time

            // max and min angular velocity
            nh.param("max_w", max_w_, 1.57);
            nh.param("min_w", min_w_, 0.0);
            nh.param("max_w_inc", max_w_inc_, 1.57); // maximum angular increase per time

            // the P,I,D coefficient of linear velocity
            nh.param("k_v_p", k_v_p_, 1.00);
            nh.param("k_v_i", k_v_i_, 0.01);
            nh.param("k_v_d", k_v_d_, 0.10);

            // the P,I,D coefficient of angular velocity
            nh.param("k_w_p", k_w_p_, 1.00);
            nh.param("k_w_i", k_w_i_, 0.01);
            nh.param("k_w_d", k_w_d_, 0.10);

            // 无人车当前位置到目标位置和目标位置到下一个目标位置两者分别的权重
            // not used
            nh.param("k_theta", k_theta_, 0.5);

            // test code
            nh.param("one_point_move_test", one_point_move_test, false);

            // control frequency
            nh.param("controller_frequency", controller_freqency_, 10.0);
            d_t_ = 1 / controller_freqency_;

            nh.param("tb_id", id, -1);
            // for (int i = 0; i < 4; i++)
            // {
            //     nh.param("global_goal/relative_pos_" + std::to_string(i) + "/x", swarm_relative_pts_[i][0], -1.0);
            //     nh.param("global_goal/relative_pos_" + std::to_string(i) + "/y", swarm_relative_pts_[i][1], -1.0);
            //     nh.param("global_goal/relative_pos_" + std::to_string(i) + "/z", swarm_relative_pts_[i][2], -1.0);
            // }

            // ROS_WARN("==== id:%d, x: %.2lf,y: %.2lf ====", id, swarm_relative_pts_[id][0], swarm_relative_pts_[id][1]);
            //
            e_v_ = i_v_ = 0.0;
            e_w_ = i_w_ = 0.0;

            // 输出控制速度 /robot_x/cmd_vel
            vel_pub_ = nh.advertise<geometry_msgs::Twist>("rb_vel", 10);

            // 订阅里程计话题
            odom_sub_ = nh.subscribe("odom", 50, &PIDPlanner::subOdom, this);

            // 订阅轨迹点 若one_point_move_test==true则不订阅，使用全局目标跟踪
            if (!one_point_move_test)
            {
                target_point_sub_ = nh.subscribe<geometry_msgs::Point>("point_tracking", 50, &PIDPlanner::pointHandler, this);
            }

            // 订阅总目标
            goal_sub_ = nh.subscribe("global_goal", 50, &PIDPlanner::setGoal, this);

            // 显示目标点
            marker_pub = nh.advertise<visualization_msgs::Marker>("local_goal_vis", 1);
            runtime_timer_ = nh.createTimer(ros::Duration(0.05), &PIDPlanner::runCallback, this, false, false);

            timer_debug_pid = nh.createTimer(ros::Duration(0.2), [&](const ros::TimerEvent &)
                                             {
                                                 // debug_error();
                                             },
                                             false, false);

            ROS_INFO("PID planner initialized!");
        }
        else
        {
            ROS_WARN("PID planner has already been initialized.");
        }
    }

    // void PIDPlanner::subOdom(const nav_msgs::OdometryPtr &msg)
    // {
    //     current_rb_odom.pose.pose.position.x = msg->pose.position.x;
    //     current_rb_odom.pose.pose.position.y = msg->pose.position.y;
    //     current_rb_odom.pose.pose.position.z = 0.0;
    //     current_rb_odom.pose.pose.orientation.x = msg->pose.orientation.x;
    //     current_rb_odom.pose.pose.orientation.y = msg->pose.orientation.y;
    //     current_rb_odom.pose.pose.orientation.z = msg->pose.orientation.z;
    //     current_rb_odom.pose.pose.orientation.w = msg->pose.orientation.w;
    //     // ROS_WARN("sub odom");
    // }

    void PIDPlanner::subOdom(const nav_msgs::OdometryPtr &msg)
    {
        current_rb_odom=*msg;
    }

    void PIDPlanner::pointHandler(const geometry_msgs::Point::ConstPtr &recv_point)
    {
        cur_target_x_ = recv_point->x;
        cur_target_y_ = recv_point->y;

        if (init_run_flag)
        {
            if (getGoalPositionDistance(cur_target_x_, cur_target_y_, current_rb_odom.pose.pose.position.x, current_rb_odom.pose.pose.position.y) > 0.1)
            {
                runtime_timer_.start();
                init_run_flag = false;
                ROS_WARN("start");
                timer_debug_pid.start();
            }
        }

        // show local point
        [&]()
        {
            visualization_msgs::Marker point;
            point.header.frame_id = "world";
            point.header.stamp = ros::Time::now();
            point.id = 0;
            point.type = visualization_msgs::Marker::SPHERE;

            point.action = visualization_msgs::Marker::ADD;

            point.pose.position.x = recv_point->x;
            point.pose.position.y = recv_point->y;
            point.pose.position.z = 0;
            point.pose.orientation.x = 0.0;
            point.pose.orientation.y = 0.0;
            point.pose.orientation.z = 0.0;
            point.pose.orientation.w = 1.0;

            point.scale.x = 0.1;
            point.scale.y = 0.1;
            point.scale.z = 0.1;

            point.color.r = 0.0f;
            point.color.g = 1.0f;
            point.color.b = 0.0f;
            point.color.a = 1.0;

            point.lifetime = ros::Duration();

            marker_pub.publish(point);
        }();
    }

    void PIDPlanner::setGoal(const geometry_msgs::PoseStampedPtr &msg)
    {
        goal_x_ = msg->pose.position.x;
        goal_y_ = msg->pose.position.y;

        ROS_INFO("got new goal");

        init_run_flag = true;

        e_v_ = i_v_ = 0.0;
        e_w_ = i_w_ = 0.0;

        if (one_point_move_test)
        {
            cur_target_x_ = goal_x_;
            cur_target_y_ = goal_y_;

            runtime_timer_.start();
            init_run_flag = false;
            ROS_WARN("start");

            timer_debug_pid.start();
        }
    }

    void PIDPlanner::isGoalReached()
    {
        if (goal_reached_)
        {
            runtime_timer_.stop();
            timer_debug_pid.stop();
        }
    }

    void PIDPlanner::runCallback(const ros::TimerEvent &e)
    {

        computeVelocityCommands(cur_target_x_, cur_target_y_);
    }

    void PIDPlanner::computeVelocityCommands(double local_target_x_, double local_target_y_)
    {

        // 机器人当前状态
        x_ = current_rb_odom.pose.pose.position.x;
        y_ = current_rb_odom.pose.pose.position.y;
        theta_ = tf2::getYaw(current_rb_odom.pose.pose.orientation);

        double theta_d, theta_dir; /*theta_trj*/
        ;
        double b_x_d, b_y_d;
        double e_theta;

        double x_d = local_target_x_;
        double y_d = local_target_y_;

        // 期望的位置和偏差的角度
        theta_dir = atan2((y_d - y_), (x_d - x_));
        b_x_d = x_d;
        b_y_d = y_d;

        //...省略了融合下个目标点的角度

        theta_d = theta_dir;
        regularizeAngle(theta_d);

        e_theta = theta_d - theta_;
        regularizeAngle(e_theta);

        // debug
        // ROS_WARN("===============");
        // ROS_WARN("vel_x:%f,ang_z:%f",current_rb_odom.twist.twist.linear.x, current_rb_odom.twist.twist.angular.z);
        // ROS_INFO("%f < %f",getGoalPositionDistance(goal_x_,goal_y_, x_, y_),p_precision_);

        // odom_helper_->getOdom(current_rb_odom);
        geometry_msgs::Twist cmd_vel;
        if (getGoalPositionDistance(goal_x_, goal_y_, x_, y_) < p_precision_)
        {
            e_theta = 0.0 - theta_;
            regularizeAngle(e_theta);

            // if (std::fabs(e_theta) < o_precision_)
            if (true)
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;

                goal_reached_ = true;
                // ROS_ERROR("STOP");
                this->isGoalReached();
            }
            // orientation not reached
            else
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                cmd_vel.angular.z = AngularPIDController(current_rb_odom, 0, theta_);

                // std::cout<<"adjust angle!!"<<std::endl;
            }
        }
        else if (std::fabs(e_theta) > 0.5)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = AngularPIDController(twist_odom, theta_d, theta_);
            // ROS_INFO("angle big");
        }
        else if (getGoalPositionDistance(local_target_x_, local_target_y_, x_, y_) < p_window_ - 0.05)
        {
            // cmd_vel.linear.x = 0.0;//aaa
            // cmd_vel.angular.z = AngularPIDController(current_rb_odom, theta_d, theta_);
            // ROS_INFO("Too close to the target");
        }
        else
        {
            cmd_vel.linear.x = LinearPIDController(twist_odom, b_x_d, b_y_d);
            cmd_vel.angular.z = AngularPIDController(twist_odom, theta_d, theta_);
        }

        // ROS_WARN("[debug]:angular.z=%f",cmd_vel.angular.z);
        vel_pub_.publish(cmd_vel);

        ROS_WARN("vel_x:%f,ang_z:%f", cmd_vel.linear.x, cmd_vel.angular.z);
    }

    double PIDPlanner::LinearPIDController(nav_msgs::Odometry &base_odometry, double b_x_d, double b_y_d)
    {
        double v = base_odometry.twist.twist.linear.x;
        // ROS_WARN("x:%.2lf,y:%.2lf",base_odometry.twist.twist.linear.x, base_odometry.twist.twist.linear.y);

        // 距离越大，期望的速度越大
        double v_d = std::hypot(b_x_d, b_y_d) / d_t_;
        if (std::fabs(v_d) > max_v_)
            v_d = std::copysign(max_v_, v_d);
        // ROS_WARN("v_d: %.2f", v_d);

        double e_v = v_d - v;
        i_v_ += e_v * d_t_;
        double d_v = (e_v - e_v_) / d_t_;
        e_v_ = e_v;

        // v_inc <---> v increase
        double v_inc = k_v_p_ * e_v + k_v_i_ * i_v_ + k_v_d_ * d_v;

        if (std::fabs(v_inc) > max_v_inc_)
            v_inc = std::copysign(max_v_inc_, v_inc);

        double v_cmd = v + v_inc;
        if (std::fabs(v_cmd) > max_v_)
            v_cmd = std::copysign(max_v_, v_cmd);
        else if (std::fabs(v_cmd) < min_v_)
            v_cmd = std::copysign(min_v_, v_cmd);

        // ROS_INFO("v_d: %.2lf, e_v: %.2lf, i_v: %.2lf, d_v: %.2lf, v_cmd: %.2lf", v_d, e_v, i_v_, d_v, v_cmd);
        // ROS_INFO("v: %.2lf, v_inc: %.2lf, v_cmd: %.2lf", v, v_inc, v_cmd);

        twist_odom.twist.twist.linear.x = v_cmd;
        return v_cmd;
    }

    double PIDPlanner::AngularPIDController(nav_msgs::Odometry &base_odometry, double theta_d, double theta)
    {
        double e_theta = theta_d - theta;
        /// ROS_WARN("theta_d: %.2lf,theta: %.2lf",theta_d,theta);
        regularizeAngle(e_theta);

        double w_d = e_theta / d_t_;
        if (std::fabs(w_d) > max_w_)
            w_d = std::copysign(max_w_, w_d);
        // ROS_WARN("w_d: %.2f", w_d);

        double w = base_odometry.twist.twist.angular.z;
        double e_w = w_d - w;
        i_w_ += e_w * d_t_;
        double d_w = (e_w - e_w_) / d_t_;
        e_w_ = e_w;

        double w_inc = k_w_p_ * e_w + k_w_i_ * i_w_ + k_w_d_ * d_w;

        if (std::fabs(w_inc) > max_w_inc_)
            w_inc = std::copysign(max_w_inc_, w_inc);

        double w_cmd = w + w_inc;
        if (std::fabs(w_cmd) > max_w_)
            w_cmd = std::copysign(max_w_, w_cmd);
        else if (std::fabs(w_cmd) < min_w_)
            w_cmd = std::copysign(min_w_, w_cmd);

        // ROS_INFO("w_d: %.2lf, e_w: %.2lf, i_w: %.2lf, d_w: %.2lf, w_cmd: %.2lf", w_d, e_w, i_w_, d_w, w_cmd);
        // ROS_INFO("w_d: %.2lf, w: %.2lf,w_inc: %.2lf,  w_cmd: %.2lf", w_d, w,w_inc, w_cmd);
        w_cmd = k_w_p_ * w_d;
        twist_odom.twist.twist.angular.z = w_cmd;
        return w_cmd;
    }

    double PIDPlanner::getGoalPositionDistance(double x_g, double y_g, double x, double y)
    {
        return std::hypot(x - x_g, y - y_g);
    }

    // 将角度转换为[-pi,pi]
    void PIDPlanner::regularizeAngle(double &angle)
    {
        angle = angle - 2.0 * M_PI * std::floor((angle + M_PI) / (2.0 * M_PI));
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pid_planner");
    ros::NodeHandle nh("~");

    pid_planner::PIDPlanner point_tracking;
    point_tracking.init(nh);

    ros::spin();
    return 0;
}
