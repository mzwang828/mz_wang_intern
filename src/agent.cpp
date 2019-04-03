#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include "mz_wang_intern/UpdateGoal.h"
#include "mz_wang_intern/GetPlan.h"
/*
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
The agent node provides 1 service and 2 publisher.
feedback_pub --- publishes the agent position
path_pub --- publishes the path return by /get_plan service
update_goal service --- calls the /get_plan service and passes the agent serial_ID and goal as request
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/
class Agent
{
  public:
    Agent(ros::NodeHandle n) : nh_(n)
    {
        // initialize publisher, service and subscriber
        update_goal_srv_ = nh_.advertiseService("update_goal", &Agent::UpdateGoalCallback, this);
        get_plan_client_ = nh_.serviceClient<mz_wang_intern::GetPlan>("/path_planner/get_plan");
        feedback_pub_ = nh_.advertise<geometry_msgs::Pose>("/agent_feedback", 10);
        path_pub_ = nh_.advertise<nav_msgs::Path>("agent_path", 10);
        // get serial ID and start position from rosparam
        nh_.getParam("serial_id", serial_id_);
        nh_.getParam("start_position", start_position_);
    }

    bool UpdateGoalCallback(mz_wang_intern::UpdateGoal::Request &req, mz_wang_intern::UpdateGoal::Response &res)
    {
        // construct and publish the start pose of the agent
        geometry_msgs::Pose start_pose;
        start_pose.position.x = start_position_[0];
        start_pose.position.y = start_position_[1];
        feedback_pub_.publish(start_pose);
        // construct the GetPlan request
        mz_wang_intern::GetPlan plan_srv;
        plan_srv.request.serial_id = serial_id_;
        plan_srv.request.goal = req.goal;
        // call the /get_plan service
        if (get_plan_client_.call(plan_srv))
        {
            // if a valid path is returned
            ROS_INFO("Successfully getting path from service /get_plan.");
            path_pub_.publish(plan_srv.response.path);
            res.success = true;
        }
        else
        {
            // if no valid path is returned or failed calling the service
            ROS_INFO("Failed getting path from service /get_plan.");
            return false;
        }
        return true;
    }

  private:
    ros::NodeHandle nh_;
    ros::ServiceServer update_goal_srv_;
    ros::ServiceClient get_plan_client_;
    ros::Publisher feedback_pub_, path_pub_;

    std::string serial_id_;
    std::vector<double> start_position_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agent");
    ros::NodeHandle n("~");

    Agent agent(n);
    ROS_INFO("Spawn Agent Succesfully");
    ros::spin();

    return 0;
}