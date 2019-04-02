#include <ros/ros.h>
#include <math.h>
#include <iostream>

#include "mz_wang_intern/UpdateGoal.h"
#include "mz_wang_intern/GetPlan.h"
class Agent
{
  public:
    Agent(ros::NodeHandle n) : nh_(n)
    {
        // initialize publisher, service and subscriber
        update_goal_srv_ = nh_.advertiseService("update_goal", &Agent::UpdateGoalCallback, this);
        get_plan_client_ = nh_.serviceClient<mz_wang_intern::GetPlan>("/path_planner/get_plan");
        position_pub_ = nh_.advertise<geometry_msgs::Pose>("agent_feedback", 10);
        // get serial ID and start position from rosparam
        nh_.getParam("serial_ID", serial_id_);
        nh_.getParam("start_position", start_position_);
    }

    bool UpdateGoalCallback(mz_wang_intern::GetPlan::Request &req, mz_wang_intern::GetPlan::Response &res)
    {
        mz_wang_intern::GetPlan plan_srv;
        plan_srv.request.serial_id = "1";
        plan_srv.request.goal.position.x = 0;
        plan_srv.request.goal.position.y = 0;
        if (get_plan_client_.call(plan_srv))
        {
            ROS_INFO("Update goal service Called");
            if (plan_srv.response.success)
            {
                ROS_INFO("get path");
                res.success = true;
            }
            else
            {
                ROS_INFO("Cannot get path from planner.");
                return false;
            }
        }
        else
        {
            ROS_INFO("Failed call get plan service.");
            return false;
        }
        return true;
    }

  private:
    ros::NodeHandle nh_;
    ros::ServiceServer update_goal_srv_;
    ros::ServiceClient get_plan_client_;
    ros::Publisher position_pub_;

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