#include <ros/ros.h>
#include <math.h>
#include <iostream>

#include "mz_wang_intern/GetPlan.h"
#include "mz_wang_intern/Node.h"

class PathPlanner
{
  public:
    PathPlanner(ros::NodeHandle n) : nh_(n)
    {
        get_plan_server_ = nh_.advertiseService("get_plan", &PathPlanner::get_plan_callback, this);
    }

    bool get_plan_callback(mz_wang_intern::GetPlan::Request &req, mz_wang_intern::GetPlan::Response &res)
    {
        res.success = true;
        ROS_INFO("Get Plan Service Called");
        return true;
    }

  private:
    ros::NodeHandle nh_;
    ros::ServiceServer get_plan_server_;

    // use a 2D matrix to represent the grid map
    std::vector<std::vector<int> > map;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle n("~");

    PathPlanner path_planner(n);
    ROS_INFO("Path Planner Initialized");
    ros::spin();

    return 0;
}