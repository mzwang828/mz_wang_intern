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
        get_plan_server_ = nh_.advertiseService("get_plan", &PathPlanner::GetPlanCallback, this);
        edge_cost_ = 10;
    }

    // calculate cost G for target node
    int calcG(mz_wang_intern::Node current, mz_wang_intern::Node target)
    {
        return current.G + edge_cost_;
    }
    // calculate cost H for target node, manhattan distance is used here
    int calcH(mz_wang_intern::Node target, mz_wang_intern::Node goal)
    {
        int H = edge_cost_ * abs(goal.position.x-target.position.x)+abs(goal.position.y-target.position.y);
        return H;
    }
    // calculate total cost F
    int calcF(mz_wang_intern::Node target)
    {
        return target.H + target.G;
    }
    // find the node with the least F from the open list, which will be visited as next step 
    mz_wang_intern::Node GetNextNode()
    {
        if(!open_list_.empty())
        {
            mz_wang_intern::Node next_node = open_list_.front();
            for (auto node:open_list_)
            {
                if (node.F < next_node.F)
                    next_node = node;
            }
            return next_node;
        }
        // if the open list is already empty
        return NULL;
    }
    // check if one node is in the list(open or close)
    mz_wang_intern::Node isInList(std::list<mz_wang_intern::Node> list, mz_wang_intern::Node target)
    {
        for (auto node:list)
        {
            if (node.position.x == target.position.x && node.position.y == target.position.y)
                return node;
        }
        return NULL;
    }
    // check is the target node is reachable
    bool isReachable(mz_wang_intern::Node target)
    {
        // return false if the target node is outside of the map, is occupied by path of another agent, or is in the closed list
        if(target.position.x < 0 || target.position.x > 10 || target.position.y < 0 || target.position.y > 10
            || map[target.position.x][target.position.y] == 1 || isInlist(close_list_, target))
            return false;
        else
            return true;
    }
    // get 4 connected nodes for a given node
    std::vector<mz_wang_intern::Node> GetConnectedNodes(mz_wang_intern::Node current)
    {   
        std::vector<mz_wang_intern::Node> connected_nodes;
        for (int i = -1; i <=1; i = i+2)
        {
            // for each iteration, check the reachability of two connected nodes
            mz_wang_intern::Node node_x, node_y;
            node_x.position.x = current.position.x + i;
            node_x.position.y = current.position.y;
            node_y.position.x = current.position.x;
            node_y.position.y = current.position.y + i;
            if (isReachable(node_x))
                connected_nodes.push_back(node_x);
            if (isReachable(node_y))
                connected_nodes.push_back(node_y);
        }
        return connected_nodes;
    }
    // find optimal path using A*
    mz_wang_intern::Node FindPath(mz_wang_intern::Node &start, mz_wang_intern::Node &goal)
    {
        // add the start node to the open list 
        open_list_.push_back(start);
        while(!open_list_.empty())
        {
            /*
            Find the node in open list with least F, which will be visited in next step
            Remove that node from open list and add it to close list
            Now visit the next node
            */
            mz_wang_intern::Node next_node = GetNextNode();
            open_list_.remove(next_node);
            close_list_.push_back(next_node);
            /*
            Find the 4 connected nodes to the new node
            For each connected node, check if it is alread in the open list
            If no, add that node to the open list
            If yes, compare its cost G with new calculated cost G from current node and rewire the route
            */
            std::vector<mz_wang_intern::Node> connected_nodes = GetConnectedNodes(next_node);


        }
    }
    bool GetPlanCallback(mz_wang_intern::GetPlan::Request &req, mz_wang_intern::GetPlan::Response &res)
    {
        res.success = true;
        ROS_INFO("Get Plan Service Called");
        return true;
    }

  private:
    ros::NodeHandle nh_;
    ros::ServiceServer get_plan_server_;
    int edge_cost_;
    // use a 2D matrix to represent the grid map
    std::vector<std::vector<int> > map;
    // open list to save nodes to be visited
    std::list<mz_wang_intern::Node> open_list_;
    // close list to save nodes that have been visited
    std::list<mz_wang_intern::Node> close_list_;
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