#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include "mz_wang_intern/GetPlan.h"
#include "mz_wang_intern/Node.h"
#include "mz_wang_intern/LinkedNode.h"

class PathPlanner
{
  public:
    PathPlanner(ros::NodeHandle n) : nh_(n)
    {
        get_plan_server_ = nh_.advertiseService("get_plan", &PathPlanner::GetPlanCallback, this);
        edge_cost_ = 10;
        // create the map
        for (int i; i < 11; i++)
        {
            for (int j; j < 11; j++)
                map_[i][j] = 0;
        }
    }

    // calculate cost G for target node
    int calcG(mz_wang_intern::LinkedNode current, mz_wang_intern::LinkedNode target)
    {
        return current.self.G + edge_cost_;
    }
    // calculate cost H for target node, manhattan distance is used here
    int calcH(mz_wang_intern::LinkedNode target, mz_wang_intern::LinkedNode goal)
    {
        int H = edge_cost_ * abs(goal.self.position.x - target.self.position.x) + abs(goal.self.position.y - target.self.position.y);
        return H;
    }
    // calculate total cost F
    int calcF(mz_wang_intern::LinkedNode target)
    {
        return target.self.H + target.self.G;
    }

    // find the node with the least F from the open list, which will be visited as next step
    mz_wang_intern::LinkedNode GetNextNode()
    {
        if (!open_list_.empty())
        {
            mz_wang_intern::LinkedNode next_node = open_list_.front();
            for (auto node : open_list_)
            {
                if (node.self.F < next_node.self.F)
                    next_node = node;
            }
            return next_node;
        }
        // if the open list is already empty
        return NULL;
    }

    // check if one node is in the list(open or close)
    mz_wang_intern::LinkedNode isInList(std::list<mz_wang_intern::LinkedNode> list, mz_wang_intern::LinkedNode target)
    {
        for (auto node : list)
        {
            if (node.self.position.x == target.self.position.x && node.self.position.y == target.self.position.y)
                return node;
        }
        return NULL;
    }

    // check is the target node is reachable
    bool isReachable(mz_wang_intern::LinkedNode target)
    {
        // return false if the target node is outside of the map, is occupied by path of another agent, or is in the closed list
        if (target.self.position.x < 0 || target.self.position.x > 10 || target.self.position.y < 0 || target.self.position.y > 10 || map_[target.self.position.x][target.self.position.y] == 1 || isInlist(close_list_, target))
            return false;
        else
            return true;
    }

    // get 4 connected nodes for a given node
    std::vector<mz_wang_intern::LinkedNode> GetConnectedNodes(mz_wang_intern::LinkedNode current)
    {
        std::vector<mz_wang_intern::LinkedNode> connected_nodes;
        for (int i = -1; i <= 1; i = i + 2)
        {
            // for each iteration, check the reachability of two connected nodes
            // new node is only added to the open list if reachable
            mz_wang_intern::LinkedNode node_x, node_y;
            node_x.self.position.x = current.self.position.x + i;
            node_x.self.position.y = current.self.position.y;
            node_y.self.position.x = current.self.position.x;
            node_y.self.position.y = current.self.position.y + i;
            if (isReachable(node_x))
                connected_nodes.push_back(node_x);
            if (isReachable(node_y))
                connected_nodes.push_back(node_y);
        }
        return connected_nodes;
    }

    // find optimal path using A*
    mz_wang_intern::LinkedNode FindPath(mz_wang_intern::LinkedNode &start, mz_wang_intern::LinkedNode &goal)
    {
        // add the start node to the open list
        open_list_.push_back(start);
        while (!open_list_.empty())
        {
            /*
            Find the node in open list with least F, which will be visited in next step
            Remove that node from open list and add it to close list
            Now visit the next node
            */
            mz_wang_intern::LinkedNode next_node = GetNextNode();
            open_list_.remove(next_node);
            close_list_.push_back(next_node);
            /*
            Find the 4 connected nodes to the new node
            For each connected node, check if it is alread in the open list
            If no, add that node to the open list
            If yes, compare its cost G with new calculated cost G from current node and rewire the route
            */
            std::vector<mz_wang_intern::LinkedNode> connected_nodes = GetConnectedNodes(next_node);
            for (auto future_node : connected_nodes)
            {
                if (!isInList(open_list_, future_node))
                {
                    /*
                    For the connected node that is not in the open list, calcualte its cost and assigned current node as its parent.
                    */
                    future_node.self.G = calcG(next_node, future_node);
                    future_node.self.H = calcH(future_node, goal);
                    future_node.self.F = calcF(future_node);
                    future_node.parent = next_node;
                    open_list_.push_back(future_node);
                }
                else
                {
                    // For the connected node that is already in the open list, check if current node gives a smaller cost G
                    if (calcG(next_node, future_node) < node.G)
                    {
                        // if new cost G is smaller, assign new G to this node and change its parent to current node
                        future_node.self.G = calcG(next_node, future_node);
                        future_node.self.F = calcF(future_node);
                        future_node.parent = next_node;
                    }
                }
                // Check if the goal node is found; if so, exit the loop and we have the path
                mz_wang_intern::LinkedNode final_node = isInList(open_list_, goal);
                if (final_node != NULL)
                    return final_node;
            }
        }
        return NULL;
    }

    bool GetPlanCallback(mz_wang_intern::GetPlan::Request &req, mz_wang_intern::GetPlan::Response &res)
    {
        /* 
        call the FindPath function to explore the map and then construct the path backward from the goal node
        */
        mz_wang_intern::LinkedNode goal, start;
        goal.self.position = req.goal.position;

        mz_wang_intern::LinkedNode result = FindPath(start, goal);
        geometry_msgs::PoseStamped pose;
        while (result != NULL)
        {
            pose.pose.position = result.self.position;
            res.path.poses.insert(res.path.poses.begin(), pose);
            result = result.parent;
        }
        open_list_.clear();
        close_list_.clear();
        if (res.path.poses.size() != 0)
        {
            res.success = true;
            ROS_INFO("Get Path Succesfully");
            return true;
        }
        else
        {
            ROS_INFO("Get Path Failed");
            return false;
        }
    }

  private:
    ros::NodeHandle nh_;
    ros::ServiceServer get_plan_server_;
    int edge_cost_;
    // use a 2D matrix to represent the grid map
    std::vector<std::vector<int>> map_;
    // open list to save nodes to be visited
    std::list<mz_wang_intern::LinkedNode> open_list_;
    // close list to save nodes that have been visited
    std::list<mz_wang_intern::LinkedNode> close_list_;
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