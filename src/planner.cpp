#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include "mz_wang_intern/GetPlan.h"

struct Node
{
    int x, y;
    int F, G, H;
    Node *parent;
    Node(int _x, int _y) : x(_x), y(_y), F(0), G(0), H(0), parent(NULL) {}
};

class PathPlanner
{
  public:
    PathPlanner(ros::NodeHandle n) : nh_(n), map_(11, std::vector<int>(11, 0))
    {
        get_plan_server_ = nh_.advertiseService("get_plan", &PathPlanner::GetPlanCallback, this);
        agent_feedback_sub_ = nh_.subscribe("/agent_feedback", 10, &PathPlanner::AgentFeedbackCallback, this);
        edge_cost_ = 10;
        // create the map
        for (int i = 0; i < 11; i++)
        {
            for (int j = 0; j < 11; j++)
            {
                map_[i][j] = 0;
            }
        }
    }

    // get agent position from /agent_feedback topic
    void AgentFeedbackCallback(const geometry_msgs::Pose &msg)
    {
        agent_x_ = msg.position.x;
        agent_y_ = msg.position.y;
    }

    // calculate cost G for target node
    int calcG(Node *current, Node *target)
    {
        return current->G + edge_cost_;
    }
    // calculate cost H for target node, manhattan distance is used here
    int calcH(Node *target, Node *goal)
    {
        int H = edge_cost_ * abs(goal->x - target->x) + abs(goal->y - target->y);
        return H;
    }
    // calculate total cost F
    int calcF(Node *target)
    {
        return target->H + target->G;
    }

    // find the node with the least F from the open list, which will be visited as next step
    Node *GetNextNode()
    {
        if (!open_list_.empty())
        {
            Node *next_node = open_list_.front();
            for (auto node : open_list_)
            {
                if (node->F < next_node->F)
                    next_node = node;
            }
            return next_node;
        }
        // if the open list is already empty
        return NULL;
    }

    // check if one node is in the list(open or close)
    Node *isInList(const std::list<Node *> &list, const Node *target)
    {
        for (auto node : list)
        {
            if (node->x == target->x && node->y == target->y)
                return node;
        }
        return NULL;
    }

    // check is the target node is reachable
    bool isReachable(Node *target)
    {
        // return false if the target node is outside of the map, is occupied by path of another agent, or is in the closed list
        if (target->x < 0 || target->x > 10 || target->y < 0 || target->y > 10 || map_[target->x][target->y] == 1 || isInList(close_list_, target))
            return false;
        else
            return true;
    }

    // get 4 connected nodes for a given node
    std::vector<Node *> GetConnectedNodes(Node *current)
    {
        std::vector<Node *> connected_nodes;
        for (int i = -1; i <= 1; i = i + 2)
        {
            // for each iteration, check the reachability of two connected nodes
            // new node is only added to the open list if reachable
            Node *node_x = new Node(current->x + i, current->y);
            Node *node_y = new Node(current->x, current->y + i);
            if (isReachable(node_x))
                connected_nodes.push_back(node_x);
            if (isReachable(node_y))
                connected_nodes.push_back(node_y);
        }
        return connected_nodes;
    }

    // find optimal path using A*
    Node *FindPath(Node &start, Node &goal)
    {
        // add the start node to the open list
        open_list_.push_back(new Node(start.x, start.y));
        while (!open_list_.empty())
        {
            /*
            Find the node in open list with least F, which will be visited in next step
            Remove that node from open list and add it to close list
            Now visit the next node
            */
            Node *next_node = GetNextNode();
            open_list_.remove(next_node);
            close_list_.push_back(next_node);
            /*
            Find the 4 connected nodes to the new node
            For each connected node, check if it is alread in the open list
            If no, add that node to the open list
            If yes, compare its cost G with new calculated cost G from current node and rewire the route
            */
            std::vector<Node *> connected_nodes = GetConnectedNodes(next_node);
            for (auto future_node : connected_nodes)
            {
                if (!isInList(open_list_, future_node))
                {
                    /*
                    For the connected node that is not in the open list, calcualte its cost and assigned current node as its parent.
                    */
                    future_node->G = calcG(next_node, future_node);
                    future_node->H = calcH(future_node, &goal);
                    future_node->F = calcF(future_node);
                    future_node->parent = next_node;
                    open_list_.push_back(future_node);
                }
                else
                {
                    // For the connected node that is already in the open list, check if current node gives a smaller cost G
                    if (calcG(next_node, future_node) < future_node->G)
                    {
                        // if new cost G is smaller, assign new G to this node and change its parent to current node
                        future_node->G = calcG(next_node, future_node);
                        future_node->F = calcF(future_node);
                        future_node->parent = next_node;
                    }
                }
                // Check if the goal node is found; if so, exit the loop and we have the path
                Node *final_node = isInList(open_list_, &goal);
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
        Node start(agent_x_, agent_y_);
        Node goal(int(req.goal.position.x), int(req.goal.position.y));
        Node *result = FindPath(start, goal);
        geometry_msgs::PoseStamped pose;
        while (result != NULL)
        {
            pose.pose.position.x = result->x;
            pose.pose.position.y = result->y;
            res.path.poses.insert(res.path.poses.begin(), pose);
            result = result->parent;
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
    ros::Subscriber agent_feedback_sub_;
    int edge_cost_, agent_x_, agent_y_;
    // use a 2D matrix to represent the grid map
    std::vector<std::vector<int>> map_;
    // open list to save nodes to be visited
    std::list<Node *> open_list_;
    // close list to save nodes that have been visited
    std::list<Node *> close_list_;
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