#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

class ActionClient
{
private:
    double timeout;
    Client client;
public:
    ActionClient(double timeout) : client("move_base", true)
    {   
        this->timeout = timeout;
        ROS_INFO("Waiting for action server to start.");
        client.waitForServer();
    }

    bool waitForResult()
    {
        return client.waitForResult(ros::Duration(timeout));
    }

    void sendGoal(geometry_msgs::PoseStamped &pose)
    {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = pose;
        client.sendGoal(goal);
    }
};
