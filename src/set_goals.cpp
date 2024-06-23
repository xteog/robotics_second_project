#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include "client_goals.cpp"
#include <tf2/LinearMath/Quaternion.h>

std::vector<std::vector<double>> readCSV(const std::string &);
geometry_msgs::PoseStamped vectorToPose(const std::vector<double> &);

class SetGoals
{
private:
    ros::Publisher pub;
    ros::NodeHandle n;
    ActionClient client;

public:
    SetGoals() : client(getTimeout())
    {
        pub = n.advertise<geometry_msgs::PoseStamped>("/move_base/current_goal", 1);
    }

    double getTimeout()
    {
        double timeout;
        n.param("timeout", timeout, 60.0);
        return timeout;
    }

    void publishWaypoints()
    {
        geometry_msgs::PoseStamped goal;

        std::string path = ros::package::getPath("second_project");

        std::vector<std::vector<double>> data = readCSV(path + "/waypoints.csv");

        for (int i = 0; i < data.size(); i++)
        {
            goal = vectorToPose(data[i]);

            client.sendGoal(goal);
            bool done = client.waitForResult();
            ros::spinOnce();
        }
    }
};

std::vector<std::vector<double>> readCSV(const std::string &filename)
{
    std::vector<std::vector<double>> data;

    std::ifstream file(filename);

    if (!file.is_open())
    {
        ROS_INFO("Failed to open waypoints file");
        return data;
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::vector<double> row;
        std::stringstream ss(line);

        std::string field;
        while (std::getline(ss, field, ','))
        {
            double f = std::stof(field);
            row.push_back(f);
        }

        data.push_back(row);
    }

    file.close();

    return data;
}

geometry_msgs::PoseStamped vectorToPose(const std::vector<double> &vector)
{
    geometry_msgs::PoseStamped goal;

    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();

    goal.pose.position.x = vector[0];
    goal.pose.position.y = vector[1];
    goal.pose.position.z = 0.0;

    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, vector[2]);

    goal.pose.orientation.x = quaternion.x();
    goal.pose.orientation.y = quaternion.y();
    goal.pose.orientation.z = quaternion.z();
    goal.pose.orientation.w = quaternion.w();

    return goal;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_goals");

    SetGoals node;

    while (ros::ok())
    {
        node.publishWaypoints();
    }

    ros::spin();
    return 0;
}
