#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <geometry_msgs/PoseStamped.h>

std::vector<std::vector<double>> readCSV(const std::string &);
geometry_msgs::PoseStamped vectorToPose(const std::vector<double> &);

class set_goals
{
private:
    ros::Publisher pub;
    ros::NodeHandle n;

public:
    set_goals()
    {
        pub = n.advertise<geometry_msgs::PoseStamped>("/move_base/current_goal", 1);
    }

    void publishWaypoints()
    {
        geometry_msgs::PoseStamped goal;

        std::vector<std::vector<double>> data = readCSV("/home/gallo/catkin_ws/src/robotics_second_project/waypoints.csv"); // TODO Relative path

        for (int i = 0; i < data.size(); i++)
        {
            goal = vectorToPose(data[i]);
            pub.publish(goal);
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
        ROS_INFO("Failed to open file");
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

    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;
    goal.pose.orientation.w = vector[2];

    return goal;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_goals");

    set_goals node;

    while (ros::ok())
    {
        node.publishWaypoints();
    }

    ros::spin();
    return 0;
}
