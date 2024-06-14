#include <ros/ros.h>
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
        pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    }

    void publishWaypoints()
    {
        geometry_msgs::PoseStamped goal;

        ROS_INFO("-2");
        std::vector<std::vector<double>> data = readCSV("waypoints.csv");
        ROS_INFO("-1");

        for (int i = 0; i < 5; i++) // TODO vector size
        {
            ROS_INFO("0");
            goal = vectorToPose(data[i]);
            ROS_INFO("0");
            pub.publish(goal);
            ROS_INFO("0");
            ros::spinOnce();
        }
    }
};

std::vector<std::vector<double>> readCSV(const std::string &filename)
{
    std::vector<std::vector<double>> data;

    std::ifstream file(filename);

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
    try
    {
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();

        goal.pose.position.x = vector[0];
        goal.pose.position.y = vector[1];
        goal.pose.position.z = 0.0;
    }
    catch (const std::exception &exc)
    {
        ROS_ERROR_STREAM("Out of range error: " << exc.what());
    }

    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;
    goal.pose.orientation.w = vector[2];

    return goal;
}

int main(int argc, char **argv)
{
    // Node name: odom_to_tf
    ros::init(argc, argv, "set_goals");

    set_goals node;

    while (ros::ok())
    {
        node.publishWaypoints();
    }

    ros::spin();
    return 0;
}
