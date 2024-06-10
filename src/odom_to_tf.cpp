#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"

class odom_to_tf
{
public:
    odom_to_tf()
    {
        sub = n.subscribe(n.resolveName("/ugv/odom"), 1000, &odom_to_tf::callback, this);
    }

    void callback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // Access pose from the Odometry message
        // ros::NodeHandle nh_private("~");
        tf::Transform transform;
        //n.getParam("root_frame", root);
        //nh_private.getParam("child_frame", child);
        root = "map";
        child = "UGV_odom";
        transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
        transform.setRotation(tf::Quaternion(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
        //ros::Time current_time = ros::Time::now();
        //br.sendTransform(tf::StampedTransform(transform, current_time, root, child));
        br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, root, child));
        // ROS_INFO("Root param name: %s", root.c_str());
        // ROS_INFO("Child param name: %s", child.c_str());
    }

private:
    tf::TransformBroadcaster br;
    ros::Subscriber sub;
    ros::NodeHandle n;
    std::string root;
    std::string child;
    std::string group_name;
};

int main(int argc, char **argv)
{
    // Node name: odom_to_tf
    ros::init(argc, argv, "odom_to_tf");

    odom_to_tf my_odom_to_tf;

    ros::spin();
    return 0;
}