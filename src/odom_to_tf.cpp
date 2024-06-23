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
        tf::Transform transform;
        root = "odom_frame";
        child = "UGV_odom";
        transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
        transform.setRotation(tf::Quaternion(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
        br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, root, child));
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