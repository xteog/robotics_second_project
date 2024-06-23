#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>

#include <dynamic_reconfigure/server.h>
#include <second_project/parametersConfig.h>

class lidar_remap
{
public:
    lidar_remap()
    {

        pub = n.advertise<sensor_msgs::PointCloud2>("pointcloud_remapped", 1000);
        sub = n.subscribe<sensor_msgs::PointCloud2>("/ugv/rslidar_points", 1, &lidar_remap::subCallback, this);
    }


    void subCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        sensor_msgs::PointCloud2 modifiedMsg = *msg;
        modifiedMsg.header.frame_id = "UGV_odom";
        modifiedMsg.header.stamp = msg->header.stamp;
        pub.publish(modifiedMsg);

    }

private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;

    dynamic_reconfigure::Server<second_project::parametersConfig> server;
    dynamic_reconfigure::Server<second_project::parametersConfig>::CallbackType f;
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "lidar_remap");

    lidar_remap my_lidar_remap;

    ros::spin();
    return 0;
}
