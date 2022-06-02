#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcd_msgs");
    ros::NodeHandle n;
    std::string topic_name = argv[1];
    std::string pcd_file = argv[2];
    
    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>(topic_name, 1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile(pcd_file, *cloud);
    
    sensor_msgs::PointCloud2 temp;
    pcl::toROSMsg(*cloud, temp);
    temp.header.frame_id = "camera_depth_optical_frame";
    temp.header.stamp = ros::Time::now();

    while(pub.getNumSubscribers() < 1) {
        if (!ros::ok())
        {
            return -1;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }
    
    pub.publish(temp);
}