#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <interactive_markers/interactive_marker_server.h>

#include <string.h>
#include <stdio.h>
#include <vector>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <sensor_msgs/PointCloud2.h>

class Selector {

private:
    ros::NodeHandle n;
    ros::Publisher cube_pub = n.advertise<visualization_msgs::Marker>("/sel_data/sel_sphere", 1);
    ros::Publisher frame_pub = n.advertise<sensor_msgs::PointCloud2>("/sel_data/cur_frame", 1);

    std::string frame_id;
    double radius = 0.33;
    std::vector<double> center;

    std::vector<sensor_msgs::PointCloud2> frames;

    visualization_msgs::Marker cube_marker;

public:
    Selector(std::string frame): frame_id(frame) {
        center.assign(3, 0);

        // Read in the rosbag
        rosbag::Bag bag;
        bag.open("garden_high_accuracy.bag");

        std::vector<std::string> topics;
        topics.push_back(std::string("/camera/depth/color/points"));

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        foreach(rosbag::MessageInstance const m, view)
        {
            sensor_msgs::PointCloud2::ConstPtr s = m.instantiate<sensor_msgs::PointCloud2>();
            

            // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

            frames.push_back(*s);

            // pcl::fromROSMsg(*s, *cloud);

            //break;
        }
        
        bag.close();

        std::cout << "Finished reading bag" << std::endl;
        frame_pub.publish(frames[0]);

        setup_sphere();

        setup_interactive_marker();
    }

    void move_center(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ) {
        // Check why the order here is whack
        center[0] = feedback->pose.position.x;
        center[1] = - feedback->pose.position.z;
        center[2] = feedback->pose.position.y;

        cube_marker.pose.position.x = center[0];
        cube_marker.pose.position.y = center[1];
        cube_marker.pose.position.z = center[2];

        cube_pub.publish(cube_marker);
    }

    void setup_interactive_marker() {
        // create an interactive marker server on the topic namespace simple_marker
        interactive_markers::InteractiveMarkerServer server("interactive_marker");

        // create an interactive marker for our server
        visualization_msgs::InteractiveMarker int_marker;
        int_marker.header.frame_id = frame_id;
        int_marker.header.stamp=ros::Time::now();

        // create a control which will move the box
        // this control does not contain any markers,
        // which will cause RViz to insert two arrows
        visualization_msgs::InteractiveMarkerControl control;
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "move_x";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "move_y";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "move_z";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        // add the interactive marker to our collection &
        // tell the server to call processFeedback() when feedback arrives for it
        server.insert(int_marker, boost::bind(&Selector::move_center, this, _1));
        // 'commit' changes and send to all clients
        server.applyChanges();

        ros::spin();
    }

    void setup_sphere() {
        // create a grey box marker
        cube_marker.header.frame_id = frame_id;
        cube_marker.header.stamp = ros::Time::now();

        cube_marker.ns = "sphere";
        cube_marker.id = 0;

        cube_marker.type = visualization_msgs::Marker::SPHERE;
        cube_marker.scale.x = radius * 2;
        cube_marker.scale.y = radius * 2;
        cube_marker.scale.z = radius * 2;
        cube_marker.color.r = 0.0;
        cube_marker.color.g = 0.5;
        cube_marker.color.b = 0.5;
        cube_marker.color.a = 0.3;

        while (cube_pub.getNumSubscribers() < 1) {
            if (!ros::ok())
            {
                return;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }

        cube_pub.publish(cube_marker);
    }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_marker");

  // Create class for selector
  Selector selector("camera_depth_optical_frame");
}
