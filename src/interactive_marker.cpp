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

#include <math.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class Selector {

private:
    ros::NodeHandle n;
    ros::Publisher cube_pub = n.advertise<visualization_msgs::Marker>("/sel_data/sel_sphere", 1);
    ros::Publisher frame_pub = n.advertise<sensor_msgs::PointCloud2>("/sel_data/cur_frame", 1);
    ros::Publisher sel_pub = n.advertise<sensor_msgs::PointCloud2>("/sel_data/selected_pc", 1);

    std::string frame_id;
    double radius;
    double cen_x;
    double cen_y;
    double cen_z;
    uint32_t frame_index;

    std::vector<std::vector<double> > points;
    std::vector<std::vector<double> > selected_points;

    std::vector<sensor_msgs::PointCloud2> frames;

    visualization_msgs::Marker cube_marker;

public:
    Selector(std::string frame, std::string bag_file_path): frame_id(frame) {
        radius = 0.33;
        cen_x = 0;
        cen_y = 0;
        cen_z = 0;
        frame_index = 1000;

        // Read in the rosbag
        rosbag::Bag bag;
        bag.open(bag_file_path);

        std::vector<std::string> topics;
        topics.push_back(std::string("/camera/depth/color/points"));

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        foreach(rosbag::MessageInstance const m, view) {
            sensor_msgs::PointCloud2::ConstPtr temp = m.instantiate<sensor_msgs::PointCloud2>();
            
            frames.push_back(*temp);
        }
        
        bag.close();

        std::cout << "Finished reading bag" << std::endl;
        frame_pub.publish(frames[frame_index]);

        setup_sphere();

        select_points();

        setup_interactive_marker();
    }

    void update_frame() {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::fromROSMsg(frames[frame_index], *cloud);

        points.clear();
        foreach(const pcl::PointXYZRGB& pt, cloud->points) {
            std::vector<double> point{pt.x, pt.y, pt.z};
            points.push_back(point);
        }
    }

    void select_points() {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::fromROSMsg(frames[frame_index], *cloud);

        pcl::PointCloud<pcl::PointXYZRGB> msg;
        foreach(const pcl::PointXYZRGB& pt, cloud->points) {
            double dist = sqrt((pt.x - cen_x) * (pt.x - cen_x) +
                               (pt.y - cen_y) * (pt.y - cen_y) +
                               (pt.z - cen_z) * (pt.z - cen_z));
            if(dist < radius) {
                msg.points.push_back(pt);
            }
        }

        sensor_msgs::PointCloud2 temp;
        pcl::toROSMsg(msg, temp);
        temp.header.frame_id = frame_id;
        temp.header.stamp = ros::Time::now();

        sel_pub.publish(temp);
    }

    void publish_points() {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr msg (new pcl::PointCloud<pcl::PointXYZRGB>);
        for(int i = 0; i < selected_points.size(); ++i) {
            pcl::PointXYZRGB pt;
            pt.x = selected_points[i][0];
            pt.y = selected_points[i][1];
            pt.z = selected_points[i][2];
            msg->points.push_back(pt);
        }
        
        sensor_msgs::PointCloud2::Ptr temp;

        pcl::toROSMsg(*msg, *temp);
        temp->header.frame_id = frame_id;
        temp->header.stamp = ros::Time::now();

        //sel_pub.publish(temp);
    }

    void move_center(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
        // Check why the order here is whack, definitely something with coordinate frames
        cen_x = feedback->pose.position.x;
        cen_y = - feedback->pose.position.z;
        cen_z = feedback->pose.position.y;

        cube_marker.pose.position.x = cen_x;
        cube_marker.pose.position.y = cen_y;
        cube_marker.pose.position.z = cen_z;

        cube_pub.publish(cube_marker);

        select_points();
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

        //ros::Duration(10).sleep();
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

// rosrun point_cloud_selector point_cloud_selector camera_depth_optical_frame /home/christianforeman/catkin_ws/src/point_cloud_selector/garden_high_accuracy.bag
int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_selector");
    std::string frame = std::string(argv[1]);
    std::string bag_file_path = std::string(argv[2]);

    // Create class for selector
    Selector selector(frame, bag_file_path);
}
