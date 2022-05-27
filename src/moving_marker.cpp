#include "ros/ros.h"

#include <interactive_markers/interactive_marker_server.h>

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"

class IM {
private:
    std::string frame_id;
    ros::NodeHandle n;
    
    ros::Subscriber toggle_sub;
    ros::Publisher center_pub = n.advertise<std_msgs::Float32MultiArray>("/sel_data/center", 1000);

    double cen_x;
    double cen_y;
    double cen_z;

    bool is_active;
    
public:
    IM(std::string frame): frame_id(frame) {
        toggle_sub = n.subscribe("/sel_data/toggle", 1000, &IM::toggle_im, this);  
        cen_x = cen_y = cen_z = 0;
        is_active = false;
        ros::spin();
    }

    void move_center(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
        // Check why the order here is whack, definitely something with coordinate frames
        cen_x = feedback->pose.position.x;
        cen_y = - feedback->pose.position.z;
        cen_z = feedback->pose.position.y;

        std_msgs::Float32MultiArray center_msg;
        center_msg.data.clear();
        center_msg.data.push_back(cen_x);
        center_msg.data.push_back(cen_y);
        center_msg.data.push_back(cen_z);

        center_pub.publish(center_msg);
    }

    void toggle_im(std_msgs::Bool msg) {
        if(msg.data) {
            is_active = true;
            setup_interactive_marker();
            ROS_INFO_STREAM("Starting Selection");
        }
        else {
            is_active = false;
            // delete the marker
            ROS_INFO_STREAM("Ending Selection");
        }
    }

    void setup_interactive_marker() {
        // create an interactive marker server on the topic namespace simple_marker
        interactive_markers::InteractiveMarkerServer server("interactive_marker");

        // create an interactive marker for our server
        visualization_msgs::InteractiveMarker int_marker;
        int_marker.header.frame_id = frame_id;
        int_marker.header.stamp = ros::Time::now();
        
        int_marker.pose.position.x = cen_x;
        int_marker.pose.position.y = cen_y;
        int_marker.pose.position.z = cen_z;

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
        server.insert(int_marker, boost::bind(&IM::move_center, this, _1));
        // 'commit' changes and send to all clients
        server.applyChanges();
        if(is_active) {
            ros::spin();
        }
    }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "moving_marker");

  IM marker("camera_depth_optical_frame");

  return 0;
}