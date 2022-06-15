#include "ros/ros.h"

#include <interactive_markers/interactive_marker_server.h>

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"

class IM {
private:
    std::string frame_id;
    ros::NodeHandle n;
    
    ros::Subscriber toggle_sub;
    ros::Publisher center_pub;

    // very strange syntax
    interactive_markers::InteractiveMarkerServer server{"interactive_marker"};
    visualization_msgs::InteractiveMarker int_marker;

    double cen_x;
    double cen_y;
    double cen_z;
    
public:
    /**
     * Initializes member variables and starts the rospin, waits for a message to spawn in
     * the interactive marker 
     */
    IM(std::string frame): frame_id(frame) {
        // set up subscribe/publish
        center_pub = n.advertise<std_msgs::Float32MultiArray>("/sel_data/center", 1000);
        toggle_sub = n.subscribe("/sel_data/toggle", 1000, &IM::toggle_im, this);  

        // initialize center
        cen_x = cen_y = cen_z = 0;

        // setup int_marker, don't add it to the scene yet
        setup_marker_controls();

        // wait for messages
        ros::spin();
    }

    /**
     * Callback to starting/ending selection
     * 
     * msg = true means create an interactive marker, msg = false means delete the current interactive marker/
     */
    void toggle_im(std_msgs::Bool msg) {
        // create an interactive marker on the scene
        if(msg.data) {
            setup_interactive_marker();
            ROS_INFO_STREAM("Starting Selection");
        }
        // remove the interactive marker, selection time is over
        else {
            server.clear();
            server.applyChanges();
        }
    }

    /**
     * Callback for the interactive marker when it moves, send a message to the plugin about the new center 
     * 
     * TODO: Check why the order of the center coordinates is so whack, has to do with coordinate frames,
     * but I believe I properly initialized the frame 
     */
    void move_center(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
        // Check why the order here is whack, definitely something with coordinate frames
        cen_x = feedback->pose.position.x;
        cen_y = - feedback->pose.position.z;
        cen_z = feedback->pose.position.y;

        // publish to the plugin the new center
        std_msgs::Float32MultiArray center_msg;
        center_msg.data.clear();
        center_msg.data.push_back(cen_x);
        center_msg.data.push_back(cen_y);
        center_msg.data.push_back(cen_z);

        center_pub.publish(center_msg);
    }

    /**
     * Called when we want to spawn in a new interactive marker 
     */
    void setup_interactive_marker() {
        // create an interactive marker for our server
        int_marker.header.frame_id = frame_id;
        int_marker.header.stamp = ros::Time::now();
        
        int_marker.pose.position.x = cen_x;
        int_marker.pose.position.y = cen_y;
        int_marker.pose.position.z = cen_z;

        // add the int_marker (the arrows around the box), and setup the callback func
        server.insert(int_marker, boost::bind(&IM::move_center, this, _1));

        server.applyChanges();
    }

    /**
     * Initializes the control of the marker, only called once on initialization
     */
    void setup_marker_controls() {
        // initialize the aspects of int_marker that never change
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
    }
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "moving_marker");

  // TODO: Make this a command line argument
  IM marker("camera_depth_optical_frame");

  return 0;
}