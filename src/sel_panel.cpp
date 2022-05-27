#include "sel_panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <QColor>
#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QPushButton>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"

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
#include <cmath>

#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"

PLUGINLIB_EXPORT_CLASS(rviz_panel::SelPanel, rviz::Panel)

namespace rviz_panel
{
    SelPanel::SelPanel(QWidget * parent)
    :   rviz::Panel(parent)
    {
        center_sub = n.subscribe("/sel_data/center", 1000, &SelPanel::new_center, this);

        // Initialize member variables
        frame_id = "camera_depth_optical_frame";
        radius = 0.33;
        frame_idx = 0;
        cen_x = 0;
        cen_y = 0;
        cen_z = 0;
        is_selecting = false;

        // Construct and lay out labels and slider controls.
        QPushButton* sel_bag = new QPushButton("&Bag Select", this);
        sel_region = new QPushButton("&Start Selection", this);
        QPushButton* end_selection = new QPushButton("&End Selection", this);
        QLabel* frame_label = new QLabel("Frame");
        frame_slider = new QSlider(Qt::Horizontal);
        frame_slider->setMinimum(0);
        frame_slider->setMaximum(0);
        QLabel* radius_label = new QLabel("Radius");
        radius_slider = new QSlider(Qt::Horizontal);
        radius_slider->setMinimum(0);
        radius_slider->setMaximum(100);
        QGridLayout* controls_layout = new QGridLayout();
        controls_layout->addWidget(sel_bag, 0, 0);
        controls_layout->addWidget(sel_region, 0, 1);
        controls_layout->addWidget(end_selection, 0, 2);
        controls_layout->addWidget(frame_label, 1, 0);
        controls_layout->addWidget(frame_slider, 1, 1);
        controls_layout->addWidget(radius_label, 2, 0);
        controls_layout->addWidget(radius_slider, 2, 1);

        // Construct and lay out render panel.
        render_panel = new rviz::RenderPanel();
        QVBoxLayout* main_layout = new QVBoxLayout;
        main_layout->addLayout(controls_layout);

        // Set the top-level layout for this MyViz widget.
        setLayout(main_layout);
        frame_slider->setValue(0);
        radius_slider->setValue(33);
 
        // Make signal/slot connections.
        connect(sel_bag, &QPushButton::clicked, this, &SelPanel::set_bag);
        connect(sel_region, &QPushButton::clicked, this, &SelPanel::select_region);
        connect(end_selection, &QPushButton::clicked, this, &SelPanel::end_selection);
        connect(frame_slider, SIGNAL(valueChanged(int)), this, SLOT(set_frame(int)));
        connect(radius_slider, SIGNAL(valueChanged(int)), this, SLOT(set_radius(int)));

        // Next we initialize the main RViz classes.
        //
        // The VisualizationManager is the container for Display objects,
        // holds the main Ogre scene, holds the ViewController, etc.  It is
        // very central and we will probably need one in every usage of
        // librviz.
        manager = new rviz::VisualizationManager(render_panel);
        manager->initialize();
        manager->startUpdate();
    }

    /**
     *  Save all configuration data from this panel to the given
     *  Config object. It is important here that you call save()
     *  on the parent class so the class id and panel name get saved.
     */
    void SelPanel::save(rviz::Config config) const {
        rviz::Panel::save(config);
    }

    /**
     *  Load all configuration data for this panel from the given Config object.
     */
    void SelPanel::load(const rviz::Config & config) {
        rviz::Panel::load(config);
    }

    void SelPanel::set_bag() {
        bag_filepath = QFileDialog::getOpenFileName(this, tr("Open Bag"), "/home/christianforeman/catkin_ws/src/point_cloud_selector", tr("Bags (*.bag)")).toStdString();
        ROS_INFO_STREAM(bag_filepath);

        // read in the rosbag
        rosbag::Bag bag;
        bag.open(bag_filepath);

        std::vector<std::string> topics;
        topics.push_back(std::string("/camera/depth/color/points"));

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        foreach(rosbag::MessageInstance const m, view)
        {
            sensor_msgs::PointCloud2::ConstPtr temp = m.instantiate<sensor_msgs::PointCloud2>();
            
            frames.push_back(*temp);
        }
        
        bag.close();

        frame_pub.publish(frames[frame_idx]);

        frame_slider->setMaximum(frames.size() - 1);

        ROS_INFO_STREAM("Finished Reading in Bag");
    }


    void SelPanel::setup_cube() {
        // create a grey box marker


        cube_marker.ns = "cube";
        cube_marker.id = 0;

        cube_marker.type = visualization_msgs::Marker::CUBE;
        cube_marker.color.r = 0.0;
        cube_marker.color.g = 0.5;
        cube_marker.color.b = 0.5;

        update_marker();
    }

    void SelPanel::update_marker() {
        cube_marker.header.frame_id = frame_id; 
        cube_marker.header.stamp = ros::Time::now();

        if(is_selecting) {
            cube_marker.color.a = 0.3;
        }
        else {
            cube_marker.color.a = 0.0;
        }

        cube_marker.scale.x = radius * 2;
        cube_marker.scale.y = radius * 2;
        cube_marker.scale.z = radius * 2;
  
        cube_marker.pose.position.x = cen_x;
        cube_marker.pose.position.y = cen_y;
        cube_marker.pose.position.z = cen_z;

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

    void SelPanel::new_center(const std_msgs::Float32MultiArray new_cen) {
        // gets message of new center from moving marker
        cen_x = new_cen.data[0];
        cen_y = new_cen.data[1];
        cen_z = new_cen.data[2];

        update_marker();
    }
    
    void SelPanel::select() {
        // get the points that are within the range
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::fromROSMsg(frames[frame_idx], *cloud);

        double x_diff, y_diff, z_diff;

        foreach(const pcl::PointXYZRGB& pt, cloud->points) {
            x_diff = std::abs(pt.x - cen_x);
            y_diff = std::abs(pt.y - cen_y);
            z_diff = std::abs(pt.z - cen_z);

            if(x_diff < radius && y_diff < radius && z_diff < radius) {
                current_selection.points.push_back(pt);
            }
        }

        sensor_msgs::PointCloud2 temp;
        pcl::toROSMsg(current_selection, temp);
        temp.header.frame_id = frame_id;
        temp.header.stamp = ros::Time::now();

        sel_pub.publish(temp);
        // TODO: check for duplicates
    }
    
    void SelPanel::select_region() {
        if(is_selecting == false) {
            is_selecting = true;
            sel_region->setText("&Select");
            setup_cube();
            
            // Tell the other node to create the IM
            std_msgs::Bool flag;
            flag.data = is_selecting;

            toggle_pub.publish(flag);
        }
        else {
            select();
        }
    }

    void SelPanel::end_selection() {
        is_selecting = false;
        update_marker();
        std_msgs::Bool flag;
        flag.data = is_selecting;
        toggle_pub.publish(flag);
        current_selection.points.clear();
        //TODO: Save selected region to a file? Make the marker invisible
        sel_region->setText("&Start Selection");
        // here should save the current selection into a file and clear out the past selection
    }

    void SelPanel::set_frame(int frame_num) {
        // reset collision statistics
        end_selection(); // may want to add a default arg to end selection which signifies saving or not

        frame_idx = frame_num;

        frame_pub.publish(frames[frame_idx]);
    }

    void SelPanel::set_radius(int new_radius) {
        radius = new_radius/100.0;
        update_marker();
    }
} // namespace rviz_panel
