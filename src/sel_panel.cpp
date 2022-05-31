#include "sel_panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <QColor>
#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QPushButton>
#include <QWheelEvent>

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
    /**
     * Constructor of the panel, initializes member variables and creates the UI
     */
    SelPanel::SelPanel(QWidget * parent):rviz::Panel(parent) {
        cube_pub = n.advertise<visualization_msgs::Marker>("/sel_data/sel_sphere", 1);
        frame_pub = n.advertise<sensor_msgs::PointCloud2>("/sel_data/cur_frame", 1);
        sel_pub = n.advertise<sensor_msgs::PointCloud2>("/sel_data/selected_pc", 1);
        toggle_pub = n.advertise<std_msgs::Bool>("/sel_data/toggle", 1);
        center_sub = n.subscribe("/sel_data/center", 1000, &SelPanel::new_center, this);

        // Initialize member variables
        frame_id = "camera_depth_optical_frame";  // TODO: make this an option in the panel
        radius = 0.33;
        frame_idx = 0;
        cen_x = 0;
        cen_y = 0;
        cen_z = 0;
        is_selecting = false;

        // Construct and lay out labels and slider controls.
        QPushButton* sel_bag = new QPushButton("&Bag Select", this);
        sel_region = new QPushButton("&Start Selection", this);
        unsel_region = new QPushButton("", this);
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
        controls_layout->addWidget(unsel_region, 1, 0);
        controls_layout->addWidget(end_selection, 1, 1);
        controls_layout->addWidget(frame_label, 2, 0);
        controls_layout->addWidget(frame_slider, 2, 1);
        controls_layout->addWidget(radius_label, 3, 0);
        controls_layout->addWidget(radius_slider, 3, 1);

        // Construct and lay out render panel.
        render_panel = new rviz::RenderPanel();
        QVBoxLayout* main_layout = new QVBoxLayout;
        main_layout->addLayout(controls_layout);

        // Set the top-level layout for this widget.
        setLayout(main_layout);
        frame_slider->setValue(0);
        radius_slider->setValue(33);
 
        // Make signal/slot connections.
        connect(sel_bag, &QPushButton::clicked, this, &SelPanel::set_bag);
        connect(sel_region, &QPushButton::clicked, this, &SelPanel::select_region);
        connect(unsel_region, &QPushButton::clicked, this, &SelPanel::unselect_region);
        connect(end_selection, &QPushButton::clicked, this, &SelPanel::end_selection);
        connect(frame_slider, SIGNAL(valueChanged(int)), this, SLOT(set_frame(int)));
        connect(radius_slider, SIGNAL(valueChanged(int)), this, SLOT(set_radius(int)));

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
    
    /**
     * After pressing "Bag Select", prompt the user with a file system to choose a bag, after that, load the first frame of the bag into rviz 
     */
    void SelPanel::set_bag() {
        
        bag_filepath = QFileDialog::getOpenFileName(this, tr("Open Bag"), "/home/christianforeman/catkin_ws/src/point_cloud_selector", tr("Bags (*.bag)")).toStdString();
        ROS_INFO_STREAM(bag_filepath);

        // read in the rosbag
        rosbag::Bag bag;
        bag.open(bag_filepath);

        std::vector<std::string> topics;
        topics.push_back(std::string("/camera/depth/color/points"));

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        foreach(rosbag::MessageInstance const m, view) {
            sensor_msgs::PointCloud2::ConstPtr temp = m.instantiate<sensor_msgs::PointCloud2>();
            
            frames.push_back(*temp);
        }
        
        bag.close();

        // publish first frame of the bag, set the range of the slider
        frame_pub.publish(frames[frame_idx]);
        frame_slider->setMaximum(frames.size() - 1);
    }

    /**
     * Set up the constants of the cube, aka the selection area when the interactive marker is active
     */
    void SelPanel::setup_cube() {
        cube_marker.ns = "cube";
        cube_marker.id = 0;

        cube_marker.type = visualization_msgs::Marker::CUBE;
        cube_marker.color.r = 0.0;
        cube_marker.color.g = 0.5;
        cube_marker.color.b = 0.5;

        update_marker();
    }

    /**
     * Update the visuales of the cube, this happens when the radius or center is changed 
     * It also occurs when the user begins/ends selection by making the marker transparent or not
     */
    void SelPanel::update_marker() {
        // need to setup frame id everytime or rviz complains for some reason
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

    /**
     * Callback to when the interactive marker is moved, it updates the position
     */
    void SelPanel::new_center(const std_msgs::Float32MultiArray new_cen) {
        // gets message of new center from moving marker
        cen_x = new_cen.data[0];
        cen_y = new_cen.data[1];
        cen_z = new_cen.data[2];

        update_marker();
    }

    /**
     * Adds on the points within the square to the current selection 
     */
    void SelPanel::select() {
        // get the points that are within the range
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::fromROSMsg(frames[frame_idx], *cloud);

        double x_diff, y_diff, z_diff;

        // get points within the cube
        foreach(const pcl::PointXYZRGB& pt, cloud->points) {
            x_diff = std::abs(pt.x - cen_x);
            y_diff = std::abs(pt.y - cen_y);
            z_diff = std::abs(pt.z - cen_z);

            if(x_diff < radius && y_diff < radius && z_diff < radius) {
                current_selection.points.push_back(pt);
            }
        }

        remove_duplicates();
        
        publish_selected();
    }

    /**
     * Removes points that appear on top of each other
     * 
     * TODO: Make this faster, currently the erasing of the vector takes a really long time
     * May want to investigate KD-trees more for this
     */
    void SelPanel::remove_duplicates() {
        std::cout << current_selection.points.size() << " Points" << std::endl;
        pcl::PointXYZRGB pt_i, pt_j;
        for(uint32_t i = 0; i < current_selection.points.size() - 1; ++i) {
            pt_i = current_selection.points[i];
            for(uint32_t j = i + 1; j < current_selection.points.size(); ++j) {
                pt_j = current_selection.points[j];
                if(pt_i.x == pt_j.x && pt_i.y == pt_j.y && pt_i.z == pt_j.z) {
                    // remove the duplicate
                    current_selection.points.erase(current_selection.points.begin() + j);
                    --j;
                    break;
                }
            }
        }
        std::cout << current_selection.points.size() << " Points" << std::endl;
    }
    
    /**
     * This is the callback to when the user presses the "Select" button. If it is first selection, add in the markers,
     * if it is pressed after the first time, select the region within the cube 
     * 
     */
    void SelPanel::select_region() {
        if(is_selecting == false) {
            is_selecting = true;
            sel_region->setText("&Select");
            unsel_region->setText("&Unselect");
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

    /**
     *  This function is the callback to pressing the "unselect" button. It simply removes the points within the cube that
     *  are also in the current selection.
     * 
     *  TODO: This is pretty slow because of erasing in a vector, should look into speeding this up (KD trees?)
     */
    void SelPanel::unselect_region() {
        double x_diff, y_diff, z_diff;
        pcl::PointXYZRGB pt;
        for(uint32_t i = 0; i < current_selection.points.size(); ++i) {
            pt = current_selection.points[i];
            x_diff = std::abs(pt.x - cen_x);
            y_diff = std::abs(pt.y - cen_y);
            z_diff = std::abs(pt.z - cen_z);

            if(x_diff < radius && y_diff < radius && z_diff < radius) {
                current_selection.points.erase(current_selection.points.begin() + i);
                --i;
            }
        }

        publish_selected();
    } 

    /**
     * This function is called when the user wants to end their selection and save it to a file 
     *
     * TODO: May want to let the user select the save path for the pc 
     */
    void SelPanel::end_selection() {
        // if user pressed when selection is not active, do nothing
        if(is_selecting = false) {
            return;
        }
        is_selecting = false;

        // make the cube transparent
        update_marker();

        // notify the interactive marker that selection is over
        std_msgs::Bool flag;
        flag.data = is_selecting;
        toggle_pub.publish(flag);

        sel_region->setText("&Start Selection");
        unsel_region->setText("");

        // save to currentselection.pcd, don't care if its empty
        if(current_selection.points.empty()) {
            return;
        }
        pcl::io::savePCDFile("/home/christianforeman/catkin_ws/src/point_cloud_selector/pcs/current_selection.pcd", current_selection, true);
        current_selection.points.clear();
        publish_selected(); 
    }

    /**
     * This function is called when the frame of the bag is changed on the slider, needs to wipe out any previous selections 
     */
    void SelPanel::set_frame(int frame_num) {
        // reset the selection
        current_selection.points.clear();
        publish_selected();

        if(is_selecting) {
            end_selection();
        }

        frame_idx = frame_num;

        frame_pub.publish(frames[frame_idx]);
    }

    /**
     * Callback to radius slider, simply changes the size of the cube 
     */
    void SelPanel::set_radius(int new_radius) {
        radius = new_radius / 100.0;
        update_marker();
    }

    /**
     * This is the callback for scrolling the mouse within the panel. 
     * It increases/decreases the size of the cube to make adjusting easier
     * 
     */
    void SelPanel::wheelEvent(QWheelEvent * event) {
        if(is_selecting == false) {
            return;
        }
        QPoint delta = event->angleDelta();
        if(delta.y() > 0) {
            int new_radius = (int) (radius * 100) + 5;
            if(new_radius <= 100) {
                // setValue triggers the callback
                radius_slider->setValue(new_radius);
            }
        }
        else {
            int new_radius = (int) (radius * 100) - 5;
            if(new_radius >= 0) {
                // setValue triggers the callback
                radius_slider->setValue(new_radius);
            }
        }
        event->accept();
    }

    /**
     * Helper to publish the selected region to rviz 
     */
    void SelPanel::publish_selected() {
        sensor_msgs::PointCloud2 temp;
        pcl::toROSMsg(current_selection, temp);
        temp.header.frame_id = frame_id;
        temp.header.stamp = ros::Time::now();

        sel_pub.publish(temp);
    }
} // namespace rviz_panel
