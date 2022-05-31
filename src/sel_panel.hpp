#ifndef rviz_panel_H_
#define rviz_panel_H_

#include <ros/ros.h>
#include <rviz/panel.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Other ROS dependencies
#include <string.h>
#include <QSlider>
#include <QPushButton>
#include <QLabel>
#include <QWheelEvent>

#include <sensor_msgs/PointCloud2.h>
#include <interactive_markers/interactive_marker_server.h>

#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}


namespace rviz_panel
{
    /**
     *  Here we declare our new subclass of rviz::Panel. Every panel which
     *  can be added via the Panels/Add_New_Panel menu is a subclass of
     *  rviz::Panel.
     */

    class SelPanel : public rviz::Panel
    {
        /**
         * This class uses Qt slots and is a subclass of QObject, so it needs
         * the Q_OBJECT macro.
         */
        Q_OBJECT

        public:
            /**
             *  QWidget subclass constructors usually take a parent widget
             *  parameter (which usually defaults to 0).  At the same time,
             *  pluginlib::ClassLoader creates instances by calling the default
             *  constructor (with no arguments). Taking the parameter and giving
             *  a default of 0 lets the default constructor work and also lets
             *  someone using the class for something else to pass in a parent
             *  widget as they normally would with Qt.
             */
            SelPanel(QWidget * parent = 0);

            /**
             *  Now we declare overrides of rviz::Panel functions for saving and
             *  loading data from the config file.  Here the data is the topic name.
             */
            virtual void save(rviz::Config config) const;
            virtual void load(const rviz::Config & config);

        private:
            void setup_cube();
            void new_center(const std_msgs::Float32MultiArray new_cen);
            void select();
            void update_marker();
            void remove_duplicates();
            void publish_selected();
        /**
         *  Here we declare some internal slots.
         */
        private Q_SLOTS:

            void set_bag();
            void select_region();
            void unselect_region();
            void end_selection(); 
            void set_frame(int frame_num);
            void set_radius(int new_radius);
            void wheelEvent(QWheelEvent * event);

        /**
         *  Finally, we close up with protected member variables
         */
        protected:
            // ROS declaration
            ros::NodeHandle n;
            ros::Publisher cube_pub;
            ros::Publisher frame_pub;
            ros::Publisher sel_pub;
            ros::Publisher toggle_pub;
            ros::Subscriber center_sub;

            rviz::VisualizationManager* manager;
            rviz::RenderPanel* render_panel;

            std::string bag_filepath;
            int frame_idx;
            std::string frame_id;
            double radius;
            double cen_x;
            double cen_y;
            double cen_z;
            QSlider* frame_slider;
            QSlider* radius_slider;
            QPushButton* sel_region;
            QPushButton* unsel_region;
            bool is_selecting;

            std::vector<sensor_msgs::PointCloud2> frames;

            // TODO: May want to make this dynamic memory
            pcl::PointCloud<pcl::PointXYZRGB> current_selection;

            visualization_msgs::Marker cube_marker;
            
    };
} // namespace rviz_panel

#endif