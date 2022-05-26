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

PLUGINLIB_EXPORT_CLASS(rviz_panel::SelPanel, rviz::Panel)

namespace rviz_panel
{
    SelPanel::SelPanel(QWidget * parent)
    :   rviz::Panel(parent)
    {
        // Construct and lay out labels and slider controls.
        QLabel* frame_label = new QLabel("Frame");
        QSlider* frame_slider = new QSlider(Qt::Horizontal);
        frame_slider->setMinimum(0);
        frame_slider->setMaximum(100);
        QLabel* radius_label = new QLabel("Radius");
        QSlider* radius_slider = new QSlider(Qt::Horizontal);
        radius_slider->setMinimum(0);
        radius_slider->setMaximum(100);
        QGridLayout* controls_layout = new QGridLayout();
        controls_layout->addWidget(frame_label, 0, 0);
        controls_layout->addWidget(frame_slider, 0, 1);
        controls_layout->addWidget(radius_label, 1, 0);
        controls_layout->addWidget(radius_slider, 1, 1);

        QPushButton* sel_bag = new QPushButton("&Bag Select", this);
        controls_layout->addWidget(sel_bag, 2, 0);


        // QString filename = QFileDialog::getOpenFileName(this, tr("Open Bag"), "/home", tr("Bags (*.bag)"));

        // QFileDialog* file_sel = new QFileDialog(this);
        // file_sel->setFileMode(QFileDialog::AnyFile);
        // file_sel->setWindowTitle("Select Bag File");
        // file_sel->setDefaultSuffix("bag");
        // file_sel->setWindowIcon(QIcon(Resources::Export16));
        // QFileDialog.->setNameFilter(tr("Bags (*.bag)"));
        // controls_layout->addWidget(file_sel, 2, 0);

        // Construct and lay out render panel.
        render_panel = new rviz::RenderPanel();
        QVBoxLayout* main_layout = new QVBoxLayout;
        main_layout->addLayout( controls_layout );

        // Set the top-level layout for this MyViz widget.
        setLayout(main_layout);

        // Make signal/slot connections.
        connect(sel_bag, &QPushButton::clicked, this, &SelPanel::set_bag);
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

        // Initialize the slider values.
        frame_slider->setValue( 0 );
        radius_slider->setValue( 33 );
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
        bag_filepath = QFileDialog::getOpenFileName(this, tr("Open Bag"), "/home", tr("Bags (*.bag)")).toStdString();
        ROS_INFO_STREAM(bag_filepath);
    }

    // This function is a Qt slot connected to a QSlider's valueChanged()
    // signal.  It sets the line thickness of the grid by changing the
    // grid's "Line Width" property.
    void SelPanel::set_frame(int frame_num) {
        return;
    }

    // This function is a Qt slot connected to a QSlider's valueChanged()
    // signal.  It sets the cell size of the grid by changing the grid's
    // "Cell Size" Property.
    void SelPanel::set_radius(int new_radius) {
        return;
    }

} // namespace rviz_panel
