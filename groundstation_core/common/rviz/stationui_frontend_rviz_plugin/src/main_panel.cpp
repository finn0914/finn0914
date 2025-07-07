#include "main_panel.hpp"

#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

namespace stationui_frontend_rviz_plugin
{

MainPanel::MainPanel(QWidget *parent)
    : rviz_common::Panel(parent)
{
}

void MainPanel::onInitialize()
{
    auto* display_context = getDisplayContext();
    auto rviz_ros_node = display_context->getRosNodeAbstraction().lock();
    node_ = rviz_ros_node->get_raw_node();
    if (!node_) {
        throw std::runtime_error("Failed to get RosNodeAbstraction from display context");
    }
    tf_listener_ = new TfListener(node_);

    drone_panel1_ = new Drone(this, node_, tf_listener_, 1);
    drone_panel2_ = new Drone(this, node_, tf_listener_, 2);
    drone_panel3_ = new Drone(this, node_, tf_listener_, 3);

    layout_ = new QHBoxLayout;
    layout_->addWidget(drone_panel1_);
    layout_->addWidget(drone_panel2_);
    layout_->addWidget(drone_panel3_);
    setLayout(layout_);
}

void MainPanel::save(rviz_common::Config config) const
{
    rviz_common::Panel::save(config);
}

void MainPanel::load(const rviz_common::Config &config)
{
    rviz_common::Panel::load(config);
}

}

#include <pluginlib/class_list_macros.hpp>
// This needs to match plugins_description.xml
PLUGINLIB_EXPORT_CLASS(stationui_frontend_rviz_plugin::MainPanel, rviz_common::Panel)
