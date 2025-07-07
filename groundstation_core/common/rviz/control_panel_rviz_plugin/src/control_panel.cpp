#include "control_panel_rviz_plugin/control_panel.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace control_panel_rviz_plugin
{

HealthCheckPanel::HealthCheckPanel(QWidget *parent)
    : rviz_common::Panel(parent)
{
  button_ = new QPushButton("Publish", this);
  connect(button_, SIGNAL(clicked()), this, SLOT(onButtonClicked()));

  node_ = std::make_shared<rclcpp::Node>("control_panel");
  publisher_ = node_->create_publisher<std_msgs::msg::String>("rviz_panel_example_topic", 10);

  auto layout = new QVBoxLayout;
  layout->addWidget(button_);
  setLayout(layout);
}

void HealthCheckPanel::onButtonClicked()
{
  auto message = std_msgs::msg::String();
  message.data = "Button Pressed!";
  publisher_->publish(message);
}

} // namespace rviz_panel_example

PLUGINLIB_EXPORT_CLASS(control_panel_rviz_plugin::HealthCheckPanel, rviz_common::Panel)