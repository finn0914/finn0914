#ifndef CONTROL_PANEL_RVIZ_PLUGIN_HEALTH_CHECK_PANEL_HPP
#define CONTROL_PANEL_RVIZ_PLUGIN_HEALTH_CHECK_PANEL_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rviz_common/panel.hpp>
#include <QPushButton>
#include <QVBoxLayout>

namespace control_panel_rviz_plugin
{

class HealthCheckPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  HealthCheckPanel(QWidget *parent = nullptr);

private Q_SLOTS:
  void onButtonClicked();

private:
  QPushButton *button_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Node::SharedPtr node_;
};

} // namespace control_panel_rviz_plugin

#endif // CONTROL_PANEL_RVIZ_PLUGIN_HEALTH_CHECK_PANEL_HPP
