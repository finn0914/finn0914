#include "send_armed.hpp"


namespace stationui_frontend_rviz_plugin
{

SendArmed::SendArmed(QWidget *parent, rclcpp::Node::SharedPtr rosnode, int drone_id)
    : QGroupBox("Arming", parent)
{
    arm_ = new QPushButton("Arm", this);
    connect(arm_, SIGNAL(clicked()), this, SLOT(onArmClicked()));
    disarm_ = new QPushButton("Disarm", this);
    connect(disarm_, SIGNAL(clicked()), this, SLOT(onDisarmClicked()));

    row_ = new QHBoxLayout;
    row_->addWidget(arm_);
    row_->addWidget(disarm_);
    setLayout(row_);

    drone_id_ = drone_id;
    node_ = rosnode;
    char buffer[30];
    using SendArmedMsg = stationui_frontend_rviz_plugin_msgs::msg::SendArmed;
    sprintf(buffer, "~/drone10%d/send_armed", drone_id_);
    pub_send_armed_ = node_->create_publisher<SendArmedMsg>(buffer, 10);
}

void SendArmed::onArmClicked()
{
    using SendArmedMsg = stationui_frontend_rviz_plugin_msgs::msg::SendArmed;
    auto msg = SendArmedMsg();
    msg.armed = true;
    pub_send_armed_->publish(msg);
}
void SendArmed::onDisarmClicked()
{
    using SendArmedMsg = stationui_frontend_rviz_plugin_msgs::msg::SendArmed;
    auto msg = SendArmedMsg();
    msg.armed = false;
    pub_send_armed_->publish(msg);
}

}
