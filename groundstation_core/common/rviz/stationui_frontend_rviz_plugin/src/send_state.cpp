#include "send_state.hpp"


namespace stationui_frontend_rviz_plugin
{

SendState::SendState(QWidget *parent, rclcpp::Node::SharedPtr rosnode, int drone_id)
    : QGroupBox("Command/PX4 Mode", parent)
{
    reboot_fcu_ = new QPushButton("Reboot FCU", this);
    connect(reboot_fcu_, SIGNAL(clicked()), this, SLOT(onRebootFCUClicked()));
    offboard_ = new QPushButton("OFFBOARD", this);
    connect(offboard_, SIGNAL(clicked()), this, SLOT(onOffboardClicked()));

    row_ = new QHBoxLayout;
    row_->addWidget(reboot_fcu_);
    row_->addWidget(offboard_);
    setLayout(row_);

    drone_id_ = drone_id;
    node_ = rosnode;
    char buffer[30];
    using SendStateMsg = stationui_frontend_rviz_plugin_msgs::msg::SendState;
    sprintf(buffer, "~/drone10%d/send_state", drone_id_);
    pub_send_state_ = node_->create_publisher<SendStateMsg>(buffer, 10);
}

void SendState::onRebootFCUClicked()
{
    using SendStateMsg = stationui_frontend_rviz_plugin_msgs::msg::SendState;
    auto msg = SendStateMsg();
    msg.state_string = "REBOOT_FCU";
    pub_send_state_->publish(msg);
}
void SendState::onOffboardClicked()
{
    using SendStateMsg = stationui_frontend_rviz_plugin_msgs::msg::SendState;
    auto msg = SendStateMsg();
    msg.state_string = "OFFBOARD";
    pub_send_state_->publish(msg);
}

}
