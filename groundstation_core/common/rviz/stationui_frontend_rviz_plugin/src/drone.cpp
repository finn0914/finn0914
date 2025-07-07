#include "drone.hpp"

namespace stationui_frontend_rviz_plugin
{

Drone::Drone(
    QWidget *parent, rclcpp::Node::SharedPtr rosnode, TfListener* tf_listener, int drone_id)
    : QWidget(parent)
{
    drone_status_ = new DroneStatus(this, rosnode, tf_listener, drone_id);
    send_state_ = new SendState(this, rosnode, drone_id);
    send_armed_ = new SendArmed(this, rosnode, drone_id);
    send_setpoint_ = new SendSetpoint(this, rosnode, tf_listener, drone_id);

    layout_ = new QVBoxLayout;
    layout_->addWidget(drone_status_);
    layout_->addWidget(send_state_);
    layout_->addWidget(send_armed_);
    layout_->addWidget(send_setpoint_);
    setLayout(layout_);

    drone_id_ = drone_id;
    node_ = rosnode;
}

}
