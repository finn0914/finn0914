#ifndef DRONE_H
#define DRONE_H

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <QtWidgets>
#include "tf_listener.hpp"
#include "drone_status.hpp"
#include "send_state.hpp"
#include "send_armed.hpp"
#include "send_setpoint.hpp"
#endif

namespace stationui_frontend_rviz_plugin
{

class Drone : public QWidget
{
    Q_OBJECT // Enables Qt's meta-object features

public:
    Drone(
        QWidget *parent = nullptr,
        rclcpp::Node::SharedPtr rosnode = nullptr,
        TfListener* tf_listener_ = nullptr,
        int drone_id = 0
    );

private Q_SLOTS:
  void onButtonClicked();
  void onIndexChanged(int index);

private:
    QVBoxLayout *layout_;

    DroneStatus *drone_status_;
    SendState *send_state_;
    SendArmed *send_armed_;
    SendSetpoint *send_setpoint_;

    int drone_id_; // counts up from 1
    rclcpp::Node::SharedPtr node_;
};

}

#endif // DRONE_H
