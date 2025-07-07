#ifndef SEND_STATE_H
#define SEND_STATE_H

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include "stationui_frontend_rviz_plugin_msgs/msg/send_state.hpp"
#include <QtWidgets>
#endif

namespace stationui_frontend_rviz_plugin
{

class SendState : public QGroupBox
{
    Q_OBJECT // Enables Qt's meta-object features

public:
    SendState(
        QWidget *parent = nullptr,
        rclcpp::Node::SharedPtr rosnode = nullptr,
        int drone_id = 0
    );

private Q_SLOTS:
  void onRebootFCUClicked();
  void onOffboardClicked();

private:
    QHBoxLayout *row_;
    QPushButton *reboot_fcu_;
    QPushButton *offboard_;

    int drone_id_; // counts up from 1
    rclcpp::Node::SharedPtr node_;
    using SendStateMsg = stationui_frontend_rviz_plugin_msgs::msg::SendState;
    rclcpp::Publisher<SendStateMsg>::SharedPtr pub_send_state_;
};

}

#endif // SEND_STATE_H
