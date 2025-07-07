#ifndef SEND_ARMED_H
#define SEND_ARMED_H

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include "stationui_frontend_rviz_plugin_msgs/msg/send_armed.hpp"
#include <QtWidgets>
#endif

namespace stationui_frontend_rviz_plugin
{

class SendArmed : public QGroupBox
{
    Q_OBJECT // Enables Qt's meta-object features

public:
    SendArmed(
        QWidget *parent = nullptr,
        rclcpp::Node::SharedPtr rosnode = nullptr,
        int drone_id = 0
    );

private Q_SLOTS:
  void onArmClicked();
  void onDisarmClicked();

private:
    QHBoxLayout *row_;
    QPushButton *arm_;
    QPushButton *disarm_;

    int drone_id_; // counts up from 1
    rclcpp::Node::SharedPtr node_;
    using SendArmedMsg = stationui_frontend_rviz_plugin_msgs::msg::SendArmed;
    rclcpp::Publisher<SendArmedMsg>::SharedPtr pub_send_armed_;
};

}

#endif // SEND_ARMED_H
