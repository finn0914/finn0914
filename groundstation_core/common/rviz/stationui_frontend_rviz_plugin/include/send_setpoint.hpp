#ifndef SEND_SETPOINT_H
#define SEND_SETPOINT_H

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "stationui_frontend_rviz_plugin_msgs/msg/send_setpoint_mode.hpp"
#include "stationui_frontend_rviz_plugin_msgs/msg/send_origin.hpp"
#include <QtWidgets>
#include "tf_listener.hpp"
#endif

namespace stationui_frontend_rviz_plugin
{

class SendSetpoint : public QGroupBox
{
    Q_OBJECT // Enables Qt's meta-object features

public:
    SendSetpoint(
        QWidget *parent = nullptr,
        rclcpp::Node::SharedPtr rosnode = nullptr,
        TfListener* tf_listener_ = nullptr,
        int drone_id = 0
    );

private Q_SLOTS:
  void onSendSetpointModeOff();
  void onSendSetpointModeXyz();
  void onSendSetpointModeXyzyaw();
  void onSendSetpointModeXyzautoyaw();
  void onSendSetpointModeVxvyvz();
  void onSendSetpointModeLand();
  void onSendClicked();
  void onSendOriginClicked();

private:
    QVBoxLayout *column_;
    QHBoxLayout *row1_;
    QHBoxLayout *row2_;
    QHBoxLayout *row3_;

    QPushButton *off_;
    QPushButton *land_;
    QPushButton *xyz_;
    QPushButton *xyzyaw_;
    QPushButton *xyzautoyaw_;
    QPushButton *vxvyvz_;

    QLabel *label_mode_;
    int mode_;

    QDoubleSpinBox *x_;
    QDoubleSpinBox *y_;
    QDoubleSpinBox *z_;
    QSpinBox *yaw_degree_;

    QPushButton *send_;
    QPushButton *send_origin_;

    int drone_id_; // counts up from 1
    rclcpp::Node::SharedPtr node_;
    TfListener* tf_listener_;
    using SendSetpointMode = stationui_frontend_rviz_plugin_msgs::msg::SendSetpointMode;
    using TrajectorySetpoint = px4_msgs::msg::TrajectorySetpoint;
    using SendOrigin = stationui_frontend_rviz_plugin_msgs::msg::SendOrigin;
    rclcpp::Publisher<SendSetpointMode>::SharedPtr pub_mode_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr pub_xyz_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr pub_xyzyaw_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr pub_vxvyvz_;
    rclcpp::Publisher<SendOrigin>::SharedPtr pub_send_origin_;
};

}

#endif // SEND_SETPOINT_H
