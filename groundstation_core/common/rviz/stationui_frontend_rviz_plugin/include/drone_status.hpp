#ifndef DRONE_STATUS_H
#define DRONE_STATUS_H

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include "stationui_frontend_rviz_plugin_msgs/msg/recv_state.hpp"
#include "stationui_frontend_rviz_plugin_msgs/msg/recv_armed.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include <QtWidgets>
#include "tf_listener.hpp"
#include "led_indicator.hpp"
#endif

namespace stationui_frontend_rviz_plugin
{

class DroneStatus : public QGroupBox
{
    Q_OBJECT // Enables Qt's meta-object features

public:
    DroneStatus(
        QWidget *parent = nullptr,
        rclcpp::Node::SharedPtr rosnode = nullptr,
        TfListener* tf_listener_ = nullptr,
        int drone_id = 0
    );

private Q_SLOTS:

private:
    using RecvState = stationui_frontend_rviz_plugin_msgs::msg::RecvState;
    using RecvArmed = stationui_frontend_rviz_plugin_msgs::msg::RecvArmed;
    using VehicleOdometry = px4_msgs::msg::VehicleOdometry;
    using TrajectorySetpoint = px4_msgs::msg::TrajectorySetpoint;
    // using RecvSetpointXyz = stationui_frontend_rviz_plugin_msgs::msg::RecvSetpointXyz;
    // using RecvSetpointXyzyaw = stationui_frontend_rviz_plugin_msgs::msg::RecvSetpointXyzyaw;
    // using RecvSetpointVxvyvz = stationui_frontend_rviz_plugin_msgs::msg::RecvSetpointVxvyvz;
    // using RecvPositionRaw = stationui_frontend_rviz_plugin_msgs::msg::RecvPositionRaw;
    // using RecvPose = stationui_frontend_rviz_plugin_msgs::msg::RecvPose;
    void onRecvState(const RecvState& msg) const;
    void onRecvArmed(const RecvArmed& msg) const;
    void onRecvPose(const VehicleOdometry& msg) const;
    void onRecvPositionRaw(const VehicleOdometry& msg) const;
    void onRecvSetpointXyz(const TrajectorySetpoint& msg) const;
    void onRecvSetpointXyzyaw(const TrajectorySetpoint& msg) const;
    void onRecvSetpointVxvyvz(const TrajectorySetpoint& msg) const;
    void onStateWatchdogTimeout() const;
    void onSetpointWatchdogTimeout() const;
    void onSetpointVelocityWatchdogTimeout() const;
    void onPositionRawWatchdogTimeout() const;
    void onPoseWatchdogTimeout() const;

    QGridLayout *grid_;

    QLabel *label_state_;
    QHBoxLayout *state_and_state_led_;
    QLabel *state_;
    LedIndicator *state_led_;
    QTimer *state_watchdog_;

    QLabel *label_armed_;
    QLabel *armed_;

    QLabel *label_setpoint_;
    QHBoxLayout *setpoint_;
    QLineEdit *setpoint_x_;
    QLineEdit *setpoint_y_;
    QLineEdit *setpoint_z_;
    QLineEdit *setpoint_yaw_;
    LedIndicator *setpoint_led_;
    QTimer *setpoint_watchdog_;

    QLabel *label_setpoint_velocity_;
    QHBoxLayout *setpoint_velocity_;
    QLineEdit *setpoint_vx_;
    QLineEdit *setpoint_vy_;
    QLineEdit *setpoint_vz_;
    LedIndicator *setpoint_velocity_led_;
    QTimer *setpoint_velocity_watchdog_;

    QLabel *label_position_raw_;
    QHBoxLayout *position_raw_;
    QLineEdit *x_raw_;
    QLineEdit *y_raw_;
    QLineEdit *z_raw_;
    LedIndicator *position_raw_led_;
    QTimer *position_raw_watchdog_;

    QLabel *label_position_;
    QHBoxLayout *position_;
    QLineEdit *x_;
    QLineEdit *y_;
    QLineEdit *z_;
    LedIndicator *pose_led_;
    QTimer *pose_watchdog_;

    QLabel *label_velocity_;
    QHBoxLayout *velocity_;
    QLineEdit *vx_;
    QLineEdit *vy_;
    QLineEdit *vz_;

    QLabel *label_attitude_;
    QHBoxLayout *attitude_;
    QLineEdit *roll_degree_;
    QLineEdit *pitch_degree_;
    QLineEdit *yaw_degree_;

    int drone_id_; // counts up from 1
    rclcpp::Node::SharedPtr node_;
    TfListener* tf_listener_;
    rclcpp::Subscription<RecvState>::SharedPtr sub_state_;
    rclcpp::Subscription<RecvArmed>::SharedPtr sub_armed_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr sub_pose_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr sub_position_raw_;
    rclcpp::Subscription<TrajectorySetpoint>::SharedPtr sub_setpoint_xyz_;
    rclcpp::Subscription<TrajectorySetpoint>::SharedPtr sub_setpoint_xyzyaw_;
    rclcpp::Subscription<TrajectorySetpoint>::SharedPtr sub_setpoint_vxvyvz_;
};

}

#endif // DRONE_STATUS_H
