#include "drone_status.hpp"
using std::placeholders::_1;


namespace stationui_frontend_rviz_plugin
{


const int STATE_TIMEOUT_INTERVAL_MS = 1200;
const int SETPOINT_TIMEOUT_INTERVAL_MS = 200;
const int POSITION_RAW_TIMEOUT_INTERVAL_MS = 100;
const int POSE_TIMEOUT_INTERVAL_MS = 100;

DroneStatus::DroneStatus(
    QWidget *parent, rclcpp::Node::SharedPtr rosnode, TfListener* tf_listener, int drone_id)
    : QGroupBox("Current Status", parent)
{
    label_state_ = new QLabel("State:", this);
    state_ = new QLabel("--", this);
    state_led_ = new LedIndicator(this);
    state_watchdog_ = new QTimer(this);
    connect(state_watchdog_, &QTimer::timeout, this, &DroneStatus::onStateWatchdogTimeout);
    state_watchdog_->setInterval(STATE_TIMEOUT_INTERVAL_MS);
    state_watchdog_->start();

    label_armed_ = new QLabel("Armed:", this);
    armed_ = new QLabel("--", this);

    label_setpoint_ = new QLabel("Setpt. Pos.:", this);
    setpoint_x_ = new QLineEdit("-.--", this);
    setpoint_y_ = new QLineEdit("-.--", this);
    setpoint_z_ = new QLineEdit("-.--", this);
    setpoint_yaw_ = new QLineEdit("--°", this);
    setpoint_x_->setReadOnly(true);
    setpoint_y_->setReadOnly(true);
    setpoint_z_->setReadOnly(true);
    setpoint_yaw_->setReadOnly(true);
    setpoint_led_ = new LedIndicator(this);
    setpoint_watchdog_ = new QTimer(this);
    connect(setpoint_watchdog_, &QTimer::timeout, this, &DroneStatus::onSetpointWatchdogTimeout);
    setpoint_watchdog_->setInterval(SETPOINT_TIMEOUT_INTERVAL_MS);
    setpoint_watchdog_->start();

    label_setpoint_velocity_ = new QLabel("Setpt. Vel.:", this);
    setpoint_vx_ = new QLineEdit("-.--", this);
    setpoint_vy_ = new QLineEdit("-.--", this);
    setpoint_vz_ = new QLineEdit("-.--", this);
    setpoint_vx_->setReadOnly(true);
    setpoint_vy_->setReadOnly(true);
    setpoint_vz_->setReadOnly(true);
    setpoint_velocity_led_ = new LedIndicator(this);
    setpoint_velocity_watchdog_ = new QTimer(this);
    connect(setpoint_velocity_watchdog_, &QTimer::timeout, this, &DroneStatus::onSetpointVelocityWatchdogTimeout);
    setpoint_velocity_watchdog_->setInterval(SETPOINT_TIMEOUT_INTERVAL_MS);
    setpoint_velocity_watchdog_->start();

    label_position_raw_ = new QLabel("Pos(Raw):", this);
    x_raw_ = new QLineEdit("-.--", this);
    y_raw_ = new QLineEdit("-.--", this);
    z_raw_ = new QLineEdit("-.--", this);
    x_raw_->setReadOnly(true);
    y_raw_->setReadOnly(true);
    z_raw_->setReadOnly(true);
    position_raw_led_ = new LedIndicator(this);
    position_raw_watchdog_ = new QTimer(this);
    connect(position_raw_watchdog_, &QTimer::timeout, this, &DroneStatus::onPositionRawWatchdogTimeout);
    position_raw_watchdog_->setInterval(POSITION_RAW_TIMEOUT_INTERVAL_MS);
    position_raw_watchdog_->start();

    label_position_ = new QLabel("Pos:", this);
    x_ = new QLineEdit("-.--", this);
    y_ = new QLineEdit("-.--", this);
    z_ = new QLineEdit("-.--", this);
    x_->setReadOnly(true);
    y_->setReadOnly(true);
    z_->setReadOnly(true);
    pose_led_ = new LedIndicator(this);
    pose_watchdog_ = new QTimer(this);
    connect(pose_watchdog_, &QTimer::timeout, this, &DroneStatus::onPoseWatchdogTimeout);
    pose_watchdog_->setInterval(POSE_TIMEOUT_INTERVAL_MS);
    pose_watchdog_->start();

    label_velocity_ = new QLabel("Vel:", this);
    vx_ = new QLineEdit("-.--", this);
    vy_ = new QLineEdit("-.--", this);
    vz_ = new QLineEdit("-.--", this);
    vx_->setReadOnly(true);
    vy_->setReadOnly(true);
    vz_->setReadOnly(true);

    label_attitude_ = new QLabel("Att:", this);
    roll_degree_ = new QLineEdit("--°", this);
    pitch_degree_ = new QLineEdit("--°", this);
    yaw_degree_ = new QLineEdit("--°", this);
    roll_degree_->setReadOnly(true);
    pitch_degree_->setReadOnly(true);
    yaw_degree_->setReadOnly(true);

    state_and_state_led_ = new QHBoxLayout;
    state_and_state_led_->addWidget(state_);
    state_and_state_led_->addWidget(state_led_);
    setpoint_ = new QHBoxLayout;
    setpoint_->addWidget(setpoint_x_);
    setpoint_->addWidget(setpoint_y_);
    setpoint_->addWidget(setpoint_z_);
    setpoint_->addWidget(setpoint_yaw_);
    setpoint_->addWidget(setpoint_led_);
    setpoint_velocity_ = new QHBoxLayout;
    setpoint_velocity_->addWidget(setpoint_vx_);
    setpoint_velocity_->addWidget(setpoint_vy_);
    setpoint_velocity_->addWidget(setpoint_vz_);
    setpoint_velocity_->addWidget(setpoint_velocity_led_);
    position_raw_ = new QHBoxLayout;
    position_raw_->addWidget(x_raw_);
    position_raw_->addWidget(y_raw_);
    position_raw_->addWidget(z_raw_);
    position_raw_->addWidget(position_raw_led_);
    position_ = new QHBoxLayout;
    position_->addWidget(x_);
    position_->addWidget(y_);
    position_->addWidget(z_);
    position_->addWidget(pose_led_);
    velocity_ = new QHBoxLayout;
    velocity_->addWidget(vx_);
    velocity_->addWidget(vy_);
    velocity_->addWidget(vz_);
    attitude_ = new QHBoxLayout;
    attitude_->addWidget(roll_degree_);
    attitude_->addWidget(pitch_degree_);
    attitude_->addWidget(yaw_degree_);

    grid_ = new QGridLayout;
    grid_->addWidget(label_state_, 0, 0);
    grid_->addWidget(label_armed_, 1, 0);
    grid_->addWidget(label_setpoint_, 2, 0);
    grid_->addWidget(label_setpoint_velocity_, 3, 0);
    grid_->addWidget(label_position_raw_, 4, 0);
    grid_->addWidget(label_position_, 5, 0);
    grid_->addWidget(label_velocity_, 6, 0);
    grid_->addWidget(label_attitude_, 7, 0);
    grid_->addLayout(state_and_state_led_, 0, 1);
    grid_->addWidget(armed_, 1, 1);
    grid_->addLayout(setpoint_, 2, 1);
    grid_->addLayout(setpoint_velocity_, 3, 1);
    grid_->addLayout(position_raw_, 4, 1);
    grid_->addLayout(position_, 5, 1);
    grid_->addLayout(velocity_, 6, 1);
    grid_->addLayout(attitude_, 7, 1);
    setLayout(grid_);

    drone_id_ = drone_id;
    node_ = rosnode;
    tf_listener_ = tf_listener;
    char buffer[60];
    using RecvState = stationui_frontend_rviz_plugin_msgs::msg::RecvState;
    using RecvArmed = stationui_frontend_rviz_plugin_msgs::msg::RecvArmed;
    using VehicleOdometry = px4_msgs::msg::VehicleOdometry;
    using TrajectorySetpoint = px4_msgs::msg::TrajectorySetpoint;

    sprintf(buffer, "/stationui_backend/drone10%d/recv_state", drone_id_);
    sub_state_ = node_->create_subscription<RecvState>(
        buffer, 10, std::bind(&DroneStatus::onRecvState, this, std::placeholders::_1));

    sprintf(buffer, "/stationui_backend/drone10%d/recv_armed", drone_id_);
    sub_armed_ = node_->create_subscription<RecvArmed>(
        buffer, 10, std::bind(&DroneStatus::onRecvArmed, this, std::placeholders::_1));

    sprintf(buffer, "/stationui_backend/drone10%d/recv_pose", drone_id_);
    sub_pose_ = node_->create_subscription<VehicleOdometry>(
        buffer, 10, std::bind(&DroneStatus::onRecvPose, this, std::placeholders::_1));

    sprintf(buffer, "/stationui_backend/drone10%d/recv_position_raw", drone_id_);
    sub_position_raw_ = node_->create_subscription<VehicleOdometry>(
        buffer, 10, std::bind(&DroneStatus::onRecvPositionRaw, this, std::placeholders::_1));

    sprintf(buffer, "/stationui_backend/drone10%d/recv_setpoint_xyz", drone_id_);
    sub_setpoint_xyz_ = node_->create_subscription<TrajectorySetpoint>(
        buffer, 10, std::bind(&DroneStatus::onRecvSetpointXyz, this, std::placeholders::_1));

    sprintf(buffer, "/stationui_backend/drone10%d/recv_setpoint_xyzyaw", drone_id_);
    sub_setpoint_xyzyaw_ = node_->create_subscription<TrajectorySetpoint>(
        buffer, 10, std::bind(&DroneStatus::onRecvSetpointXyzyaw, this, std::placeholders::_1));

    sprintf(buffer, "/stationui_backend/drone10%d/recv_setpoint_vxvyvz", drone_id_);
    sub_setpoint_vxvyvz_ = node_->create_subscription<TrajectorySetpoint>(
        buffer, 10, std::bind(&DroneStatus::onRecvSetpointVxvyvz, this, std::placeholders::_1));
}

using RecvState = stationui_frontend_rviz_plugin_msgs::msg::RecvState;
void DroneStatus::onRecvState(const RecvState& msg) const
{
    state_->setText(msg.state_string.c_str());
    state_->repaint();

    state_led_->setState(true);
    state_watchdog_->start();
}
using RecvArmed = stationui_frontend_rviz_plugin_msgs::msg::RecvArmed;
void DroneStatus::onRecvArmed(const RecvArmed& msg) const
{
    armed_->setText(msg.armed ? "ARMED" : "DISARMED");
    armed_->repaint();
}
using VehicleOdometry = px4_msgs::msg::VehicleOdometry;
using TrajectorySetpoint = px4_msgs::msg::TrajectorySetpoint;
using PointStamped = geometry_msgs::msg::PointStamped;
using QuaternionStamped = geometry_msgs::msg::QuaternionStamped;
using Quaternion = tf2::Quaternion;
int on_recv_pose_counter = 0;
void DroneStatus::onRecvPose(const VehicleOdometry& msg) const
{
    pose_led_->setState(true);
    pose_watchdog_->start();

    if (++on_recv_pose_counter % 5 != 0) {
        return;
    }

    PointStamped position_px4;
    position_px4.point.x = msg.position[0];
    position_px4.point.y = msg.position[1];
    position_px4.point.z = msg.position[2];
    PointStamped position_enu = tf_listener_->px4_to_enu(position_px4);
    PointStamped velocity_px4;
    velocity_px4.point.x = msg.velocity[0];
    velocity_px4.point.y = msg.velocity[1];
    velocity_px4.point.z = msg.velocity[2];
    PointStamped velocity_enu = tf_listener_->px4_to_enu(velocity_px4);

    QuaternionStamped attitude_px4;
    attitude_px4.quaternion.x = msg.q[0];
    attitude_px4.quaternion.y = msg.q[1];
    attitude_px4.quaternion.z = msg.q[2];
    attitude_px4.quaternion.w = msg.q[3];
    QuaternionStamped attitude_enu = tf_listener_->px4_to_enu_quat(attitude_px4);
    Quaternion q_enu;
    tf2::fromMsg(attitude_enu.quaternion, q_enu);
    double enu_roll_radian, enu_pitch_radian, enu_yaw_radian;
    TfListener::my_getRPY(q_enu, enu_roll_radian, enu_pitch_radian, enu_yaw_radian);
    int enu_roll_degree = enu_roll_radian * 180.0 / M_PI;
    int enu_pitch_degree = enu_pitch_radian * 180.0 / M_PI;
    int enu_yaw_degree = enu_yaw_radian * 180.0 / M_PI;

    char buffer[10];
    sprintf(buffer, "%.2f", position_enu.point.x);
    x_->setText(buffer);
    x_->repaint();
    sprintf(buffer, "%.2f", position_enu.point.y);
    y_->setText(buffer);
    y_->repaint();
    sprintf(buffer, "%.2f", position_enu.point.z);
    z_->setText(buffer);
    z_->repaint();
    sprintf(buffer, "%.2f", velocity_enu.point.x);
    vx_->setText(buffer);
    vx_->repaint();
    sprintf(buffer, "%.2f", velocity_enu.point.y);
    vy_->setText(buffer);
    vy_->repaint();
    sprintf(buffer, "%.2f", velocity_enu.point.z);
    vz_->setText(buffer);
    vz_->repaint();
    sprintf(buffer, "%d°", enu_roll_degree);
    roll_degree_->setText(buffer);
    roll_degree_->repaint();
    sprintf(buffer, "%d°", enu_pitch_degree);
    pitch_degree_->setText(buffer);
    pitch_degree_->repaint();
    sprintf(buffer, "%d°", enu_yaw_degree);
    yaw_degree_->setText(buffer);
    yaw_degree_->repaint();
}
int on_recv_position_raw_counter = 0;
void DroneStatus::onRecvPositionRaw(const VehicleOdometry& msg) const
{
    position_raw_led_->setState(true);
    position_raw_watchdog_->start();

    if (++on_recv_position_raw_counter % 5 != 0) {
        return;
    }

    // Already in ENU.
    char buffer[10];
    sprintf(buffer, "%.2f", msg.position[0]);
    x_raw_->setText(buffer);
    x_raw_->repaint();
    sprintf(buffer, "%.2f", msg.position[1]);
    y_raw_->setText(buffer);
    y_raw_->repaint();
    sprintf(buffer, "%.2f", msg.position[2]);
    z_raw_->setText(buffer);
    z_raw_->repaint();
}
int on_recv_setpoint_counter = 0;
void DroneStatus::onRecvSetpointXyz(const TrajectorySetpoint& msg) const
{
    setpoint_led_->setState(true);
    setpoint_watchdog_->start();

    if (++on_recv_setpoint_counter % 3 != 0) {
        return;
    }

    PointStamped position_px4;
    position_px4.point.x = msg.position[0];
    position_px4.point.y = msg.position[1];
    position_px4.point.z = msg.position[2];
    PointStamped position_enu = tf_listener_->px4_to_enu(position_px4);

    char buffer[10];
    sprintf(buffer, "%.2f", position_enu.point.x);
    setpoint_x_->setText(buffer);
    setpoint_x_->repaint();
    sprintf(buffer, "%.2f", position_enu.point.y);
    setpoint_y_->setText(buffer);
    setpoint_y_->repaint();
    sprintf(buffer, "%.2f", position_enu.point.z);
    setpoint_z_->setText(buffer);
    setpoint_z_->repaint();
    setpoint_yaw_->setEnabled(false);
}
void DroneStatus::onRecvSetpointXyzyaw(const TrajectorySetpoint& msg) const
{
    setpoint_led_->setState(true);
    setpoint_watchdog_->start();
    if (++on_recv_setpoint_counter % 3 != 0) {
        return;
    }

    PointStamped position_px4;
    position_px4.point.x = msg.position[0];
    position_px4.point.y = msg.position[1];
    position_px4.point.z = msg.position[2];
    PointStamped position_enu = tf_listener_->px4_to_enu(position_px4);

    Quaternion q_px4 = tf2::Quaternion::getIdentity();
    q_px4.setRPY(0.0, 0.0, msg.yaw);
    QuaternionStamped attitude_px4;
    attitude_px4.quaternion.x = q_px4[0];
    attitude_px4.quaternion.y = q_px4[1];
    attitude_px4.quaternion.z = q_px4[2];
    attitude_px4.quaternion.w = q_px4[3];
    QuaternionStamped attitude_enu = tf_listener_->px4_to_enu_quat(attitude_px4);
    Quaternion q_enu;
    tf2::fromMsg(attitude_enu.quaternion, q_enu);
    double enu_roll_radian, enu_pitch_radian, enu_yaw_radian;
    TfListener::my_getRPY(q_enu, enu_roll_radian, enu_pitch_radian, enu_yaw_radian);
    int enu_yaw_degree = enu_yaw_radian * 180.0 / M_PI;

    char buffer[10];
    sprintf(buffer, "%.2f", position_enu.point.x);
    setpoint_x_->setText(buffer);
    setpoint_x_->repaint();
    sprintf(buffer, "%.2f", position_enu.point.y);
    setpoint_y_->setText(buffer);
    setpoint_y_->repaint();
    sprintf(buffer, "%.2f", position_enu.point.z);
    setpoint_z_->setText(buffer);
    setpoint_z_->repaint();
    sprintf(buffer, "%d°", enu_yaw_degree);
    setpoint_yaw_->setText(buffer);
    setpoint_yaw_->repaint();
    setpoint_yaw_->setEnabled(true);
}
int on_recv_setpoint_velocity_counter = 0;
void DroneStatus::onRecvSetpointVxvyvz(const TrajectorySetpoint& msg) const
{
    setpoint_velocity_led_->setState(true);
    setpoint_velocity_watchdog_->start();

    if (++on_recv_setpoint_velocity_counter % 3 != 0) {
        return;
    }

    PointStamped velocity_px4;
    velocity_px4.point.x = msg.velocity[0];
    velocity_px4.point.y = msg.velocity[1];
    velocity_px4.point.z = msg.velocity[2];
    PointStamped velocity_enu = tf_listener_->px4_to_enu(velocity_px4);

    char buffer[10];
    sprintf(buffer, "%.2f", velocity_enu.point.x);
    setpoint_vx_->setText(buffer);
    setpoint_vx_->repaint();
    sprintf(buffer, "%.2f", velocity_enu.point.y);
    setpoint_vy_->setText(buffer);
    setpoint_vy_->repaint();
    sprintf(buffer, "%.2f", velocity_enu.point.z);
    setpoint_vz_->setText(buffer);
    setpoint_vz_->repaint();
}
void DroneStatus::onStateWatchdogTimeout() const
{
    state_led_->setState(false);
}
void DroneStatus::onSetpointWatchdogTimeout() const
{
    setpoint_led_->setState(false);
}
void DroneStatus::onSetpointVelocityWatchdogTimeout() const
{
    setpoint_velocity_led_->setState(false);
}
void DroneStatus::onPositionRawWatchdogTimeout() const
{
    position_raw_led_->setState(false);
}
void DroneStatus::onPoseWatchdogTimeout() const
{
    pose_led_->setState(false);
}

}
