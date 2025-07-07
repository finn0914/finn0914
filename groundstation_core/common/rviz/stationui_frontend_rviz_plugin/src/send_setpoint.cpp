#include "send_setpoint.hpp"
#include <cmath> // NAN


namespace stationui_frontend_rviz_plugin
{

const int SETPOINT_MODE_OFF = 0;
const int SETPOINT_MODE_LAND = 1;
const int SETPOINT_MODE_XYZ = 2;
const int SETPOINT_MODE_XYZYAW = 3;
const int SETPOINT_MODE_VXVYVZ = 4;
int SETPOINT_MODE_STRINGS_COUNT = 5;
const char* SETPOINT_MODE_STRINGS[] = {
    "(Setpt. OFF)",
    "Setpt. Mode: Land",
    "Setpt. Mode: Position",
    "Setpt. Mode: Position + Yaw (Degree)",
    "Setpt. Mode: Velocity"
};

SendSetpoint::SendSetpoint(
    QWidget *parent, rclcpp::Node::SharedPtr rosnode, TfListener* tf_listener, int drone_id)
    : QGroupBox("Setpoint", parent)
{
    off_ = new QPushButton("OFF", this);
    connect(off_, SIGNAL(clicked()), this, SLOT(onSendSetpointModeOff()));
    land_ = new QPushButton("Land", this);
    connect(land_, SIGNAL(clicked()), this, SLOT(onSendSetpointModeLand()));
    xyz_ = new QPushButton("Pos.", this);
    connect(xyz_, SIGNAL(clicked()), this, SLOT(onSendSetpointModeXyz()));
    xyzyaw_ = new QPushButton("Pos. + Yaw deg.", this);
    connect(xyzyaw_, SIGNAL(clicked()), this, SLOT(onSendSetpointModeXyzyaw()));
    xyzautoyaw_ = new QPushButton("Pos. + Auto Yaw", this);
    connect(xyzautoyaw_, SIGNAL(clicked()), this, SLOT(onSendSetpointModeXyzautoyaw()));
    vxvyvz_ = new QPushButton("Vel.", this);
    connect(vxvyvz_, SIGNAL(clicked()), this, SLOT(onSendSetpointModeVxvyvz()));

    label_mode_ = new QLabel(SETPOINT_MODE_STRINGS[SETPOINT_MODE_OFF], this);
    mode_ = SETPOINT_MODE_OFF;

    x_ = new QDoubleSpinBox(this);
    x_->setSingleStep(0.1);
    x_->setMinimum(-1.0);
    x_->setMaximum(15.0);
    x_->setEnabled(false);
    y_ = new QDoubleSpinBox(this);
    y_->setSingleStep(0.1);
    y_->setMinimum(-1.0);
    y_->setMaximum(15.0);
    y_->setEnabled(false);
    z_ = new QDoubleSpinBox(this);
    z_->setSingleStep(0.1);
    z_->setMinimum(-0.5);
    z_->setMaximum(3.0);
    z_->setValue(1.0);
    z_->setEnabled(false);
    yaw_degree_ = new QSpinBox(this);
    yaw_degree_->setSingleStep(45);
    yaw_degree_->setMinimum(0);
    yaw_degree_->setMaximum(360);
    yaw_degree_->setSuffix("°");
    yaw_degree_->setEnabled(false);

    send_ = new QPushButton("Send", this);
    connect(send_, SIGNAL(clicked()), this, SLOT(onSendClicked()));

    send_origin_ = new QPushButton("Set Origin", this);
    connect(send_origin_, SIGNAL(clicked()), this, SLOT(onSendOriginClicked()));

    row1_ = new QHBoxLayout;
    row2_ = new QHBoxLayout;
    row3_ = new QHBoxLayout;
    row1_->addWidget(off_);
    row1_->addWidget(land_);
    row2_->addWidget(xyz_);
    row2_->addWidget(xyzyaw_);
    row2_->addWidget(xyzautoyaw_);
    row2_->addWidget(vxvyvz_);
    row3_->addWidget(x_);
    row3_->addWidget(y_);
    row3_->addWidget(z_);
    row3_->addWidget(yaw_degree_);

    column_ = new QVBoxLayout;
    column_->addLayout(row1_);
    column_->addLayout(row2_);
    column_->addWidget(label_mode_);
    column_->addLayout(row3_);
    column_->addWidget(send_);
    column_->addWidget(send_origin_);
    setLayout(column_);

    drone_id_ = drone_id;
    node_ = rosnode;
    tf_listener_ = tf_listener;
    char buffer[45];
    using SendSetpointMode = stationui_frontend_rviz_plugin_msgs::msg::SendSetpointMode;
    using TrajectorySetpoint = px4_msgs::msg::TrajectorySetpoint;
    using SendOrigin = stationui_frontend_rviz_plugin_msgs::msg::SendOrigin;
    sprintf(buffer, "~/drone10%d/send_setpoint_mode", drone_id_);
    pub_mode_ = node_->create_publisher<SendSetpointMode>(buffer, 10);
    sprintf(buffer, "~/drone10%d/send_setpoint_xyz", drone_id_);
    pub_xyz_ = node_->create_publisher<TrajectorySetpoint>(buffer, 10);
    sprintf(buffer, "~/drone10%d/send_setpoint_xyzyaw", drone_id_);
    pub_xyzyaw_ = node_->create_publisher<TrajectorySetpoint>(buffer, 10);
    sprintf(buffer, "~/drone10%d/send_setpoint_vxvyvz", drone_id_);
    pub_vxvyvz_ = node_->create_publisher<TrajectorySetpoint>(buffer, 10);
    sprintf(buffer, "~/drone10%d/send_origin", drone_id_);
    pub_send_origin_ = node_->create_publisher<SendOrigin>(buffer, 10);
}

void SendSetpoint::onSendSetpointModeOff() {
    mode_ = SETPOINT_MODE_OFF;
    label_mode_->setText(SETPOINT_MODE_STRINGS[SETPOINT_MODE_OFF]);
    label_mode_->repaint();
    x_->setEnabled(false);
    y_->setEnabled(false);
    z_->setEnabled(false);
    yaw_degree_->setEnabled(false);

    using SendSetpointMode = stationui_frontend_rviz_plugin_msgs::msg::SendSetpointMode;
    auto msg = SendSetpointMode();
    msg.setpoint_mode = "off";
    pub_mode_->publish(msg);
}
void SendSetpoint::onSendSetpointModeXyz() {
    mode_ = SETPOINT_MODE_XYZ;
    label_mode_->setText(SETPOINT_MODE_STRINGS[SETPOINT_MODE_XYZ]);
    label_mode_->repaint();
    x_->setEnabled(true);
    y_->setEnabled(true);
    z_->setEnabled(true);
    yaw_degree_->setEnabled(false);

    using SendSetpointMode = stationui_frontend_rviz_plugin_msgs::msg::SendSetpointMode;
    auto msg = SendSetpointMode();
    msg.setpoint_mode = "xyz";
    pub_mode_->publish(msg);
}
void SendSetpoint::onSendSetpointModeXyzyaw() {
    mode_ = SETPOINT_MODE_XYZYAW;
    label_mode_->setText(SETPOINT_MODE_STRINGS[SETPOINT_MODE_XYZYAW]);
    label_mode_->repaint();
    x_->setEnabled(true);
    y_->setEnabled(true);
    z_->setEnabled(true);
    yaw_degree_->setEnabled(true);

    using SendSetpointMode = stationui_frontend_rviz_plugin_msgs::msg::SendSetpointMode;
    auto msg = SendSetpointMode();
    msg.setpoint_mode = "xyzyaw";
    pub_mode_->publish(msg);
}
void SendSetpoint::onSendSetpointModeVxvyvz() {
    mode_ = SETPOINT_MODE_VXVYVZ;
    label_mode_->setText(SETPOINT_MODE_STRINGS[SETPOINT_MODE_VXVYVZ]);
    label_mode_->repaint();
    x_->setEnabled(true);
    y_->setEnabled(true);
    z_->setEnabled(true);
    yaw_degree_->setEnabled(false);

    using SendSetpointMode = stationui_frontend_rviz_plugin_msgs::msg::SendSetpointMode;
    auto msg = SendSetpointMode();
    msg.setpoint_mode = "vxvyvz";
    pub_mode_->publish(msg);
}
void SendSetpoint::onSendSetpointModeLand() {
    mode_ = SETPOINT_MODE_LAND;
    label_mode_->setText(SETPOINT_MODE_STRINGS[SETPOINT_MODE_LAND]);
    label_mode_->repaint();
    x_->setEnabled(false);
    y_->setEnabled(false);
    z_->setEnabled(false);
    yaw_degree_->setEnabled(false);

    using SendSetpointMode = stationui_frontend_rviz_plugin_msgs::msg::SendSetpointMode;
    auto msg = SendSetpointMode();
    msg.setpoint_mode = "land";
    pub_mode_->publish(msg);
}

void SendSetpoint::onSendClicked()
{
    using TrajectorySetpoint = px4_msgs::msg::TrajectorySetpoint;
    using PointStamped = geometry_msgs::msg::PointStamped;
    using QuaternionStamped = geometry_msgs::msg::QuaternionStamped;
    using Quaternion = tf2::Quaternion;
    auto msg = TrajectorySetpoint();
    msg.position = {NAN, NAN, NAN};
    msg.velocity = {NAN, NAN, NAN};
    msg.acceleration = {NAN, NAN, NAN};
    msg.jerk = {NAN, NAN, NAN};
    msg.yaw = NAN;
    msg.yawspeed = NAN;
    switch(mode_) {
        case SETPOINT_MODE_OFF:
        case SETPOINT_MODE_LAND:
            break;
        case SETPOINT_MODE_XYZ: {
            PointStamped position_enu;
            position_enu.point.x = x_->value();
            position_enu.point.y = y_->value();
            position_enu.point.z = z_->value();
            PointStamped position_px4 = tf_listener_->enu_to_px4(position_enu);
            msg.position[0] = position_px4.point.x;
            msg.position[1] = position_px4.point.y;
            msg.position[2] = position_px4.point.z;
            pub_xyz_->publish(msg);
        } break;
        case SETPOINT_MODE_XYZYAW: {
            PointStamped position_enu;
            position_enu.point.x = x_->value();
            position_enu.point.y = y_->value();
            position_enu.point.z = z_->value();
            PointStamped position_px4 = tf_listener_->enu_to_px4(position_enu);
            msg.position[0] = position_px4.point.x;
            msg.position[1] = position_px4.point.y;
            msg.position[2] = position_px4.point.z;

            Quaternion q_enu = tf2::Quaternion::getIdentity();
            q_enu.setRPY(0.0, 0.0, (yaw_degree_->value() * M_PI) / 180.0);
            QuaternionStamped attitude_enu;
            attitude_enu.quaternion.x = q_enu[0];
            attitude_enu.quaternion.y = q_enu[1];
            attitude_enu.quaternion.z = q_enu[2];
            attitude_enu.quaternion.w = q_enu[3];
            QuaternionStamped attitude_px4 = tf_listener_->enu_to_px4_quat(attitude_enu);
            Quaternion q_px4;
            tf2::fromMsg(attitude_enu.quaternion, q_px4);
            double px4_roll_radian, px4_pitch_radian, px4_yaw_radian;
            TfListener::my_getRPY(q_px4, px4_roll_radian, px4_pitch_radian, px4_yaw_radian);
            msg.yaw = px4_yaw_radian;
            
            pub_xyzyaw_->publish(msg);
        } break;
        case SETPOINT_MODE_VXVYVZ: {
            PointStamped velocity_enu;
            velocity_enu.point.x = x_->value();
            velocity_enu.point.y = y_->value();
            velocity_enu.point.z = z_->value();
            PointStamped velocity_px4 = tf_listener_->enu_to_px4(velocity_enu);
            msg.velocity[0] = velocity_px4.point.x;
            msg.velocity[1] = velocity_px4.point.y;
            msg.velocity[2] = velocity_px4.point.z;
            pub_vxvyvz_->publish(msg);
        } break;
    }
}

void SendSetpoint::onSendOriginClicked()
{
    using SendOrigin = stationui_frontend_rviz_plugin_msgs::msg::SendOrigin;
    auto msg = SendOrigin();
    pub_send_origin_->publish(msg);

}

}
