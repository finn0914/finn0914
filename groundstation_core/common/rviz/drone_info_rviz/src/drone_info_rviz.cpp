#include <drone_info_rviz/drone_info_rviz.hpp>

DroneRviz::DroneRviz() : Node("drone_info_rviz")
{
  try
  {

    init_commons();
  }
  catch (std::exception &ex)
  {
    RCLCPP_ERROR(this->get_logger(), ex.what());
  }
}
DroneRviz::~DroneRviz()
{
}
void DroneRviz::lateInit()
{
  try
  {
    this->vt_.reset(new rviz_visual_tools::RvizVisualTools("map_enu", "/visualization_marker", this->shared_from_this()));
    this->vt_->loadMarkerPub();
    // optional: clear old markers on startup
    this->vt_->deleteAllMarkers();
    //////////////////////

    this->text_instant_err_101_.ns = "metrics";
    this->text_instant_err_101_.id = 0;
    this->text_instant_err_101_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    this->text_instant_err_101_.action = visualization_msgs::msg::Marker::ADD;
    this->text_instant_err_101_.scale.z = 0.4; // text height (m)
    this->text_instant_err_101_.color.r = this->text_instant_err_101_.color.g = this->text_instant_err_101_.color.b = 1.0;
    this->text_instant_err_101_.color.a = 1.0;
    this->text_instant_err_101_.header.frame_id = "map_enu";

    ///////////////////////////
    this->text_instant_err_102_.ns = "metrics";
    this->text_instant_err_102_.id = 0;
    this->text_instant_err_102_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    this->text_instant_err_102_.action = visualization_msgs::msg::Marker::ADD;
    this->text_instant_err_102_.scale.z = 0.4; // text height (m)
    this->text_instant_err_102_.color.r = this->text_instant_err_102_.color.g = this->text_instant_err_102_.color.b = 1.0;
    this->text_instant_err_102_.color.a = 1.0;
    this->text_instant_err_102_.header.frame_id = "map_enu";

    ///////////////////////////
    this->text_instant_err_103_.ns = "metrics";
    this->text_instant_err_103_.id = 0;
    this->text_instant_err_103_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    this->text_instant_err_103_.action = visualization_msgs::msg::Marker::ADD;
    this->text_instant_err_103_.scale.z = 0.4; // text height (m)
    this->text_instant_err_103_.color.r = this->text_instant_err_103_.color.g = this->text_instant_err_103_.color.b = 1.0;
    this->text_instant_err_103_.color.a = 1.0;
    this->text_instant_err_103_.header.frame_id = "map_enu";
    ///
    this->post_init_timer_->cancel();
  }
  catch (std::exception &ex)
  {
    RCLCPP_ERROR(this->get_logger(), ex.what());
  }
}
void DroneRviz::init_commons()
{
  try
  {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

    std::string topic_name_vehicle_odom_101 = "/px4_" + std::to_string(1) + "/rviz/path/flown";
    std::string topic_name_traj_ref_101 = "/px4_" + std::to_string(1) + "/rviz/path/ref";
    std::string topic_name_marker_101 = "/px4_" + std::to_string(1) + "/drone_marker";
    std::string topic_name_tracker_error_101 = "/px4_" + std::to_string(1) + "/tracker_error";
    /////////////////////////////////////////////////

    std::string topic_name_vehicle_odom_102 = "/px4_" + std::to_string(2) + "/rviz/path/flown";
    std::string topic_name_traj_ref_102 = "/px4_" + std::to_string(2) + "/rviz/path/ref";
    std::string topic_name_marker_102 = "/px4_" + std::to_string(2) + "/drone_marker";
    std::string topic_name_tracker_error_102 = "/px4_" + std::to_string(2) + "/tracker_error";
    ////////////////////////////////////////////////

    std::string topic_name_vehicle_odom_103 = "/px4_" + std::to_string(3) + "/rviz/path/flown";
    std::string topic_name_traj_ref_103 = "/px4_" + std::to_string(3) + "/rviz/path/ref";
    std::string topic_name_marker_103 = "/px4_" + std::to_string(3) + "/drone_marker";
    std::string topic_name_tracker_error_103 = "/px4_" + std::to_string(3) + "/tracker_error";

    ////////////////////////////////////////////////////

    DroneRviz::traj_setpoints_101_sub_ = this->create_subscription<TrajectorySetpoint>("/px4_1/setpoint_ref", qos, std::bind(&DroneRviz::traj_setpoins_101_callback, this, std::placeholders::_1));
    DroneRviz::odom_101_sub_ = this->create_subscription<VehicleOdometry>("px4_1/odom", qos, std::bind(&DroneRviz::vehicle_odom_unbiase_101_callback, this, std::placeholders::_1));
    DroneRviz::path_flown_101_pub_ = this->create_publisher<nav_msgs::msg::Path>(topic_name_vehicle_odom_101, 10);
    DroneRviz::path_ref_101_pub_ = this->create_publisher<nav_msgs::msg::Path>(topic_name_traj_ref_101, 1);
    DroneRviz::marker_101_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(topic_name_marker_101, 1);
    DroneRviz::metric_101_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(topic_name_tracker_error_101, 1);
    ////////////////////////////////////////////////

    DroneRviz::traj_setpoints_102_sub_ = this->create_subscription<TrajectorySetpoint>("px4_2/setpoint_ref", qos, std::bind(&DroneRviz::traj_setpoins_102_callback, this, std::placeholders::_1));
    DroneRviz::odom_102_sub_ = this->create_subscription<VehicleOdometry>("px4_2/odom", qos, std::bind(&DroneRviz::vehicle_odom_unbiase_102_callback, this, std::placeholders::_1));
    DroneRviz::path_flown_102_pub_ = this->create_publisher<nav_msgs::msg::Path>(topic_name_vehicle_odom_102, 10);
    DroneRviz::path_ref_102_pub_ = this->create_publisher<nav_msgs::msg::Path>(topic_name_traj_ref_102, 1);
    DroneRviz::marker_102_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(topic_name_marker_102, 1);
    DroneRviz::metric_102_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(topic_name_tracker_error_102, 1);

    ////////////////////////////////////////////////////

    DroneRviz::traj_setpoints_103_sub_ = this->create_subscription<TrajectorySetpoint>("px4_3/setpoint_ref", qos, std::bind(&DroneRviz::traj_setpoins_103_callback, this, std::placeholders::_1));
    DroneRviz::odom_103_sub_ = this->create_subscription<VehicleOdometry>("px4_3/odom", qos, std::bind(&DroneRviz::vehicle_odom_unbiase_103_callback, this, std::placeholders::_1));
    DroneRviz::path_flown_103_pub_ = this->create_publisher<nav_msgs::msg::Path>(topic_name_vehicle_odom_103, 10);
    DroneRviz::path_ref_103_pub_ = this->create_publisher<nav_msgs::msg::Path>(topic_name_traj_ref_103, 1);
    DroneRviz::marker_103_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(topic_name_marker_103, 1);
    DroneRviz::metric_103_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(topic_name_tracker_error_103, 1);

    ///////////////////////////
    // DroneRviz::initial_visual_text();
    RCLCPP_DEBUG(this->get_logger(), "initial the subs and pubs 1");

    DroneRviz::post_init_timer_ = create_wall_timer(10ms, std::bind(&DroneRviz::lateInit, this));
    RCLCPP_DEBUG(this->get_logger(), "initial the subs and pubs 2");

    using namespace std::chrono_literals;
    DroneRviz::error_timer_ = this->create_wall_timer(100ms, std::bind(&DroneRviz::publish_error, this));
    RCLCPP_DEBUG(this->get_logger(), "initial the subs and pubs 3");
  }
  catch (std::exception &ex)
  {
    RCLCPP_ERROR(this->get_logger(), ex.what());
  }
}
void DroneRviz::vehicle_odom_unbiase_101_callback(const VehicleOdometry &msg)
{
  RCLCPP_DEBUG(this->get_logger(), "vehicle_odom_unbiase_101_callback");

  auto msg_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();

  msg_pose->pose.position.x = msg.position[0]; 
  msg_pose->pose.position.y = msg.position[1];
  msg_pose->pose.position.z = msg.position[2];

  msg_pose->pose.orientation.w = msg.q[0];
  msg_pose->pose.orientation.x = msg.q[1]; 
  msg_pose->pose.orientation.y = msg.q[2];
  msg_pose->pose.orientation.z = msg.q[3];

  msg_pose->header.frame_id = "map_enu";
  msg_pose->header.stamp = this->now();

  this->flown_101_.header = msg_pose->header; // this->flown_101_ is a class member Path
  this->flown_101_.poses.push_back(*msg_pose);
  this->path_flown_101_pub_->publish(this->flown_101_);

  visualization_msgs::msg::Marker mk;
  mk.header = msg_pose->header;
  mk.ns = "drone101";
  mk.id = 0;
  mk.type = visualization_msgs::msg::Marker::CUBE;
  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.pose = msg_pose->pose;
  mk.scale.x = mk.scale.y = mk.scale.z = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.color.a = 1.0;
  marker_101_pub_->publish(mk);
}
void DroneRviz::vehicle_odom_unbiase_102_callback(const VehicleOdometry &msg)
{
  RCLCPP_DEBUG(this->get_logger(), "vehicle_odom_unbiase_102_callback ");

  auto msg_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();

  msg_pose->pose.position.x = msg.position[0]; 
  msg_pose->pose.position.y = msg.position[1];
  msg_pose->pose.position.z = msg.position[2];

  msg_pose->pose.orientation.w = msg.q[0];
  msg_pose->pose.orientation.x = msg.q[1]; 
  msg_pose->pose.orientation.y = msg.q[2];
  msg_pose->pose.orientation.z = msg.q[3];

  msg_pose->header.frame_id = "map_enu";
  msg_pose->header.stamp = this->now();

  this->flown_102_.header = msg_pose->header; // this->flown_102_ is a class member Path
  this->flown_102_.poses.push_back(*msg_pose);
  this->path_flown_102_pub_->publish(this->flown_102_);

  visualization_msgs::msg::Marker mk;
  mk.header = msg_pose->header;
  mk.ns = "drone102";
  mk.id = 0;
  mk.type = visualization_msgs::msg::Marker::CUBE;
  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.pose = msg_pose->pose;
  mk.scale.x = mk.scale.y = mk.scale.z = 0.3;
  mk.color.r = 0.0;
  mk.color.g = 1.0;
  mk.color.b = 0.0;
  mk.color.a = 1.0;
  marker_102_pub_->publish(mk);
}
void DroneRviz::vehicle_odom_unbiase_103_callback(const VehicleOdometry &msg)
{
  RCLCPP_DEBUG(this->get_logger(), "vehicle_odom_unbiase_103_callback");

  auto msg_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();

  msg_pose->pose.position.x = msg.position[0]; 
  msg_pose->pose.position.y = msg.position[1];
  msg_pose->pose.position.z = msg.position[2];

  msg_pose->pose.orientation.w = msg.q[0];
  msg_pose->pose.orientation.x = msg.q[1]; 
  msg_pose->pose.orientation.y = msg.q[2];
  msg_pose->pose.orientation.z = msg.q[3];

  msg_pose->header.frame_id = "map_enu";
  msg_pose->header.stamp = this->now();

  this->flown_103_.header = msg_pose->header; // this->flown_103_ is a class member Path
  this->flown_103_.poses.push_back(*msg_pose);
  this->path_flown_103_pub_->publish(this->flown_103_);

  visualization_msgs::msg::Marker mk;
  mk.header = msg_pose->header;
  mk.ns = "drone103";
  mk.id = 0;
  mk.type = visualization_msgs::msg::Marker::CUBE;
  mk.action = visualization_msgs::msg::Marker::ADD;
  mk.pose = msg_pose->pose;
  mk.scale.x = mk.scale.y = mk.scale.z = 0.3;
  mk.color.r = 0.0;
  mk.color.g = 0.0;
  mk.color.b = 1.0;
  mk.color.a = 1.0;
  marker_103_pub_->publish(mk);
}
// class member that keeps the whole reference line
void DroneRviz::traj_setpoins_101_callback(const TrajectorySetpoint &msg) // or TrajectorySetpoint
{
  /* -------- 1. build a PoseStamped from the incoming message -------- */
  RCLCPP_DEBUG(this->get_logger(), "traj_setpoins_101_callback");

  geometry_msgs::msg::PoseStamped pose;

  // a) header
  pose.header.frame_id = "map_enu";    // same world frame you use in RViz
  pose.header.stamp = this->now(); // rclcpp::Node::now()

  // b) position  (E-N-U for RViz)
  pose.pose.position.x = msg.position[0];
  pose.pose.position.y = msg.position[1];
  pose.pose.position.z = msg.position[2];

  // c) orientation  (swap + negate to ENU)
  // pose.pose.orientation.w = msg.q[0];
  // pose.pose.orientation.x = msg.q[1];
  // pose.pose.orientation.y = msg.q[2];
  // pose.pose.orientation.z = msg.q[3];

  /* -------- 2. append to a persistent Path  -------- */
  this->ref_path_101_.header = pose.header; // keep header up to date
  this->ref_path_101_.poses.push_back(pose);

  this->path_ref_101_pub_->publish(this->ref_path_101_); // Path display grows over time
}
void DroneRviz::traj_setpoins_102_callback(const TrajectorySetpoint &msg) // or TrajectorySetpoint
{
  /* -------- 1. build a PoseStamped from the incoming message -------- */
  RCLCPP_DEBUG(this->get_logger(), "traj_setpoins_102_callback");

  geometry_msgs::msg::PoseStamped pose;

  // a) header
  pose.header.frame_id = "map_enu";    // same world frame you use in RViz
  pose.header.stamp = this->now(); // rclcpp::Node::now()

  // b) position  (E-N-U for RViz)
  pose.pose.position.x = msg.position[0];
  pose.pose.position.y = msg.position[1];
  pose.pose.position.z = msg.position[2];

  // c) orientation  (swap + negate to ENU)
  // pose.pose.orientation.w = msg.q[0];
  // pose.pose.orientation.x = msg.q[1];
  // pose.pose.orientation.y = msg.q[2];
  // pose.pose.orientation.z = msg.q[3];

  /* -------- 2. append to a persistent Path  -------- */
  this->ref_path_102_.header = pose.header; // keep header up to date
  this->ref_path_102_.poses.push_back(pose);

  this->path_ref_102_pub_->publish(this->ref_path_102_); // Path display grows over time
}
void DroneRviz::traj_setpoins_103_callback(const TrajectorySetpoint &msg) // or TrajectorySetpoint
{
  /* -------- 1. build a PoseStamped from the incoming message -------- */
  RCLCPP_DEBUG(this->get_logger(), "traj_setpoins_103_callback");

  geometry_msgs::msg::PoseStamped pose;

  // a) header
  pose.header.frame_id = "map_enu";    // same world frame you use in RViz
  pose.header.stamp = this->now(); // rclcpp::Node::now()

  // b) position  (E-N-U for RViz)
  pose.pose.position.x = msg.position[0];
  pose.pose.position.y = msg.position[1];
  pose.pose.position.z = msg.position[2];

  // c) orientation  (swap + negate to ENU)
  // pose.pose.orientation.w = msg.q[0];
  // pose.pose.orientation.x = msg.q[1];
  // pose.pose.orientation.y = msg.q[2];
  // pose.pose.orientation.z = msg.q[3];

  /* -------- 2. append to a persistent Path  -------- */
  this->ref_path_103_.header = pose.header; // keep header up to date
  this->ref_path_103_.poses.push_back(pose);

  this->path_ref_103_pub_->publish(this->ref_path_103_); // Path display grows over time
}
double DroneRviz::tracking_error(nav_msgs::msg::Path ref_path, const geometry_msgs::msg::Pose &cur) const
{
  double min_sq = std::numeric_limits<double>::max();

  for (const auto &p : ref_path.poses)
  { // ref_path_ already stored
    double dx = cur.position.x - p.pose.position.x;
    double dy = cur.position.y - p.pose.position.y;
    double dz = cur.position.z - p.pose.position.z;
    double sq = dx * dx + dy * dy + dz * dz;
    if (sq < min_sq)
      min_sq = sq;
  }
  return std::sqrt(min_sq); // metres
}

void DroneRviz::publish_error()
{
  if (!flown_101_.poses.empty() && !ref_path_101_.poses.empty()) {

    RCLCPP_DEBUG(this->get_logger(), "  this->vt_->trigger(); 0");

    const auto &cur_pose_101 = this->flown_101_.poses.back().pose;
    double err = DroneRviz::tracking_error(this->ref_path_101_, cur_pose_101);
    this->accum_error_101_ += err;
    // overwrite reusable marker
    this->text_instant_err_101_.header.stamp = this->now();
    this->text_instant_err_101_.pose.position = cur_pose_101.position; // put number above vehicle
    this->text_instant_err_101_.pose.position.z += 1.0;                // 1 m up
    std::ostringstream ss_101;
    ss_101 << "D1:"<<std::fixed << std::setprecision(2) << err << "m";
    this->text_instant_err_101_.text = ss_101.str();

    this->metric_101_pub_->publish(this->text_instant_err_101_);

  };


  ///////////////////////////////////////////////////////////////////////
  if (!flown_102_.poses.empty() && !ref_path_102_.poses.empty()) {

    const auto &cur_pose_102 = this->flown_102_.poses.back().pose;
    double err = DroneRviz::tracking_error(this->ref_path_102_,cur_pose_102);
    this->accum_error_102_+=err;

    // overwrite reusable marker
    this->text_instant_err_102_.header.stamp    = this->now();
    this->text_instant_err_102_.pose.position   = cur_pose_102.position;    // put number above vehicle
    this->text_instant_err_102_.pose.position.z += 1.0;                 // 1 m up
    std::ostringstream ss_102;
    ss_102 << "D2:"<<std::fixed << std::setprecision(2) << err << "m";
    this->text_instant_err_102_.text = ss_102.str();

    this->metric_102_pub_->publish(this->text_instant_err_102_);
    }
  // ////////////////////////////////////////////////////////////////////////
  if (!flown_103_.poses.empty() && !ref_path_103_.poses.empty()) {

    const auto &cur_pose_103 = this->flown_103_.poses.back().pose;
    double err = DroneRviz::tracking_error(this->ref_path_103_,cur_pose_103);
    this->accum_error_103_+=err;

    // overwrite reusable marker
    this->text_instant_err_103_.header.stamp    = this->now();
    this->text_instant_err_103_.pose.position   = cur_pose_103.position;    // put number above vehicle
    this->text_instant_err_103_.pose.position.z += 1.0;                 // 1 m up
    std::ostringstream ss_103;
    ss_103 << "D3:"<<std::fixed << std::setprecision(2) << err << "m";
    this->text_instant_err_103_.text = ss_103.str();

    this->metric_103_pub_->publish(this->text_instant_err_103_);
    }
  ////////////////////////////////////////////////////////////////////////
  RCLCPP_DEBUG(this->get_logger(), "  this->vt_->trigger(); 1");

  this->vt_->deleteAllMarkers(); // optional: clear old text so you don't accumulate markers

  ////// Drone101 total RMSE fixed at left top corner
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation() << 0.0, 0.0, 0.0;
  RCLCPP_DEBUG(this->get_logger(), "  this->vt_->trigger(); 2");

  // helper lambda to publish one line
  auto pub_line = [&](double error, std::string drone_name,double x_offset,double y_offset,double z_offset)
  {
    std::ostringstream ss;
    ss <<drone_name<<":"<<std::fixed << std::setprecision(2) << error << "m";

    // move the text pose down by z_offset (so lines don’t overlap)
    text_pose.translation().x() =  x_offset;
    text_pose.translation().y() =  y_offset;
    text_pose.translation().z() =  z_offset;
    RCLCPP_DEBUG(this->get_logger(), "  this->vt_->trigger(); 3");

    this->vt_->publishText(
        text_pose, // world‐frame pose
        ss.str(),  // the string
        rviz_visual_tools::WHITE,
        rviz_visual_tools::XXXXLARGE, // pick a readable size
        false                         // static_id=false to allow multiple texts
    );
  };
  RCLCPP_DEBUG(this->get_logger(), "  this->vt_->trigger(); 4");

  // Drone 101 at z=2.0
  pub_line(accum_error_101_,"D101", 0.0, -6.0, 2.0);
  // Drone 102 just below (e.g. z=1.9)
  pub_line(accum_error_102_,"D102", -1.0, -6.0, 2.0);
  // Drone 103 below that (z=1.8)
  pub_line(accum_error_101_,"D103", -2.0, -6.0, 2.0);

  // 3) send them all to RViz in one go
  RCLCPP_DEBUG(this->get_logger(), "  this->vt_->trigger(); 5");

  this->vt_->trigger();
}
