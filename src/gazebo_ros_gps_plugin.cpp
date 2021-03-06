/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 * Copyright (C) 2017-2018 PX4 Pro Development Team
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
/**
 * @brief GPS Plugin
 *
 * This plugin publishes GPS and Groundtruth data to be used and propagated
 *
 * @author Amy Wagoner <arwagoner@gmail.com>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <gazebo_ros_gps_plugin.h>

#include <boost/algorithm/string.hpp>
#include <memory>
#include <rclcpp/utilities.hpp>

using namespace std;

namespace gazebo
{
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosGpsPlugin)

GazeboRosGpsPlugin::GazeboRosGpsPlugin() : SensorPlugin() {
}

GazeboRosGpsPlugin::~GazeboRosGpsPlugin() {
  if (updateSensorConnection_)
    updateSensorConnection_->~Connection();
  parentSensor_.reset();
  world_->Reset();
}

void GazeboRosGpsPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
  // Get then name of the parent sensor
  parentSensor_ = std::dynamic_pointer_cast<sensors::GpsSensor>(_parent);

  if (!parentSensor_)
    gzthrow("GazeboRosGpsPlugin requires a GPS Sensor as its parent");

  // Get the root model name
  const string        scopedName = _parent->ParentName();
  vector<std::string> names_splitted;
  boost::split(names_splitted, scopedName, boost::is_any_of("::"));
  names_splitted.erase(std::remove_if(begin(names_splitted), end(names_splitted), [](const string& name) { return name.size() == 0; }), end(names_splitted));
  const string rootModelName = names_splitted.front();  // The first element is the name of the root model

  // the second to the last name is the model name
  const string parentSensorModelName = names_splitted.rbegin()[1];

  // store the model name
  model_name_ = names_splitted[0];

  // get gps topic name
  if (_sdf->HasElement("topic")) {
    gps_topic_ = _sdf->GetElement("topic")->Get<std::string>();
  } else {
    // if not set by parameter, get the topic name from the model name
    gps_topic_ = parentSensorModelName;
    gzwarn << "[gazebo_ros_gps_plugin]: " + names_splitted.front() + "::" + names_splitted.rbegin()[1] + " using gps topic \"" << parentSensorModelName
           << "\"\n";
  }

  // Store the pointer to the world.
  world_ = physics::get_world(parentSensor_->WorldName());

#if GAZEBO_MAJOR_VERSION >= 9
  last_time_     = world_->SimTime();
  last_gps_time_ = world_->SimTime();
  start_time_    = world_->StartTime();
#else
  last_time_     = world_->GetSimTime();
  last_gps_time_ = world_->GetSimTime();
  start_time_    = world_->GetStartTime();
#endif

  // Use environment variables if set for home position.
  const char* env_lat = std::getenv("PX4_HOME_LAT");
  const char* env_lon = std::getenv("PX4_HOME_LON");
  const char* env_alt = std::getenv("PX4_HOME_ALT");

  // Get noise param
  if (_sdf->HasElement("gpsNoise")) {
    getSdfParam<bool>(_sdf, "gpsNoise", gps_noise_, gps_noise_);
  } else {
    gps_noise_ = false;
  }

  const bool world_has_origin = checkWorldHomePosition(world_, world_latitude_, world_longitude_, world_altitude_);

  if (env_lat) {
    lat_home_ = std::stod(env_lat) * M_PI / 180.0;
    gzmsg << "[gazebo_ros_gps_plugin] Home latitude is set to " << std::stod(env_lat) << ".\n";
  } else if (world_has_origin) {
    lat_home_ = world_latitude_;
    gzmsg << "[gazebo_ros_gps_plugin] Home latitude is set to " << lat_home_ << ".\n";
  } else if (_sdf->HasElement("homeLatitude")) {
    double latitude;
    getSdfParam<double>(_sdf, "homeLatitude", latitude, lat_home_);
    lat_home_ = latitude * M_PI / 180.0;
  }

  if (env_lon) {
    lon_home_ = std::stod(env_lon) * M_PI / 180.0;
    gzmsg << "[gazebo_ros_gps_plugin] Home longitude is set to " << std::stod(env_lon) << ".\n";
  } else if (world_has_origin) {
    lon_home_ = world_longitude_;
    gzmsg << "[gazebo_ros_gps_plugin] Home longitude is set to " << lon_home_ << ".\n";
  } else if (_sdf->HasElement("homeLongitude")) {
    double longitude;
    getSdfParam<double>(_sdf, "homeLongitude", longitude, lon_home_);
    lon_home_ = longitude * M_PI / 180.0;
  }

  if (env_alt) {
    alt_home_ = std::stod(env_alt);
    gzmsg << "[gazebo_ros_gps_plugin] Home altitude is set to " << alt_home_ << ".\n";
  } else if (world_has_origin) {
    alt_home_ = world_altitude_;
    gzmsg << "[gazebo_ros_gps_plugin] Home altitude is set to " << alt_home_ << ".\n";
  } else if (_sdf->HasElement("homeAltitude")) {
    getSdfParam<double>(_sdf, "homeAltitude", alt_home_, alt_home_);
  }

  // get random walk in XY plane
  if (_sdf->HasElement("gpsXYRandomWalk")) {
    getSdfParam<double>(_sdf, "gpsXYRandomWalk", gps_xy_random_walk_, kDefaultGpsXYRandomWalk);
  } else {
    gzwarn << "[gazebo_ros_gps_plugin] Using default random walk in XY plane: " << kDefaultGpsXYRandomWalk << "\n";
  }

  // get random walk in Z
  if (_sdf->HasElement("gpsZRandomWalk")) {
    getSdfParam<double>(_sdf, "gpsZRandomWalk", gps_z_random_walk_, kDefaultGpsZRandomWalk);
  } else {
    gzwarn << "[gazebo_ros_gps_plugin] Using default random walk in Z: " << kDefaultGpsZRandomWalk << "\n";
  }

  // get position noise density in XY plane
  if (_sdf->HasElement("gpsXYNoiseDensity")) {
    getSdfParam<double>(_sdf, "gpsXYNoiseDensity", gps_xy_noise_density_, kDefaultGpsXYNoiseDensity);
  } else {
    gzwarn << "[gazebo_ros_gps_plugin] Using default position noise density in XY plane: " << kDefaultGpsXYNoiseDensity << "\n";
  }

  // get position noise density in Z
  if (_sdf->HasElement("gpsZNoiseDensity")) {
    getSdfParam<double>(_sdf, "gpsZNoiseDensity", gps_z_noise_density_, kDefaultGpsZNoiseDensity);
  } else {
    gzwarn << "[gazebo_ros_gps_plugin] Using default position noise density in Z: " << kDefaultGpsZNoiseDensity << "\n";
  }

  // get velocity noise density in XY plane
  if (_sdf->HasElement("gpsVXYNoiseDensity")) {
    getSdfParam<double>(_sdf, "gpsVXYNoiseDensity", gps_vxy_noise_density_, kDefaultGpsVXYNoiseDensity);
  } else {
    gzwarn << "[gazebo_ros_gps_plugin] Using default velocity noise density in XY plane: " << kDefaultGpsVXYNoiseDensity << "\n";
  }

  // get velocity noise density in Z
  if (_sdf->HasElement("gpsVZNoiseDensity")) {
    getSdfParam<double>(_sdf, "gpsVZNoiseDensity", gps_vz_noise_density_, kDefaultGpsVZNoiseDensity);
  } else {
    gzwarn << "[gazebo_ros_gps_plugin] Using default velocity noise density in Z: " << kDefaultGpsVZNoiseDensity << "\n";
  }

  namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_ros_gps_plugin] Please specify a robotNamespace.\n";
  }

  // get update rate
  if (_sdf->HasElement("update_rate")) {
    getSdfParam<double>(_sdf, "update_rate", update_rate_, kDefaultUpdateRate);
  } else {
    update_rate_ = kDefaultUpdateRate;
    gzwarn << "[gazebo_ros_gps_plugin] Using default update rate of " << kDefaultUpdateRate << "hz \n";
  }

  // Get groundtruth rate
  if (_sdf->HasElement("groundtruth_update_rate")) {
    getSdfParam<double>(_sdf, "groundtruth_update_rate", groundtruth_update_rate_, kDefaultGroundthruthUpdateRate);
  } else {
    groundtruth_update_rate_ = kDefaultGroundthruthUpdateRate;
    gzwarn << "[gazebo_ros_gps_plugin] Using default update rate for groundtruth of " << kDefaultGroundthruthUpdateRate << "hz \n";
  }

  parentSensor_->SetUpdateRate(update_rate_);

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  parentSensor_->SetActive(false);
  updateSensorConnection_ = parentSensor_->ConnectUpdated(boost::bind(&GazeboRosGpsPlugin::OnSensorUpdate, this));
  parentSensor_->SetActive(true);

  // Listen to the update event. This event is broadcast every simulation iteration.
  updateWorldConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosGpsPlugin::OnWorldUpdate, this, _1));

  gravity_W_ = world_->Gravity();

  gps_pub_ = node_handle_->Advertise<sensor_msgs::msgs::SITLGps>("~/" + model_name_ + "/link/" + gps_topic_, 10);

  // Initiate ROS2 stuff
  int    argc = 0;
  char** argv = NULL;
  if (!rclcpp::ok()) {
    rclcpp::init(argc, argv);
  }
  ros_node_           = gazebo_ros::Node::Get();
  activation_service_ = ros_node_->create_service<std_srvs::srv::SetBool>(
      "/gazebo/gps_plugin/activate", std::bind(&GazeboRosGpsPlugin::activationCallback, this, std::placeholders::_1, std::placeholders::_2));
  set_bad_gps_service_ = ros_node_->create_service<std_srvs::srv::SetBool>(
      "/gazebo/gps_plugin/set_bad_gps", std::bind(&GazeboRosGpsPlugin::setBadGpsCallback, this, std::placeholders::_1, std::placeholders::_2));

  rclcpp::QoS qos(rclcpp::KeepLast(3));
  groundtruth_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>("/" + model_name_ + "/groundtruth", qos);

  groundtruth_msg_.header.frame_id = "gazebo_world";

  timer_groundtruth_ =
      ros_node_->create_wall_timer(std::chrono::duration<double>(1.0 / groundtruth_update_rate_), std::bind(&GazeboRosGpsPlugin::GroundtruthRoutine, this));
}

void GazeboRosGpsPlugin::GroundtruthRoutine(void) {
    std::scoped_lock lock(groundtruth_mutex_);
    groundtruth_pub_->publish(groundtruth_msg_);
}

void GazeboRosGpsPlugin::OnWorldUpdate(const common::UpdateInfo& /*_info*/) {

  {
    std::scoped_lock lock(active_mutex_);
    if (!active_) {
      return;
    }
  }

  // Store the pointer to the model.
  if (model_ == NULL)
#if GAZEBO_MAJOR_VERSION >= 9
    model_ = world_->ModelByName(model_name_);
#else
    model_ = world_->GetModel(model_name_);
#endif

  common::Time current_time;

#if GAZEBO_MAJOR_VERSION >= 9
  current_time = world_->SimTime();
#else
  current_time                                        = world_->GetSimTime();
#endif
  double dt = (current_time - last_time_).Double();

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d T_W_I = model_->WorldPose();
#else
  ignition::math::Pose3d   T_W_I                      = ignitionFromGazeboMath(model_->GetWorldPose());
#endif
  // Use the model world position for GPS
  ignition::math::Vector3d&                     pos_W_I = T_W_I.Pos();
  [[maybe_unused]] ignition::math::Quaterniond& att_W_I = T_W_I.Rot();

  // Use the models' world position for GPS velocity.
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d velocity_current_W         = model_->WorldLinearVel();
  ignition::math::Vector3d angular_velocity_current_W = model_->WorldAngularVel();
#else
  ignition::math::Vector3d velocity_current_W         = ignitionFromGazeboMath(model_->GetWorldLinearVel());
  ignition::math::Vector3d angular_velocity_current_W = ignitionFromGazeboMath(model_->GetWorldAngularVel());
#endif

  ignition::math::Vector3d velocity_current_W_xy = velocity_current_W;
  velocity_current_W_xy.Z()                      = 0;

  // update noise parameters if gps_noise_ is set
  if (gps_noise_) {
    noise_gps_pos_.X()   = gps_xy_noise_density_ * sqrt(dt) * randn_(rand_);
    noise_gps_pos_.Y()   = gps_xy_noise_density_ * sqrt(dt) * randn_(rand_);
    noise_gps_pos_.Z()   = gps_z_noise_density_ * sqrt(dt) * randn_(rand_);
    noise_gps_vel_.X()   = gps_vxy_noise_density_ * sqrt(dt) * randn_(rand_);
    noise_gps_vel_.Y()   = gps_vxy_noise_density_ * sqrt(dt) * randn_(rand_);
    noise_gps_vel_.Z()   = gps_vz_noise_density_ * sqrt(dt) * randn_(rand_);
    random_walk_gps_.X() = gps_xy_random_walk_ * sqrt(dt) * randn_(rand_);
    random_walk_gps_.Y() = gps_xy_random_walk_ * sqrt(dt) * randn_(rand_);
    random_walk_gps_.Z() = gps_z_random_walk_ * sqrt(dt) * randn_(rand_);
  } else {
    noise_gps_pos_.X()   = 0.0;
    noise_gps_pos_.Y()   = 0.0;
    noise_gps_pos_.Z()   = 0.0;
    noise_gps_vel_.X()   = 0.0;
    noise_gps_vel_.Y()   = 0.0;
    noise_gps_vel_.Z()   = 0.0;
    random_walk_gps_.X() = 0.0;
    random_walk_gps_.Y() = 0.0;
    random_walk_gps_.Z() = 0.0;
  }

  // gps bias integration
  gps_bias_.X() += random_walk_gps_.X() * dt - gps_bias_.X() / gps_corellation_time_;
  gps_bias_.Y() += random_walk_gps_.Y() * dt - gps_bias_.Y() / gps_corellation_time_;
  gps_bias_.Z() += random_walk_gps_.Z() * dt - gps_bias_.Z() / gps_corellation_time_;

  // reproject position with noise into geographic coordinates
  auto pos_with_noise = pos_W_I + noise_gps_pos_ + gps_bias_;
  auto latlon         = reproject(pos_with_noise, lat_home_, lon_home_, alt_home_);

  // fill groundtruth
  {
    std::lock_guard<std::mutex> lock(groundtruth_mutex_);
    groundtruth_msg_.header.stamp = ros_node_->get_clock()->now();

    groundtruth_msg_.pose.pose.position.x = pos_W_I.X();
    groundtruth_msg_.pose.pose.position.y = pos_W_I.Y();
    groundtruth_msg_.pose.pose.position.z = pos_W_I.Z();

    groundtruth_msg_.pose.pose.orientation.x = att_W_I.X();
    groundtruth_msg_.pose.pose.orientation.y = att_W_I.Y();
    groundtruth_msg_.pose.pose.orientation.z = att_W_I.Z();
    groundtruth_msg_.pose.pose.orientation.w = att_W_I.W();

    groundtruth_msg_.twist.twist.linear.x = velocity_current_W.X();
    groundtruth_msg_.twist.twist.linear.y = velocity_current_W.Y();
    groundtruth_msg_.twist.twist.linear.z = velocity_current_W.Z();

    groundtruth_msg_.twist.twist.angular.x = angular_velocity_current_W.X();
    groundtruth_msg_.twist.twist.angular.y = angular_velocity_current_W.Y();
    groundtruth_msg_.twist.twist.angular.z = angular_velocity_current_W.Z();
  }

  // fill SITLGps msg
  sensor_msgs::msgs::SITLGps gps_msg;

  gps_msg.set_time_usec(current_time.Double() * 1e6);
  gps_msg.set_time_utc_usec((current_time.Double() + start_time_.Double()) * 1e6);

  // @note Unfurtonately the Gazebo GpsSensor seems to provide bad readings,
  // starting to drift and leading to global position loss
  // gps_msg.set_latitude_deg(parentSensor_->Latitude().Degree());
  // gps_msg.set_longitude_deg(parentSensor_->Longitude().Degree());
  // gps_msg.set_altitude(parentSensor_->Altitude());
  gps_msg.set_latitude_deg(latlon.first * 180.0 / M_PI);
  gps_msg.set_longitude_deg(latlon.second * 180.0 / M_PI);
  gps_msg.set_altitude(pos_W_I.Z() + alt_home_ - noise_gps_pos_.Z() + gps_bias_.Z());

  std_xy_ = 1.0;
  std_z_  = 1.0;

  {
    std::scoped_lock lock(set_bad_gps_mutex_);
    if (bad_gps_) {
      gps_msg.set_eph(2.5);
    } else {
      gps_msg.set_eph(std_xy_);
    }
  }

  gps_msg.set_epv(std_z_);

  gps_msg.set_velocity_east(velocity_current_W.X() + noise_gps_vel_.Y());
  gps_msg.set_velocity(velocity_current_W_xy.Length());
  gps_msg.set_velocity_north(velocity_current_W.Y() + noise_gps_vel_.X());
  gps_msg.set_velocity_up(velocity_current_W.Z() - noise_gps_vel_.Z());

  {
    // protect shared variables
    std::lock_guard<std::mutex> lock(data_mutex_);

    // add msg to buffer
    gps_delay_buffer_.push(gps_msg);
    current_time_ = current_time;
  }

  last_time_ = current_time;
}

void GazeboRosGpsPlugin::OnSensorUpdate() {
  // protect shared variables
  std::lock_guard<std::mutex> lock(data_mutex_);

  sensor_msgs::msgs::SITLGps gps_msg;
  // apply GPS delay
  if ((current_time_ - last_gps_time_).Double() > 1 / parentSensor_->UpdateRate()) {
    last_gps_time_ = current_time_;

    // do not sent empty msg
    // abort if buffer is empty
    if (gps_delay_buffer_.empty()) {
      return;
    }

    while (true) {

      if (gps_delay_buffer_.empty()) {
        // abort if buffer is empty already
        break;
      }

      gps_msg                  = gps_delay_buffer_.front();
      double gps_current_delay = current_time_.Double() - gps_delay_buffer_.front().time_usec() / 1e6f;

      // remove data that is too old or if buffer size is too large
      if (gps_current_delay >= gps_delay_) {
        gps_delay_buffer_.pop();
        // remove data if buffer too large
      } else if (gps_delay_buffer_.size() > gps_buffer_size_max_) {
        gps_delay_buffer_.pop();
      } else {
        // if we get here, we have good data, stop
        break;
      }
    }
    // publish SITLGps msg at the defined update rate
    gps_pub_->Publish(gps_msg);
  }
}

bool GazeboRosGpsPlugin::activationCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                            std::shared_ptr<std_srvs::srv::SetBool::Response>      response) {
  std::scoped_lock lock(active_mutex_);
  active_ = request->data;
  RCLCPP_INFO(ros_node_->get_logger(), "[%s]: GPS plugin %s", ros_node_->get_name(), active_ ? "active" : "inactive");

  response->message = "Success";
  response->success = true;
  return true;
}

bool GazeboRosGpsPlugin::setBadGpsCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                           std::shared_ptr<std_srvs::srv::SetBool::Response>      response) {
  std::scoped_lock lock(set_bad_gps_mutex_);
  bad_gps_ = request->data;
  RCLCPP_INFO(ros_node_->get_logger(), "[%s]: GPS quality is %s", ros_node_->get_name(), bad_gps_ ? "bad" : "good");

  response->message = "Success";
  response->success = true;
  return true;
}

}  // namespace gazebo
