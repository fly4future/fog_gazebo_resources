// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <boost/make_shared.hpp>
#include <boost/variant.hpp>
#include <boost/algorithm/string.hpp>
#include <gazebo/transport/transport.hh>
#include <gazebo_range_plugin.hpp>
#include <gazebo_ros/conversions/sensor_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/detail/range__struct.hpp>
#include <string>
#include <algorithm>
#include <limits>
#include <memory>

#include <Range.pb.h>

namespace gazebo_plugins
{
  /* static constexpr double kSensorMinDistance = 0.06;     // values smaller than that cause issues */
  /* static constexpr double kSensorMaxDistance = 35.0;     // values bigger than that cause issues */
  /* static constexpr double kDefaultMinDistance = 0.2; */
  /* static constexpr double kDefaultMaxDistance = 15.0; */
  static constexpr double kDefaultFOV = 0.00872665;         // standard 0.5 degrees

class GazeboRosRangeSensorPrivate {
public:
  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  // Aliases
  using LaserScan = sensor_msgs::msg::LaserScan;
  using Range     = sensor_msgs::msg::Range;
  using RangePub  = rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr;


  /// Publisher of output
  RangePub pub_;

  /// TF frame output is published in
  std::string frame_name_;

  /// Subscribe to gazebo's laserscan, calling the appropriate callback based on output type
  void SubscribeGazeboLaserScan();

  /// Publish a sensor_msgs/Range message from a gazebo laser scan
  void PublishRange(ConstLaserScanStampedPtr& _msg);

  /// Gazebo transport topic to subscribe to for laser scan
  std::string sensor_topic_;

  /// brief Radiation type to report when output type is range
  uint8_t range_radiation_type_;

  // orientation of sensor
  gazebo::msgs::Quaternion orientation_;

  /// Gazebo node used to subscribe to laser scan
  gazebo::transport::NodePtr gazebo_node_;

  /// Gazebo data publisher
  gazebo::transport::PublisherPtr gazebo_pub_;

  /// Gazebo subscribe to parent sensor's data
  gazebo::transport::SubscriberPtr range_sub_;
};

GazeboRosRangeSensor::GazeboRosRangeSensor() : impl_(std::make_unique<GazeboRosRangeSensorPrivate>()) {
}

GazeboRosRangeSensor::~GazeboRosRangeSensor() {
  // Must release subscriber and then call fini on node to remove it from topic manager.
  impl_->range_sub_.reset();
  if (impl_->gazebo_node_) {
    impl_->gazebo_node_->Fini();
  }
  impl_->gazebo_node_.reset();
}

void GazeboRosRangeSensor::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {
  // Create ros_node configured from sdf
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS& qos = impl_->ros_node_->get_qos();

  // Get QoS profile for the publisher
  rclcpp::QoS pub_qos = qos.get_publisher_qos("~/out", rclcpp::SensorDataQoS().reliable());

  impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::Range>("~/out", pub_qos);

  // Get tf frame for output
  impl_->frame_name_ = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  // Get the root model name
  const std::string        scopedName = _sensor->ParentName();
  std::vector<std::string> names_splitted;
  boost::split(names_splitted, scopedName, boost::is_any_of("::"));
  names_splitted.erase(std::remove_if(begin(names_splitted), end(names_splitted), [](const std::string& name) { return name.size() == 0; }), end(names_splitted));

  // store the model name
  const std::string model_name_ = names_splitted[0];
  std::string gazebo_sensor_name;

  if (_sdf->HasElement("gazebo_sensor_name")) {
    gazebo_sensor_name = _sdf->Get<std::string>("gazebo_sensor_name");
  }else{
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Missing <gazebo_sensor_name>");
    return;
  }

  if (!_sdf->HasElement("radiation_type")) {
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "missing <radiation_type>, defaulting to infrared");
    impl_->range_radiation_type_ = sensor_msgs::msg::Range::INFRARED;
  } else if ("ultrasound" == _sdf->Get<std::string>("radiation_type")) {
    impl_->range_radiation_type_ = sensor_msgs::msg::Range::ULTRASOUND;
  } else if ("infrared" == _sdf->Get<std::string>("radiation_type")) {
    impl_->range_radiation_type_ = sensor_msgs::msg::Range::INFRARED;
  } else {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Invalid <radiation_type> [%s]. Can be ultrasound or infrared",
                 _sdf->Get<std::string>("radiation_type").c_str());
    return;
  }
  // Calculate parent sensor rotation WRT `base_link`
  const ignition::math::Quaterniond q_ls = _sensor->Pose().Rot();

  // Set the orientation
  impl_->orientation_.set_x(q_ls.X());
  impl_->orientation_.set_y(q_ls.Y());
  impl_->orientation_.set_z(q_ls.Z());
  impl_->orientation_.set_w(q_ls.W());

  // Create gazebo transport node and subscribe to sensor's laser scan
  impl_->gazebo_node_ = boost::make_shared<gazebo::transport::Node>();
  impl_->gazebo_node_->Init(_sensor->WorldName());

  impl_->gazebo_pub_ = impl_->gazebo_node_->Advertise<sensor_msgs::msgs::Range>("~/" + model_name_ + "/link/" + gazebo_sensor_name, 10);

  // TODO(ironmig): use lazy publisher to only process laser data when output has a subscriber
  impl_->sensor_topic_ = _sensor->Topic();
  impl_->SubscribeGazeboLaserScan();
}

void GazeboRosRangeSensorPrivate::SubscribeGazeboLaserScan() {
  range_sub_ = gazebo_node_->Subscribe(sensor_topic_, &GazeboRosRangeSensorPrivate::PublishRange, this);
}

void GazeboRosRangeSensorPrivate::PublishRange(ConstLaserScanStampedPtr& _msg) {
  // Convert Laser scan to range
  auto range_msg = gazebo_ros::Convert<sensor_msgs::msg::Range>(*_msg);

  /* // check current distance measured from the sensor */
  /* double current_distance = range_msg.range; */

  /* // set distance to min/max if actual value is smaller/bigger */
  /* if (current_distance <  range_msg.min_range || std::isinf(current_distance)) { */
  /*   current_distance = range_msg.min_range; */
  /* } else if (current_distance > range_msg.max_range) { */
  /*   current_distance = range_msg.max_range; */
  /* } */

  /* range_msg.range = current_distance; */

  // Set tf frame
  range_msg.header.frame_id = frame_name_;
  // Set radiation type from sdf
  range_msg.radiation_type = range_radiation_type_;

  // fill protobuf Range msg
  sensor_msgs::msgs::Range proto_range_msg;
  proto_range_msg.set_time_usec(range_msg.header.stamp.nanosec * 1e-3);
  proto_range_msg.set_min_distance(range_msg.min_range);
  proto_range_msg.set_max_distance(range_msg.max_range);
  proto_range_msg.set_current_distance(range_msg.range);
  proto_range_msg.set_h_fov(kDefaultFOV);
  proto_range_msg.set_v_fov(kDefaultFOV);

  // Compute signal strength
  // Other effects like target size, shape or reflectivity are not considered
  const double signal_strength = sqrt(range_msg.range);
  const double low_signal_strength_ = sqrt(range_msg.min_range);
  const double high_signal_strength_ = sqrt(range_msg.max_range + 0.02); // extend the threshold so there is still quality at max distance

  // Compute and set the signal quality
  // The signal quality is normalized between 1 and 100 using the absolute
  // signal strength (DISTANCE_SENSOR signal_quality value of 0 means invalid)
  uint8_t signal_quality = 1;
  if (signal_strength > low_signal_strength_) {
    signal_quality = static_cast<uint8_t>(99 * ((high_signal_strength_ - signal_strength) /
            (high_signal_strength_ - low_signal_strength_)) + 1);
  }

  proto_range_msg.set_signal_quality(signal_quality);
  proto_range_msg.set_allocated_orientation(new gazebo::msgs::Quaternion(orientation_));

  // Publish ROS msg output
  pub_->publish(range_msg);

  // Publish gazebo msg output
  gazebo_pub_->Publish(proto_range_msg);

}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosRangeSensor)

}  // namespace gazebo_plugins
