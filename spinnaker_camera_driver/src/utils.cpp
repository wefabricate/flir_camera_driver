// -*-c++-*--------------------------------------------------------------------
// Copyright 2025 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#include <spinnaker_camera_driver/utils.hpp>

namespace spinnaker_camera_driver
{
namespace utils
{
template <class T>
T safeDeclare(
  const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> & pi,
  const std::string & name, const T & def)
{
  try {
    return (pi->declare_parameter(name, rclcpp::ParameterValue(def)).get<T>());
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
    const rclcpp::Parameter p = pi->get_parameter(name);
    return (p.get_parameter_value().get<T>());
  }
}

std::shared_ptr<camera_info_manager::CameraInfoManager> makeCameraInfoManager(
  const std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> & bi,
  const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> & pi,
  const std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> & li,
  const std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> & si,
  const std::string & cameraName, const std::string & urlParameterName, uint32_t qSize)
{
  const auto calib = safeDeclare<std::string>(pi, urlParameterName, "");
  // CameraInfoManager's node-interfaces ctor takes an rmw_qos_profile_t (rclcpp::QoS
  // does not implicitly convert to it), so hand it the underlying profile.
  auto mgr = std::make_shared<camera_info_manager::CameraInfoManager>(
    bi, si, li, cameraName, calib, rclcpp::QoS(qSize).get_rmw_qos_profile());
  return (mgr);
}

}  // namespace utils
}  // namespace spinnaker_camera_driver
