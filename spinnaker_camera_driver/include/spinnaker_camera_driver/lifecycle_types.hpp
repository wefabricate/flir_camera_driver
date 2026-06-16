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

#ifndef SPINNAKER_CAMERA_DRIVER__LIFECYCLE_TYPES_HPP_
#define SPINNAKER_CAMERA_DRIVER__LIFECYCLE_TYPES_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

// The driver node is ALWAYS a lifecycle node, independent of the image_transport
// version. The image_transport version only decides which PUBLISHER is used.
using NodeType = rclcpp_lifecycle::LifecycleNode;
using LCState = rclcpp_lifecycle::State;
using CbReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// Publisher-selection guard: the image_transport publisher can only attach to a
// lifecycle node from image_transport >= 6.4.0, which is exactly when the build
// defines IMAGE_TRANSPORT_SUPPORTS_LIFECYCLE_NODE (see CMakeLists.txt). Alias it
// to a name that reads as a publisher choice rather than a node-type choice.
// When it is NOT defined, the driver falls back to plain lifecycle publishers
// for sensor_msgs/Image (+ CameraInfo), with no image_transport involved.
#ifdef IMAGE_TRANSPORT_SUPPORTS_LIFECYCLE_NODE
#define USE_IMAGE_TRANSPORT_PUBLISHER
#endif

#endif  // SPINNAKER_CAMERA_DRIVER__LIFECYCLE_TYPES_HPP_
