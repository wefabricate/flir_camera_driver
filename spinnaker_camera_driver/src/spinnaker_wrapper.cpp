// -*-c++-*--------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#include <Spinnaker.h>

#include <chrono>
#include <functional>
#include <spinnaker_camera_driver/logging.hpp>
#include <spinnaker_camera_driver/spinnaker_wrapper.hpp>
#include <string>
#include <thread>

#include "./spinnaker_wrapper_impl.hpp"

namespace spinnaker_camera_driver
{
// transient GigE transport errors that clear on a retry, unlike deterministic
// errors (out-of-range, invalid value) which must not be retried
static bool isTransientTransportError(const Spinnaker::Exception & e)
{
  const auto code = e.GetError();
  return code == Spinnaker::SPINNAKER_ERR_IO || code == Spinnaker::SPINNAKER_ERR_TIMEOUT;
}

SpinnakerWrapper::SpinnakerWrapper(rclcpp::Logger logger) : logger_(logger)
{
  wrapperImpl_.reset(new SpinnakerWrapperImpl(logger));
}

// fn() must be idempotent: a retry re-runs the whole impl call, re-issuing any
// write. Only absolute-value sets are routed here; command nodes use execute().
std::string SpinnakerWrapper::callWithRetry(
  const std::string & op, const std::function<std::string()> & fn)
{
  const int maxAttempts = 4;
  for (int attempt = 1;; ++attempt) {
    try {
      return fn();
    } catch (const Spinnaker::Exception & e) {
      if (isTransientTransportError(e) && attempt < maxAttempts) {
        LOG_WARN(
          "transient device I/O on " << op << " (attempt " << attempt << "/" << maxAttempts
                                     << "), retrying: " << e.what());
        // impl call has returned, so cameraMutex_ is not held during the backoff
        std::this_thread::sleep_for(std::chrono::milliseconds(50 * attempt));
        continue;
      }
      throw SpinnakerWrapper::Exception(e.what());
    }
  }
}

std::string SpinnakerWrapper::getLibraryVersion() const
{
  return wrapperImpl_->getLibraryVersion();
}

void SpinnakerWrapper::refreshCameraList() { wrapperImpl_->refreshCameraList(); }

std::vector<std::string> SpinnakerWrapper::getSerialNumbers() const
{
  return wrapperImpl_->getSerialNumbers();
}

bool SpinnakerWrapper::initCamera(const std::string & serialNumber)
{
  return wrapperImpl_->initCamera(serialNumber);
}

bool SpinnakerWrapper::deInitCamera() { return wrapperImpl_->deInitCamera(); }

bool SpinnakerWrapper::startCamera(const Callback & cb) { return wrapperImpl_->startCamera(cb); }

bool SpinnakerWrapper::stopCamera() { return wrapperImpl_->stopCamera(); }

std::string SpinnakerWrapper::getPixelFormat() const { return wrapperImpl_->getPixelFormat(); }

void SpinnakerWrapper::getAndClearStatistics(Stats * stats)
{
  wrapperImpl_->getAndClearStatistics(stats);
}

std::string SpinnakerWrapper::getNodeMapAsString() { return (wrapperImpl_->getNodeMapAsString()); }

std::string SpinnakerWrapper::setEnum(
  const std::string & nodeName, const std::string & val, std::string * retVal)
{
  return callWithRetry(
    "setEnum(" + nodeName + ")", [&] { return wrapperImpl_->setEnum(nodeName, val, retVal); });
}

std::string SpinnakerWrapper::setDouble(const std::string & nodeName, double val, double * retVal)
{
  return callWithRetry(
    "setDouble(" + nodeName + ")", [&] { return wrapperImpl_->setDouble(nodeName, val, retVal); });
}

std::string SpinnakerWrapper::setBool(const std::string & nodeName, bool val, bool * retVal)
{
  return callWithRetry(
    "setBool(" + nodeName + ")", [&] { return wrapperImpl_->setBool(nodeName, val, retVal); });
}

std::string SpinnakerWrapper::setInt(const std::string & nodeName, int val, int * retVal)
{
  return callWithRetry(
    "setInt(" + nodeName + ")", [&] { return wrapperImpl_->setInt(nodeName, val, retVal); });
}

std::string SpinnakerWrapper::getEnum(const std::string & nodeName, std::string * retVal)
{
  return callWithRetry(
    "getEnum(" + nodeName + ")", [&] { return wrapperImpl_->getEnum(nodeName, retVal); });
}

std::string SpinnakerWrapper::getDouble(const std::string & nodeName, double * retVal)
{
  return callWithRetry(
    "getDouble(" + nodeName + ")", [&] { return wrapperImpl_->getDouble(nodeName, retVal); });
}

// not retried: execute() fires command nodes (e.g. TriggerSoftware) a retry could double-fire
std::string SpinnakerWrapper::execute(const std::string & nodeName)
{
  try {
    return (wrapperImpl_->execute(nodeName));
  } catch (const Spinnaker::Exception & e) {
    throw SpinnakerWrapper::Exception(e.what());
  }
}

void SpinnakerWrapper::setComputeBrightness(bool b) { wrapperImpl_->setComputeBrightness(b); }

void SpinnakerWrapper::setAcquisitionTimeout(double t) { wrapperImpl_->setAcquisitionTimeout(t); }

void SpinnakerWrapper::useIEEE1588(bool b) { wrapperImpl_->useIEEE1588(b); }
std::string SpinnakerWrapper::getIEEE1588Status() const
{
  return (wrapperImpl_->getIEEE1588Status());
}

void SpinnakerWrapper::setDebug(bool b) { wrapperImpl_->setDebug(b); }

}  // namespace spinnaker_camera_driver
