/*
This code was developed by the National Robotics Engineering Center (NREC), part of the Robotics Institute at Carnegie Mellon University.
Its development was funded by DARPA under the LS3 program and submitted for public release on June 7th, 2012.
Release was granted on August, 21st 2012 with Distribution Statement "A" (Approved for Public Release, Distribution Unlimited).

This software is released under a BSD license:

Copyright (c) 2012, Carnegie Mellon University. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
Neither the name of the Carnegie Mellon University nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/



/**
   @file nodelet.cpp
   @author Chad Rockey
   @author James Servos
   @date Nov 9, 2016
   @brief ROS nodelet for a pair of Point Grey Chameleon Cameras

   @attention Copyright (C) 2011
   @attention National Robotics Engineering Center
   @attention Carnegie Mellon University
*/

// ROS and associated nodelet interface and PLUGINLIB declaration header
#include "ros/ros.h"
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "pointgrey_camera_driver/PointGreyCamera.h" // The actual standalone library for the PointGreys

#include <image_transport/image_transport.h> // ROS library that allows sending compressed images
#include <camera_info_manager/camera_info_manager.h> // ROS library that publishes CameraInfo topics
#include <sensor_msgs/CameraInfo.h> // ROS message header for CameraInfo

#include <wfov_camera_msgs/WFOVImage.h>
#include <image_exposure_msgs/ExposureSequence.h> // Message type for configuring gain and white balance.

#include <diagnostic_updater/diagnostic_updater.h> // Headers for publishing diagnostic messages.
#include <diagnostic_updater/publisher.h>

#include <boost/thread.hpp> // Needed for the nodelet to launch the reading thread.

#include <dynamic_reconfigure/server.h> // Needed for the dynamic_reconfigure gui service to run

namespace pointgrey_camera_driver
{

class PointGreyCustomStereoNodelet: public nodelet::Nodelet
{
public:
  PointGreyCustomStereoNodelet() {}

  ~PointGreyCustomStereoNodelet()
  {
    if(pubThread_)
    {
      pubThread_->interrupt();
      pubThread_->join();
    }

    try
    {
      NODELET_DEBUG("Stopping camera capture.");
      left_cam_.stop();
      right_cam_.stop();
      NODELET_DEBUG("Disconnecting from camera.");
      left_cam_.disconnect();
      right_cam_.disconnect();
    }
    catch(std::runtime_error& e)
    {
      NODELET_ERROR("%s", e.what());
    }
  }

private:
  /*!
  * \brief Function that allows reconfiguration of the camera.
  *
  * This function serves as a callback for the dynamic reconfigure service.  It simply passes the configuration object to the driver to allow the camera to reconfigure.
  * \param config  camera_library::CameraConfig object passed by reference.  Values will be changed to those the driver is currently using.
  * \param level driver_base reconfiguration level.  See driver_base/SensorLevels.h for more information.
  */
  void paramCallback(pointgrey_camera_driver::PointGreyConfig &config, uint32_t level)
  {
    config_ = config;

    try
    {
      NODELET_DEBUG("Dynamic reconfigure callback with level: %d", level);
      left_cam_.setNewConfiguration(config, level);
      right_cam_.setNewConfiguration(config, level);

      // Store needed parameters for the metadata message
      gain_ = config.gain;
      wb_blue_ = config.white_balance_blue;
      wb_red_ = config.white_balance_red;

      // Store CameraInfo binning information
      binning_x_ = 1;
      binning_y_ = 1;

      // Store CameraInfo RegionOfInterest information
      if(config.video_mode == "format7_mode0" || config.video_mode == "format7_mode1" || config.video_mode == "format7_mode2")
      {
        roi_x_offset_ = config.format7_x_offset;
        roi_y_offset_ = config.format7_y_offset;
        roi_width_ = config.format7_roi_width;
        roi_height_ = config.format7_roi_height;
        do_rectify_ = true; // Set to true if an ROI is used.
      }
      else
      {
        // Zeros mean the full resolution was captured.
        roi_x_offset_ = 0;
        roi_y_offset_ = 0;
        roi_height_ = 0;
        roi_width_ = 0;
        do_rectify_ = false; // Set to false if the whole image is captured.
      }
    }
    catch(std::runtime_error& e)
    {
      NODELET_ERROR("Reconfigure Callback failed with error: %s", e.what());
    }
  }

  /*!
  * \brief Connection callback to only do work when someone is listening.
  *
  * This function will connect/disconnect from the camera depending on who is using the output.
  */
  void connectCb()
  {
    NODELET_DEBUG("Connect callback!");
    boost::mutex::scoped_lock scopedLock(connect_mutex_); // Grab the mutex.  Wait until we're done initializing before letting this function through.
    // Check if we should disconnect (there are 0 subscribers to our data)
    if(left_it_pub_.getNumSubscribers() == 0 && left_pub_->getPublisher().getNumSubscribers() == 0
    && right_it_pub_.getNumSubscribers() == 0 && right_pub_->getPublisher().getNumSubscribers() == 0)
    {
      NODELET_DEBUG("Disconnecting.");
      pubThread_->interrupt();
      scopedLock.unlock();
      pubThread_->join();
      scopedLock.lock();
      sub_.shutdown();

      try
      {
        NODELET_DEBUG("Stopping camera capture.");
        left_cam_.stop();
        right_cam_.stop();
      }
      catch(std::runtime_error& e)
      {
        NODELET_ERROR("%s", e.what());
      }

      try
      {
        NODELET_DEBUG("Disconnecting from camera.");
        left_cam_.disconnect();
        right_cam_.disconnect();
      }
      catch(std::runtime_error& e)
      {
        NODELET_ERROR("%s", e.what());
      }
    }
    else if(!sub_)     // We need to connect
    {
      // Start the thread to loop through and publish messages
      pubThread_.reset(new boost::thread(boost::bind(&pointgrey_camera_driver::PointGreyCustomStereoNodelet::devicePoll, this)));
    }
    else
    {
      NODELET_DEBUG("Do nothing in callback.");
    }
  }

  int getSerial(const std::string& prefix, ros::NodeHandle nh)
  {
    int serial = 0;
    XmlRpc::XmlRpcValue serial_xmlrpc;
    nh.getParam(prefix + "_serial", serial_xmlrpc);
    if (serial_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      nh.param<int>(prefix + "_serial", serial, 0);
    }
    else if (serial_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      std::string serial_str;
      nh.param<std::string>(prefix + "_serial", serial_str, "0");
      std::istringstream(serial_str) >> serial;
    }
    else
    {
      NODELET_DEBUG("Serial XMLRPC type.");
      serial = 0;
    }
    NODELET_INFO("Using camera serial %d", serial);
    return serial;
  }

  /*!
  * \brief Serves as a psuedo constructor for nodelets.
  *
  * This function needs to do the MINIMUM amount of work to get the nodelet running.  Nodelets should not call blocking functions here.
  */
  void onInit()
  {
    // Get nodeHandles
    ros::NodeHandle &nh = getMTNodeHandle();
    ros::NodeHandle &pnh = getMTPrivateNodeHandle();
    ros::NodeHandle left_nh(nh, "left");
    ros::NodeHandle right_nh(nh, "right");

    // Get a serial number through ros
    int left_serial = getSerial("left", pnh);
    int right_serial = getSerial("right", pnh);


    left_cam_.setDesiredCamera((uint32_t)left_serial);
    right_cam_.setDesiredCamera((uint32_t)right_serial);

    // Get GigE camera parameters:
    pnh.param<int>("packet_size", packet_size_, 1400);
    pnh.param<bool>("auto_packet_size", auto_packet_size_, true);
    pnh.param<int>("packet_delay", packet_delay_, 4000);

    // Set GigE parameters:
    //pg_.setGigEParameters(auto_packet_size_, packet_size_, packet_delay_);

    // Get the location of our camera config yaml
    std::string right_camera_info_url, left_camera_info_url;
    pnh.param<std::string>("right_camera_info_url", right_camera_info_url, "");
    pnh.param<std::string>("left_camera_info_url", left_camera_info_url, "");
    // Get the desired frame_id, set to 'camera' if not found
    pnh.param<std::string>("frame_id", frame_id_, "camera");

    // Do not call the connectCb function until after we are done initializing.
    boost::mutex::scoped_lock scopedLock(connect_mutex_);

    // Start up the dynamic_reconfigure service, note that this needs to stick around after this function ends
    srv_ = boost::make_shared <dynamic_reconfigure::Server<pointgrey_camera_driver::PointGreyConfig> > (pnh);
    dynamic_reconfigure::Server<pointgrey_camera_driver::PointGreyConfig>::CallbackType f =
      boost::bind(&pointgrey_camera_driver::PointGreyCustomStereoNodelet::paramCallback, this, _1, _2);
    srv_->setCallback(f);

    it_.reset(new image_transport::ImageTransport(nh));
    image_transport::SubscriberStatusCallback cb = boost::bind(&PointGreyCustomStereoNodelet::connectCb, this);

    // Start the camera info manager and attempt to load any configurations
    // Left
    std::stringstream left_cinfo_name;
    left_cinfo_name << left_serial;
    left_cinfo_.reset(new camera_info_manager::CameraInfoManager(left_nh, left_cinfo_name.str(), left_camera_info_url));
    left_it_pub_ = it_->advertiseCamera("left/image_raw", 5, cb, cb);

    // Left
    std::stringstream right_cinfo_name;
    right_cinfo_name << right_serial;
    right_cinfo_.reset(new camera_info_manager::CameraInfoManager(right_nh, right_cinfo_name.str(), right_camera_info_url));
    right_it_pub_ = it_->advertiseCamera("right/image_raw", 5, cb, cb);

    // Set up diagnostics
    updater_.setHardwareID("pointgrey custom stereo camera");

    // Set up a diagnosed publisher
    double desired_freq;
    pnh.param<double>("desired_freq", desired_freq, 7.0);
    pnh.param<double>("min_freq", min_freq_, desired_freq);
    pnh.param<double>("max_freq", max_freq_, desired_freq);
    double freq_tolerance; // Tolerance before stating error on publish frequency, fractional percent of desired frequencies.
    pnh.param<double>("freq_tolerance", freq_tolerance, 0.1);
    int window_size; // Number of samples to consider in frequency
    pnh.param<int>("window_size", window_size, 100);
    double min_acceptable; // The minimum publishing delay (in seconds) before warning.  Negative values mean future dated messages.
    pnh.param<double>("min_acceptable_delay", min_acceptable, 0.0);
    double max_acceptable; // The maximum publishing delay (in seconds) before warning.
    pnh.param<double>("max_acceptable_delay", max_acceptable, 0.2);
    ros::SubscriberStatusCallback cb2 = boost::bind(&PointGreyCustomStereoNodelet::connectCb, this);

    left_pub_.reset(new diagnostic_updater::DiagnosedPublisher<wfov_camera_msgs::WFOVImage>(nh.advertise<wfov_camera_msgs::WFOVImage>("left/image", 5, cb2, cb2),
               updater_,
               diagnostic_updater::FrequencyStatusParam(&min_freq_, &max_freq_, freq_tolerance, window_size),
               diagnostic_updater::TimeStampStatusParam(min_acceptable, max_acceptable)));

    right_pub_.reset(new diagnostic_updater::DiagnosedPublisher<wfov_camera_msgs::WFOVImage>(nh.advertise<wfov_camera_msgs::WFOVImage>("right/image", 5, cb2, cb2),
               updater_,
               diagnostic_updater::FrequencyStatusParam(&min_freq_, &max_freq_, freq_tolerance, window_size),
               diagnostic_updater::TimeStampStatusParam(min_acceptable, max_acceptable)));
  }


  bool publishImage(const std::string& camera_name, PointGreyCamera& cam, 
                  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo,
                  image_transport::CameraPublisher it_pub,
                  boost::shared_ptr<diagnostic_updater::DiagnosedPublisher<wfov_camera_msgs::WFOVImage> > wfov_pub,
                  ros::Time timestamp)
  {
    try
    {
      wfov_camera_msgs::WFOVImagePtr wfov_image(new wfov_camera_msgs::WFOVImage);
      // Get the image from the camera library
      NODELET_DEBUG("Starting a new grab from camera.");
      cam.grabImage(wfov_image->image, frame_id_ + "_" + camera_name);

      // Set other values
      wfov_image->header.frame_id = frame_id_ + "_" + camera_name;

      wfov_image->gain = gain_;
      wfov_image->white_balance_blue = wb_blue_;
      wfov_image->white_balance_red = wb_red_;

      wfov_image->temperature = cam.getCameraTemperature();

      wfov_image->header.stamp = timestamp;
      wfov_image->image.header.stamp = timestamp;

      // Set the CameraInfo message
      sensor_msgs::CameraInfoPtr ci; ///< Camera Info message.
      ci.reset(new sensor_msgs::CameraInfo(cinfo->getCameraInfo()));
      ci->header.stamp = wfov_image->image.header.stamp;
      ci->header.frame_id = wfov_image->header.frame_id;
      //The height, width, distortion model, and parameters are all filled in by camera info manager.
      ci->binning_x = binning_x_;
      ci->binning_y = binning_y_;
      ci->roi.x_offset = roi_x_offset_;
      ci->roi.y_offset = roi_y_offset_;
      ci->roi.height = roi_height_;
      ci->roi.width = roi_width_;
      ci->roi.do_rectify = do_rectify_;

      wfov_image->info = *ci;

      // Publish the full message
      wfov_pub->publish(wfov_image);

      // Publish the message using standard image transport
      if(it_pub.getNumSubscribers() > 0)
      {
        sensor_msgs::ImagePtr image(new sensor_msgs::Image(wfov_image->image));
        it_pub.publish(image, ci);
      }
    }
    catch(CameraTimeoutException& e)
    {
      NODELET_WARN("%s", e.what());
    }
    catch(std::runtime_error& e)
    {
      NODELET_ERROR("%s", e.what());

      return false;
    }

    return true;
  }

  /*!
  * \brief Function for the boost::thread to grabImages and publish them.
  *
  * This function continues until the thread is interupted.  Responsible for getting sensor_msgs::Image and publishing them.
  */
  void devicePoll()
  {
    enum State
    {
        NONE
      , ERROR
      , STOPPED
      , DISCONNECTED
      , CONNECTED
      , STARTED
    };

    State state = DISCONNECTED;
    State previous_state = NONE;

    while(!boost::this_thread::interruption_requested())   // Block until we need to stop this thread.
    {
      bool state_changed = state != previous_state;

      previous_state = state;

      switch(state)
      {
        case ERROR:
          // Generally there's no need to stop before disconnecting after an
          // error. Indeed, stop will usually fail.
#if STOP_ON_ERROR
          // Try stopping the camera
          {
            boost::mutex::scoped_lock scopedLock(connect_mutex_);
            sub_.shutdown();
          }

          try
          {
            NODELET_DEBUG("Stopping camera.");
            left_cam_.stop();
            right_cam_.stop();
            NODELET_INFO("Stopped camera.");

            state = STOPPED;
          }
          catch(std::runtime_error& e)
          {
            NODELET_ERROR_COND(state_changed,
                "Failed to stop error: %s", e.what());
            ros::Duration(1.0).sleep(); // sleep for one second each time
          }

          break;
#endif
        case STOPPED:
          // Try disconnecting from the camera
          try
          {
            NODELET_DEBUG("Disconnecting from camera.");
            left_cam_.disconnect();
            right_cam_.disconnect();
            NODELET_INFO("Disconnected from camera.");

            state = DISCONNECTED;
          }
          catch(std::runtime_error& e)
          {
            NODELET_ERROR_COND(state_changed,
                "Failed to disconnect with error: %s", e.what());
            ros::Duration(1.0).sleep(); // sleep for one second each time
          }

          break;
        case DISCONNECTED:
          // Try connecting to the camera
          try
          {
            NODELET_DEBUG("Connecting to camera.");
            left_cam_.connect();
            right_cam_.connect();
            NODELET_INFO("Connected to camera.");

            // Set last configuration, forcing the reconfigure level to stop
            left_cam_.setNewConfiguration(config_, PointGreyCamera::LEVEL_RECONFIGURE_STOP);
            right_cam_.setNewConfiguration(config_, PointGreyCamera::LEVEL_RECONFIGURE_STOP);

            // Set the timeout for grabbing images.
            try
            {
              double timeout;
              getMTPrivateNodeHandle().param("timeout", timeout, 1.0);

              NODELET_DEBUG("Setting timeout to: %f.", timeout);
              left_cam_.setTimeout(timeout);
              right_cam_.setTimeout(timeout);
            }
            catch(std::runtime_error& e)
            {
              NODELET_ERROR("%s", e.what());
            }

            // Subscribe to gain and white balance changes
            {
              boost::mutex::scoped_lock scopedLock(connect_mutex_);
              sub_ = getMTNodeHandle().subscribe("image_exposure_sequence", 10, &pointgrey_camera_driver::PointGreyCustomStereoNodelet::gainWBCallback, this);
            }

            state = CONNECTED;
          }
          catch(std::runtime_error& e)
          {
            NODELET_ERROR_COND(state_changed,
                "Failed to connect with error: %s", e.what());
            ros::Duration(1.0).sleep(); // sleep for one second each time
          }

          break;
        case CONNECTED:
          // Try starting the camera
          try
          {
            NODELET_DEBUG("Starting camera.");
            left_cam_.start();
            right_cam_.start();
            NODELET_INFO("Started camera.");

            state = STARTED;
          }
          catch(std::runtime_error& e)
          {
            NODELET_ERROR_COND(state_changed,
                "Failed to start with error: %s", e.what());
            ros::Duration(1.0).sleep(); // sleep for one second each time
          }

          break;
        case STARTED:
          {
            // Trigger cameras if using software triggering
            if (config_.enable_trigger == true && config_.trigger_source == "software")
            {
              if (!left_cam_.trigger() || !right_cam_.trigger())
              {
                NODELET_ERROR("Could not trigger camera");
                state = ERROR;
                break;
              }
            }

            ros::Time stamp = ros::Time::now();
            
            // Get Images
            if (!publishImage("left", left_cam_, left_cinfo_, left_it_pub_, left_pub_, stamp) 
                || !publishImage("right", right_cam_, right_cinfo_, right_it_pub_, right_pub_, stamp))
            {
              state = ERROR;
            }
          }
          break;
        default:
          NODELET_ERROR("Unknown camera state %d!", state);
      }

      // Update diagnostics
      updater_.update();
    }
    NODELET_DEBUG("Leaving thread.");
  }

  void gainWBCallback(const image_exposure_msgs::ExposureSequence &msg)
  {
    try
    {
      NODELET_DEBUG("Gain callback:  Setting gain to %f and white balances to %u, %u", msg.gain, msg.white_balance_blue, msg.white_balance_red);
      gain_ = msg.gain;
      left_cam_.setGain(gain_);
      right_cam_.setGain(gain_);
      wb_blue_ = msg.white_balance_blue;
      wb_red_ = msg.white_balance_red;
      left_cam_.setBRWhiteBalance(false, wb_blue_, wb_red_);
      right_cam_.setBRWhiteBalance(false, wb_blue_, wb_red_);
    }
    catch(std::runtime_error& e)
    {
      NODELET_ERROR("gainWBCallback failed with error: %s", e.what());
    }
  }

  boost::shared_ptr<dynamic_reconfigure::Server<pointgrey_camera_driver::PointGreyConfig> > srv_; ///< Needed to initialize and keep the dynamic_reconfigure::Server in scope.

  boost::shared_ptr<image_transport::ImageTransport> it_; ///< Needed to initialize and keep the ImageTransport in scope.

  PointGreyCamera left_cam_; ///< Instance of the PointGreyCamera library, used to interface with the hardware.
  boost::shared_ptr<camera_info_manager::CameraInfoManager> left_cinfo_; ///< Needed to initialize and keep the CameraInfoManager in scope.
  image_transport::CameraPublisher left_it_pub_; ///< CameraInfoManager ROS publisher

  PointGreyCamera right_cam_; ///< Instance of the PointGreyCamera library, used to interface with the hardware.
  boost::shared_ptr<camera_info_manager::CameraInfoManager> right_cinfo_; ///< Needed to initialize and keep the CameraInfoManager in scope.
  image_transport::CameraPublisher right_it_pub_; ///< CameraInfoManager ROS publisher

   boost::shared_ptr<diagnostic_updater::DiagnosedPublisher<wfov_camera_msgs::WFOVImage> > left_pub_; ///< Diagnosed publisher, has to be a pointer because of constructor requirements
   boost::shared_ptr<diagnostic_updater::DiagnosedPublisher<wfov_camera_msgs::WFOVImage> > right_pub_; ///< Diagnosed publisher, has to be a pointer because of constructor requirements

  ros::Subscriber sub_; ///< Subscriber for gain and white balance changes.

  boost::mutex connect_mutex_;

  diagnostic_updater::Updater updater_; ///< Handles publishing diagnostics messages.
  double min_freq_;
  double max_freq_;

  std::string frame_id_; ///< Frame id for the camera messages, defaults to 'camera'
  boost::shared_ptr<boost::thread> pubThread_; ///< The thread that reads and publishes the images.

  double gain_;
  uint16_t wb_blue_;
  uint16_t wb_red_;

  // Parameters for cameraInfo
  size_t binning_x_; ///< Camera Info pixel binning along the image x axis.
  size_t binning_y_; ///< Camera Info pixel binning along the image y axis.
  size_t roi_x_offset_; ///< Camera Info ROI x offset
  size_t roi_y_offset_; ///< Camera Info ROI y offset
  size_t roi_height_; ///< Camera Info ROI height
  size_t roi_width_; ///< Camera Info ROI width
  bool do_rectify_; ///< Whether or not to rectify as if part of an image.  Set to false if whole image, and true if in ROI mode.

  // For GigE cameras:
  /// If true, GigE packet size is automatically determined, otherwise packet_size_ is used:
  bool auto_packet_size_;
  /// GigE packet size:
  int packet_size_;
  /// GigE packet delay:
  int packet_delay_;

  /// Configuration:
  pointgrey_camera_driver::PointGreyConfig config_;
};

PLUGINLIB_DECLARE_CLASS(pointgrey_camera_driver, PointGreyCustomStereoNodelet, pointgrey_camera_driver::PointGreyCustomStereoNodelet, nodelet::Nodelet);  // Needed for Nodelet declaration
}
