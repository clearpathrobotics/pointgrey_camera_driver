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



/*-*-C++-*-*/
/**
   @file PointGreyCamera.cpp
   @author Chad Rockey
   @date July 11, 2011
   @brief Interface to Point Grey cameras

   @attention Copyright (C) 2011
   @attention National Robotics Engineering Center
   @attention Carnegie Mellon University
*/

#include "pointgrey_camera_driver/PointGreyCamera.h"

#include <iostream>
#include <sstream>

PointGreyCamera::PointGreyCamera()
{
  system_ = Spinnaker::System::GetInstance();
  camList_ = system_->GetCameras();

  unsigned int num_cameras = camList_.GetSize();
  ROS_INFO_STREAM("[PointGreyCamera]: Number of cameras detected: " <<  num_cameras);

  pCam_ = NULL;

  serial_ = 0;
  captureRunning_ = false;
}

PointGreyCamera::~PointGreyCamera()
{
}


bool PointGreyCamera::setNewConfiguration(PointGreyCameraConfig &config, const uint32_t &level)
{
  // Check if camera is connected
  if(!pCam_)
  {
    PointGreyCamera::connect();
  }

  // Activate mutex to prevent us from grabbing images during this time
  boost::mutex::scoped_lock scopedLock(mutex_);


  // return true if we can set values as desired.
  bool retVal = true;

  PointGreyCamera::setVideoMode(config.video_mode);

  // TODO: @tthomas - set format 7 (image control params)

  // Set frame rate
  retVal = setProperty("AcquisitionFrameRateAuto", config.acquisition_frame_rate_auto);
  retVal = setProperty("AcquisitionFrameRateEnabled", config.acquisition_frame_rate_enabled);
  retVal = setProperty("AcquisitionFrameRate", config.acquisition_frame_rate);


  // Set auto exposure
  retVal = setProperty("ExposureMode", std::string("Timed"));
  retVal = setProperty("ExposureAuto", config.exposure_auto);


  // TODO: Set sharpness


  // Set saturation
  if (config.saturation_enabled)
  {
    retVal = setProperty("SaturationEnabled", config.saturation_enabled);
    retVal = setProperty("Saturation", config.saturation);
  }


  // Set shutter time/speed
  if (config.exposure_auto.compare(std::string("Off")) == 0)
  {
    retVal = setProperty("ExposureTime", config.exposure_time);
  }
  retVal = setProperty("AutoExposureTimeUpperLimit", config.auto_exposure_exposure_time_upper_limit);


  // TODO: Set gain

  // TODO: Set pan

  // TODO: Set tilt


  // Set brightness
  retVal = setProperty("BlackLevel", config.black_level);

  // Set gamma
  if (config.gamma_enabled)
  {
    retVal = setProperty("GammaEnabled", config.gamma_enabled);
    retVal = setProperty("Gamma", config.gamma);
  }

  // Set white balance
  retVal = setProperty("BalanceWhiteAuto", config.balance_white_auto);
  retVal = setProperty("BalanceRatioSelector", "Blue");
  retVal = setProperty("BalanceRatio", config.balance_ratio_blue);
  retVal = setProperty("BalanceRatioSelector", "Red");
  retVal = setProperty("BalanceRatio", config.balance_ratio_red);


  // Set Trigger and Strobe
  if (config.is_left)
  {
    // Set strobe (LEFT CAMERA ONLY)
    // and AcquisitionFrameRate (only if Trigger Mode is Off)
    retVal = setProperty("LineSelector", config.line_selector);
    retVal = setProperty("LineMode", config.line_mode);
    retVal = setProperty("LineSource", config.line_source);
    retVal = setProperty("TriggerMode", std::string("Off"));
    retVal = setProperty("AcquisitionFrameRateAuto", config.acquisition_frame_rate_auto);
    retVal = setProperty("AcquisitionFrameRateEnabled", config.acquisition_frame_rate_enabled);
    retVal = setProperty("AcquisitionFrameRate", config.acquisition_frame_rate);
  }
  else
  {
    // Trigger (Must be switched off to change source!)
    // (RIGHT CAMERA ONLY)
    retVal = setProperty("TriggerMode", std::string("Off"));
    retVal = setProperty("TriggerSource", config.trigger_source);
    retVal = setProperty("TriggerMode", std::string("On"));
    retVal = setProperty("TriggerSelector", config.trigger_selector);
    retVal = setProperty("TriggerActivation", config.trigger_activation);
  }

  return retVal;

}  // end setNewConfiguration


void PointGreyCamera::setGain(float& gain)
{
  // Turn auto gain off
  pCam_->GainAuto.SetValue(Spinnaker::GainAutoEnums::GainAuto_Off);

  // Set gain
  setProperty("Gain", gain);
}




void PointGreyCamera::setVideoMode(const std::string& videoMode)
{
  setProperty("VideoMode", videoMode);
}



/*

TODO: @tthomas - implement the following
bool PointGreyCamera::setFormat7(FlyCapture2::Mode &fmt7Mode, FlyCapture2::PixelFormat &fmt7PixFmt, uint16_t &roi_width, uint16_t &roi_height, uint16_t &roi_offset_x, uint16_t &roi_offset_y)
{
  return retVal;
}

bool PointGreyCamera::getVideoModeFromString(std::string &vmode, FlyCapture2::VideoMode &vmode_out, FlyCapture2::Mode &fmt7Mode)
{
  return retVal;
}

bool PointGreyCamera::getFormat7PixelFormatFromString(std::string &sformat, FlyCapture2::PixelFormat &fmt7PixFmt)
{
  return retVal;
}


void PointGreyCamera::setTimeout(const double &timeout)
{
}

float PointGreyCamera::getCameraTemperature()
{
}

float PointGreyCamera::getCameraFrameRate()
{
}


void PointGreyCamera::setGigEParameters(bool auto_packet_size, unsigned int packet_size, unsigned int packet_delay)
{
}

void PointGreyCamera::setupGigEPacketSize(PGRGuid & guid)
{
}

void PointGreyCamera::setupGigEPacketSize(PGRGuid & guid, unsigned int packet_size)
{

}

void PointGreyCamera::setupGigEPacketDelay(PGRGuid & guid, unsigned int packet_delay)
{
}

*/

int PointGreyCamera::connect()
{
  int result = 0;
  int err = 0;

  if(!pCam_)
  {
    // If we have a specific camera to connect to (specified by a serial number)
    if(serial_ != 0)
    {
      std::string serial_string = std::to_string(serial_);

      try
      {
        pCam_ = camList_.GetBySerial(serial_string);
      }
      catch (Spinnaker::Exception &e)
      {
        ROS_ERROR("PointGreyCamera::connect Could not find camera with serial number: %s Is that camera plugged in?");

        std::cout << "Error: " << e.what() << std::endl;
        result = -1;
      }
    }
    else
    {
      // Connect to any camera (the first)

      try
      {
        pCam_ = camList_.GetByIndex(0);
      }
      catch (Spinnaker::Exception &e)
      {
        ROS_ERROR("PointGreyCamera::connect Failed to get first connected camera");
        std::cout << "Error: " << e.what() << std::endl;
        result = -1;
      }
    }

    // TODO @tthomas - check if interface is GigE and connect to GigE cam

    try
    {
      // Initialize Camera
      pCam_->Init();

      // Retrieve GenICam nodemap
      node_map_ = &pCam_->GetNodeMap();

      // Configure chunk data - Enable Metadata
      err = ConfigureChunkData(*node_map_);
      if (err < 0)
      {
        return err;
      }

      // NOTE: Brightness is termed black level in GenICam
      float black_level = 1.7;
      setProperty("BlackLevel", black_level);
    }
    catch (Spinnaker::Exception &e)
    {
      ROS_ERROR("PointGreyCamera::connect Failed to connect to camera");
      std::cout << "Error: " << e.what() << std::endl;
      result = -1;
    }
    return result;

    // TODO: Get camera info to check if camera is running in color or mono mode
    /*
    CameraInfo cInfo;
    error = cam_.GetCameraInfo(&cInfo);
    PointGreyCamera::handleError("PointGreyCamera::connect  Failed to get camera info.", error);
    isColor_ = cInfo.isColorCamera;
    */
  }
}

int PointGreyCamera::disconnect()
{
  int result = 0;

  boost::mutex::scoped_lock scopedLock(mutex_);
  captureRunning_ = false;

  // Check if camera is connected
  if(pCam_)
  {
    try
    {
      pCam_->DeInit();
    }
    catch (Spinnaker::Exception &e)
    {
      std::cout << "PointGreyCamera::disconnect Failed to disconnect camera with Error: " << e.what() << std::endl;
      result = -1;
    }
  }

  return result;
}

int PointGreyCamera::start()
{
  int result = 0;

  try
  {
    // Check if camera is connected
    if(pCam_ && !captureRunning_)
    {
      // Start capturing images
      pCam_->BeginAcquisition();
      captureRunning_ = true;
    }
  }
  catch (Spinnaker::Exception &e)
  {
    ROS_ERROR_STREAM("PointGreyCamera::start Failed to start capture with Error: " << e.what());
    int result = -1;
  }
  return result;
}


bool PointGreyCamera::stop()
{
  if (pCam_ && captureRunning_)
  {
    // Stop capturing images
    try
    {
      captureRunning_ = false;
      pCam_->EndAcquisition();
      return true;
    }
    catch (Spinnaker::Exception &e)
    {
      ROS_ERROR_STREAM("PointGreyCamera::stop Failed to stop capture with Error: " << e.what());
      return false;
    }
  }
  return false;
}


int PointGreyCamera::grabImage(sensor_msgs::Image &image, const std::string &frame_id)
{
  int result = 0;

  boost::mutex::scoped_lock scopedLock(mutex_);

  // Check if Camera is connected and Running
  if(pCam_ && captureRunning_)
  {
    // Handle "Image Retrieval" Exception
    try
    {
      Spinnaker::ImagePtr image_ptr = pCam_->GetNextImage();
      std::string format(image_ptr->GetPixelFormatName());

      // Set Image Time Stamp
      image.header.stamp.sec = image_ptr->GetTimeStamp() * 1e-9;
      image.header.stamp.nsec = image_ptr->GetTimeStamp();

      if (image_ptr->IsIncomplete())
      {
        ROS_ERROR("Camera %d Received but is Incomplete", serial_);
      }
      else
      {
        int width = image_ptr->GetWidth();
        int height = image_ptr->GetHeight();
        int stride = image_ptr->GetStride();

        // ROS_INFO("\033[93m wxh: (%d, %d) \n", width, height);

        // Check the bits per pixel.
        size_t bitsPerPixel = image_ptr->GetBitsPerPixel();


        // --------------------------------------------------
        // Set the image encoding
        std::string imageEncoding = sensor_msgs::image_encodings::MONO8;

        Spinnaker::GenApi::IEnumerationT<Spinnaker::PixelColorFilterEnums>& bayer_format = pCam_->PixelColorFilter;

        // if(isColor_ && bayer_format != NONE)
        if (bayer_format != Spinnaker::PixelColorFilter_None)
        {
          if(bitsPerPixel == 16)
          {
            // 16 Bits per Pixel
            if (bayer_format == Spinnaker::PixelColorFilter_BayerRG)
            {
              imageEncoding = sensor_msgs::image_encodings::BAYER_RGGB16;
            }
            else if (bayer_format == Spinnaker::PixelColorFilter_BayerGR)
            {
              imageEncoding = sensor_msgs::image_encodings::BAYER_GRBG16;
            }
            else if (bayer_format == Spinnaker::PixelColorFilter_BayerGB)
            {
              imageEncoding = sensor_msgs::image_encodings::BAYER_GBRG16;
            }
            else if (bayer_format == Spinnaker::PixelColorFilter_BayerBG)
            {
              imageEncoding = sensor_msgs::image_encodings::BAYER_BGGR16;
            }
            else
            {
              ROS_ERROR("Bayer Format Not Recognized!");
            }
          }
          else
          {
            // 8 Bits per Pixel
            if (bayer_format == Spinnaker::PixelColorFilter_BayerRG)
            {
              imageEncoding = sensor_msgs::image_encodings::BAYER_RGGB8;
            }
            else if (bayer_format == Spinnaker::PixelColorFilter_BayerGR)
            {
              imageEncoding = sensor_msgs::image_encodings::BAYER_GRBG8;
            }
            else if (bayer_format == Spinnaker::PixelColorFilter_BayerGB)
            {
              imageEncoding = sensor_msgs::image_encodings::BAYER_GBRG8;
            }
            else if (bayer_format == Spinnaker::PixelColorFilter_BayerBG)
            {
              imageEncoding = sensor_msgs::image_encodings::BAYER_BGGR8;
            }
            else
            {
              ROS_ERROR("Bayer Format Not Recognized!");
            }
          }
        }
        else     // Mono camera or in pixel binned mode.
        {
          if(bitsPerPixel == 16)
          {
            imageEncoding = sensor_msgs::image_encodings::MONO16;
          }
          else if(bitsPerPixel==24)
          {
            imageEncoding = sensor_msgs::image_encodings::RGB8;
          }
          else
          {
            imageEncoding = sensor_msgs::image_encodings::MONO8;
          }
        }

        // Check the bayer/pixel format with:
        // std::string format(image_ptr->GetPixelFormatName());
        // const std::string  BAYER_GBRG8 = "bayer_gbrg8"
        // std::string imageEncoding = sensor_msgs::image_encodings::BAYER_GBRG8;

        fillImage(image, imageEncoding, height, width, stride, image_ptr->GetData());
        image.header.frame_id = frame_id;

      }  // end else
    }
    catch (Spinnaker::Exception& e)
    {
      ROS_ERROR_STREAM("PointGreyCamera::grabImage Failed to retrieve buffer with Error: " << e.what());
      result = -1;
    }

  }
  else if(pCam_)
  {
    throw CameraNotRunningException("PointGreyCamera::grabImage: Camera is currently not running.  Please start the capture.");
  }
  else
  {
    throw std::runtime_error("PointGreyCamera::grabImage not connected!");
  }

  return result;

}  // end grabImage


// TODO: @tthomas - Implement later if needed
void PointGreyCamera::grabStereoImage(sensor_msgs::Image &image, const std::string &frame_id,
  sensor_msgs::Image &second_image, const std::string &second_frame_id)
{

}

// TODO @tthomas
// uint PointGreyCamera::getGain()
// {
//   return metadata_.embeddedGain >> 20;
// }

// uint PointGreyCamera::getShutter()
// {
//   return metadata_.embeddedShutter >> 20;
// }

// uint PointGreyCamera::getBrightness()
// {
//   return metadata_.embeddedTimeStamp >> 20;
// }

// uint PointGreyCamera::getExposure()
// {
//   return metadata_.embeddedBrightness >> 20;
// }

// uint PointGreyCamera::getWhiteBalance()
// {
//   return metadata_.embeddedExposure >> 8;
// }

// uint PointGreyCamera::getROIPosition()
// {
//   return metadata_.embeddedROIPosition >> 24;
// }

void PointGreyCamera::setDesiredCamera(const uint32_t &id)
{
  serial_ = id;
}

// std::vector<uint32_t> PointGreyCamera::getAttachedCameras()
// {
//   std::vector<uint32_t> cameras;
//   unsigned int num_cameras;
//   Error error = busMgr_.GetNumOfCameras(&num_cameras);
//   PointGreyCamera::handleError("PointGreyCamera::getAttachedCameras: Could not get number of cameras", error);
//   for(unsigned int i = 0; i < num_cameras; i++)
//   {
//     unsigned int this_serial;
//     error = busMgr_.GetCameraSerialNumberFromIndex(i, &this_serial);
//     PointGreyCamera::handleError("PointGreyCamera::getAttachedCameras: Could not get get serial number from index", error);
//     cameras.push_back(this_serial);
//   }
//   return cameras;
// }



bool PointGreyCamera::setProperty(const std::string &property_name, const std::string &entry_name)
{
  // *** NOTES ***
  // Enumeration nodes are slightly more complicated to set than other
  // nodes. This is because setting an enumeration node requires working
  // with two nodes instead of the usual one.
  //
  // As such, there are a number of steps to setting an enumeration node:
  // retrieve the enumeration node from the nodemap, retrieve the desired
  // entry node from the enumeration node, retrieve the integer value from
  // the entry node, and set the new value of the enumeration node with
  // the integer value from the entry node.
  Spinnaker::GenApi::CEnumerationPtr enumerationPtr = node_map_->GetNode(property_name.c_str());

  if (!Spinnaker::GenApi::IsImplemented(enumerationPtr))
  {
    ROS_ERROR_STREAM("[PointGreyCamera]: (" << serial_ << ") Enumeration name " << property_name << " not implemented.");
    return false;
  }

  if (Spinnaker::GenApi::IsAvailable(enumerationPtr))
  {
    if (Spinnaker::GenApi::IsWritable(enumerationPtr))
    {
      Spinnaker::GenApi::CEnumEntryPtr enumEmtryPtr = enumerationPtr->GetEntryByName(entry_name.c_str());

      if (Spinnaker::GenApi::IsAvailable(enumEmtryPtr))
      {
        if (Spinnaker::GenApi::IsReadable(enumEmtryPtr))
        {
          enumerationPtr->SetIntValue(enumEmtryPtr->GetValue());

          ROS_INFO_STREAM("[PointGreyCamera]: (" << serial_ <<  ") " << property_name << " set to " <<
            enumerationPtr->GetCurrentEntry()->GetSymbolic() << ".");

          return true;
        }
        else
        {
          ROS_WARN_STREAM("[PointGreyCamera]: (" << serial_ << ") Entry name " << entry_name << " not writable.");
        }
      }
      else
      {
        ROS_WARN_STREAM("[PointGreyCamera]: (" << serial_ << ") Entry name " << entry_name << " not available.");
      }
    }
    else
    {
      ROS_WARN_STREAM("[PointGreyCamera]: (" << serial_ << ") Enumeration " << property_name << " not writable.");
    }
  }
  else
  {
    ROS_WARN_STREAM("[PointGreyCamera]: (" << serial_ << ") Enumeration " << property_name << " not available.");
  }
  return false;
}

bool PointGreyCamera::setProperty(const std::string &property_name, const float& value)
{
  Spinnaker::GenApi::CFloatPtr floatPtr = node_map_->GetNode(property_name.c_str());

  if (!Spinnaker::GenApi::IsImplemented(floatPtr))
  {
    ROS_ERROR_STREAM("[PointGreyCamera]: (" << serial_ << ") Feature name " << property_name << " not implemented.");
    return false;
  }
  if (Spinnaker::GenApi::IsAvailable(floatPtr))
  {
    if (Spinnaker::GenApi::IsWritable(floatPtr))
    {
      floatPtr->SetValue(value);
      ROS_INFO_STREAM("[PointGreyCamera]: (" << serial_ << ") " <<  property_name << " set to " << floatPtr->GetValue() << ".");
      return true;
    } else {
      ROS_WARN_STREAM("[PointGreyCamera]: (" << serial_ << ") Feature " <<
       property_name << " not writable.");
    }
  } else {
    ROS_WARN_STREAM("[PointGreyCamera]: (" << serial_ << ") Feature " <<
      property_name << " not available.");
  }
  return false;
}


bool PointGreyCamera::setProperty(const std::string &property_name, const bool &value)
{
  Spinnaker::GenApi::CBooleanPtr boolPtr = node_map_->GetNode(property_name.c_str());
  if (!Spinnaker::GenApi::IsImplemented(boolPtr))
  {
    ROS_ERROR_STREAM("[PointGreyCamera]: (" << serial_ << ") Feature name " << property_name << " not implemented.");
    return false;
  }
  if (Spinnaker::GenApi::IsAvailable(boolPtr))
  {
    if (Spinnaker::GenApi::IsWritable(boolPtr))
    {
      boolPtr->SetValue(value);
      ROS_INFO_STREAM("[PointGreyCamera]: (" << serial_ << ") " << property_name << " set to " << boolPtr->GetValue() << ".");
      return true;
    }
    else
    {
      ROS_WARN_STREAM("[PointGreyCamera]: (" << serial_ << ") Feature " << property_name << " not writable.");
    }
  }
  else
  {
    ROS_WARN_STREAM("[PointGreyCamera]: (" << serial_ << ") Feature " <<  property_name << " not available.");
  }
  return false;
}

bool PointGreyCamera::setProperty(const std::string &property_name, const int &value)
{
  Spinnaker::GenApi::CIntegerPtr intPtr = node_map_->GetNode(property_name.c_str());
  if (!Spinnaker::GenApi::IsImplemented(intPtr))
  {
    ROS_ERROR_STREAM("[PointGreyCamera]: (" << serial_ <<  ") Feature name " << property_name << " not implemented.");
    return false;
  }
  if (Spinnaker::GenApi::IsAvailable(intPtr))
  {
    if (Spinnaker::GenApi::IsWritable(intPtr))
    {
      intPtr->SetValue(value);
      ROS_INFO_STREAM("[PointGreyCamera]: (" << serial_ << ") " << property_name << " set to " << intPtr->GetValue() << ".");
      return true;
    }
    else
    {
      ROS_WARN_STREAM("[PointGreyCamera]: (" << serial_ << ") Feature " << property_name << " not writable.");
    }
  }
  else
  {
    ROS_WARN_STREAM("[PointGreyCamera]: (" << serial_ << ") Feature " << property_name << " not available.");
  }
  return false;
}

bool PointGreyCamera::setMaxInt(const std::string &property_name)
{
  Spinnaker::GenApi::CIntegerPtr intPtr =  node_map_->GetNode(property_name.c_str());

  if (Spinnaker::GenApi::IsAvailable(intPtr))
  {
    if (Spinnaker::GenApi::IsWritable(intPtr))
    {
      intPtr->SetValue(intPtr->GetMax());
      ROS_INFO_STREAM("[PointGreyCamera]: (" << serial_ << ") " << property_name << " set to " << intPtr->GetValue() << ".");
      return true;
    }
    else
    {
      ROS_WARN_STREAM("[PointGreyCamera]: (" << serial_ << ") Feature " << property_name << " not writable.");
    }
  }
  else
  {
    ROS_WARN_STREAM("[PointGreyCamera]: (" << serial_ << ") Feature " << property_name << " not available.");
  }
  return false;
}


int ConfigureChunkData(Spinnaker::GenApi::INodeMap & nodeMap)
{
  int result = 0;
  ROS_INFO_STREAM("*** CONFIGURING CHUNK DATA ***");
  try
  {
    // Activate chunk mode
    //
    // *** NOTES ***
    // Once enabled, chunk data will be available at the end of the payload
    // of every image captured until it is disabled. Chunk data can also be
    // retrieved from the nodemap.
    //
    Spinnaker::GenApi::CBooleanPtr ptrChunkModeActive = nodeMap.GetNode("ChunkModeActive");
    if (!Spinnaker::GenApi::IsAvailable(ptrChunkModeActive) || !Spinnaker::GenApi::IsWritable(ptrChunkModeActive))
    {
      ROS_ERROR_STREAM("Unable to activate chunk mode. Aborting...");
      return -1;
    }
    ptrChunkModeActive->SetValue(true);
    ROS_INFO_STREAM("Chunk mode activated...");

    // Enable all types of chunk data
    //
    // *** NOTES ***
    // Enabling chunk data requires working with nodes: "ChunkSelector"
    // is an enumeration selector node and "ChunkEnable" is a boolean. It
    // requires retrieving the selector node (which is of enumeration node
    // type), selecting the entry of the chunk data to be enabled, retrieving
    // the corresponding boolean, and setting it to true.
    //
    // In this example, all chunk data is enabled, so these steps are
    // performed in a loop. Once this is complete, chunk mode still needs to
    // be activated.
    //
    Spinnaker::GenApi::NodeList_t entries;
    // Retrieve the selector node
    Spinnaker::GenApi::CEnumerationPtr ptrChunkSelector = nodeMap.GetNode("ChunkSelector");
    if (!Spinnaker::GenApi::IsAvailable(ptrChunkSelector) || !Spinnaker::GenApi::IsReadable(ptrChunkSelector))
    {
      ROS_ERROR_STREAM("Unable to retrieve chunk selector. Aborting...");
      return -1;
    }
    // Retrieve entries
    ptrChunkSelector->GetEntries(entries);

    ROS_INFO_STREAM("Enabling entries...");

    for (int i = 0; i < entries.size(); i++)
    {
      // Select entry to be enabled
      Spinnaker::GenApi::CEnumEntryPtr ptrChunkSelectorEntry = entries.at(i);
      // Go to next node if problem occurs
      if (!Spinnaker::GenApi::IsAvailable(ptrChunkSelectorEntry) || !Spinnaker::GenApi::IsReadable(ptrChunkSelectorEntry))
      {
        continue;
      }
      ptrChunkSelector->SetIntValue(ptrChunkSelectorEntry->GetValue());

      ROS_INFO_STREAM("\t" << ptrChunkSelectorEntry->GetSymbolic() << ": ");
      // Retrieve corresponding boolean
      Spinnaker::GenApi::CBooleanPtr ptrChunkEnable = nodeMap.GetNode("ChunkEnable");
      // Enable the boolean, thus enabling the corresponding chunk data
      if (!Spinnaker::GenApi::IsAvailable(ptrChunkEnable))
      {
        ROS_INFO("Node not available");
      }
      else if (ptrChunkEnable->GetValue())
      {
        ROS_INFO("Enabled");
      }
      else if (Spinnaker::GenApi::IsWritable(ptrChunkEnable))
      {
        ptrChunkEnable->SetValue(true);
        ROS_INFO("Enabled");
      }
      else
      {
        ROS_INFO("Node not writable");
      }
    }
  }
  catch (Spinnaker::Exception &e)
  {
    ROS_ERROR_STREAM("Error: " << e.what());
    result = -1;
  }
  return result;
}
