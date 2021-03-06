/******************************************************************************

Copyright 2015  Abdelhamid El-Bably (University of Waterloo)
                      [ahelbably@uwaterloo.ca]
                Arun Das (University of Waterloo)
                      [adas@uwaterloo.ca]
                Michael Tribou (University of Waterloo)
                      [mjtribou@uwaterloo.ca]

All rights reserved.

********************************************************************************/
#ifndef XIMEA_CAMERA_XIMEA_DRIVER_H
#define XIMEA_CAMERA_XIMEA_DRIVER_H

// Sample for XIMEA Software Package V2.57
#include <m3api/xiApi.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/UInt8.h>
#include <camera_info_manager/camera_info_manager.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <fstream>
#include <yaml-cpp/yaml.h>

class ximea_driver
{
public:
  explicit ximea_driver(int serial_no = 0 , std::string cam_name = "");  // if no serial no is specified select the first cam on the bus
  explicit ximea_driver(std::string file_name);
  explicit ximea_driver(int serial_no, std::string cam_name, std::string file_name, std::string frame_id);

  int readParamsFromFile(std::string file_name);
  void applyParameters();
  void errorHandling(XI_RETURN ret, std::string message);
  void enableTrigger(unsigned char trigger_mode);  // 0 none, 1 soft_trigger, 2 hard_trigger_rising edge (unsupported)
  void limitBandwidth(int mbps);
  void openDevice();
  void closeDevice();
  void startAcquisition();
  void stopAcquisition();
  int acquireImage();
  void triggerDevice();
  int getSerialNo() const
  {
    return serial_no_;
  }
  virtual void setImageDataFormat(std::string s);  // this is virtual because the ros class needs to do a bit more work to publish the right image
  void setROI(int rect_left, int rect_top, int rect_width, int rect_height);
  void setExposure(int time);
  void setAutoExposure(int auto_exposure);
  void setAutoExposureLimit(int ae_limit);
  void setAutoGainLimit(int ag_limit);
  void setAutoExposurePriority(float exp_priority);
  void setAutoWB(int auto_wb);
  void setOtherParams();
  void setTrigger(int trigger_type);
  bool hasValidHandle()
  {
    return xiH_ == NULL ? false : true;
  }
  const XI_IMG& getImage()const
  {
    return image_;
  }

  float getCameraTemperature();
  float getGain();
  float getWBRed();
  float getWBGreen();
  float getWBBlue();

  void setGain(float gain);
  void setWhiteBalance(float red, float green, float blue);
  XI_IMG* getAcquiredImage();


  XI_IMG image_;

protected:
  void assignDefaultValues();

  // variables for ximea api internals
  std::string cam_name_;
  int serial_no_;
  std::string frame_id_;
  int cams_on_bus_;
  int bandwidth_safety_margin_;
  int frame_rate_;
  int bandwidth_;
  int exposure_time_;
  int auto_exposure_;
  int auto_exposure_limit_;
  int auto_gain_limit_;
  float auto_exposure_priority_;
  bool binning_enabled_;
  int downsample_factor_;
  int rect_left_;
  int rect_top_;
  int rect_width_;
  int rect_height_;
  bool acquisition_active_;
  int trigger_source_;
  std::string image_data_format_;  // One of XI_MONO8, XI_RGB24, XI_RGB32, XI_RAW
  std::string yaml_url_;
  HANDLE xiH_;  
  int image_capture_timeout_;  // max amount of time to wait for an image to come in
  unsigned char trigger_mode_;
  int auto_wb_;
};

#endif  // XIMEA_CAMERA_XIMEA_DRIVER_H
