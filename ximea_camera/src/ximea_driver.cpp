/******************************************************************************

Copyright 2015  Abdelhamid El-Bably (University of Waterloo)
                      [ahelbably@uwaterloo.ca]
                Arun Das (University of Waterloo)
                      [adas@uwaterloo.ca]
                Michael Tribou (University of Waterloo)
                      [mjtribou@uwaterloo.ca]

All rights reserved.

********************************************************************************/

#include <ximea_camera/ximea_driver.h>
#include <stdexcept>
#include <sstream>
#include <string>

ximea_driver::ximea_driver(int serial_no, std::string cam_name)
{
  serial_no_ = serial_no;
  cam_name_ = cam_name;
  assignDefaultValues();
}

ximea_driver::ximea_driver(int serial_no, std::string cam_name, std::string file_name, std::string frame_id)
{ 
  assignDefaultValues();
  readParamsFromFile(file_name);
  serial_no_ = serial_no;
  cam_name_ = cam_name;
  frame_id_ = frame_id;  
}


void ximea_driver::assignDefaultValues()
{
  cams_on_bus_ = 4;
  bandwidth_safety_margin_ = 30;
  binning_enabled_ = false;
  downsample_factor_ = false;
  auto_exposure_ = 0;
  auto_exposure_limit_ = 100000;
  auto_gain_limit_ = 14;
  auto_exposure_priority_ = 0.8;
  trigger_source_ = 0;

  exposure_time_ = 1000;
  image_data_format_ = "XI_MONO8";
  rect_left_ = 0;
  rect_top_ = 0;
  rect_width_ = 1280;
  rect_height_ = 1024;
  xiH_ = NULL;
  image_.size = sizeof(XI_IMG);
  image_.bp = NULL;
  image_.bp_size = 0;
  acquisition_active_ = false;
  image_capture_timeout_ = 1000;
  frame_id_ = "";
  auto_wb_ = 1;
}

ximea_driver::ximea_driver(std::string file_name)
{
  assignDefaultValues();
  readParamsFromFile(file_name);
  ROS_INFO_STREAM("ximea_driver: reading paramter values from file: " << file_name);
}

void ximea_driver::errorHandling(XI_RETURN ret, std::string message)
{
  if (ret != XI_OK)
  {
    std::stringstream errMsg;
    std::cout << "Error after "  << message << std::endl;
    closeDevice();
  }
}

void ximea_driver::applyParameters()
{
  std::cout << " Applying Parameters\r\n";  
  setImageDataFormat(image_data_format_);
  if (0 == auto_exposure_) {
      setExposure(exposure_time_);
  } else {
      std::cout << "Setting autoexposure\r\n";
      setAutoExposure(auto_exposure_);
      setAutoExposureLimit(auto_exposure_limit_);
      setAutoGainLimit(auto_gain_limit_);
      setAutoExposurePriority(auto_exposure_priority_);
  }
  setROI(rect_left_, rect_top_, rect_width_, rect_height_);
  setAutoWB(auto_wb_);
  
  // setOtherParams();
}

void ximea_driver::openDevice()
{
  XI_RETURN stat;
  if (serial_no_ == 0)
  {
    stat = xiOpenDevice(0, &xiH_);
    errorHandling(stat, "Open Device");
  }
  else
  {
    std::stringstream conv;
    conv << serial_no_;
    stat = xiOpenDeviceBy(XI_OPEN_BY_SN, conv.str().c_str(), &xiH_);
    errorHandling(stat, "Open Device");
  }
  applyParameters();
}

void ximea_driver::closeDevice()
{
  if (xiH_)
  {
    xiCloseDevice(xiH_);
    xiH_ = NULL;
  }
}

void ximea_driver::stopAcquisition()
{
  XI_RETURN stat;
  if (!hasValidHandle())
  {
    return;
  }
  stat = xiStopAcquisition(xiH_);
  errorHandling(stat, "Starting Acquisition");  // die if we cannot stop acquisition.
  acquisition_active_ = false;
}

void ximea_driver::startAcquisition()
{
  if (!hasValidHandle())
  {
    return;
  }
  XI_RETURN stat = xiStartAcquisition(xiH_);
  errorHandling(stat, "Starting Acquisition");
  acquisition_active_ = true;
}

int ximea_driver::acquireImage()
{
  if (!hasValidHandle())
  {
    return -1;
  }
  XI_RETURN stat = xiGetImage(xiH_, image_capture_timeout_, &image_);
  if (stat != 0)
  {
    std::cout << "Error on" << cam_name_ << " with error " <<  stat << std::endl;
    return (int) stat;
  }

  return 0;
}

XI_IMG* ximea_driver::getAcquiredImage()
{
    return &image_;
}

void ximea_driver::setImageDataFormat(std::string image_format)
{
  XI_RETURN stat;
  int image_data_format;

  if (!hasValidHandle())
  {
    return;
  }
  if (image_format == std::string("XI_MONO16"))
  {
    image_data_format = XI_MONO16;
  }

  else if (image_format == std::string("XI_RGB24"))
  {
    image_data_format = XI_RGB24;
  }

  else if (image_format == std::string("XI_RGB32"))
  {
    image_data_format = XI_RGB32;
  }

  else if (image_format == std::string("XI_RGB_PLANAR"))
  {
    image_data_format = XI_MONO8;
    std::cout << "This is unsupported in ROS default to XI_MONO8" << std::endl;
  }

  else if (image_format == std::string("XI_RAW8"))
  {
    image_data_format = XI_RAW8;
  }

  else if (image_format == std::string("XI_RAW16"))
  {
    image_data_format = XI_RAW16;
  }

  else
  {
    image_data_format = XI_MONO8;
  }

  stat = xiSetParamInt(xiH_, XI_PRM_IMAGE_DATA_FORMAT, image_data_format);
  errorHandling(stat, "image_format");  // if we cannot set the format then there is something wrong we should probably quit then
  image_data_format_ = image_data_format;
}

void ximea_driver::setROI(int l, int t, int w, int h)
{
  XI_RETURN stat;

  if (!hasValidHandle())
  {
    return;
  }

  if (l < 0 || l > 1280) rect_left_ = 0;
  else rect_left_ = l;
  if (t < 0 || t > 1024) rect_top_ = 0;
  else rect_top_ = t;
  if (w < 0 || w > 1280) rect_width_ = 1280;
  else rect_width_ = w;
  if (h < 0 || h > 1024) rect_height_ = 1024;
  else rect_height_ = h;
  if (l + w > 1280)
  {
    rect_left_ =  0;
    rect_width_ = 1280;
  }
  if (h + t > 1024)
  {
    rect_top_ =  0;
    rect_height_ = 1024;
  }

  std::cout << rect_height_ << " " << rect_width_ << " " << rect_left_ << " " << rect_top_ << std::endl;

  int tmp;
  stat = xiSetParamInt(xiH_, XI_PRM_WIDTH, rect_width_);
  errorHandling(stat, "xiSetParamInt (aoi width)");
  xiGetParamInt(xiH_, XI_PRM_WIDTH XI_PRM_INFO_INCREMENT, &tmp);
  std::cout << "width increment " << tmp << std::endl;

  stat = xiSetParamInt(xiH_, XI_PRM_HEIGHT, rect_height_);
  errorHandling(stat, "xiSetParamInt (aoi height)");
  xiGetParamInt(xiH_, XI_PRM_HEIGHT XI_PRM_INFO_INCREMENT, &tmp);
  std::cout << "height increment " << tmp << std::endl;

  stat = xiSetParamInt(xiH_, XI_PRM_OFFSET_X, rect_left_);
  errorHandling(stat, "xiSetParamInt (aoi left)");
  xiGetParamInt(xiH_, XI_PRM_OFFSET_X XI_PRM_INFO_INCREMENT, &tmp);
  std::cout << "left increment " << tmp << std::endl;

  stat = xiSetParamInt(xiH_, XI_PRM_OFFSET_Y, rect_top_);
  errorHandling(stat, "xiSetParamInt (aoi top)");
  xiGetParamInt(xiH_, XI_PRM_OFFSET_Y XI_PRM_INFO_INCREMENT, &tmp);
  std::cout << "top increment " << tmp << std::endl;
}

void ximea_driver::setOtherParams()
{
  xiSetParamInt(xiH_, XI_PRM_TEMP_SELECTOR, XI_TEMP_SENSOR_BOARD); 
}

void ximea_driver::setExposure(int time)
{
  XI_RETURN stat = xiSetParamInt(xiH_, XI_PRM_EXPOSURE, time);
  errorHandling(stat, "xiOSetParamInt (Exposure Time)");
  if (!stat)
  {
    exposure_time_ = time;
  }
}

void ximea_driver::setAutoExposure(int auto_exposure)
{
  XI_RETURN stat = xiSetParamInt(xiH_, XI_PRM_AEAG, auto_exposure);
  errorHandling(stat, "xiOSetParamInt (AutoExposure Time)");
  if (!stat)
  {
    // auto_exposureme_ = time;
  }
}

void ximea_driver::setAutoWB(int auto_wb)
{
  XI_RETURN stat = xiSetParamInt(xiH_, XI_PRM_AUTO_WB, auto_wb);
  errorHandling(stat, "xiOSetParamInt (AutoExposure Time)");
  if (!stat)
  {
    // auto_exposureme_ = time;
  }
}

void ximea_driver::setAutoExposureLimit(int ae_limit)
{
  XI_RETURN stat = xiSetParamFloat(xiH_, XI_PRM_AE_MAX_LIMIT, ae_limit);
  errorHandling(stat, "xiOSetParamInt (AutoExposure Limit Time)");
  if (!stat)
  {
    // auto_exposureme_ = time;
  }
}

void ximea_driver::setAutoGainLimit(int ag_limit)
{
  XI_RETURN stat = xiSetParamInt(xiH_, XI_PRM_AG_MAX_LIMIT, ag_limit);
  errorHandling(stat, "xiOSetParamInt (AutoExposure Limit GAIN)");
  if (!stat)
  {
    // auto_exposureme_ = time;
  }
}


void ximea_driver::setAutoExposurePriority(float exp_priority)
{
  XI_RETURN stat = xiSetParamFloat(xiH_, XI_PRM_EXP_PRIORITY, exp_priority);
  errorHandling(stat, "xiOSetParamInt (AutoExposure Priority)");
  if (!stat)
  {
    // auto_exposureme_ = time;
  }
}


int ximea_driver::readParamsFromFile(std::string file_name)
{
  std::ifstream fin(file_name.c_str());
  if (fin.fail())
  {
    ROS_ERROR_STREAM("could not open file " << file_name.c_str() << std::endl);
    exit(-1);
  }


  YAML::Node doc = YAML::LoadFile(file_name);
  std::string tmpS;
  int tmpI1, tmpI2, tmpI3, tmpI4;
  bool tmpB;


  try
  {
    serial_no_ = doc["serial_no"].as<int>();
  }
  catch (std::runtime_error) {}
  try
  {
    cam_name_ =  doc["cam_name"].as<std::string>();
  }
  catch (std::runtime_error) {}
  try
  {
    frame_id_ =  doc["frame_id"].as<std::string>();
  }
  catch (std::runtime_error) {}

  try
  {
    yaml_url_ = doc["yaml_url"].as<std::string>();
  }
  catch (std::runtime_error) {}

  try
  {
    cams_on_bus_ = doc["cams_on_bus"].as<int>();
  }
  catch (std::runtime_error) {}

  try
  {
    bandwidth_safety_margin_ = doc["bandwidth_safety_margin"].as<int>();
  }
  catch (std::runtime_error) {}

  try
  {
    frame_rate_ = doc["frame_rate"].as<int>();
  }
  catch (std::runtime_error) {}

  try
  {
    exposure_time_ = doc["exposure_time"].as<int>();
  }
  catch (std::runtime_error) {}

  try
  {
    auto_wb_ = doc["auto_wb"].as<int>();
  }
  catch (std::runtime_error) {}


  try
  {
    auto_exposure_ = doc["auto_exposure"].as<int>();
  }
  catch (std::runtime_error) {}

  try
  {
    auto_exposure_limit_ = doc["auto_exposure_limit"].as<int>();
  }
  catch (std::runtime_error) {}

  try
  {
    auto_gain_limit_ = doc["auto_gain_limit"].as<int>();
  }
  catch (std::runtime_error) {}

  try
  {
    auto_exposure_priority_ = doc["auto_exposure_priority"].as<float>();
  }
  catch (std::runtime_error) {}

  try
  {
    binning_enabled_ = doc["binning_enabled"].as<bool>();
  }
  catch (std::runtime_error) {}

  try
  {
    downsample_factor_ = doc["downsample_factor_"].as<int>();
  }
  catch (std::runtime_error) {}

  try
  {
    rect_left_ = doc["rect_left"].as<int>();
  }
  catch (std::runtime_error) {}
  try
  {
    rect_top_ = doc["rect_top"].as<int>();
  }
  catch (std::runtime_error) {}
  try
  {
    rect_width_ = doc["rect_width"].as<int>();
  }
  catch (std::runtime_error) {}

  try
  {
    trigger_source_ = doc["trigger_source"].as<int>();
  }
  catch (std::runtime_error) {}
  setTrigger(trigger_source_);

  try
  {
    rect_height_ = doc["rect_height"].as<int>();
  }
  catch (std::runtime_error) {}
  setROI(rect_left_, rect_top_, rect_width_, rect_height_);
  try
  {
    image_data_format_ = doc["image_data_format"].as<std::string>();
  }
  catch (std::runtime_error) {}
  setImageDataFormat(image_data_format_);
}
void ximea_driver::enableTrigger(unsigned char trigger_mode)
{
  if (!xiH_) return;
  trigger_mode_ = trigger_mode;
  if (trigger_mode_ > 3 || trigger_mode_ < 0)
  {
    trigger_mode_ = 0;
  }
  XI_RETURN stat;
  switch (trigger_mode_)
  {
  case (0):
    break;
  case (1):
    stat = xiSetParamInt(xiH_, XI_PRM_TRG_SOURCE, XI_TRG_SOFTWARE);
    errorHandling(stat, "Could not enable software trigger");
    break;
  case (2):
    break;
  default:
    break;
  }
}

void ximea_driver::triggerDevice()
{
  if (!xiH_) return;
  XI_RETURN stat;
  switch (trigger_mode_)
  {
  case (0):
    break;
  case (1):
    stat = xiSetParamInt(xiH_, XI_PRM_TRG_SOFTWARE, 1);
    errorHandling(stat, "Error During triggering");
    std::cout << "triggering " << cam_name_ <<  std::endl;
    break;
  case (2):
    break;
  default:
    break;
  }
}

void ximea_driver::setTrigger(int trigger_type)
{
  if (!xiH_) return;

  if (acquisition_active_)
  {
    stopAcquisition();
    xiSetParamInt(xiH_, XI_PRM_TRG_SOURCE, trigger_type);
    startAcquisition();
  }
  else
    xiSetParamInt(xiH_, XI_PRM_TRG_SOURCE, trigger_type);

}

void ximea_driver::limitBandwidth(int mbps)
{
  if (!xiH_) return;
  XI_RETURN stat;
  // stat = xiSetParamInt(xiH_, XI_PRM_LIMIT_BANDWIDTH , mbps);
  // errorHandling(stat, "could not limit bandwidth");
}

float ximea_driver::getCameraTemperature()
{
  if (!xiH_) return -1.0;

  float temperature_c=0;  
  xiGetParamFloat(xiH_, XI_PRM_TEMP, &temperature_c);

  return temperature_c;
}


float ximea_driver::getGain()
{
  if (!xiH_) return -1.0;

  float gain=0;
  xiGetParamFloat(xiH_, XI_PRM_GAIN, &gain);

  return gain;
}


float ximea_driver::getWBRed()
{
  if (!xiH_) return -1.0;

  float value=0;
  xiGetParamFloat(xiH_, XI_PRM_WB_KR, &value);

  return value;
}


float ximea_driver::getWBGreen()
{
  if (!xiH_) return -1.0;

  float value=0;
  xiGetParamFloat(xiH_, XI_PRM_WB_KG, &value);

  return value;
}

float ximea_driver::getWBBlue()
{
  if (!xiH_) return -1.0;

  float value=0;
  xiGetParamFloat(xiH_, XI_PRM_WB_KB, &value);

  return value;
}


  void ximea_driver::setGain(float gain)
  {
    if (!xiH_) return;

    xiSetParamFloat(xiH_, XI_PRM_GAIN, gain);    
  }
  void ximea_driver::setWhiteBalance(float red, float green, float blue)
  {
    if (!xiH_) return;

    if (red > 0.0)
      xiSetParamFloat(xiH_, XI_PRM_WB_KR, red);

    if (green > 0.0)
      xiSetParamFloat(xiH_, XI_PRM_WB_KG, green);

    if (blue > 0.0)
      xiSetParamFloat(xiH_, XI_PRM_WB_KB, blue);
  }
