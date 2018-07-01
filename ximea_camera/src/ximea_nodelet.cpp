/******************************************************************************

Copyright 2015  Abdelhamid El-Bably (University of Waterloo)
                      [ahelbably@uwaterloo.ca]
                Arun Das (University of Waterloo)
                      [adas@uwaterloo.ca]
                Michael Tribou (University of Waterloo)
                      [mjtribou@uwaterloo.ca]

All rights reserved.

********************************************************************************/
#include "ros/ros.h"
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <image_transport/image_transport.h> // ROS library that allows sending compressed images
#include <camera_info_manager/camera_info_manager.h> // ROS library that publishes CameraInfo topics
#include <sensor_msgs/CameraInfo.h> // ROS message header for CameraInfo

#include <image_exposure_msgs/ExposureSequence.h> // Message type for configuring gain and white balance.

#include <diagnostic_updater/diagnostic_updater.h> // Headers for publishing diagnostic messages.
#include <diagnostic_updater/publisher.h>

#include <ximea_camera/ximea_driver.h>


#include <wfov_camera_msgs/WFOVImage.h>

#include <string>
#include <algorithm>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp> // Needed for the nodelet to launch the reading thread.



namespace ximea_camera
{
  
class XimeaCameraNodelet: public nodelet::Nodelet
{
public:
  XimeaCameraNodelet() {}

  ~XimeaCameraNodelet()
  {
    boost::mutex::scoped_lock scopedLock(connect_mutex_);

    if(pubThread_)
    {
      pubThread_->interrupt();
      pubThread_->join();

      try
      {
        NODELET_DEBUG("Stopping camera capture.");
        xm_.stopAcquisition();        
        NODELET_DEBUG("Disconnecting from camera.");
        xm_.closeDevice();        
      }
      catch(std::runtime_error& e)
      {
        NODELET_ERROR("%s", e.what());
      }
    }
  }

private:
  void onInit()
  {
    // Get nodeHandles
    ros::NodeHandle &nh = getMTNodeHandle();
    ros::NodeHandle &pnh = getMTPrivateNodeHandle();

    // Get a serial number through ros
    int serial = 0;

    XmlRpc::XmlRpcValue serial_xmlrpc;
    pnh.getParam("serial", serial_xmlrpc);
    if (serial_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      pnh.param<int>("serial", serial, 0);
    }
    else if (serial_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      std::string serial_str;
      pnh.param<std::string>("serial", serial_str, "0");
      std::istringstream(serial_str) >> serial;
    }
    else
    {
      NODELET_DEBUG("Serial XMLRPC type.");
      serial = 0;
    }

    NODELET_INFO("Using camera serial %d", serial);
    // Get the location of our camera config yaml
    std::string camera_info_url;
    std::string configuration_url;    
    std::string image_format;


    pnh.param<std::string>("yaml_url", configuration_url, "");
    pnh.param<std::string>("camera_info_url", camera_info_url, "");
    // Get the desired frame_id, set to 'camera' if not found
    pnh.param<std::string>("frame_id", frame_id_, "camera");
    // Get the camera name, set to 'camera' if not found
    pnh.param<std::string>("camera_name", camera_name_, "camera");
    pnh.param<std::string>("image_data_format", image_format, "");

    NODELET_INFO("Image format %s.", image_format.c_str());

    ros::NodeHandle named_nh(getMTNodeHandle(), camera_name_);

    xm_ = ximea_driver(serial, camera_name_);
    xm_.readParamsFromFile(configuration_url);
    setImageDataFormat(image_format);    

    // Do not call the connectCb function until after we are done initializing.
    boost::mutex::scoped_lock scopedLock(connect_mutex_);

    // Start up the dynamic_reconfigure service, note that this needs to stick around after this function ends
    // srv_ = boost::make_shared <dynamic_reconfigure::Server<pointgrey_camera_driver::PointGreyConfig> > (pnh);
    // dynamic_reconfigure::Server<pointgrey_camera_driver::PointGreyConfig>::CallbackType f =
    //   boost::bind(&pointgrey_camera_driver::PointGreyCameraNodelet::paramCallback, this, _1, _2);
    // srv_->setCallback(f);

    // Start the camera info manager and attempt to load any configurations
    std::stringstream cinfo_name;
    cinfo_name << serial;
    cinfo_.reset(new camera_info_manager::CameraInfoManager(named_nh, cinfo_name.str(), camera_info_url));

    // Publish topics using ImageTransport through camera_info_manager (gives cool things like compression)
    it_.reset(new image_transport::ImageTransport(named_nh));
    image_transport::SubscriberStatusCallback cb = boost::bind(&XimeaCameraNodelet::connectCb, this);
    it_pub_ = it_->advertiseCamera("image_raw", 5, cb, cb);

    // Set up diagnostics
    updater_.setHardwareID("ximea_camera " + cinfo_name.str());

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
    ros::SubscriberStatusCallback cb2 = boost::bind(&XimeaCameraNodelet::connectCb, this);
    pub_.reset(new diagnostic_updater::DiagnosedPublisher<wfov_camera_msgs::WFOVImage>(named_nh.advertise<wfov_camera_msgs::WFOVImage>("image", 5, cb2, cb2),
               updater_,
               diagnostic_updater::FrequencyStatusParam(&min_freq_, &max_freq_, freq_tolerance, window_size),
               diagnostic_updater::TimeStampStatusParam(min_acceptable, max_acceptable))); 
  }

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
            xm_.stopAcquisition();
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
            xm_.closeDevice();
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
            xm_.openDevice();
            NODELET_INFO("Connected to camera.");

            // Set last configuration, forcing the reconfigure level to stop
            //pg_.setNewConfiguration(config_, PointGreyCamera::LEVEL_RECONFIGURE_STOP);

            // Set the timeout for grabbing images.
            try
            {
              double timeout;
              getMTPrivateNodeHandle().param("timeout", timeout, 1.0);

              NODELET_DEBUG("Setting timeout to: %f.", timeout);
              
              //pg_.setTimeout(timeout);
            }
            catch(std::runtime_error& e)
            {
              NODELET_ERROR("%s", e.what());
            }

            // Subscribe to gain and white balance changes
            {
              boost::mutex::scoped_lock scopedLock(connect_mutex_);
              //sub_ = getMTNodeHandle().subscribe("image_exposure_sequence", 10, &pointgrey_camera_driver::PointGreyCameraNodelet::gainWBCallback, this);
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
            xm_.startAcquisition();            
            NODELET_INFO("Started camera.");
            NODELET_INFO("Attention: if nothing subscribes to the camera topic, the camera_info is not published on the correspondent topic.");
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
          try
          {
             wfov_camera_msgs::WFOVImagePtr wfov_image(new wfov_camera_msgs::WFOVImage);
            // Get the image from the camera library
            NODELET_DEBUG("Starting a new grab from camera.");
            xm_.acquireImage();
            NODELET_DEBUG("New grab from camera completed.");
            //pg_.grabImage(wfov_image->image, frame_id_);
            cam_buffer_ = reinterpret_cast<char *>(xm_.image_.bp);
            cam_buffer_size_ = xm_.image_.width * xm_.image_.height * bpp_;
            wfov_image->image.data.resize(cam_buffer_size_);
            NODELET_DEBUG("Set encoding");
            wfov_image->image.encoding = encoding_;
            NODELET_DEBUG("Set encoding complete");
            wfov_image->image.width = xm_.image_.width;
            wfov_image->image.height = xm_.image_.height;
            wfov_image->image.step = xm_.image_.width * bpp_;

            copy(reinterpret_cast<char *>(cam_buffer_),
                (reinterpret_cast<char *>(cam_buffer_)) + cam_buffer_size_,
                wfov_image->image.data.begin());

            // Set other values
            wfov_image->header.frame_id = frame_id_;

            wfov_image->gain = xm_.image_.gain_db;
            wfov_image->white_balance_blue = xm_.getWBBlue();
            wfov_image->white_balance_red = xm_.getWBRed();            
            wfov_image->white_balance_green = xm_.getWBGreen();

            wfov_image->temperature = xm_.getCameraTemperature();          
            wfov_image->exposure_time = xm_.image_.exposure_time_us;  

            ros::Time time = ros::Time::now();
            wfov_image->header.stamp = time;
            wfov_image->image.header.stamp = time;

            // Set the CameraInfo message
            ci_.reset(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
            ci_->header.stamp = wfov_image->image.header.stamp;
            ci_->header.frame_id = wfov_image->header.frame_id;
            // The height, width, distortion model, and parameters are all filled in by camera info manager.
            ci_->binning_x = 0;
            ci_->binning_y = 0;
            ci_->roi.x_offset = 0;
            ci_->roi.y_offset = 0;
            ci_->roi.height = 0;
            ci_->roi.width = 0;
            ci_->roi.do_rectify = false;

            wfov_image->info = *ci_;

            // Publish the full message
            NODELET_DEBUG("Publish WFOV");
            pub_->publish(wfov_image);
            NODELET_DEBUG("Publish WFOV Complete");

            // Publish the message using standard image transport
            if(it_pub_.getNumSubscribers() > 0)
            {
              sensor_msgs::ImagePtr image(new sensor_msgs::Image(wfov_image->image));
              NODELET_DEBUG("Publish Image");
              it_pub_.publish(image, ci_);
              NODELET_DEBUG("Publish Image Complete");
            }
          }
          catch(std::runtime_error& e)
          {
            NODELET_ERROR("%s", e.what());

            state = ERROR;
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


  void connectCb()
  {
    NODELET_DEBUG("Connect callback!");
    boost::mutex::scoped_lock scopedLock(connect_mutex_); // Grab the mutex.  Wait until we're done initializing before letting this function through.
    // Check if we should disconnect (there are 0 subscribers to our data)
    if(it_pub_.getNumSubscribers() == 0 && pub_->getPublisher().getNumSubscribers() == 0)
    {
      /* if (pubThread_)
      {
        NODELET_DEBUG("Disconnecting.");
        pubThread_->interrupt();
        scopedLock.unlock();
        pubThread_->join();
        scopedLock.lock();
        pubThread_.reset();
        sub_.shutdown();

        try
        {
          NODELET_DEBUG("Stopping camera capture.");
          xm_.stopAcquisition();          
        }
        catch(std::runtime_error& e)
        {
          NODELET_ERROR("%s", e.what());
        }

        try
        {
          NODELET_DEBUG("Disconnecting from camera.");
          xm_.closeDevice();          
        }
        catch(std::runtime_error& e)
        {
          NODELET_ERROR("%s", e.what());
        } 
      }*/
    }
    else if(!pubThread_)     // We need to connect
    {
      NODELET_INFO("Connecting.");
      // Start the thread to loop through and publish messages
      pubThread_.reset(new boost::thread(boost::bind(&XimeaCameraNodelet::devicePoll, this)));
    }
    else
    {
      NODELET_DEBUG("Do nothing in callback.");
    }
  }

void gainWBCallback(const image_exposure_msgs::ExposureSequence &msg)
  {
    try
    {
      NODELET_DEBUG("Gain callback:  Setting gain to %f and white balances to %u, %u", msg.gain, msg.white_balance_blue, msg.white_balance_red);
      gain_ = msg.gain;
      xm_.setGain(gain_);
      wb_blue_ = msg.white_balance_blue;
      wb_red_ = msg.white_balance_red;
      //pg_.setBRWhiteBalance(false, wb_blue_, wb_red_);
    }
    catch(std::runtime_error& e)
    {
      NODELET_ERROR("gainWBCallback failed with error: %s", e.what());
    }
  }

void setImageDataFormat(std::string image_format)
{
  XI_RETURN stat;
  int image_data_format;
  NODELET_INFO("SetImageDataFormat %s.", image_format.c_str());


  //if (!xm_.hasValidHandle())
  //{
  //  NODELET_INFO("SetImageDataFormat early exit");
  //  return;
  //}
  if (image_format == std::string("XI_MONO16"))
  {
    image_data_format = XI_MONO16;
    encoding_ = std::string("mono16");
    bpp_ = 2;
  }

  else if (image_format == std::string("XI_RGB24"))
  {
    image_data_format = XI_RGB24;
    encoding_ = std::string("bgr8");
    bpp_ = 3;
    NODELET_INFO("Image info %s %i.", encoding_.c_str(), bpp_);
  }

  else if (image_format == std::string("XI_RGB32"))
  {
    image_data_format = XI_RGB32;
    encoding_ = std::string("bgr16");
    bpp_ = 3;
  }

  else if (image_format == std::string("XI_RGB_PLANAR"))
  {
    image_data_format = XI_MONO8;
    std::cout << "This is unsupported in ROS default to XI_MONO8" << std::endl;
    bpp_ = 1;
  }

  else if (image_format == std::string("XI_RAW8"))
  {
    image_data_format = XI_RAW8;
    encoding_ = std::string("mono8");
    bpp_ = 1;
  }

  else if (image_format == std::string("XI_RAW16"))
  {
    image_data_format = XI_RAW16;
    encoding_ = std::string("mono16");
    bpp_ = 2;
  }

  else
  {
    image_data_format = XI_MONO8;
    encoding_ = std::string("mono8");
    bpp_ = 1;
  }

  //xm_.setImageDataFormat(image_format);  
}


  //boost::shared_ptr<dynamic_reconfigure::Server<pointgrey_camera_driver::PointGreyConfig> > srv_; ///< Needed to initialize and keep the dynamic_reconfigure::Server in scope.
  boost::shared_ptr<image_transport::ImageTransport> it_; ///< Needed to initialize and keep the ImageTransport in scope.
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_; ///< Needed to initialize and keep the CameraInfoManager in scope.
  image_transport::CameraPublisher it_pub_; ///< CameraInfoManager ROS publisher
  boost::shared_ptr<diagnostic_updater::DiagnosedPublisher<wfov_camera_msgs::WFOVImage> > pub_; ///< Diagnosed publisher, has to be a pointer because of constructor requirements
  ros::Subscriber sub_; ///< Subscriber for gain and white balance changes.

  sensor_msgs::CameraInfoPtr ci_; ///< Camera Info message.
  std::string frame_id_; ///< Frame id for the camera messages, defaults to 'camera'
  std::string camera_name_;
  boost::shared_ptr<boost::thread> pubThread_; ///< The thread that reads and publishes the images.

  diagnostic_updater::Updater updater_; ///< Handles publishing diagnostics messages.
  double min_freq_;
  double max_freq_;

  boost::mutex connect_mutex_;

  ximea_driver xm_;
  char * cam_buffer_;
  int cam_buffer_size_;

  double gain_;
  uint16_t wb_blue_;
  uint16_t wb_red_;
  
  int bpp_;  // the next 2 paramaeters are used by the ros_image_transport publisher
  std::string encoding_;

  // Parameters for cameraInfo
  size_t binning_x_; ///< Camera Info pixel binning along the image x axis.
  size_t binning_y_; ///< Camera Info pixel binning along the image y axis.
  size_t roi_x_offset_; ///< Camera Info ROI x offset
  size_t roi_y_offset_; ///< Camera Info ROI y offset
  size_t roi_height_; ///< Camera Info ROI height
  size_t roi_width_; ///< Camera Info ROI width
  bool do_rectify_; ///< Whether or not to rectify as if part of an image.  Set to false if whole image, and true if in ROI mode.
  
};

PLUGINLIB_EXPORT_CLASS(ximea_camera::XimeaCameraNodelet, nodelet::Nodelet)  // Needed for Nodelet declaration
}





/*
ximea_ros_driver::ximea_ros_driver(const ros::NodeHandle &nh, std::string cam_name, int serial_no, std::string yaml_url): ximea_driver(serial_no, cam_name)
{
  pnh_ = nh;
  cam_info_manager_ = boost::make_shared<camera_info_manager::CameraInfoManager>(pnh_, cam_name_);
  cam_info_manager_->loadCameraInfo(yaml_url);
  it_ = boost::make_shared<image_transport::ImageTransport>(nh);
  ros_cam_pub_ = it_->advertise(std::string("image_raw"), 1);
  cam_info_pub_ = pnh_.advertise<sensor_msgs::CameraInfo>(std::string("camera_info"), 1);
}

ximea_ros_driver::ximea_ros_driver(const ros::NodeHandle &nh, std::string file_name) : ximea_driver(file_name)
{
  pnh_ = nh;
  cam_info_manager_ = boost::make_shared<camera_info_manager::CameraInfoManager>(pnh_, cam_name_);
  cam_info_manager_->loadCameraInfo(yaml_url_);
  it_ = boost::make_shared<image_transport::ImageTransport>(nh);
  ros_cam_pub_ = it_->advertise(std::string("image_raw"), 1);
  cam_info_pub_ = pnh_.advertise<sensor_msgs::CameraInfo>(std::string("camera_info"), 1);
}

void ximea_ros_driver::common_initialize(const ros::NodeHandle &nh)
{
  pnh_ = nh;
  cam_info_manager_ = boost::make_shared<camera_info_manager::CameraInfoManager>(pnh_, cam_name_);
  cam_info_manager_->loadCameraInfo("");  // TODO: yaml_url
  it_ = boost::make_shared<image_transport::ImageTransport>(nh);
  ros_cam_pub_ = it_->advertise(cam_name_ + std::string("/image_raw"), 1);
  cam_info_pub_ = pnh_.advertise<sensor_msgs::CameraInfo>(cam_name_ + std::string("/camera_info"), 1);
}

void ximea_ros_driver::publishImage(const ros::Time & now)
{
  cam_buffer_ = reinterpret_cast<char *>(image_.bp);
  cam_buffer_size_ = image_.width * image_.height * bpp_;
  ros_image_.data.resize(cam_buffer_size_);
  ros_image_.encoding = encoding_;
  ros_image_.width = image_.width;
  ros_image_.height = image_.height;
  ros_image_.step = image_.width * bpp_;

  copy(reinterpret_cast<char *>(cam_buffer_),
       (reinterpret_cast<char *>(cam_buffer_)) + cam_buffer_size_,
       ros_image_.data.begin());

  ros_cam_pub_.publish(ros_image_);
}

void ximea_ros_driver::publishCamInfo(const ros::Time &now)
{
  ros_image_.header.stamp = now;
  cam_info_ = cam_info_manager_->getCameraInfo();
  cam_info_.header.frame_id = frame_id_;
  cam_info_pub_.publish(cam_info_);
}

void ximea_ros_driver::publishImageAndCamInfo()
{
  ros::Time now = ros::Time::now();
  publishImage(now);
  publishCamInfo(now);
}

void ximea_ros_driver::setImageDataFormat(std::string image_format)
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
    encoding_ = std::string("mono16");
    bpp_ = 2;
  }

  else if (image_format == std::string("XI_RGB24"))
  {
    image_data_format = XI_RGB24;
    encoding_ = std::string("bgr8");
    bpp_ = 3;
  }

  else if (image_format == std::string("XI_RGB32"))
  {
    image_data_format = XI_RGB32;
    encoding_ = std::string("bgr16");
    bpp_ = 3;
  }

  else if (image_format == std::string("XI_RGB_PLANAR"))
  {
    image_data_format = XI_MONO8;
    std::cout << "This is unsupported in ROS default to XI_MONO8" << std::endl;
    bpp_ = 1;
  }

  else if (image_format == std::string("XI_RAW8"))
  {
    image_data_format = XI_RAW8;
    encoding_ = std::string("mono8");
    bpp_ = 1;
  }

  else if (image_format == std::string("XI_RAW16"))
  {
    image_data_format = XI_RAW16;
    encoding_ = std::string("mono16");
    bpp_ = 2;
  }

  else
  {
    image_data_format = XI_MONO8;
    encoding_ = std::string("mono8");
    bpp_ = 1;
  }

  stat = xiSetParamInt(xiH_, XI_PRM_IMAGE_DATA_FORMAT, image_data_format);
  errorHandling(stat, "image_format");    // if we cannot set the format then there is something wrong we should probably quit then
  image_data_format_ = image_data_format;
}
*/