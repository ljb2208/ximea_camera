#ifndef XIMEA_CAMERA_STEREO_NODELET_H
#define XIMEA_CAMERA_STEREO_NODELET_H

#include <ximea_camera/ximea_driver.h>
#include <ximea_camera/ximea_ros_driver.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/server.h>
#include <ximea_camera/XimeaSettingsConfig.h>
#include <autorally_core/SerialInterfaceThreaded.h>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/tokenizer.hpp>

#include <stdio.h>

namespace ximea_camera
{

class StereoCameraNodelet: public nodelet::Nodelet
{
    public:
        StereoCameraNodelet();
        ~StereoCameraNodelet();
        virtual void onInit();
    protected:

    private:
        int getSerial(ros::NodeHandle pnh, std::string paramName);
        void acquireImages();
        void publishImages();
        void startAcquisition(ros::NodeHandle pnh);
        void stopAcquisition();
        void publishData();
        void triggerCameras();
        void initializeTrigger(ros::NodeHandle pnh);
        void stopTrigger();

        void reconfigureCallback(ximea_camera::XimeaSettingsConfig &config, uint32_t level);

        std::string frame_id;
        std::string serial_port;

        ximea_ros_driver* leftCamDriver;
        ximea_ros_driver* rightCamDriver;

        boost::thread* lcThread;
        boost::thread* rcThread;

        boost::shared_ptr<boost::thread> pubThread_; ///< The thread that reads and publishes the images.

        SerialInterfaceThreaded port_; ///<Serial port for arduino data

        dynamic_reconfigure::Server<ximea_camera::XimeaSettingsConfig> dr_server;        
        dynamic_reconfigure::Server<ximea_camera::XimeaSettingsConfig>::CallbackType dr_cb;
};

PLUGINLIB_EXPORT_CLASS(ximea_camera::StereoCameraNodelet, nodelet::Nodelet)  // Needed for Nodelet declaration
}
#endif // XIMEA_CAMERA_STEREO_NODELET_H
