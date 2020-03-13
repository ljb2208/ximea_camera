#ifndef XIMEA_CAMERA_NODELET2_H
#define XIMEA_CAMERA_NODELET2_H

#include <ximea_camera/ximea_driver.h>
#include <ximea_camera/ximea_ros_driver.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/server.h>
#include <ximea_camera/XimeaSettingsConfig.h>
#include <autorally_core/SerialInterfaceThreaded.h>
#include <diagnostic_updater/diagnostic_updater.h> // Headers for publishing diagnostic messages.
#include <diagnostic_updater/publisher.h>
#include <wfov_camera_msgs/WFOVImage.h>
#include <image_exposure_msgs/ExposureSequence.h> // Message type for configuring gain and white balance.
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/tokenizer.hpp>

#include <stdio.h>

namespace ximea_camera
{

class XimeaCameraNodelet2: public nodelet::Nodelet
{
    public:
        XimeaCameraNodelet2();
        ~XimeaCameraNodelet2();
        virtual void onInit();
    protected:

    private:
        void devicePoll();
        void reconfigureCallback(ximea_camera::XimeaSettingsConfig &config, uint32_t level);
        void gainWBCallback(const image_exposure_msgs::ExposureSequence &msg);
        void setEncodingFromFormat(std::string image_format);

        std::string frame_id;        
        std::string yaml_url;        

        int bpp;
        std::string encoding;

        sensor_msgs::CameraInfoPtr ci_; ///< Camera Info message.
        diagnostic_updater::Updater updater_; ///< Handles publishing diagnostics messages.
        double min_freq_;
        double max_freq_;

        ximea_driver* driver;
        ros::Subscriber sub_; ///< Subscriber for gain and white balance changes.

        float wb_blue;
        float wb_red;
        float wb_green;
        double gain;        

        // boost::thread* cThread;        

        boost::shared_ptr<boost::thread> pubThread_; ///< The thread that reads and publishes the images.
        boost::mutex connect_mutex_;        

        dynamic_reconfigure::Server<ximea_camera::XimeaSettingsConfig> dr_server;        
        dynamic_reconfigure::Server<ximea_camera::XimeaSettingsConfig>::CallbackType dr_cb;

        boost::shared_ptr<image_transport::ImageTransport> it_; ///< Needed to initialize and keep the ImageTransport in scope.
        boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_; ///< Needed to initialize and keep the CameraInfoManager in scope.
        image_transport::CameraPublisher it_pub_; ///< CameraInfoManager ROS publisher
        boost::shared_ptr<diagnostic_updater::DiagnosedPublisher<wfov_camera_msgs::WFOVImage> > pub_;
        void connectCb();
};

PLUGINLIB_EXPORT_CLASS(ximea_camera::XimeaCameraNodelet2, nodelet::Nodelet)  // Needed for Nodelet declaration
}
#endif // XIMEA_CAMERA_NODELET2_H
