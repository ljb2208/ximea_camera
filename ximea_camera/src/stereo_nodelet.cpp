#include <ximea_camera/stereo_nodelet.h>


namespace ximea_camera
{
    StereoCameraNodelet::StereoCameraNodelet()
    {

    }

    StereoCameraNodelet::~StereoCameraNodelet()
    {
        if(pubThread_)
        {
            pubThread_->interrupt();
            pubThread_->join();
        }

        stopAcquisition();
        
    }

    void StereoCameraNodelet::reconfigureCallback(ximea_camera::XimeaSettingsConfig &config, uint32_t level)
    {
        if (leftCamDriver)
        {
            leftCamDriver->setTrigger(config.trigger);
            leftCamDriver->setAutoExposure(config.auto_exposure);

            if (!config.auto_exposure)
                leftCamDriver->setExposure(config.exposure);
        }

        if (rightCamDriver)
        {
            rightCamDriver->setTrigger(config.trigger);
            rightCamDriver->setAutoExposure(config.auto_exposure);

            if (!config.auto_exposure)
                rightCamDriver->setExposure(config.exposure);
        }        
    }

    void StereoCameraNodelet::onInit()
    {
        leftCamDriver = NULL;
        rightCamDriver = NULL;

        // Get nodeHandles
        ros::NodeHandle &nh = getMTNodeHandle();
        ros::NodeHandle &pnh = getMTPrivateNodeHandle();

        // setup dyn reconfigure
        dr_cb = boost::bind(&StereoCameraNodelet::reconfigureCallback, this, _1, _2);
        dr_server.setCallback(dr_cb);

        initializeTrigger(pnh);
        startAcquisition(pnh);        

        pubThread_.reset(new boost::thread(boost::bind(&StereoCameraNodelet::publishData, this)));

    }

    void StereoCameraNodelet::publishData()
    {        
        while(!boost::this_thread::interruption_requested())   // Block until we need to stop this thread.
        {            
            acquireImages();
            publishImages();
        }
    }

    void StereoCameraNodelet::initializeTrigger(ros::NodeHandle pnh)
    {
        pnh.param<std::string>("port", serial_port, "");
        NODELET_INFO("Serial port: %s", serial_port.c_str());
        port_.init(pnh, getName(), "", "ximea_camera", serial_port, true);
    }

    void StereoCameraNodelet::stopTrigger()
    {        
    }

    void StereoCameraNodelet::triggerCameras()
    {
        port_.writePort("#trg:");
    }

    void StereoCameraNodelet::startAcquisition(ros::NodeHandle pnh)
    {
        // Get a serial number through ros
        int left_serial = getSerial(pnh, "left_serial");
        int right_serial = getSerial(pnh, "right_serial");        

        NODELET_INFO("Using camera serial Left: %d    Right: %d", left_serial, right_serial);
        // Get the location of our camera config yaml
        std::string left_camera_info_url;
        std::string right_camera_info_url;
        std::string left_configuration_url;
        std::string right_configuration_url;    
        std::string image_format;
        std::string left_camera_name;
        std::string right_camera_name;

        pnh.param<std::string>("left_yaml_url", left_configuration_url, "");
        pnh.param<std::string>("left_camera_info_url", left_camera_info_url, "");

        pnh.param<std::string>("right_yaml_url", right_configuration_url, "");
        pnh.param<std::string>("right_camera_info_url", right_camera_info_url, "");

        // Get the desired frame_id, set to 'camera' if not found
        pnh.param<std::string>("frame_id", frame_id, "stereo_camera");

        // Get the camera names, set to default if not found
        pnh.param<std::string>("left_camera_name", left_camera_name, "leftCamera");
        pnh.param<std::string>("right_camera_name", right_camera_name, "rightCamera");
        pnh.param<std::string>("image_data_format", image_format, "");

        NODELET_INFO("Image format %s.", image_format.c_str());

        leftCamDriver = new ximea_ros_driver(pnh, left_camera_name, left_serial, left_camera_info_url, left_configuration_url, frame_id);
        rightCamDriver = new ximea_ros_driver(pnh, right_camera_name, right_serial, right_camera_info_url, right_configuration_url, frame_id);

        leftCamDriver->openDevice();
        rightCamDriver->openDevice();
        leftCamDriver->startAcquisition();
        rightCamDriver->startAcquisition();
    }

    void StereoCameraNodelet::stopAcquisition()
    {
        leftCamDriver->stopAcquisition();
        rightCamDriver->stopAcquisition();
        leftCamDriver->closeDevice();
        rightCamDriver->closeDevice();

        delete leftCamDriver;
        delete rightCamDriver;
    }

    void StereoCameraNodelet::acquireImages()
    {
        lcThread = new boost::thread(&ximea_driver::acquireImage, leftCamDriver);
        rcThread = new boost::thread(&ximea_driver::acquireImage, rightCamDriver);

        triggerCameras();

        lcThread->join();
        rcThread->join();

        delete lcThread;
        delete rcThread;
    }

    void StereoCameraNodelet::publishImages()
    {
        ros::Time now = ros::Time::now();

        lcThread = new boost::thread(&ximea_ros_driver::publishImageAndCamInfoWithTime, leftCamDriver, now);
        rcThread = new boost::thread(&ximea_ros_driver::publishImageAndCamInfoWithTime, rightCamDriver, now);

        lcThread->join();
        rcThread->join();

        delete lcThread;
        delete rcThread;
    }

    int StereoCameraNodelet::getSerial(ros::NodeHandle pnh, std::string paramName)
    {
        //get serial number and return it
        int serial = 0;
        
        XmlRpc::XmlRpcValue serial_xmlrpc;
        pnh.getParam(paramName, serial_xmlrpc);
        if (serial_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeInt)
        {
        pnh.param<int>(paramName, serial, 0);
        }
        else if (serial_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeString)
        {
        std::string serial_str;
        pnh.param<std::string>(paramName, serial_str, "0");
        std::istringstream(serial_str) >> serial;
        }
        else
        {
        NODELET_DEBUG("Serial XMLRPC type.");
        serial = 0;
        }

        return serial;
    }
}