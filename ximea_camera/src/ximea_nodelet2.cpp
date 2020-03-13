#include <ximea_camera/ximea_nodelet2.h>


namespace ximea_camera
{
    XimeaCameraNodelet2::XimeaCameraNodelet2()
    {
        driver = NULL;
    }

    XimeaCameraNodelet2::~XimeaCameraNodelet2()
    {
        if(pubThread_)
        {
            pubThread_->interrupt();
            pubThread_->join();
        }

        if (driver != NULL)
        {
            try
            {                
                driver->closeDevice();
            }
            catch(std::runtime_error& e)
            {

            }
            delete driver;        
        }

        try
        {
            // stopAcquistion();
        }
        catch(std::runtime_error& e)
        {
            NODELET_ERROR("%s", e.what());
        }
        
    }

    void XimeaCameraNodelet2::reconfigureCallback(ximea_camera::XimeaSettingsConfig &config, uint32_t level)
    {
        NODELET_INFO("Reconfigure");

        if (driver != NULL && driver->hasValidHandle())
        {
            if (config.auto_exposure)
                driver->setAutoExposure(1);
            else
            {
                driver->setAutoExposure(0);
                driver->setExposure(config.exposure);
            }

            driver->setTrigger(config.trigger);
        }        
    }

    void XimeaCameraNodelet2::onInit()
    {
        driver = NULL;                

        // Get nodeHandles
        ros::NodeHandle &nh = getMTNodeHandle();
        ros::NodeHandle &pnh = getMTPrivateNodeHandle();

        std::string camera_info_url;
        std::string image_format;
        int serial = 0;        
        std::string camera_name;        

        // Get the location of our camera config yaml        
        pnh.param<std::string>("camera_info_url", camera_info_url, "");
        // Get the desired frame_id, set to 'camera' if not found
        pnh.param<std::string>("frame_id", frame_id, "camera");
        pnh.param<int>("camera_serial", serial, 0);        
        pnh.param<std::string>("camera_name", camera_name, "camera");        
        pnh.param<std::string>("yaml_url", yaml_url, "");        
        pnh.param<std::string>("image_data_format", image_format, "XI_RGB24");

        setEncodingFromFormat(image_format);

        driver = new ximea_driver(serial, camera_name, yaml_url, frame_id);        

        std::stringstream cinfo_name;
        cinfo_name << camera_name;
        cinfo_.reset(new camera_info_manager::CameraInfoManager(nh, cinfo_name.str(), camera_info_url));

        // Publish topics using ImageTransport through camera_info_manager (gives cool things like compression)
        it_.reset(new image_transport::ImageTransport(nh));
        image_transport::SubscriberStatusCallback cb = boost::bind(&XimeaCameraNodelet2::connectCb, this);
        it_pub_ = it_->advertiseCamera("image_raw", 5, cb, cb);

        ros::SubscriberStatusCallback cb2 = boost::bind(&XimeaCameraNodelet2::connectCb, this);

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

        pub_.reset(new diagnostic_updater::DiagnosedPublisher<wfov_camera_msgs::WFOVImage>(nh.advertise<wfov_camera_msgs::WFOVImage>("image", 5, cb2, cb2),
               updater_,
               diagnostic_updater::FrequencyStatusParam(&min_freq_, &max_freq_, freq_tolerance, window_size),
               diagnostic_updater::TimeStampStatusParam(min_acceptable, max_acceptable)));

        NODELET_INFO("Nodelet Init: %i %s %s %s", serial, camera_name.c_str(), yaml_url.c_str(), frame_id.c_str());                

        // setup dyn reconfigure
        dr_cb = boost::bind(&XimeaCameraNodelet2::reconfigureCallback, this, _1, _2);
        dr_server.setCallback(dr_cb);

    }

    void XimeaCameraNodelet2::connectCb()
    {
        NODELET_DEBUG("Connect callback!");
        boost::mutex::scoped_lock scopedLock(connect_mutex_); // Grab the mutex.  Wait until we're done initializing before letting this function through.
        // Check if we should disconnect (there are 0 subscribers to our data)
        if(it_pub_.getNumSubscribers() == 0 && pub_->getPublisher().getNumSubscribers() == 0)
        {
            if (pubThread_)
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
                    driver->stopAcquisition();
                }
                catch(std::runtime_error& e)
                {
                    NODELET_ERROR("%s", e.what());
                }

                try
                {
                    NODELET_DEBUG("Disconnecting from camera.");
                    driver->closeDevice();                    
                }
                catch(std::runtime_error& e)
                {
                    NODELET_ERROR("%s", e.what());
                }
            }
        }
        else if(!pubThread_)     // We need to connect
        {
            // Start the thread to loop through and publish messages
            pubThread_.reset(new boost::thread(boost::bind(&ximea_camera::XimeaCameraNodelet2::devicePoll, this)));
        }
        else
        {
            NODELET_DEBUG("Do nothing in callback.");
        }
    }

    void XimeaCameraNodelet2::devicePoll()
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
                    pg_.stop();
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
                    driver->closeDevice();
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
                    driver->openDevice();                    
                    NODELET_INFO("Connected to camera.");

                    // Set last configuration, forcing the reconfigure level to stop
                    // pg_.setNewConfiguration(config_, PointGreyCamera::LEVEL_RECONFIGURE_STOP);

                    // Set the timeout for grabbing images.
                    try
                    {
                        double timeout;
                        getMTPrivateNodeHandle().param("timeout", timeout, 1.0);

                        NODELET_DEBUG("Setting timeout to: %f.", timeout);

                        // pg_.setTimeout(timeout);
                    }
                    catch(std::runtime_error& e)
                    {
                        NODELET_ERROR("%s", e.what());
                    }

                    // Subscribe to gain and white balance changes
                    {
                        boost::mutex::scoped_lock scopedLock(connect_mutex_);
                        sub_ = getMTNodeHandle().subscribe("image_exposure_sequence", 10, &ximea_camera::XimeaCameraNodelet2::gainWBCallback, this);
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
                    driver->startAcquisition();                    
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
                    // Get the image from the camera library
                    NODELET_DEBUG("Starting a new grab from camera.");
                    
                    if (driver->acquireImage() == 0)
                    {
                        wfov_camera_msgs::WFOVImagePtr wfov_image(new wfov_camera_msgs::WFOVImage);

                        XI_IMG* img = driver->getAcquiredImage();
                        wfov_image->header.frame_id = frame_id;
                        ros::Time time = ros::Time::now();
                        wfov_image->header.stamp = time;
                        wfov_image->image.header.stamp = time;
                        
                        wfov_image->image.data.resize(img->width * img->height * bpp);
                        wfov_image->image.encoding = encoding;
                        wfov_image->image.width = img->width;
                        wfov_image->image.height = img->height;
                        wfov_image->image.step = img->width * bpp;                  

                        // NODELET_INFO("image details: %i %i %i %i %s %i", wfov_image->image.width, wfov_image->image.height, wfov_image->image.step, bpp, encoding.c_str(), img->bp_size);      

                        fillImage(wfov_image->image, encoding, img->height, img->width, img->width * bpp, img->bp);
                        
                        wfov_image->gain = img->gain_db;
                        wfov_image->exposure_time = img->exposure_time_us;
                        wfov_image->shutter = img->exposure_time_us;
                        wfov_image->white_balance_blue = img->wb_blue;
                        wfov_image->white_balance_red = img->wb_red;
                        wfov_image->white_balance_green = img->wb_green;
                        wfov_image->temperature = driver->getCameraTemperature();

                        // // Set the CameraInfo message
                        ci_.reset(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
                        ci_->header.stamp = wfov_image->image.header.stamp;
                        ci_->header.frame_id = wfov_image->header.frame_id;

                        // The height, width, distortion model, and parameters are all filled in by camera info manager.
                        // ci_->binning_x = binning_x_;
                        // ci_->binning_y = binning_y_;
                        // ci_->roi.x_offset = roi_x_offset_;
                        // ci_->roi.y_offset = roi_y_offset_;
                        // ci_->roi.height = roi_height_;
                        // ci_->roi.width = roi_width_;
                        // ci_->roi.do_rectify = do_rectify_;

                        wfov_image->info = *ci_;

                        // Publish the full message
                        pub_->publish(wfov_image);

                        // Publish the message using standard image transport
                        if(it_pub_.getNumSubscribers() > 0)
                        {
                            sensor_msgs::ImagePtr image(new sensor_msgs::Image(wfov_image->image));
                            it_pub_.publish(image, ci_);
                        }
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

    void XimeaCameraNodelet2::gainWBCallback(const image_exposure_msgs::ExposureSequence &msg)
    {
        try
        {
            NODELET_DEBUG("Gain callback:  Setting gain to %f and white balances to %u, %u", msg.gain, msg.white_balance_blue, msg.white_balance_red);
            gain = msg.gain;
            wb_blue = static_cast<float>(msg.white_balance_blue);
            wb_red = static_cast<float>(msg.white_balance_red);
            // wb_green = msg.white_balance_green;

            if (driver != NULL && driver->hasValidHandle())
            {
                driver->setGain(gain);
                driver->setWhiteBalance(wb_red, 0.0, wb_blue);
            }            
        }
        catch(std::runtime_error& e)
        {
            NODELET_ERROR("gainWBCallback failed with error: %s", e.what());
        }
    }    

    void XimeaCameraNodelet2::setEncodingFromFormat(std::string image_format)
    {
        if (image_format == std::string("XI_MONO16"))
        {
            // image_data_format = XI_MONO16;
            encoding = std::string("mono16");
            bpp = 2;
        }

        else if (image_format == std::string("XI_RGB24"))
        {
            // image_data_format = XI_RGB24;
            encoding = std::string("bgr8");
            bpp = 3;
        }

        else if (image_format == std::string("XI_RGB32"))
        {
            // image_data_format = XI_RGB32;
            encoding = std::string("bgr16");
            bpp = 3;
        }

        else if (image_format == std::string("XI_RGB_PLANAR"))
        {
            // image_data_format = XI_MONO8;
            std::cout << "This is unsupported in ROS default to XI_MONO8" << std::endl;
            bpp = 1;
        }

        else if (image_format == std::string("XI_RAW8"))
        {
            // image_data_format = XI_RAW8;
            encoding = std::string("mono8");
            bpp = 1;
        }

        else if (image_format == std::string("XI_RAW16"))
        {
            // image_data_format = XI_RAW16;
            encoding = std::string("mono16");
            bpp = 2;
        }

        else
        {
            // image_data_format = XI_MONO8;
            encoding = std::string("mono8");
            bpp = 1;
        }
    }
}