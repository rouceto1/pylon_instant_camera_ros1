#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <thread>
#include <chrono>
#include<sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_calibration_parsers/parse.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/image_encodings.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/DeviceInfo.h>
#pragma GCC diagnostic pop

namespace pylon_instant_camera_ros1 {

// this BufferFactory pre-allocates sensor_msgs::Image memory
// so the Basler Pylon SDK can write into the data vectors directly
class ImageBufferFactory : public Pylon::IBufferFactory {
public:
    ~ImageBufferFactory() {
    }
    virtual void AllocateBuffer(size_t bufferSize, void **pCreatedBuffer, intptr_t &bufferContext) {
        sensor_msgs::Image * image_message_ptr = new sensor_msgs::Image();
        image_message_ptr->data.resize(bufferSize);
        *pCreatedBuffer = image_message_ptr->data.data();
        bufferContext = reinterpret_cast<intptr_t>(image_message_ptr);
    }
    virtual void FreeBuffer(void *, intptr_t bufferContext) {
        delete ReinterpretBufferContext(bufferContext);
    }
    virtual void DestroyBufferFactory() {
    }
    static sensor_msgs::Image * ReinterpretBufferContext(intptr_t bufferContext) {
        return reinterpret_cast<sensor_msgs::Image *>(bufferContext);
    }
};

class PylonCamera {
private:
    // This smart pointer will receive the grab result data.
    Pylon::CGrabResultPtr ptrGrabResult;

public:
    Pylon::CBaslerUniversalInstantCamera * camera;
    int grab_timeout_ms = 1000; //default
    PylonCamera(const std::string & full_name, const std::string & user_defined_name, const std::string & ip_address, const std::string & serial_number) {
        Pylon::PylonInitialize();
        // Create an instant camera object with the camera device found first or found by specified parameters.
        if (!full_name.empty() && !user_defined_name.empty() && !ip_address.empty() && serial_number == "") {
            camera = new Pylon::CBaslerUniversalInstantCamera(Pylon::CTlFactory::GetInstance().CreateFirstDevice());
        } else {
            Pylon::CDeviceInfo di;
            if (!full_name.empty()) { 
                di.SetFullName(full_name.c_str()); 
            }
            if (!user_defined_name.empty()) { 
                di.SetUserDefinedName(user_defined_name.c_str()); 
            }
            if (!ip_address.empty()) { 
                di.SetIpAddress(ip_address.c_str()); 
            }
            if (serial_number != "") {
                di.SetSerialNumber(serial_number.c_str());
            }
            camera = new Pylon::CBaslerUniversalInstantCamera(Pylon::CTlFactory::GetInstance().CreateDevice(di));
        }
        camera->Open();
        // provide Pylon with sensor_msgs::msg::Image buffers
        camera->SetBufferFactory(new ImageBufferFactory());
        // The parameter MaxNumBuffer can be used to control the count of buffers
        // allocated for grabbing. The default value of this parameter is 10.
        camera->MaxNumBuffer = 5;
        //INodeMap& nodemap = camera.GetNodeMap();
        //CImageDecompressor decompressor( nodemap );
    }
    ~PylonCamera() {
        delete camera;
        Pylon::PylonTerminate(); 
    }
    Pylon::CGrabResultPtr & grab_frame() {
        if (camera->IsGrabbing()) {
            // Wait for an image and then retrieve it.
            camera->RetrieveResult(grab_timeout_ms, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded()) {
                /*
                 CompressionInfo_t info;
                if (decompressor.GetCompressionInfo( info, ptrGrabResult ))
                {
                    // Print content of CompressionInfo_t.
                    printCompressionInfo( info );
                    if (info.hasCompressedImage)
                    {
                        if (info.compressionStatus == CompressionStatus_Ok)
                        {
                            CPylonImage targetImage;
                            // Show compression ratio.
                            cout << endl << "Transferred payload \t:" << ptrGrabResult->GetPayloadSize() << endl;
                            cout << "Compression ratio \t:" << (static_cast<float>(ptrGrabResult->GetPayloadSize()) / static_cast<float>(info.decompressedPayloadSize) * 100.0f) << "%" << endl;

                            // Decompress the image.
                            decompressor.DecompressImage( targetImage, ptrGrabResult );
                             
                        }
                        else
                        {
                            cout << "There was an error while the camera was compressing the image." << endl;
                        }
                    }
                    */

                return ptrGrabResult;
            } else {
                throw std::runtime_error(std::string(ptrGrabResult->GetErrorDescription()));
            }
        } else {
            throw std::runtime_error("Camera is not grabbing.");
        }
    }
};


//const std::map<Pylon::EPixelType, const char *> Pylon2ROS {
 //                   {Pylon::EPixelType::PixelType_BayerRG8, sensor_msgs::image_encodings::BAYER_RGGB8},
  //                  {Pylon::EPixelType::PixelType_RGB8packed, sensor_msgs::image_encodings::RGB8},
   //                 {Pylon::EPixelType::PixelType_Mono8, sensor_msgs::image_encodings::MONO8}
//}

const std::string Pylon2ROS(Pylon::EPixelType type){
    if (type == Pylon::EPixelType::PixelType_BayerRG8)  return sensor_msgs::image_encodings::BAYER_RGGB8;
    if (type == Pylon::EPixelType::PixelType_RGB8packed) return sensor_msgs::image_encodings::RGB8;
    if (type == Pylon::EPixelType::PixelType_Mono8) return sensor_msgs::image_encodings::MONO8;
    return "unknown";
}


sensor_msgs::Image * pylon_result_to_image_message(Pylon::CGrabResultPtr & ptrGrabResult) {


    sensor_msgs::Image * img = pylon_instant_camera_ros1::ImageBufferFactory::ReinterpretBufferContext(ptrGrabResult->GetBufferContext());
    img->width = ptrGrabResult->GetWidth();
    img->height = ptrGrabResult->GetHeight();
    size_t stride;
    ptrGrabResult->GetStride(stride);
    img->step = stride;
    const Pylon::EPixelType pixel_type = ptrGrabResult->GetPixelType();
    try {
        img->encoding = Pylon2ROS(pixel_type);
    } catch (std::out_of_range &) {
        const std::string pixel_type_str(Pylon::CPixelTypeMapper::GetNameByPixelType(pixel_type));
        //RCLCPP_ERROR(this->get_logger(), "Captured image has Pylon pixel type %s. "\
        //            "This driver does not know the corresponding ROS image encoding. "\
        //           "Please add your preferred pixel type to the mapping code and file a pull request.",
        //           pixel_type_str.c_str());
        // I would really like to fail critically on unrecoverable errors, 
        // but throwing exeptions in threads in a composable node
        // messes up the node container
        //throw std::runtime_error("Unknown Pylon pixel type.");
    }
    //RCLCPP_DEBUG(this->get_logger(), "Got image %ix%i, stride %zu, size %zu bytes, publishing.", img->width, img->height, stride, ptrGrabResult->GetBufferSize());
    return img; // NOTE: this is NOT a bottleneck. copy-elision is strong in this one.
}


class PylonCameraNode : public nodelet::Nodelet
    {
    private:
        // the camera
        std::unique_ptr<PylonCamera> camera;
        std::string frame_id;
        std::string camera_info_path;
        std::string camera_settings_path;
        std::string full_name;
        std::string user_defined_name;
        std::string ip_address;
        std::string  serial_number; 

    public:
    virtual void onInit(){
        NODELET_INFO("Starting PylonCameraNode");
        ros::NodeHandle& nh = getNodeHandle();
        ros::NodeHandle& private_nh = getPrivateNodeHandle();   
        image_transport::ImageTransport it(private_nh);

        

        // background thread
        std::unique_ptr<std::thread> grabbing_thread;
        //------------------------------
        private_nh.param<std::string>("frame_id", this->frame_id, "");
        private_nh.param<std::string>("camera_info_yaml", this->camera_info_path, "");
        private_nh.param<std::string>("full_name", this->full_name, "");
        private_nh.param<std::string>("user_defined_name", this->user_defined_name, "");
        private_nh.param<std::string>("ip_address", this->ip_address, "");
        private_nh.param<std::string>("serial_number", this->serial_number, "");

        private_nh.param<std::string>("camera_settings_path", this->camera_settings_path, "");
        try {
            camera = std::make_unique<PylonCamera>(full_name, user_defined_name, ip_address, serial_number);
        } catch (const GenICam::RuntimeException & e) {
            NODELET_ERROR( "Exception in PylonCamera constructor: %s", e.what());
            throw e;
        }

        // camera parameters
        //nh.param<std::string>("camera_settings_path", this->camera_settings_path, "cm.conf");
        if (!this->camera_settings_path.empty()) {
            try {
                Pylon::CFeaturePersistence::Load(this->camera_settings_path.c_str(), &camera->camera->GetNodeMap(), true);
                NODELET_WARN("Loaded camra settings file %s",this->camera_settings_path.c_str());         
            } catch (GenICam::RuntimeException &e) {
                NODELET_WARN(
                    "Loading camera settings from PFS file %s failed: %s",
                    this->camera_settings_path.c_str(), e.what()
                );
            }
        }else{
            NODELET_WARN("Operation from internal settings");    
        }
        private_nh.param<int>("grab_timeout", camera->grab_timeout_ms, 30);
        //camera->grab_timeout_ms = 30;
        //NODELET_WARN("grab_timeout is %d ms.", camera->grab_timeout_ms);
        ////TODO read camera info
        sensor_msgs::CameraInfo camera_info;
        if (!this->camera_info_path.empty()) {
                bool info_loaded = camera_calibration_parsers::readCalibration(
                    this->camera_info_path, this->serial_number, camera_info);
                if (info_loaded == true){
                    NODELET_WARN("Loaded camra info file %s",this->camera_info_path.c_str());         
                }else{
                    NODELET_WARN("Loading camera info file %s failed",
                        this->camera_info_path.c_str());
                }
            
        }else{
            NODELET_WARN("Operation without info");    
        }


        std::string publish_topic = "image";
        Pylon::CPixelTypeMapper pixel_type_mapper(&camera->camera->PixelFormat);
        const Pylon::EPixelType pixel_type = pixel_type_mapper.GetPylonPixelTypeFromNodeValue(camera->camera->PixelFormat.GetIntValue());
        if (Pylon::IsBayer(pixel_type)) {
            // if PixelType is bayer, topic should be image_raw, according to
            // https://github.com/ros-perception/image_pipeline/blob/ros2/image_proc/src/debayer.cpp
            publish_topic = "image_raw";
        }
        image_transport::CameraPublisher image_publisher = it.advertiseCamera(publish_topic.c_str(), 1);

        // Camera fully set-up. Now start grabbing.
        camera->camera->StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);

        //grabbing_thread = std::make_unique<std::thread>([&](){
            int timed_out = 0;
            while(ros::ok()) {
                try {
                    sensor_msgs::Image * img_msg = pylon_result_to_image_message(camera->grab_frame());
                    img_msg->header.stamp = ros::Time::now();
                    img_msg->header.frame_id = this->frame_id;
                    camera_info.header.stamp = img_msg->header.stamp;
                    camera_info.header.frame_id = img_msg->header.frame_id;
                    image_publisher.publish(*img_msg,camera_info);
                    if (timed_out > 0){
                        NODELET_WARN( "Timeout while grabbing. Dropped %i frames", timed_out);
                        timed_out = 0;
                        }
                } catch (const std::runtime_error & e) { // TODO: use more specific exception class
                    NODELET_WARN(e.what());
                } catch (const GenICam::TimeoutException & e) {
                    NODELET_INFO( "Timeout while grabbing.");
                    timed_out++;
                }
                ros::spinOnce();
            }
        //});
    }
};
PLUGINLIB_EXPORT_CLASS(pylon_instant_camera_ros1::PylonCameraNode, nodelet::Nodelet)
} // end namespace pylon_instant_camera
