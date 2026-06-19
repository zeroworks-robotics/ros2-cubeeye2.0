#ifndef CAMERA_MODULE_H_
#define CAMERA_MODULE_H_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "CubeEyeSink.h"
#include "CubeEyeCamera.h"
#include "CubeEyeBasicFrame.h"
#include "CubeEyePointCloudFrame.h"
#include "CubeEyeIntensityPointCloudFrame.h"

#include "ModelParameter.h"

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/multi_echo_laser_scan.hpp>

class ReceivedIntensityPCLFrameSink;

class CameraModule : public std::enable_shared_from_this<CameraModule>
{
public:
    CameraModule();

    // camera operations
    std::vector<std::string> scan();
    meere::sensor::result connect(int32_t index);
    meere::sensor::result connect(std::string serialNumber);
    meere::sensor::result run(int32_t type);
    meere::sensor::result stop();
    meere::sensor::result disconnect();
    void shutdown();

    // node connections
    void connectTo(rclcpp::Node* node);
    void disconnectFrom(rclcpp::Node* node);

    int32_t getLastState() { return mLastState.load(std::memory_order_acquire); }
    int32_t getLastError() { return mLastError.load(std::memory_order_acquire); };

    // ---- depth -> MultiEchoLaserScan conversion + filter parameters ----
    // Compile-time defaults (overridable via ROS 2 params declared once in connectTo()).
    bool draw = false;
    int contour_filter_size = 3500;
    int median_filter_col = 20;
    int median_filter_row = 20;
    int padding = 10;
    int right_padding = 0;
    int left_padding = 0;
    int num_of_thread = 4;
    std::vector<float> sin_lookup;
    float ground_height_max = 0.1;
    float ground_height_min = -0.1;
    float Robot_height = 1.5;

    float scan_angle_increment = 0.5f / 180.0f * (float)M_PI;
    float scan_angle_min = -(float)M_PI / 2.0;
    float scan_angle_max = (float)M_PI / 2.0;
    float scan_range_min = 0.0f;
    float scan_range_max = 5.0f;

    float gradient_angle = 1.0f;  // declared for parity; currently unused (slope hardcoded at 1 deg)

    Eigen::Matrix3d R;
    Eigen::Vector3d T;
    bool get_tf = false;

    std::mutex push_mtx, data_mtx;
    sensor_msgs::msg::PointCloud2 main_data;

    bool ROI_initialized = false;
    cv::Mat RGBD_data, RGBD_mask;
    std::atomic<bool> thread_flag{true};
    void init_thread();
    std::shared_ptr<std::thread> d2g_thread;

private:
    void publishFrames(const meere::sensor::sptr_frame_list& frames);
    void createPublishers(rclcpp::Node* node);

    void setLastState(int32_t state) { mLastState.store(state, std::memory_order_release); }
    void setLastError(int32_t error) { mLastError.store(error, std::memory_order_release); }

    sensor_msgs::msg::Image::SharedPtr createImageMessage(meere::sensor::FrameType type, int32_t width, int32_t height);
    sensor_msgs::msg::PointCloud2::SharedPtr createPointCloudMessage(meere::sensor::FrameType type, int32_t width, int32_t height);

private:
    rclcpp::Logger mLogger;
    rclcpp::Node* mNode = nullptr;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mDepthImagePublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mAmplitudeImagePublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mRGBImagePublisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mPointCloudPublisher;
    rclcpp::Publisher<sensor_msgs::msg::MultiEchoLaserScan>::SharedPtr d2l_pub;

    std::shared_ptr<tf2_ros::Buffer> mTfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> mTfListener;
    std::string mFrameId = "pcl";

    std::shared_ptr<ReceivedIntensityPCLFrameSink> mSink;
    meere::sensor::sptr_source_list mSourceList;
    meere::sensor::sptr_camera mCamera;
    std::atomic<int32_t> mLastState;
    std::atomic<int32_t> mLastError;

    std::shared_ptr<ModelParameter> mModelParams;

    friend class ReceivedIntensityPCLFrameSink;
};

#endif // CAMERA_MODULE_H_
