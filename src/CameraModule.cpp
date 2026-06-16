#include <sstream>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <cmath>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/multi_echo_laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/time.h>
#include <cv_bridge/cv_bridge.h>

#include "ProjectDefines.h"
#include "CameraModule.h"

class ReceivedIntensityPCLFrameSink : public meere::sensor::sink
 , public meere::sensor::prepared_listener
{
public:
    // Hold a weak_ptr to the owning module: the camera HW may deliver a final
    // callback after CameraModule is destroyed; lock() returns null then (no UAF).
    ReceivedIntensityPCLFrameSink(std::weak_ptr<CameraModule> node) : mNode(node) {}
    virtual ~ReceivedIntensityPCLFrameSink() = default;

    virtual std::string name() const {
        return std::string("ReceivedIntensityPCLFrameSink");
    }

    virtual void onCubeEyeCameraState(const meere::sensor::ptr_source source, meere::sensor::CameraState state) {
        if (auto node = mNode.lock()) {
            RCLCPP_DEBUG(node->mLogger, "%s:%d source(%s) state = %d\n", __FUNCTION__, __LINE__, source->uri().c_str(), static_cast<int>(state));
            node->setLastState((int32_t)state);
        }
    }

    virtual void onCubeEyeCameraError(const meere::sensor::ptr_source source, meere::sensor::CameraError error) {
        if (auto node = mNode.lock()) {
            RCLCPP_DEBUG(node->mLogger, "%s:%d source(%s) error = %d\n", __FUNCTION__, __LINE__, source->uri().c_str(), static_cast<int>(error));
            node->setLastError((int32_t)error);
        }
    }

    virtual void onCubeEyeFrameList(const meere::sensor::ptr_source source , const meere::sensor::sptr_frame_list& frames) {
        UNUSED(source);
        if (auto node = mNode.lock()) node->publishFrames(frames);
    }

public:
    virtual void onCubeEyeCameraPrepared(const meere::sensor::ptr_camera camera) {
        if (auto node = mNode.lock())
            RCLCPP_INFO(node->mLogger, "%s:%d source(%s)\n", __FUNCTION__, __LINE__, camera->source()->uri().c_str());
    }

public:
    std::weak_ptr<CameraModule> mNode;
};

CameraModule::CameraModule() : mLogger(rclcpp::get_logger("camera_module")),
        mLastState(0), mLastError(0) {
    // Sink and worker thread are created lazily once we have a node / camera
    // (shared_from_this() is unavailable in the constructor).
}

std::vector<std::string> CameraModule::scan() {
    mSourceList = meere::sensor::search_camera_source();
    if (mSourceList == nullptr || mSourceList->size() == 0) {
        RCLCPP_ERROR(mLogger, "no searched device!");
        return std::vector<std::string>();
    }

    std::vector<std::string> connections;

    int i = 0;
    for (auto it : (*mSourceList)) {
        RCLCPP_INFO(mLogger, "%d) source name : %s, serialNumber : %s, uri : %s",
            i++, it->name().c_str(), it->serialNumber().c_str(), it->uri().c_str());

        connections.push_back(it->uri());
    }

    return connections;
}

meere::sensor::result CameraModule::connect(int32_t index) {
    if (mSourceList == nullptr || mSourceList->size() == 0) {
        RCLCPP_ERROR(mLogger, "source list is null");
        return meere::sensor::result::fail;
    }

    if (index >= static_cast<int32_t>(mSourceList->size())) {
        RCLCPP_ERROR(mLogger, "wrong source index");
        return meere::sensor::result::fail;
    }

    // create camera
    mCamera = meere::sensor::create_camera(mSourceList->at(index));
    if (mCamera == nullptr) {
        RCLCPP_ERROR(mLogger, "camera creation failed");
        return meere::sensor::result::fail;
    }

    if (!mSink)
        mSink = std::make_shared<ReceivedIntensityPCLFrameSink>(shared_from_this());

    meere::sensor::add_prepared_listener(mSink.get());

    mCamera->addSink(mSink.get());

    meere::sensor::result _rt;
    _rt = mCamera->prepare();
    assert(meere::sensor::result::success == _rt);
    if (meere::sensor::result::success != _rt) {
        RCLCPP_ERROR(mLogger, "mCamera->prepare() failed");

        meere::sensor::destroy_camera(mCamera);
        return meere::sensor::result::fail;
    }
    RCLCPP_INFO(mLogger, "camera is connected");

    return meere::sensor::result::success;
}

meere::sensor::result CameraModule::connect(std::string serialNumber) {
    if (mSourceList == nullptr || mSourceList->size() == 0) {
        RCLCPP_ERROR(mLogger, "source list is null");
        return meere::sensor::result::fail;
    }

    for (const auto& it : (*mSourceList)) {
        if (it->serialNumber() == serialNumber) {
            mCamera = meere::sensor::create_camera(it);
            break;
        }
    }

    if (!mCamera) {
        RCLCPP_ERROR(mLogger, "camera creation failed");
        return meere::sensor::result::fail;
    }

    if (!mSink)
        mSink = std::make_shared<ReceivedIntensityPCLFrameSink>(shared_from_this());

    meere::sensor::add_prepared_listener(mSink.get());

    mCamera->addSink(mSink.get());

    meere::sensor::result _rt;
    _rt = mCamera->prepare();
    assert(meere::sensor::result::success == _rt);
    if (meere::sensor::result::success != _rt) {
        RCLCPP_ERROR(mLogger, "mCamera->prepare() failed");

        meere::sensor::destroy_camera(mCamera);
        return meere::sensor::result::fail;
    }
    RCLCPP_INFO(mLogger, "camera is connected");

    return meere::sensor::result::success;
}

meere::sensor::result CameraModule::run(int32_t type) {
    if (mCamera == nullptr) {
        RCLCPP_ERROR(mLogger, "camera is not created");
        return meere::sensor::result::fail;
    }

    meere::sensor::result _rt = mCamera->run(type);
    if (_rt != meere::sensor::result::success) {
        RCLCPP_ERROR(mLogger, "mCamera->run() failed");
        return _rt;
    }

    return _rt;
}

meere::sensor::result CameraModule::stop() {
    thread_flag = false;
    if (d2g_thread && d2g_thread->joinable())
        d2g_thread->join();

    if (mCamera == nullptr) {
        RCLCPP_ERROR(mLogger, "camera is not created");
        return meere::sensor::result::fail;
    }

    meere::sensor::result _rt = mCamera->stop();
    if (_rt != meere::sensor::result::success) {
        RCLCPP_ERROR(mLogger, "mCamera->stop() failed");
        return _rt;
    }

    return _rt;
}

meere::sensor::result CameraModule::disconnect() {
    if (mCamera == nullptr) {
        RCLCPP_ERROR(mLogger, "camera is not created");
        return meere::sensor::result::fail;
    }

    mCamera->release();

    meere::sensor::remove_prepared_listener(mSink.get());
    meere::sensor::result _rt = meere::sensor::destroy_camera(mCamera);
    mCamera.reset();

    return _rt;
}

void CameraModule::shutdown() {
    thread_flag = false;
    if (d2g_thread && d2g_thread->joinable())
        d2g_thread->join();

    if (nullptr != mCamera) {
        mCamera->stop();
        mCamera->release();
        meere::sensor::destroy_camera(mCamera);
    }
}

void CameraModule::connectTo(rclcpp::Node* node) {
    mNode = node;

    // initialize model
    RCLCPP_INFO(mLogger, "make model parameter (%s)", mCamera->source()->name().c_str());
    mModelParams = ModelParameter::create(mCamera);
    if (mModelParams != nullptr) {
        mModelParams->addTo(node);
    }

    // Declare conversion params ONCE (rclcpp throws ParameterAlreadyDeclared on re-declare;
    // connectTo may be re-entered after a disconnect, so guard with has_parameter()).
    if (!node->has_parameter("frame_id"))      node->declare_parameter("frame_id", std::string("pcl"));
    if (!node->has_parameter("draw"))          node->declare_parameter("draw", draw);
    if (!node->has_parameter("right_padding")) node->declare_parameter("right_padding", right_padding);
    if (!node->has_parameter("left_padding"))  node->declare_parameter("left_padding", left_padding);
    mFrameId      = node->get_parameter("frame_id").as_string();
    draw          = node->get_parameter("draw").as_bool();
    right_padding = (int)node->get_parameter("right_padding").as_int();
    left_padding  = (int)node->get_parameter("left_padding").as_int();

    // tf2 buffer/listener need the node clock; the listener spins its own thread.
    if (!mTfBuffer) {
        mTfBuffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        mTfListener = std::make_shared<tf2_ros::TransformListener>(*mTfBuffer);
    }

    // create publishers
    createPublishers(node);

    // start the depth->laserscan worker once (was started in ctor under ROS 1)
    if (!d2g_thread) {
        thread_flag = true;
        init_thread();
    }
}

void CameraModule::disconnectFrom(rclcpp::Node* node) {
    if (mModelParams != nullptr) {
        mModelParams->removeFrom(node);
        mModelParams.reset();
    }
}

void CameraModule::publishFrames(const meere::sensor::sptr_frame_list& frames)
{
    for (auto it : (*frames)) {
        // intensity-PointCloud frame
        if (it->frameType() == meere::sensor::FrameType::Depth
            || it->frameType() == meere::sensor::FrameType::Amplitude
            || it->frameType() == meere::sensor::FrameType::RegisteredDepth) {
            if (it->frameDataType() == meere::sensor::DataType::U16) {
                auto _sptr_basic_frame = meere::sensor::frame_cast_basic16u(it);
                auto _sptr_frame_data = _sptr_basic_frame->frameData();	// data array

                sensor_msgs::msg::Image::SharedPtr _msg = createImageMessage(it->frameType(),
                                                                _sptr_basic_frame->frameWidth(), _sptr_basic_frame->frameHeight());
                uint16_t* _data = reinterpret_cast<uint16_t*>(&_msg->data[0]);
                for (int y = 0 ; y < _sptr_basic_frame->frameHeight(); y++) {
                    for (int x = 0 ; x < _sptr_basic_frame->frameWidth(); x++) {
                        int _pos = y * _sptr_basic_frame->frameWidth() + x;
                        _data[_pos] = (*_sptr_frame_data)[_pos];
                    }
                }

                if (it->frameType() == meere::sensor::FrameType::Depth
                    || it->frameType() == meere::sensor::FrameType::RegisteredDepth) {
                    mDepthImagePublisher->publish(*_msg);
                }
                else if (it->frameType() == meere::sensor::FrameType::Amplitude) {
                    mAmplitudeImagePublisher->publish(*_msg);
                }
            }
        }
        else if (it->frameType() == meere::sensor::FrameType::RGB
            || it->frameType() == meere::sensor::FrameType::RegisteredRGB) {

            auto _sptr_frame = meere::sensor::frame_cast_basic8u(it);
            auto _ptr_frame_data = _sptr_frame->frameData();

            sensor_msgs::msg::Image::SharedPtr _msg = createImageMessage(it->frameType(),
                                                            _sptr_frame->frameWidth(), _sptr_frame->frameHeight());
            uint8_t* _data = reinterpret_cast<uint8_t*>(&_msg->data[0]);
            for (int y = 0 ; y < _sptr_frame->frameHeight(); y++) {
                for (int x = 0 ; x < _sptr_frame->frameWidth(); x++) {
                    int _pos = y * (_sptr_frame->frameWidth() * 3) + x * 3;
                    _data[_pos] = (*_ptr_frame_data)[_pos];
                    _data[_pos + 1] = (*_ptr_frame_data)[_pos + 1];
                    _data[_pos + 2] = (*_ptr_frame_data)[_pos + 2];
                }
            }

            mRGBImagePublisher->publish(*_msg);
        }
        else if (it->frameType() == meere::sensor::FrameType::PointCloud
            || it->frameType() == meere::sensor::FrameType::RegisteredPointCloud) {
            if (it->frameDataType() == meere::sensor::DataType::F32) {
                auto _sptr_pointcloud_frame = meere::sensor::frame_cast_pcl32f(it);
                auto _sptr_frame_dataX = _sptr_pointcloud_frame->frameDataX();
                auto _sptr_frame_dataY = _sptr_pointcloud_frame->frameDataY();
                auto _sptr_frame_dataZ = _sptr_pointcloud_frame->frameDataZ();

                sensor_msgs::msg::PointCloud2::SharedPtr _pcl_msg = createPointCloudMessage(meere::sensor::FrameType::PointCloud,
                                                                _sptr_pointcloud_frame->frameWidth(), _sptr_pointcloud_frame->frameHeight());

                float* _pcl_data = reinterpret_cast<float*>(&_pcl_msg->data[0]);
                for (int y = 0 ; y < _sptr_pointcloud_frame->frameHeight(); y++) {
                    for (int x = 0 ; x < _sptr_pointcloud_frame->frameWidth(); x++) {
                        // left/right column crop (with oversize guard)
                        if (this->right_padding + this->left_padding > _sptr_pointcloud_frame->frameWidth())
                        {
                            this->right_padding = 0;
                            this->left_padding = 0;
                            RCLCPP_WARN(mLogger, "padding is too big for frame width, set padding to 0");
                        }

                        if (x > _sptr_pointcloud_frame->frameWidth() - this->right_padding || x < this->left_padding) continue;

                        int _pos = y * _sptr_pointcloud_frame->frameWidth() + x;
                        int _pcl_pos = _pos * 3;

                        // points
                        _pcl_data[_pcl_pos] = -(*_sptr_frame_dataY)[_pos];
                        _pcl_data[_pcl_pos + 1] = -(*_sptr_frame_dataX)[_pos];
                        _pcl_data[_pcl_pos + 2] = (*_sptr_frame_dataZ)[_pos];
                    }
                }

                mPointCloudPublisher->publish(*_pcl_msg);

                {
                    std::lock_guard<std::mutex> lk(data_mtx);
                    main_data = *_pcl_msg;
                }
            }
        }
        else if (it->frameType() == meere::sensor::FrameType::IntensityPointCloud) {
            if (it->frameDataType() == meere::sensor::DataType::F32) {
                auto _sptr_intensity_pointcloud_frame = meere::sensor::frame_cast_ipcl32f(it);
                auto _sptr_frame_dataX = _sptr_intensity_pointcloud_frame->frameDataX();
                auto _sptr_frame_dataY = _sptr_intensity_pointcloud_frame->frameDataY();
                auto _sptr_frame_dataZ = _sptr_intensity_pointcloud_frame->frameDataZ();

                sensor_msgs::msg::PointCloud2::SharedPtr _pcl_msg = createPointCloudMessage(meere::sensor::FrameType::PointCloud,
                                                                _sptr_intensity_pointcloud_frame->frameWidth(), _sptr_intensity_pointcloud_frame->frameHeight());

                float* _pcl_data = reinterpret_cast<float*>(&_pcl_msg->data[0]);
                for (int y = 0 ; y < _sptr_intensity_pointcloud_frame->frameHeight(); y++) {
                    for (int x = 0 ; x < _sptr_intensity_pointcloud_frame->frameWidth(); x++) {
                        int _pos = y * _sptr_intensity_pointcloud_frame->frameWidth() + x;
                        int _pcl_pos = _pos * 3;

                        // points
                        _pcl_data[_pcl_pos] = -(*_sptr_frame_dataY)[_pos];
                        _pcl_data[_pcl_pos + 1] = -(*_sptr_frame_dataX)[_pos];
                        _pcl_data[_pcl_pos + 2] = (*_sptr_frame_dataZ)[_pos];
                    }
                }

                {
                    std::lock_guard<std::mutex> lk(data_mtx);
                    main_data = *_pcl_msg;
                }

                mPointCloudPublisher->publish(*_pcl_msg);
            }
        }
    }
}

void CameraModule::createPublishers(rclcpp::Node* node)
{
    auto _qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

    mDepthImagePublisher = node->create_publisher<sensor_msgs::msg::Image>("~/depth", _qos);
    mAmplitudeImagePublisher = node->create_publisher<sensor_msgs::msg::Image>("~/amplitude", _qos);
    mRGBImagePublisher = node->create_publisher<sensor_msgs::msg::Image>("~/rgb", _qos);
    mPointCloudPublisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("~/points", _qos);

    // depth -> 2D laser scan output (keeps the fork's "filtered_points" topic contract)
    d2l_pub = node->create_publisher<sensor_msgs::msg::MultiEchoLaserScan>("filtered_points", _qos);
}

sensor_msgs::msg::Image::SharedPtr CameraModule::createImageMessage(meere::sensor::FrameType type, int32_t width, int32_t height)
{
    sensor_msgs::msg::Image::SharedPtr _imageMsg;

    if (type == meere::sensor::FrameType::Depth
        || type == meere::sensor::FrameType::Amplitude
        || type == meere::sensor::FrameType::RegisteredDepth) {
        _imageMsg = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1).toImageMsg();

        switch (type) {
        case meere::sensor::FrameType::Depth:
        case meere::sensor::FrameType::RegisteredDepth:
            _imageMsg->header.frame_id = "depth";
            break;
        case meere::sensor::FrameType::Amplitude:
            _imageMsg->header.frame_id = "amplitude";
            break;
        default:
            _imageMsg->header.frame_id = "depth";
            break;
        }

        _imageMsg->width = width;
        _imageMsg->height = height;
        _imageMsg->is_bigendian = false;
        _imageMsg->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        _imageMsg->step = (uint32_t)(sizeof(uint16_t) * width);
        _imageMsg->data.resize(sizeof(uint16_t) * width * height);
    }
    else if (type == meere::sensor::FrameType::RGB
        || type == meere::sensor::FrameType::RegisteredRGB) {
        _imageMsg = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8).toImageMsg();

        _imageMsg->header.frame_id = "rgb";
        _imageMsg->width = width;
        _imageMsg->height = height;
        _imageMsg->is_bigendian = false;
        _imageMsg->encoding = sensor_msgs::image_encodings::BGR8;
        _imageMsg->step = (uint32_t)(sizeof(uint8_t) * 3 * width);
        _imageMsg->data.resize(sizeof(uint8_t) * 3 * width * height);
    }

    return _imageMsg;
}

sensor_msgs::msg::PointCloud2::SharedPtr CameraModule::createPointCloudMessage(meere::sensor::FrameType type, int32_t width, int32_t height)
{
    UNUSED(type);

    // one-shot tf: camera frame -> base_link (params already declared in connectTo)
    if (!get_tf && mTfBuffer) {
        try {
            geometry_msgs::msg::TransformStamped transform =
                mTfBuffer->lookupTransform("base_link", mFrameId, tf2::TimePointZero, tf2::durationFromSec(1.0));

            T.x() = transform.transform.translation.x;
            T.y() = transform.transform.translation.y;
            T.z() = transform.transform.translation.z;

            Eigen::Quaterniond eigen_quat(transform.transform.rotation.w,
                                          transform.transform.rotation.x,
                                          transform.transform.rotation.y,
                                          transform.transform.rotation.z);
            R = eigen_quat.toRotationMatrix();

            get_tf = true;
        }
        catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(mLogger, "tf error : %s", ex.what());
        }
    }

    sensor_msgs::msg::PointCloud2::SharedPtr _pclMsg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    _pclMsg->header.frame_id = mFrameId;
    _pclMsg->width = width;
    _pclMsg->height = height;
    _pclMsg->is_bigendian = false;
    _pclMsg->is_dense = false;

    _pclMsg->point_step = (uint32_t)(3 * sizeof(float));
    _pclMsg->row_step = (uint32_t)(_pclMsg->point_step * width);
    _pclMsg->fields.resize(3);
    _pclMsg->fields[0].name = "z";
    _pclMsg->fields[0].offset = 0;
    _pclMsg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    _pclMsg->fields[0].count = 1;

    _pclMsg->fields[1].name = "y";
    _pclMsg->fields[1].offset = _pclMsg->fields[0].offset + (uint32_t)sizeof(float);
    _pclMsg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    _pclMsg->fields[1].count = 1;

    _pclMsg->fields[2].name = "x";
    _pclMsg->fields[2].offset = _pclMsg->fields[1].offset + (uint32_t)sizeof(float);
    _pclMsg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    _pclMsg->fields[2].count = 1;
    _pclMsg->data.resize(_pclMsg->point_step * _pclMsg->width * _pclMsg->height);

    return _pclMsg;
}

void CameraModule::init_thread()
{
    this->d2g_thread = std::make_shared<std::thread>([this]()
    {
        int ROI_size = 0;
        int min_ROI_size = 0;
        std::vector<std::pair<cv::Mat, cv::Mat>> ROI_list;

        cv::Mat test_RGBD_data, test_RGBD_mask;
        std::vector<cv::Mat> test_ROI_list;

        // Pre-allocate output scan message once — reset per iteration instead of reallocating
        sensor_msgs::msg::MultiEchoLaserScan output;
        output.angle_min = scan_angle_min;
        output.angle_max = scan_angle_max;
        output.angle_increment = scan_angle_increment;
        output.range_min = scan_range_min;
        output.range_max = scan_range_max;
        output.ranges.resize((int)((output.angle_max - output.angle_min) / output.angle_increment));

        // Thread pool — created once, reused every iteration
        std::mutex pool_mtx;
        std::condition_variable work_cv, done_cv;
        int work_gen = 0;
        int workers_done = 0;
        bool stop_pool = false;
        std::atomic<int> roi_idx{0};
        int roi_list_size = 0;

        std::vector<std::thread> pool(num_of_thread);
        for (int i = 0; i < num_of_thread; i++)
        {
            pool[i] = std::thread([&]()
            {
                int my_gen = 0;
                Eigen::Vector3d p(0, 0, 0);
                while (true)
                {
                    {
                        std::unique_lock<std::mutex> lk(pool_mtx);
                        work_cv.wait(lk, [&]{ return work_gen != my_gen || stop_pool; });
                        if (stop_pool) break;
                        my_gen = work_gen;
                    }

                    while (true)
                    {
                        int my_idx = roi_idx.fetch_add(1);
                        if (my_idx >= roi_list_size) break;

                        if(draw) cv::rectangle(test_ROI_list[my_idx], cv::Rect(0, 0, test_ROI_list[my_idx].cols, test_ROI_list[my_idx].rows), cv::Scalar(100, 100, 100));

                        std::vector<cv::Vec3f> sort_q;

                        std::pair<cv::Mat, cv::Mat>& ROI = ROI_list[my_idx];
                        sort_q.reserve(ROI_size);

                        ROI.first.forEach<cv::Vec3f>([&](cv::Vec3f& val, const int* pos)
                        {
                            bool is_in = ROI.second.ptr<unsigned char>(pos[0])[pos[1]]>0;

                            if(val[0]>scan_range_min && val[0]<scan_range_max && is_in)
                                sort_q.emplace_back(val);
                        });

                        if(sort_q.size() < (size_t)min_ROI_size)
                        {
                            if(draw && !sort_q.empty()) cv::rectangle(test_ROI_list[my_idx], cv::Rect(0, 0, test_ROI_list[my_idx].cols, test_ROI_list[my_idx].rows), cv::Scalar(0, 0, 255));
                            continue;
                        }

                        std::sort(sort_q.begin(), sort_q.end(), [](cv::Vec3f& a, cv::Vec3f& b)
                        {
                            return a[0] < b[0];
                        });

                        int sq_size = sort_q.size();

                        cv::Vec3f medianValue = sort_q.at(sq_size/2);

                        p[0] = medianValue[0];
                        p[1] = medianValue[1];
                        p[2] = medianValue[2];

                        Eigen::Vector3d reprj_p = R * p + T;
                        float range = std::sqrt(std::pow(reprj_p[0], 2) + std::pow(reprj_p[1], 2));
                        int range_10x = std::round(range*10.0f);

                        if (range <= scan_range_min || range >= scan_range_max)
                        {
                            if(draw) cv::rectangle(test_ROI_list[my_idx], cv::Rect(0, 0, test_ROI_list[my_idx].cols, test_ROI_list[my_idx].rows), cv::Scalar(0, 125, 0));
                            continue;
                        }

                        float scan_angle = (float)atan2(reprj_p[1], reprj_p[0]);
                        if (scan_angle < scan_angle_min || scan_angle > scan_angle_max)
                        {
                            if(draw) cv::rectangle(test_ROI_list[my_idx], cv::Rect(0, 0, test_ROI_list[my_idx].cols, test_ROI_list[my_idx].rows), cv::Scalar(0, 125, 0));
                            continue;
                        }

                        int index = (scan_angle - output.angle_min) / output.angle_increment;
                        if (index < 0 || index >= (int)output.ranges.size())
                        {
                            if(draw) cv::rectangle(test_ROI_list[my_idx], cv::Rect(0, 0, test_ROI_list[my_idx].cols, test_ROI_list[my_idx].rows), cv::Scalar(0, 125, 0));
                            continue;
                        }

                        float offset = sin_lookup[range_10x];
                        bool lower_than_top = reprj_p[2]<ground_height_max+offset;
                        bool higher_than_bot = reprj_p[2]>ground_height_min-offset;

                        if((lower_than_top&&higher_than_bot) || reprj_p[2]>Robot_height+offset)
                        {
                            if(draw) cv::rectangle(test_ROI_list[my_idx], cv::Rect(0, 0, test_ROI_list[my_idx].cols, test_ROI_list[my_idx].rows), cv::Scalar(255, 0, 0));
                            continue;
                        }
                        else
                        {
                            push_mtx.lock();
                            output.ranges[index].echoes.push_back(range);
                            push_mtx.unlock();

                            if(draw)
                            {
                                cv::rectangle(test_ROI_list[my_idx], cv::Rect(1, 1, test_ROI_list[my_idx].cols-2, test_ROI_list[my_idx].rows-2), cv::Scalar(0, 255, 0));
                            }
                        }
                    }

                    {
                        std::lock_guard<std::mutex> lk(pool_mtx);
                        if (++workers_done == num_of_thread)
                            done_cv.notify_one();
                    }
                }
            });
        }

        rclcpp::Rate r(10);
        while (this->thread_flag && rclcpp::ok())
        {
            if (this->main_data.data.empty() || !get_tf)
            {
                r.sleep();
                continue;
            }

            rclcpp::Time start_time = mNode->now();

            data_mtx.lock();
            sensor_msgs::msg::PointCloud2 data = this->main_data;
            data_mtx.unlock();

            output.header = data.header;
            output.header.frame_id = "base_link";
            output.header.stamp = mNode->now();
            for (auto& echo : output.ranges)
                echo.echoes.clear();

            double column = data.width;
            double row = data.height;

            if (this->sin_lookup.empty())
            {
                int max_range_10x = std::ceil(scan_range_max*10.0f) + 1;
                this->sin_lookup = std::vector<float>(max_range_10x, 0.0f);
                float sin_f = sin(M_PI / 180.0);
                for (int i = 0; i < (int)sin_lookup.size(); i++)
                    this->sin_lookup[i] = sin_f * (float)i * 0.1f;
            }

            if(!ROI_initialized)
            {
                RGBD_data = cv::Mat(data.height, data.width, CV_32FC3, cv::Scalar(0.0f, 0.0f, 0.0f));
                RGBD_mask = cv::Mat(data.height, data.width, CV_8UC1, cv::Scalar(0));

                if(draw)
                {
                    test_RGBD_data = cv::Mat(data.height, data.width, CV_8UC3, cv::Scalar(0, 0, 0));
                    test_RGBD_mask = cv::Mat(data.height, data.width, CV_8UC3, cv::Scalar(0, 0, 0));
                }

                ROI_size = this->median_filter_col * this->median_filter_row;
                min_ROI_size = ROI_size * 0.1;

                int col_step = this->median_filter_col / 1;
                int row_step = this->median_filter_row / 1;
                int c_size = RGBD_data.cols - this->padding;
                for (int c = this->padding; c < c_size; c += col_step)
                {
                    for (int r_ = RGBD_data.rows - row_step; r_ >= 0; r_ -= row_step)
                    {
                        if (c + this->median_filter_col > c_size || r_ + this->median_filter_row > RGBD_data.rows)
                            continue;

                        cv::Rect ROI(c, r_, this->median_filter_col, this->median_filter_row);
                        ROI_list.push_back(std::pair<cv::Mat, cv::Mat>(cv::Mat(), cv::Mat()));
                        ROI_list.back().first = RGBD_data(ROI);
                        ROI_list.back().second = RGBD_mask(ROI);

                        if(draw)
                        {
                            test_ROI_list.push_back(cv::Mat());
                            test_ROI_list.back() = test_RGBD_data(ROI);
                        }
                    }
                }
                ROI_initialized = true;
            }
            RGBD_data.setTo(0);
            RGBD_mask.setTo(0);

            if(draw)
            {
                test_RGBD_data.setTo(0);
                test_RGBD_mask.setTo(0);
            }

            sensor_msgs::PointCloud2ConstIterator<float> x_iter(data, "x");
            sensor_msgs::PointCloud2ConstIterator<float> y_iter(data, "y");
            sensor_msgs::PointCloud2ConstIterator<float> z_iter(data, "z");

            cv::Mat temp_RGBD_mask(RGBD_mask.size(), CV_8UC1, cv::Scalar(0));
            for (int r_num = 0; r_num < row; r_num++)
            {
                cv::Vec3f* RGBD_data_ptr = RGBD_data.ptr<cv::Vec3f>(r_num);
                unsigned char* RGBD_mask_ptr = temp_RGBD_mask.ptr<unsigned char>(r_num);
                for (int c_num = 0; c_num < column; c_num++)
                {
                    float val = *x_iter;

                    if(std::isfinite(val) && !std::isnan(val) && val>0.0)
                    {
                        RGBD_data_ptr[c_num][0] = val;
                        RGBD_data_ptr[c_num][1] = *y_iter;
                        RGBD_data_ptr[c_num][2] = *z_iter;
                        RGBD_mask_ptr[c_num] = 255;
                    }

                    ++x_iter;
                    ++y_iter;
                    ++z_iter;
                }
            }

            cv::dilate(temp_RGBD_mask, temp_RGBD_mask, cv::Mat());

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(temp_RGBD_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

            temp_RGBD_mask.setTo(0);
            for(int i=0; i<(int)contours.size(); i++)
            {
                double area = cv::contourArea(contours[i]);
                if(area < this->contour_filter_size)
                {
                    if(draw)
                    {
                        cv::putText(test_RGBD_mask, cv::format("%.1lf", area), contours[i][0], 0, 0.5, cv::Scalar(255, 255, 255));
                        cv::drawContours(test_RGBD_mask, contours, i, cv::Scalar(0, 0, 255), -1);
                    }
                    continue;
                }
                cv::drawContours(temp_RGBD_mask, contours, i, cv::Scalar(255), -1);
                if(draw) cv::drawContours(test_RGBD_mask, contours, i, cv::Scalar(0, 255, 0), -1);
            }
            temp_RGBD_mask.copyTo(RGBD_mask);

            if(draw)
            {
                cv::Mat test_norm_RGBD_data;
                cv::normalize(RGBD_data, test_norm_RGBD_data, 0, 255, cv::NORM_MINMAX);
                test_norm_RGBD_data.convertTo(test_norm_RGBD_data, CV_8UC3);
                test_RGBD_data = 0.5*test_RGBD_mask + 0.5*test_norm_RGBD_data;
            }

            // Dispatch work to thread pool
            {
                std::lock_guard<std::mutex> lk(pool_mtx);
                roi_idx.store(0);
                roi_list_size = (int)ROI_list.size();
                workers_done = 0;
                ++work_gen;
                work_cv.notify_all();
            }
            // Wait for all workers to finish
            {
                std::unique_lock<std::mutex> lk(pool_mtx);
                done_cv.wait(lk, [&]{ return workers_done == num_of_thread; });
            }

            if(draw)
            {
                rclcpp::Time end_time = mNode->now();
                rclcpp::Duration dt = end_time - start_time;
                cv::putText(test_RGBD_data, cv::format("%5.1lf ms", dt.seconds()*1000.0), cv::Point(10, 50), 0, 0.5, cv::Scalar(0, 255, 0));
                cv::imshow("test_RGBD_data", test_RGBD_data);
                cv::waitKey(10);
            }

            d2l_pub->publish(output);
            r.sleep();
        }

        // Stop thread pool before exiting
        {
            std::lock_guard<std::mutex> lk(pool_mtx);
            stop_pool = true;
            work_cv.notify_all();
        }
        for (auto& t : pool) t.join();
    });
}
