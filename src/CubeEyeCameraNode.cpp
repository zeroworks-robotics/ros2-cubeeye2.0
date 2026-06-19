#include <sstream>
#include <thread>
#include <mutex>
#include <atomic>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>

// service
#include "cubeeye_camera/srv/last_state.hpp"
#include "cubeeye_camera/srv/last_error.hpp"
#include "cubeeye_camera/srv/scan.hpp"
#include "cubeeye_camera/srv/connect.hpp"
#include "cubeeye_camera/srv/run.hpp"
#include "cubeeye_camera/srv/stop.hpp"
#include "cubeeye_camera/srv/disconnect.hpp"

#include "ProjectDefines.h"
#include "CameraModule.h"
#include "CubeEyeCameraNode.h"

CubeEyeCameraNode::CubeEyeCameraNode() : Node("cubeeye_camera_node"),
    mLogger(rclcpp::get_logger("camera_node")), mCamera(std::make_shared<CameraModule>()),
    mLastMsgTime(0, 0, RCL_ROS_TIME)
{
}

void CubeEyeCameraNode::init()
{
    // init services
    mLastStateService = create_service<cubeeye_camera::srv::LastState>("~/get_last_state",
                            std::bind(&CubeEyeCameraNode::getLastStateServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    mLastErrorService = create_service<cubeeye_camera::srv::LastError>("~/get_last_error",
                            std::bind(&CubeEyeCameraNode::getLastErrorServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    mScanService = create_service<cubeeye_camera::srv::Scan>("~/scan",
                            std::bind(&CubeEyeCameraNode::getScanServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    mConnectService = create_service<cubeeye_camera::srv::Connect>("~/connect",
                            std::bind(&CubeEyeCameraNode::getConnectServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    mRunService = create_service<cubeeye_camera::srv::Run>("~/run",
                            std::bind(&CubeEyeCameraNode::getRunServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    mStopService = create_service<cubeeye_camera::srv::Stop>("~/stop",
                            std::bind(&CubeEyeCameraNode::getStopServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
    mDisconnectService = create_service<cubeeye_camera::srv::Disconnect>("~/disconnect",
                            std::bind(&CubeEyeCameraNode::getDisconnectServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Declare the watchdog param here (before any connect/autorun → before
    // ModelParameter::addTo registers its param-rejecting on-set callback).
    if (!has_parameter("msg_live_check_time"))
        declare_parameter("msg_live_check_time", 10);
}

void CubeEyeCameraNode::getLastStateServiceCallback(
    const std::shared_ptr<cubeeye_camera::srv::LastState::Request> request,
    std::shared_ptr<cubeeye_camera::srv::LastState::Response>      response)
{
    UNUSED(request);
    response->state = mCamera->getLastState();
}

void CubeEyeCameraNode::getLastErrorServiceCallback(
    const std::shared_ptr<cubeeye_camera::srv::LastError::Request> request,
    std::shared_ptr<cubeeye_camera::srv::LastError::Response>      response)
{
    UNUSED(request);
    response->error = mCamera->getLastError();
}

void CubeEyeCameraNode::getScanServiceCallback(const std::shared_ptr<cubeeye_camera::srv::Scan::Request> request,
                                std::shared_ptr<cubeeye_camera::srv::Scan::Response> response)
{
    UNUSED(request);
    RCLCPP_INFO(mLogger, "scan cameras...");
    response->connections = mCamera->scan();
}

void CubeEyeCameraNode::getConnectServiceCallback(const std::shared_ptr<cubeeye_camera::srv::Connect::Request> request,
                                    std::shared_ptr<cubeeye_camera::srv::Connect::Response> response)
{
    RCLCPP_INFO(mLogger, "connect camera(index: %d)", request->index);
    response->result = mCamera->connect(request->index);
    if (response->result != meere::sensor::result::success) {
        RCLCPP_ERROR(mLogger, "camera connection failed.");
        return;
    }

    mCamera->connectTo(this);
}

void CubeEyeCameraNode::getRunServiceCallback(const std::shared_ptr<cubeeye_camera::srv::Run::Request> request,
                                std::shared_ptr<cubeeye_camera::srv::Run::Response> response)
{
    RCLCPP_INFO(mLogger, "run camera(type: %d)", request->type);
    response->result = mCamera->run(request->type);
}

void CubeEyeCameraNode::getStopServiceCallback(const std::shared_ptr<cubeeye_camera::srv::Stop::Request> request,
                                std::shared_ptr<cubeeye_camera::srv::Stop::Response> response)
{
    UNUSED(request);
    RCLCPP_INFO(mLogger, "stop camera");
    response->result = mCamera->stop();
}

void CubeEyeCameraNode::getDisconnectServiceCallback(const std::shared_ptr<cubeeye_camera::srv::Disconnect::Request> request,
                                        std::shared_ptr<cubeeye_camera::srv::Disconnect::Response> response)
{
    UNUSED(request);
    RCLCPP_INFO(mLogger, "disconnect camera");
    response->result = mCamera->disconnect();
    if (response->result != meere::sensor::result::success) {
        RCLCPP_ERROR(mLogger, "camera disconnection failed.");
        return;
    }

    mCamera->disconnectFrom(this);
}

void CubeEyeCameraNode::autorun(std::string serialNumber, uint32_t frameType) {
    RCLCPP_INFO(mLogger, "autorun (serial number: %s, frametype %d)", serialNumber.c_str(), frameType);

    std::vector<std::string> list = mCamera->scan();

    if (!list.empty()) {
        RCLCPP_INFO(mLogger, "connect camera(serialnumber: %s)", serialNumber.c_str());
        meere::sensor::result _rt = mCamera->connect(serialNumber);
        if (_rt != meere::sensor::result::success) {
            RCLCPP_ERROR(mLogger, "camera connection failed.");
            topic_check(false);
            return;
        }

        mCamera->connectTo(this);

        RCLCPP_INFO(mLogger, "run camera(type: %d)", frameType);
        _rt = mCamera->run(frameType);
        if (_rt != meere::sensor::result::success) {
            RCLCPP_ERROR(mLogger, "mCamera->run() failed");
        }

        RCLCPP_INFO(mLogger, "camera is running.");
        topic_check(true);
    }
    else {
        RCLCPP_ERROR(mLogger, "no source found.");
        topic_check(false);
    }
}

bool CubeEyeCameraNode::shutdown()
{
    RCLCPP_INFO(mLogger, "node shutdown");
    mTopicCheckStop = true;
    if (mTopicCheckThread.joinable())
        mTopicCheckThread.join();
    if (mCamera != nullptr) {
        mCamera->shutdown();
    }
    return true;
}

void CubeEyeCameraNode::msg_check_Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    UNUSED(msg);
    std::lock_guard<std::mutex> lock(mtx);
    msg_live_check = true;
}

void CubeEyeCameraNode::node_killer()
{
    // ROS 2 self-heal: stop the process so a launch with respawn=True restarts it.
    // (ROS 1 used `rosnode kill <name>`; there is no direct equivalent in ROS 2.)
    RCLCPP_ERROR(mLogger, "frame liveness lost - shutting down node for respawn");
    rclcpp::shutdown();
}

void CubeEyeCameraNode::topic_check(bool node_start_check)
{
    if (!node_start_check) {
        node_killer();
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mtx);
        msg_live_check = false;
        mLastMsgTime = now();
    }

    if (!has_parameter("msg_live_check_time"))
        declare_parameter("msg_live_check_time", 10);
    const int wait_seconds = (int)get_parameter("msg_live_check_time").as_int();

    // subscribe to this node's own point cloud to detect liveness (serviced by the
    // main executor's spin()).
    mPointsSub = create_subscription<sensor_msgs::msg::PointCloud2>(
        "~/points", 10, std::bind(&CubeEyeCameraNode::msg_check_Callback, this, std::placeholders::_1));

    mTopicCheckStop = false;
    if (mTopicCheckThread.joinable())
        mTopicCheckThread.join();

    mTopicCheckThread = std::thread([this, wait_seconds]()
    {
        rclcpp::Rate rate(16);
        while (!mTopicCheckStop && rclcpp::ok())
        {
            {
                std::lock_guard<std::mutex> lock(mtx);
                if (msg_live_check) {
                    mLastMsgTime = now();
                    msg_live_check = false;
                }
                if ((now() - mLastMsgTime).seconds() > wait_seconds) {
                    node_killer();
                    break;
                }
            }
            rate.sleep();
        }
    });
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CubeEyeCameraNode>();
    node->init();

    RCLCPP_INFO(node->get_logger(), "cubeeye camera node started");

    // declare launch parameters
    node->declare_parameter("autorun_onoff", false);
    node->declare_parameter("autorun_serialnumber", "");
    node->declare_parameter("autorun_frametype", 6);

    bool _autorun = node->get_parameter("autorun_onoff").as_bool();
    RCLCPP_INFO(node->get_logger(), "autorun_onoff %d", _autorun);

    if (_autorun) {
        std::string _serialNumber = node->get_parameter("autorun_serialnumber").as_string();
        RCLCPP_INFO(node->get_logger(), "autorun_serialnumber %s", _serialNumber.c_str());

        int _frameType = node->get_parameter("autorun_frametype").as_int();
        RCLCPP_INFO(node->get_logger(), "autorun_frametype %d", _frameType);

        if (!_serialNumber.empty()) {
            node->autorun(_serialNumber, _frameType);
        }
    }

    if (rclcpp::ok())
        rclcpp::spin(node);
    rclcpp::shutdown();

    node->shutdown();
    RCLCPP_INFO(node->get_logger(), "cubeeye camera node stopped");
    return 0;
}
