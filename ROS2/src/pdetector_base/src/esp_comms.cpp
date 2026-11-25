#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>
#include <poll.h>
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <sstream>
#include <cmath>
#include <memory>

using namespace std::chrono_literals;

class ESPComms : public rclcpp::Node {
public:
  ESPComms() : Node("esp_comms") {
    wheelRadius = this->declare_parameter("wheel_radius", 0.0325);
    trackWidth = this->declare_parameter("track_width", 0.197);
    maxWheelSpeed = this->declare_parameter("max_wheel_speed", 13.6);
    ticksPerRev = this->declare_parameter("ticks_per_rev", 0.0);

    velSubscriber = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&ESPComms::VelCallback, this, std::placeholders::_1));
    joyPublisher = this->create_publisher<sensor_msgs::msg::Joy>("joy", 10);
    odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer = this->create_wall_timer(10ms, std::bind(&ESPComms::Tick, this));

    fd = openSerial("/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0");
  }

  ~ESPComms() {
    if (fd >= 0) {
      close(fd);
    }
  }

private:
  void Tick() {
    pollfd p{fd, POLLIN, 0};
    if (poll(&p, 1, 0) <= 0) return;
 
    char buf[256];
    ssize_t n;
    while ((n = read(fd, buf, sizeof(buf))) > 0) {
      rxBuffer.append(buf, n);
    }
 
    size_t pos;
    while ((pos = rxBuffer.find('\n')) != std::string::npos) {
      std::string line = rxBuffer.substr(0, pos);
      rxBuffer.erase(0, pos + 1);
      if (!line.empty() && line.back() == '\r') line.pop_back();
      HandleLine(line);
    }
  }

  void VelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double v = msg->linear.x;
    double w = msg->angular.z;

    double leftGround  = v - w * trackWidth / 2.0;
    double rightGround = v + w * trackWidth / 2.0;

    double left  = leftGround  / wheelRadius;
    double right = rightGround / wheelRadius;

    double biggest = std::max(std::abs(left), std::abs(right));
    if (biggest > maxWheelSpeed) {
      double scale = maxWheelSpeed / biggest;
      left *= scale;
      right *= scale;
    }

    std::string line = "V " + std::to_string(left) + " " + std::to_string(right) + "\n";
    write(fd, line.data(), line.size());
  }

  int openSerial(const std::string& path) {
    int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
      throw std::runtime_error("open " + path + ": " + std::strerror(errno));
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
      ::close(fd);
      throw std::runtime_error(std::string("tcgetattr: ") + std::strerror(errno));
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      ::close(fd);
      throw std::runtime_error(std::string("tcsetattr: ") + std::strerror(errno));
    }

    std::this_thread::sleep_for(std::chrono::seconds(2));
    tcflush(fd, TCIOFLUSH);

    return fd;
  }

  void HandleLine(const std::string& line) {
    std::istringstream in(line);
    std::string type;
    in >> type;
 
    if (type == "J") {
      float leftY, rightX;
      if (!(in >> leftY >> rightX)) return; 
 
      sensor_msgs::msg::Joy msg;
      msg.header.stamp = this->now();
      msg.axes = {0.0f, leftY, rightX, 0.0f};
      joyPublisher->publish(msg);
    }
    else if (type == "E") {
      long left, right;
      unsigned long ms;
      if (!(in >> left >> right >> ms)) return;

      HandleEncoders(left, right, ms);
    }
    else if (type == "#") {
      RCLCPP_INFO(this->get_logger(), "ESP32: %s", line.c_str() + 1);
    }
  }

  void HandleEncoders(long left, long right, unsigned long ms) {
    if (!haveEncoders) {
      lastLeft = left;
      lastRight = right;
      lastMs = ms;
      haveEncoders = true;
      return;
    }

    long dTicksL = left - lastLeft;
    long dTicksR = right - lastRight;
    double dt = (ms - lastMs) / 1000.0;

    lastLeft = left;
    lastRight = right;
    lastMs = ms;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                   "L %ld R %ld", left, right);

    if (ticksPerRev <= 0.0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                     "ticks_per_rev not set, odometry disabled");
      return;
    }
    if (dt <= 0.0) return;

    double metersPerTick = 2.0 * M_PI * wheelRadius / ticksPerRev;
    double distL = dTicksL * metersPerTick;
    double distR = dTicksR * metersPerTick;

    double dist = (distL + distR) / 2.0;
    double dTheta = (distR - distL) / trackWidth;

    x += dist * std::cos(theta + dTheta / 2.0);
    y += dist * std::sin(theta + dTheta / 2.0);
    theta = std::atan2(std::sin(theta + dTheta), std::cos(theta + dTheta));

    PublishOdometry(dist / dt, dTheta / dt);
  }

  void PublishOdometry(double v, double w) {
    rclcpp::Time stamp = this->now();
    double qz = std::sin(theta / 2.0);
    double qw = std::cos(theta / 2.0);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation.z = qz;
    odom.pose.pose.orientation.w = qw;
    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = w;
    odomPublisher->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.rotation.z = qz;
    tf.transform.rotation.w = qw;
    tfBroadcaster->sendTransform(tf);
  }

  int fd = -1;
  std::string rxBuffer;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velSubscriber;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joyPublisher;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

  double wheelRadius;
  double trackWidth;
  double maxWheelSpeed;
  double ticksPerRev;

  bool haveEncoders = false;
  long lastLeft = 0;
  long lastRight = 0;
  unsigned long lastMs = 0;

  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<ESPComms>();
    RCLCPP_INFO(node->get_logger(), "ESPComms Init");
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("esp_comms"), "%s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}

