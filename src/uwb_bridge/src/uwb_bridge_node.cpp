// UWB/compass bridge to the PX4 EKF2 external-vision input (Section VI-A, Fig. 2).
//
// A 6-anchor LinkTrack UWB system supplies the 3D position of the onboard tag.
// The position is rotated from the UWB anchor frame into the ENU map frame,
// whose heading is fixed by the external compass, so that the UWB frame and
// the flight controller's internal frame share one orientation. Roll and pitch
// come from the flight-controller IMU, yaw from the compass. The pose is
// published to <ns>/mavros/vision_pose/pose and fused by EKF2.

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nlink_parser/LinktrackNodeframe2.h>
#include <sensor_msgs/Imu.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace {

// ================ Configuration Parameters ================
int tag_id = 11;
int moving_average_window = 1;
// Rotation of the UWB anchor frame relative to the compass-aligned ENU frame (rad).
double map_rotation_offset = -M_PI / 2.0;

// ================ Data Structures ================
struct TagState {
  bool has = false;
  double x = 0.0, y = 0.0, z = 0.0;
  ros::Time stamp;
} tag;

struct ImuRPState {
  bool has = false;
  double roll = 0.0, pitch = 0.0;
  ros::Time stamp;
} imu_rp;

double compass_yaw_enu_rad = 0.0;
bool compass_ready = false;
int serial_fd = -1;

// ================ Utility Functions ================

// Normalize angle to [-pi, pi].
double wrapPi(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a <= 0.0) a += 2.0 * M_PI;
  return a - M_PI;
}

// Moving-average filter of the UWB tag position.
struct TagFilter {
  std::deque<double> qx, qy, qz;
  void push(double x, double y, double z, int window) {
    qx.push_back(x); qy.push_back(y); qz.push_back(z);
    while (static_cast<int>(qx.size()) > window) {
      qx.pop_front(); qy.pop_front(); qz.pop_front();
    }
  }
  static double mean(const std::deque<double>& q) {
    double sum = 0.0;
    for (double value : q) sum += value;
    return q.empty() ? 0.0 : sum / static_cast<double>(q.size());
  }
  void get(double& x, double& y, double& z) const {
    x = mean(qx); y = mean(qy); z = mean(qz);
  }
} tag_filter;

// ================ Serial & BCD Parsing (compass) ================

bool initSerial(const char* port_name, int baud_rate) {
  serial_fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
  if (serial_fd == -1) {
    ROS_ERROR("Unable to open serial port %s", port_name);
    return false;
  }
  struct termios options;
  tcgetattr(serial_fd, &options);
  speed_t baud;
  switch (baud_rate) {
    case 115200: baud = B115200; break;
    case 38400:  baud = B38400;  break;
    case 9600:   baud = B9600;   break;
    default:     baud = B115200; break;
  }
  cfsetispeed(&options, baud);
  cfsetospeed(&options, baud);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cflag |= (CLOCAL | CREAD);
  // Raw input mode.
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;
  tcsetattr(serial_fd, TCSANOW, &options);
  ROS_INFO("Serial port %s initialized at %d baud", port_name, baud_rate);
  return true;
}

int bcd2int(uint8_t value) {
  return (value >> 4) * 10 + (value & 0x0F);
}

// Decode a BCD heading to degrees; a high nibble of 0x1 in the first byte is negative.
double decodeBCD(const uint8_t* data) {
  const int sign = ((data[0] & 0xF0) == 0x10) ? -1 : 1;
  const int hundreds = data[0] & 0x0F;
  const int tens_ones = bcd2int(data[1]);
  const int decimal_part = bcd2int(data[2]);
  return sign * (hundreds * 100.0 + tens_ones + decimal_part / 100.0);
}

// Extract compass heading packets (header 0x68, command 0x84, 14 bytes).
void processSerialData(std::vector<uint8_t>& buffer) {
  while (buffer.size() >= 14) {
    if (buffer[0] != 0x68) {
      buffer.erase(buffer.begin());
      continue;
    }
    if (buffer[3] == 0x84) {
      const double heading_deg = decodeBCD(&buffer[10]);
      // Compass heading is clockwise from north; ENU yaw is counterclockwise from east.
      compass_yaw_enu_rad = wrapPi(M_PI / 2.0 - heading_deg * M_PI / 180.0);
      compass_ready = true;
      buffer.erase(buffer.begin(), buffer.begin() + 14);
    } else {
      buffer.erase(buffer.begin());
    }
  }
}

// ================ ROS Callbacks ================

// LinkTrack Nodeframe2: 3D position of the onboard tag in the anchor frame.
void uwbNodeframe2Cb(const nlink_parser::LinktrackNodeframe2::ConstPtr& msg) {
  if (msg->id != tag_id) return;
  tag_filter.push(msg->pos_3d[0], msg->pos_3d[1], msg->pos_3d[2], moving_average_window);
  tag_filter.get(tag.x, tag.y, tag.z);
  tag.has = true;
  tag.stamp = ros::Time::now();
}

// Flight-controller IMU: roll and pitch of the vision attitude.
void imuCb(const sensor_msgs::Imu::ConstPtr& msg) {
  tf2::Quaternion q;
  tf2::fromMsg(msg->orientation, q);
  double roll, pitch, yaw_unused;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_unused);
  imu_rp.has = true;
  imu_rp.roll = roll;
  imu_rp.pitch = pitch;
  imu_rp.stamp = msg->header.stamp;
}

}  // namespace

// ================ Main Node ================
int main(int argc, char** argv) {
  ros::init(argc, argv, "uwb_compass_node");
  ros::NodeHandle nh("~");

  std::string drone_ns, compass_port;
  int compass_baud = 115200;
  nh.param("tag_id_head", tag_id, 11);
  nh.param("ma_window", moving_average_window, 1);
  nh.param("map_rotation_offset", map_rotation_offset, -M_PI / 2.0);
  nh.param<std::string>("drone_namespace", drone_ns, "drone1");
  nh.param<std::string>("compass_port", compass_port, "/dev/ttyAMA1");
  nh.param("compass_baud", compass_baud, 115200);
  if (moving_average_window < 1) moving_average_window = 1;
  const std::string prefix = "/" + drone_ns + "/";

  if (!initSerial(compass_port.c_str(), compass_baud)) {
    ROS_ERROR("Failed to initialize the compass serial port");
  }

  ros::Subscriber sub_uwb = nh.subscribe<nlink_parser::LinktrackNodeframe2>(
      prefix + "nlink_linktrack_nodeframe2", 100, uwbNodeframe2Cb);
  ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>(prefix + "mavros/imu/data", 50, imuCb);

  // External-vision pose fused by the PX4 EKF2.
  ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>(prefix + "mavros/vision_pose/pose", 30);
  // The same pose for visualization and logging.
  ros::Publisher pub_user_pose = nh.advertise<geometry_msgs::PoseStamped>(prefix + "uwb_compass/pose", 30);

  ros::Rate rate(30.0);
  std::vector<uint8_t> serial_buffer;
  uint8_t tmp_buf[128];

  while (ros::ok()) {
    if (serial_fd != -1) {
      const int n = read(serial_fd, reinterpret_cast<void*>(tmp_buf), sizeof(tmp_buf));
      if (n > 0) {
        serial_buffer.insert(serial_buffer.end(), tmp_buf, tmp_buf + n);
        processSerialData(serial_buffer);
      }
    }
    ros::spinOnce();

    if (tag.has && imu_rp.has && compass_ready) {
      // Rotate the UWB anchor frame into the compass-aligned ENU map frame.
      const double c = std::cos(map_rotation_offset), s = std::sin(map_rotation_offset);
      geometry_msgs::PoseStamped out;
      out.header.stamp = ros::Time::now();
      out.header.frame_id = "map";
      out.pose.position.x = c * tag.x - s * tag.y;
      out.pose.position.y = s * tag.x + c * tag.y;
      out.pose.position.z = tag.z;
      tf2::Quaternion q_out;
      q_out.setRPY(imu_rp.roll, imu_rp.pitch, compass_yaw_enu_rad);
      out.pose.orientation = tf2::toMsg(q_out);
      pub_pose.publish(out);
      pub_user_pose.publish(out);
      ROS_INFO_THROTTLE(1.0, "[UWB pose] XYZ: [%.3f, %.3f, %.3f] | Yaw: %.2f deg",
                        out.pose.position.x, out.pose.position.y, out.pose.position.z,
                        compass_yaw_enu_rad * 180.0 / M_PI);
    } else {
      ROS_WARN_THROTTLE(2.0, "Waiting for data streams... UWB tag:%d, IMU:%d, compass:%d",
                        tag.has, imu_rp.has, compass_ready);
    }
    rate.sleep();
  }

  if (serial_fd != -1) close(serial_fd);
  return 0;
}
