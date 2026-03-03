#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nlink_parser/LinktrackNodeframe2.h>
#include <nlink_parser/TofsenseFrame0.h>
#include <sensor_msgs/Imu.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <deque>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <sstream>

// ================ Configuration Parameters ================
static double BASELINE_L = 0.16;
static int    TAG_ID_HEAD = 11;
static int    MA_WINDOW = 1; 
// UWB map rotation offset relative to true north (radians)
static double MAP_ROTATION_OFFSET = -M_PI / 2.0; 

static const double EPS = 1e-6;

// ================ Data Structures ================
struct TagState {
  bool has = false;
  double x = 0, y = 0, z = 0;
  ros::Time stamp;
} tagA, tagB;

struct ImuRPState {
  bool has = false;
  double roll = 0.0, pitch = 0.0;
  ros::Time stamp;
} imuRP;

// Sensor states
double tof_z = 0.0;
bool tof_ready = false;

double compass_yaw_enu_rad = 0.0; // Converted ENU Yaw
bool compass_ready = false;
int serial_fd = -1;

// ================ Utility Functions ================

// Normalize angle to [-pi, pi]
inline double wrapPi(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a <= 0) a += 2.0 * M_PI;
  return a - M_PI;
}

// Compute the mean of a deque
template<typename T>
static T meanDeque(const std::deque<T>& q) {
  if (q.empty()) return T();
  T s = 0;
  for (const auto& v : q) s += v;
  return s / static_cast<T>(q.size());
}

// Moving average filter for UWB tags
struct TagFilter {
  std::deque<double> qx, qy, qz;
  void push(double x, double y, double z, int win) {
    qx.push_back(x); qy.push_back(y); qz.push_back(z);
    if ((int)qx.size() > win) { 
        qx.pop_front(); 
        qy.pop_front(); 
        qz.pop_front(); 
    }
  }
  void get(double& x, double& y, double& z) const {
    x = meanDeque(qx); y = meanDeque(qy); z = meanDeque(qz);
  }
} filtA, filtB;

// ================ Serial & BCD Parsing ================

// Initialize serial port for the compass
bool initSerial(const char* port_name, int baud_rate) {
    serial_fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd == -1) {
        ROS_ERROR("Unable to open serial port %s", port_name);
        return false;
    }

    struct termios options;
    tcgetattr(serial_fd, &options);

    speed_t baud;
    switch(baud_rate) {
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
    
    // Raw input mode
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;

    tcsetattr(serial_fd, TCSANOW, &options);
    ROS_INFO("Serial port %s initialized at %d baud", port_name, baud_rate);
    return true;
}

int bcd2int(uint8_t val) {
    return (val >> 4) * 10 + (val & 0x0F);
}

// Decode BCD array to a floating-point degree value
double decodeBCD(const uint8_t* data) {
    // data[0] high 4 bits determine the sign: 0x10 is negative
    int sign = ((data[0] & 0xF0) == 0x10) ? -1 : 1;
    int hundreds = data[0] & 0x0F;
    int tens_ones = bcd2int(data[1]);
    int decimal_part = bcd2int(data[2]);
    return sign * (hundreds * 100.0 + tens_ones + decimal_part / 100.0);
}

// Extract and process compass packets from the serial buffer
void processSerialData(std::vector<uint8_t>& buffer) {
    // Look for frame header 0x68, ensure buffer has at least 14 bytes
    while (buffer.size() >= 14) {
        if (buffer[0] != 0x68) {
            buffer.erase(buffer.begin());
            continue;
        }

        // Check for command byte 0x84
        if (buffer[3] == 0x84) {
            double yaw_compass_deg = decodeBCD(&buffer[10]);
            double compass_rad = yaw_compass_deg * M_PI / 180.0;
            double enu_yaw = M_PI / 2.0 - compass_rad;

            compass_yaw_enu_rad = wrapPi(enu_yaw);
            compass_ready = true;

            buffer.erase(buffer.begin(), buffer.begin() + 14);
        } else {
            buffer.erase(buffer.begin());
        }
    }
}

// ================ ROS Callbacks ================

// TOF sensor callback for altitude
void tofCallback(const nlink_parser::TofsenseFrame0::ConstPtr& msg) {
  tof_z = msg->dis; 
  tof_ready = true;
}

// UWB Nodeframe2 callback for X, Y position
void uwbNodeframe2Cb(const nlink_parser::LinktrackNodeframe2::ConstPtr& msg) {
  const double rx = msg->pos_3d[0];
  const double ry = msg->pos_3d[1];
  const double rz = msg->pos_3d[2];

  if (msg->id == TAG_ID_HEAD) {
    filtA.push(rx, ry, rz, MA_WINDOW);
    double mx, my, mz; 
    filtA.get(mx, my, mz);
    
    tagA.has = true; 
    tagA.x = mx; 
    tagA.y = my; 
    tagA.z = mz;
    tagA.stamp = ros::Time::now();
  } 
}

// IMU callback for roll and pitch
void imuCb(const sensor_msgs::Imu::ConstPtr& msg) {
  tf2::Quaternion q;
  tf2::fromMsg(msg->orientation, q);
  double roll, pitch, yaw_unused;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_unused);
  
  imuRP.has = true; 
  imuRP.roll = roll; 
  imuRP.pitch = pitch;
  imuRP.stamp = msg->header.stamp;
}

// ================ Main Node ================
int main(int argc, char **argv)
{
  ros::init(argc, argv, "uwb_compass_node");
  ros::NodeHandle nh("~");

  // Load Parameters
  nh.param("tag_id_head", TAG_ID_HEAD, 11);
  nh.param("ma_window", MA_WINDOW, 1);
  nh.param("map_rotation_offset", MAP_ROTATION_OFFSET, -M_PI / 2.0);

  // Initialize serial port
  if (!initSerial("/dev/ttyAMA1", 115200)) {
      ROS_ERROR("Failed to initialize serial port!");
  }

  // Initialize Subscribers
  ros::Subscriber sub_uwb = nh.subscribe<nlink_parser::LinktrackNodeframe2>("/drone1/nlink_linktrack_nodeframe2", 100, uwbNodeframe2Cb);
  ros::Subscriber sub_tof = nh.subscribe<nlink_parser::TofsenseFrame0>("/drone1/nlink_tofsense_frame0", 30, tofCallback);
  ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("/drone1/mavros/imu/data", 50, imuCb);

  // Initialize Publishers
  // Publisher 1: MAVROS Vision Pose
  ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/drone1/mavros/vision_pose/pose", 30);
  
  // Publisher 2: User Debug / RViz Display
  ros::Publisher pub_user_pose = nh.advertise<geometry_msgs::PoseStamped>("/drone1/uwb_compass/pose", 30);

  ros::Rate rate(30.0);
  std::vector<uint8_t> serial_buffer;
  uint8_t tmp_buf[128]; 

  while (ros::ok()) {
    // 1. Read Serial Data
    if (serial_fd != -1) {
        int n = read(serial_fd, reinterpret_cast<void*>(tmp_buf), sizeof(tmp_buf));
        if (n > 0) {
            serial_buffer.insert(serial_buffer.end(), tmp_buf, tmp_buf + n);
            processSerialData(serial_buffer);
        }
    }

    // 2. Process ROS Callbacks
    ros::spinOnce();

    // 3. Sensor Fusion and Publishing
    if (tagA.has && tof_ready && imuRP.has && compass_ready) {
      
      double uwb_x = tagA.x; 
      double uwb_y = tagA.y;

      // Rotate coordinates to map frame
      double px = uwb_x * cos(MAP_ROTATION_OFFSET) - uwb_y * sin(MAP_ROTATION_OFFSET);
      double py = uwb_x * sin(MAP_ROTATION_OFFSET) + uwb_y * cos(MAP_ROTATION_OFFSET);

      double yaw = compass_yaw_enu_rad;
      double roll  = imuRP.roll;
      double pitch = imuRP.pitch;

      // Construct PoseStamped message
      geometry_msgs::PoseStamped out;
      out.header.stamp = ros::Time::now();
      out.header.frame_id = "map";

      out.pose.position.x = px;
      out.pose.position.y = py;
      
      // Pitch and roll compensation for ToF altitude
      const double c = std::abs(std::cos(roll) * std::cos(pitch));
      out.pose.position.z = tof_z * c;

      // Convert Euler angles to Quaternion
      tf2::Quaternion q_out;
      q_out.setRPY(roll, pitch, yaw);
      out.pose.orientation = tf2::toMsg(q_out);

      // Publish to MAVROS
      pub_pose.publish(out);
      
      // Publish for RViz and user debugging
      pub_user_pose.publish(out);
      
      ROS_INFO_THROTTLE(1.0, "[Pose Update] XYZ: [%.3f, %.3f, %.3f] | Yaw: %.2f deg", 
                        px, py, out.pose.position.z, yaw * 180.0 / M_PI);
    }
    else {
        // Warn if waiting on specific sensor data streams
        ROS_WARN_THROTTLE(2.0, "Waiting for data streams... Tag:%d, Tof:%d, IMU:%d, Compass:%d", 
                          tagA.has, tof_ready, imuRP.has, compass_ready);
    }

    rate.sleep();
  }

  // Cleanup
  if (serial_fd != -1) close(serial_fd);
  return 0;
}