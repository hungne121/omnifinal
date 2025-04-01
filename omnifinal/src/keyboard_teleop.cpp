#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <map>

// Map for movement keys
std::map<char, std::vector<float>> moveBindings
{
  {'i', {1, 0, 0, 0}},   // Tiến theo trục x
  {',', {-1, 0, 0, 0}},  // Lùi theo trục x
  {'j', {0, 1, 0, 0}},   // Ngang trái theo trục y
  {'l', {0, -1, 0, 0}},  // Ngang phải theo trục y
  {'u', {0, 0, 0, 1}},   // Xoay trái
  {'o', {0, 0, 0, -1}},  // Xoay phải
  {'k', {0, 0, 0, 0}},   // Dừng lại
};

// Map for speed keys
std::map<char, std::vector<float>> speedBindings
{
  {'q', {1.1, 1.1}},
  {'z', {0.9, 0.9}}
};

// Map for duration adjustment keys
std::map<char, float> durationBindings
{
  {'w', 1.0},  // Tăng thời gian
  {'x', -1.0}  // Giảm thời gian
};

const char* msg = R"(
---------------------------
Moving around:
   i    
 j  k  l
   ,    
u/o : rotate left/right
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease move duration by 1 sec
CTRL-C to quit
)";

float speed(1.0);
float turn(1.0);
float x(0), y(0), z(0), th(0);
float move_duration = 1.0; // Thời gian di chuyển mặc định
char key(' ');

// Biến lưu trữ vị trí hiện tại
float pos_x = 0.0, pos_y = 0.0, yaw = 0.0;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    pos_x = msg->pose.pose.position.x;
    pos_y = msg->pose.pose.position.y;
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
}

int getch(void)
{
    int ch;
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    ros::Subscriber sub = nh.subscribe("odom", 100, odomCallback);

    geometry_msgs::Twist twist;
    printf("%s", msg);
    printf("\rCurrent: speed %f m/s\tturn %f rad/s | Awaiting command...\r", speed, turn);

    while (true) {
        key = getch();
        if (moveBindings.count(key) == 1)
        {
            x = moveBindings[key][0];
            y = moveBindings[key][1];
            z = moveBindings[key][2];
            th = moveBindings[key][3];
            printf("\rCurrent: speed %f m/s\tturn %f rad/s | Last command: %c   ", speed, turn, key);
            
            twist.linear.x = x * speed;
            twist.linear.y = y * speed;
            twist.linear.z = 0;
            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = th * turn;
            
            pub.publish(twist);
            ros::Duration(move_duration).sleep(); // Dừng sau khoảng thời gian di chuyển
            twist.linear.x = 0;
            twist.linear.y = 0;
            twist.angular.z = 0;
            pub.publish(twist);
        }
        else if (speedBindings.count(key) == 1)
        {
            speed *= speedBindings[key][0];
            turn *= speedBindings[key][1];
            printf("\rCurrent: speed %f m/s\tturn %f rad/s | Last command: %c   ", speed, turn, key);
        }
        else if (durationBindings.count(key) == 1)
        {
            move_duration += durationBindings[key];
            if (move_duration < 1.0) move_duration = 1.0; // Đảm bảo ít nhất 1 giây
            printf("\rMove duration: %f sec | Last command: %c   ", move_duration, key);
        }
        else
        {
            x = 0;
            y = 0;
            z = 0;
            th = 0;
            if (key == '\x03') break;
            printf("\rCurrent: speed %f m/s\tturn %f rad/s| Invalid command! %c", speed, turn, key);
        }
        printf("\nPosition: x = %f, y = %f, yaw = %f", pos_x, pos_y, yaw);
        ros::spinOnce();
    }
    return 0;
}
