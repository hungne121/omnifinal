#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <map>

// Map for controlling arm joints
std::map<char, float> armBindings
{
    {'e', 1.0},  // Increase arm_link1 joint angle
    {'c', -1.0}, // Decrease arm_link1 joint angle
    {'r', 1.0},  // Increase arm_link2 joint angle
    {'v', -1.0}  // Decrease arm_link2 joint angle
};

const char* msg = R"(
---------------------------
Control the arm:
e/c : increase/decrease arm_link1 joint angle
r/v : increase/decrease arm_link2 joint angle
CTRL-C to quit
)";

float arm1_velocity = 0.0;
float arm2_velocity = 0.0;
char key(' ');

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
    ros::init(argc, argv, "keyboard_arm_control");
    ros::NodeHandle nh;

    // Publishers for arm_link1 and arm_link2 joint velocities
    ros::Publisher arm1_pub = nh.advertise<std_msgs::Float64>("/arm_link1_controller/command", 10);
    ros::Publisher arm2_pub = nh.advertise<std_msgs::Float64>("/arm_link2_controller/command", 10);

    std_msgs::Float64 arm1_cmd;
    std_msgs::Float64 arm2_cmd;

    printf("%s", msg);

    while (ros::ok())
    {
        key = getch();
        if (armBindings.count(key) == 1)
        {
            if (key == 'e' || key == 'c')
            {
                arm1_velocity = armBindings[key];
                arm1_cmd.data = arm1_velocity;
                arm1_pub.publish(arm1_cmd);
                printf("\rArm Link 1 Velocity: %f | Last command: %c   ", arm1_velocity, key);
            }
            else if (key == 'r' || key == 'v')
            {
                arm2_velocity = armBindings[key];
                arm2_cmd.data = arm2_velocity;
                arm2_pub.publish(arm2_cmd);
                printf("\rArm Link 2 Velocity: %f | Last command: %c   ", arm2_velocity, key);
            }
        }
        else
        {
            if (key == '\x03') // CTRL-C to quit
                break;
            printf("\rInvalid command! %c", key);
        }
        ros::spinOnce();
    }

    return 0;
}