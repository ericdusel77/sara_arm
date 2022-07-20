#include "kinova_driver/kinova_api.h"
#include <ros/ros.h>
#include "std_msgs/Float64.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joystick_pub");
    ros::NodeHandle nh;

    ros::Publisher joy_pub = nh.advertise<std_msgs::Float64>("joystick_command", 10);

    kinova::KinovaAPI kinova_api_;
    kinova_api_.initializeKinovaAPIFunctions(kinova::KinovaAPIType::USB);
    int result;

    result = kinova_api_.startControlAPI();

    while (ros::ok())
    {

        JoystickCommand j;
        j.InitStruct();

        result = kinova_api_.getJoystickValue(j);

        std_msgs::Float64 b_val;

        b_val.data = j.MoveLeftRight;
        joy_pub.publish(b_val);

    }

    return 0;
}
