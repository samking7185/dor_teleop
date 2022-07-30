#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>

class TeleopJoy
{
public:
    TeleopJoy();

private:
    void joyCallback (const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;

    int linearBase, angularBase, linearXP, linearXN, linearY, linearZ;

    ros::Publisher vel_base_pub;
    ros::Publisher vel_X_pub;
    ros::Publisher vel_Y_pub;
    ros::Publisher vel_Z_pub;
    ros::Subscriber joy_sub;
};

TeleopJoy::TeleopJoy():
    linearBase(1),
    angularBase(0),
    linearXN(2),
    linearXP(5),
    linearY(3),
    linearZ(4)
{
    nh_.param("base_linear", linearBase, linearBase);
    nh_.param("base_angular", angularBase, angularBase);
    nh_.param("device_linear_x_p", linearXP, linearXP);
    nh_.param("device_linear_x_n", linearXN, linearXN);
    nh_.param("device_linear_y", linearY, linearY);
    nh_.param("device_linear_z", linearZ, linearZ);

    vel_base_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    vel_Z_pub = nh_.advertise<std_msgs::Int32>("/robo1_cmd", 1);
    vel_Y_pub = nh_.advertise<std_msgs::Int32>("/robo2_cmd", 1);
    vel_X_pub = nh_.advertise<std_msgs::Int32>("/robo3_cmd", 1);
    joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::joyCallback, this);
}

void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist twist;
    twist.angular.z = 1*joy->axes[angularBase];
    twist.linear.x = 1*joy->axes[linearBase];

    std_msgs::Int32 deviceX;
    std_msgs::Int32 deviceY;
    std_msgs::Int32 deviceZ;

    if ( joy->axes[linearZ] > 0.8 )
        deviceZ.data = 1;
    else if (joy->axes[linearZ] < -0.8)
        deviceZ.data = -1;
    else
        deviceZ.data = 0;

    if ( joy->axes[linearY] > 0.8 )
        deviceY.data = 1;
    else if (joy->axes[linearY] < -0.8)
        deviceY.data = -1;
    else
        deviceY.data = 0;

    if ( joy->axes[linearXP] < -0.8 )
        deviceX.data = 1;
    else if (joy->axes[linearXN] < -0.8)
        deviceX.data = -1;
    else if ( joy->axes[linearXP < -0.8 && joy->axes[linearXN] < -0.8])
        deviceX.data = 0;
    else
        deviceX.data = 0;

    vel_base_pub.publish(twist);
    vel_X_pub.publish(deviceX);
    vel_Y_pub.publish(deviceY);
    vel_Z_pub.publish(deviceZ);
}

int main(int argc, char** argv)
{
    ros::init (argc, argv, "teleop_dor");
    TeleopJoy teleop_dor;

    ros::spin();
}
