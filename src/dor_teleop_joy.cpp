#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>

// References
// https://github.com/ros2/teleop_twist_joy
// http://wiki.ros.org/joy/Tutorials

class TeleopJoy
{
public:
    TeleopJoy();

private:

    struct Axis
    {
        Axis():axis(0),scale(0.0),offset(0.0){}
        int axis;
        double scale;
        double offset;
    };

    struct Button
    {
        Button():button(0){}
        int button;
    };

    struct
    {
        Axis linear;
        Axis angular;
        Axis horizontal;
        Axis vertical;
    } axes;

    struct {
        Button stop;
        Button forward;
        Button backward;
    } buttons;
    
    void joyCallback (const sensor_msgs::Joy::ConstPtr& joy);
    void motorCallback (const std_msgs::Float32MultiArray::ConstPtr& status);
    double getAxis(const sensor_msgs::JoyConstPtr& joy, const Axis &axis);
    bool getButton(const sensor_msgs::JoyConstPtr& joy, const Axis &axis);

    ros::NodeHandle nh_;
    ros::Publisher vel_base_pub;
    ros::Publisher vel_X_pub;
    ros::Publisher vel_Y_pub;
    ros::Publisher vel_Z_pub;
    ros::Subscriber joy_sub;
    ros::Subscriber motor_sub;
};

TeleopJoy::TeleopJoy():
{
    ros::param::get("/teleop/linear_axis", axes.linear.axis);
    ros::param::get("/teleop/angular_axis", axes.angular.axis);
    ros::param::get("/teleop/vertical_axis", axes.vertical.axis);
    ros::param::get("/teleop/horizontal_axis", axes.horizontal.axis);
    ros::param::get("/teleop/stop_btn", buttons.stop.button);
    ros::param::get("/teleop/forward_btn", buttons.forward.button);
    ros::param::get("/teleop/backward_btn", buttons.backward.button);
    ros::param::get("/teleop/linear_scale", axes.linear.scale);
    ros::param::get("/teleop/angular_scale", axes.angular.scale);
    ros::param::get("/teleop/horizontal_scale", axes.horizontal.scale);
    ros::param::get("/teleop/vertical_scale", axes.vertical.scale);
    
    vel_base_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    vel_Z_pub = nh_.advertise<std_msgs::Int32>("/robo3_cmd", 1);
    vel_Y_pub = nh_.advertise<std_msgs::Int32>("/robo2_cmd", 1);
    vel_X_pub = nh_.advertise<std_msgs::Int32>("/robo1_cmd", 1);
    joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::joyCallback, this);
}

void TeleopJoy::motorCallback(const std_msgs::Float32MultiArray::ConstPtr& status)
{
    // std::cout <<"status" << std::endl;
}

void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

    if (getButton(joy, buttons.stop))
    {
        stop();
    }

    bool push = getButton(joy, buttons.forward);
    bool pull = getButton(joy, buttons.backward);
    if (push || pull)
    {
        std_msgs::Int32 msg;
        msg.data = push ? 1 : -1;
        vel_X_pub.publish(msg);
    }

    double h = getAxis(joy, axes.horizontal);
    if (std::abs(h) > 0)
    {
        std_msgs::Int32 msg;
        msg.data = (h > 0) ? 1 : -1;
        vel_Y_pub.publish(msg);
    }

    double v = getAxis(joy, axes.vertical);
    if (std::abs(v) > 0)
    {
        std_msgs::Int32 msg;
        msg.data = (v > 0) ? 1 : -1;
        vel_Z_pub.publish(msg);
    }

    double l = getAxis(joy, axes.linear);
    double a = getAxis(joy, axes.angular);
    if (std::abs(l) > 0 || std::abs(a) > 0)
    {
        geometry_msgs::Twist twist;
        twist.angular.z = a;
        twist.linear.x = l;
        vel_base_pub.publish(twist);
    }

}

double TeleopJoy::getAxis(const sensor_msgs::JoyConstPtr &joy, const Axis &axis)
{
    if (axis.axis < 0 || std::abs(axis.axis) > joy->axes.size())
    {
        ROS_ERROR_STREAM("Axis " << axis.axis << " out of range, joy has " << joy->axes.size() << " axes");
        return 0;
    }

    double output = joy->axes[std::abs(axis.axis)] * axis.factor + axis.offset;
    return output;

}

bool TeleopJoy::getButton(const sensor_msgs::JoyConstPtr &joy, const Button &button)
{
    if (button.button < 0 || button.button > joy->buttons.size())
    {
        ROS_ERROR_STREAM("Button " << button.button << " is out of range, joystick has " << joy->buttons.size() << " buttons");
        return false;
    }

    return joy->buttons[button.button];
}

void TeleopJoy::stop()
{
    std_mgs::Int32 stop;
    stop.data = 0;
    vel_X_pub.publish(stop);
    vel_Y_pub.publish(stop);
    vel_Z_pub.publish(stop);

}

int main(int argc, char** argv)
{
    ros::init (argc, argv, "teleop_dor");
    TeleopJoy teleop_dor;
    ros::spin();
    return 0;
}
