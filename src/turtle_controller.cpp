#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

class TurtleControllerNode : public rclcpp::Node // MODIFY NAME
{
public:
    TurtleControllerNode() : Node("turtle_controller"), turtlesim_up_(false) // MODIFY NAME
    {
        pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&TurtleControllerNode::callback_turtle_pose, this));
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        control_loo_timer_ = this.create_wall_timer(std::chrono::milliseconds(10),std::bind(&TurtleControllerNode::controlLoop, this));
    }

private:
    void callback_turtle_pose(const turtlesim::msg::Pose::SharedPtr pose)
    {
        pose = *pose.get();
        turtlesim_up_ = true;
    }

    void control_loop()
    {
        if (!turtlesim_up_)
            return;
    
        double x_dist = x_target - pose_.x;
        double y_dist = y_target - pose_.y;
        double target_dist = std::sqrt(x_dist * x_dist + y_dist * y_dist)

        auto msg = geometry_msgs::msg::Twist();

        if (target_dist > 0.5)
        {
            //position 
            msg.linear.x = 2 * target_dist;

            //ORIENTATION 
            double target_theta = std::atan2(y_dist, x_dist);
            double ang_diff = target_theta - pose_.theta;
            if(ang_diff > M_PI)
            {
                ang_diff -=  2 * M_PI;
            }
            else if(ang_diff < -M_PI)
            {
                ang_diff +=  2 * M_PI;
            }

        }
        else
        {
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
        }

        cmd_vel_publisher_->publish(msg)
    }

    double x_target;
    double y_target;
    turtlesim::msg::Pose pose_;
    bool turtlesim_up_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscriber<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
