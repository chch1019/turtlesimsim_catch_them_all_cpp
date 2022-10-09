#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include <string>


class TurtleSpawnerNode : public rclcpp::Node // MODIFY NAME
{
public:
    TurtleSpawnerNode() : Node("turtle_spawner") // MODIFY NAME
    {
        turtle_name_prefix_ = "turtle";
        spawn_turtle_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2000), std::bind(&TurtleSpawnerNode::spawn_new_turtle, this));
    
    }
    
private:
    double random_dNumber()
    {
        return double(std::rand()) / (double(RAND_MAX) + 1.0);
    }
    void spawn_new_turtle()
    {
        turtle_counter_ += 1;
        turtle_name_prefix_ = turtle_name_prefix_ + std::to_string(turtle_counter_);
        double x = random_dNumber() * 10;
        double y = random_dNumber() * 10;
        double theta = random_dNumber() * 2 * M_PI;
        
        spawn_turtle_threads_.push_back(std::make_shared<std::thread>(std::bind(&TurtleSpawnerNode::call_spawn_server, this, x, y, theta)));

    }

    void call_spawn_server(double x,double y,double theta)
        {
            auto client = this->create_client<turtlesim::srv::Spawn>("spawn");
            while (!client->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
            }

            auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
            request->x = x;
            request->y = y;
            request->theta = theta;
            

            auto future = client->async_send_request(request);

            try
            {
                auto response = future.get();
                if (response->name != "")
                {
                    RCLCPP_INFO(this->get_logger(), "Turtle %s is now alive.", response->name.c_str());
                }
                
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Service call failed");
            }
        }
    
    int turtle_counter_ = 0;
    std::string turtle_name_prefix_;
    rclcpp::TimerBase::SharedPtr spawn_turtle_timer_;
    std::vector<std::shared_ptr<std::thread>> spawn_turtle_threads_;
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawnerNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
