
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Simulator : public rclcpp::Node
{
public:

    Simulator() : Node("simulator")
    {
        // Receive pump command
        pump_sub_ = create_subscription<std_msgs::msg::Bool>("/rov/pump_cmd", 10,[this](std_msgs::msg::Bool::SharedPtr msg)
            {
                if (msg->data && !pump_on_)
                    volume_ = 0.0F;

                pump_on_ = msg->data;

                RCLCPP_INFO(get_logger(),"Pump: %s",pump_on_ ? "ON" : "OFF");
            });

        // Receive valve command
        valve_sub_ = create_subscription<std_msgs::msg::Bool>("/rov/valve_cmd", 10,[this](std_msgs::msg::Bool::SharedPtr msg)
            {
                valve_open_ = msg->data;
            });

        // Receive manipulator command
        arm_sub_ = create_subscription<std_msgs::msg::String>("/rov/arm_cmd", 10,[this](std_msgs::msg::String::SharedPtr msg)
            {
                arm_state_ = msg->data;
            });

        // Fault injection
        pressure_fault_sub_ =create_subscription<std_msgs::msg::Bool>("/sim/pressure_fault", 10,[this](std_msgs::msg::Bool::SharedPtr msg)
            {
                pressure_fault_ = msg->data;
            });

        sensor_fault_sub_ =
            create_subscription<std_msgs::msg::Bool>("/sim/sensor_fault", 10,[this](std_msgs::msg::Bool::SharedPtr msg)
            {
                sensor_fault_ = msg->data;
            });

        // Publishers
        pressure_pub_ = create_publisher<std_msgs::msg::Float32>("/rov/pressure", 10);

        volume_pub_ = create_publisher<std_msgs::msg::Float32>("/rov/sample_volume", 10);

        sensor_pub_ = create_publisher<std_msgs::msg::Bool>("/rov/sensor_ok", 10);

        pump_state_pub_ = create_publisher<std_msgs::msg::Bool>("/rov/pump_state", 10);

        valve_state_pub_ = create_publisher<std_msgs::msg::Bool>("/rov/valve_state", 10);

        arm_state_pub_ = create_publisher<std_msgs::msg::String>("/rov/arm_state", 10);

        // Update every 100 milliseconds
        timer_ = create_wall_timer(100ms,std::bind(&Simulator::update, this)
        );

        RCLCPP_INFO(get_logger(),"ROV Hardware Simulator Started");
    }

private:

    bool pump_on_{false};
    bool valve_open_{false};

    bool pressure_fault_{false};
    bool sensor_fault_{false};

    float volume_{0.0F};

    std::string arm_state_{"HOME"};

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pump_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr valve_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pressure_fault_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sensor_fault_sub_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pressure_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr volume_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sensor_pub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pump_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valve_state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arm_state_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    void update()
    {
        constexpr float dt = 0.1F;
        constexpr float rate = 15.0F;

        // Simulate sample collection
        if (pump_on_ && valve_open_ && !pressure_fault_ && !sensor_fault_)
        {
            volume_ += rate * dt;
        }

        // Pressure feedback
        std_msgs::msg::Float32 pressure;
        pressure.data = pressure_fault_ ? 3.0F : 1.0F;
        pressure_pub_->publish(pressure);

        // Sample volume
        std_msgs::msg::Float32 volume;
        volume.data = volume_;
        volume_pub_->publish(volume);

        // Sensor health
        std_msgs::msg::Bool sensor;
        sensor.data = !sensor_fault_;
        sensor_pub_->publish(sensor);

        // Actuator states
        std_msgs::msg::Bool pump;
        pump.data = pump_on_;
        pump_state_pub_->publish(pump);

        std_msgs::msg::Bool valve;
        valve.data = valve_open_;
        valve_state_pub_->publish(valve);

        std_msgs::msg::String arm;
        arm.data = arm_state_;
        arm_state_pub_->publish(arm);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Simulator>());

    rclcpp::shutdown();

    return 0;
}
