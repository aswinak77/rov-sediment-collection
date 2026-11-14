#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

#include "rov_interface/action/collect_sediment.hpp"

using namespace std::chrono_literals;

class SedimentController : public rclcpp_lifecycle::LifecycleNode
{
public:
  using Collect = rov_interface::action::CollectSediment;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Collect>;
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  SedimentController()
  : LifecycleNode("controller")
  {
    // Commands sent to the ROV.
    pump_pub_ = create_publisher<std_msgs::msg::Bool>("/rov/pump_cmd", 10);
    valve_pub_ = create_publisher<std_msgs::msg::Bool>("/rov/valve_cmd", 10);
    arm_pub_ = create_publisher<std_msgs::msg::String>("/rov/arm_cmd", 10);

    // Sensor data received from the ROV.
    pressure_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/rov/pressure", 10,
      [this](std_msgs::msg::Float32::SharedPtr msg) {
        pressure_ = msg->data;
        pressure_received_ = true;
      });

    volume_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/rov/sample_volume", 10,
      [this](std_msgs::msg::Float32::SharedPtr msg) {
        volume_ = msg->data;
        volume_received_ = true;
      });

    sensor_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/rov/sensor_ok", 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        sensor_ok_ = msg->data;
        sensor_received_ = true;
      });

    // Action server: clients send collection requests here.
    action_server_ = rclcpp_action::create_server<Collect>(
      this,
      "collect_sediment",
      std::bind(&SedimentController::handle_goal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(&SedimentController::handle_cancel, this,
        std::placeholders::_1),
      std::bind(&SedimentController::handle_accepted, this,
        std::placeholders::_1));

    // Run one small step of the collection sequence every 100 ms.
    timer_ = create_wall_timer(
      100ms, std::bind(&SedimentController::run_step, this));

    RCLCPP_INFO(get_logger(), "Sediment controller created");
  }

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Controller configured");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    pump_pub_->on_activate();
    valve_pub_->on_activate();
    arm_pub_->on_activate();

    active_ = true;
    stop_collection("STOP");

    RCLCPP_INFO(get_logger(), "Controller activated");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    active_ = false;

    if (goal_handle_) {
      abort_goal("Controller deactivated");
    }

    stop_collection("STOP");

    pump_pub_->on_deactivate();
    valve_pub_->on_deactivate();
    arm_pub_->on_deactivate();

    RCLCPP_WARN(get_logger(), "Controller deactivated");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    active_ = false;
    RCLCPP_INFO(get_logger(), "Controller cleanup complete");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    active_ = false;

    if (goal_handle_) {
      abort_goal("Controller shutting down");
    }

    if (pump_pub_->is_activated()) {
      stop_collection("STOP");
      pump_pub_->on_deactivate();
      valve_pub_->on_deactivate();
      arm_pub_->on_deactivate();
    }

    RCLCPP_WARN(get_logger(), "Controller shutting down");
    return CallbackReturn::SUCCESS;
  }

private:
  enum class Step
  {
    IDLE,
    PRECHECK,
    POSITIONING,
    COLLECTING,
    RETRACTING
  };

  rclcpp_action::Server<Collect>::SharedPtr action_server_;

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr pump_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr valve_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr arm_pub_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pressure_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr volume_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sensor_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<GoalHandle> goal_handle_;

  bool active_{false};
  bool pressure_received_{false};
  bool volume_received_{false};
  bool sensor_received_{false};

  float pressure_{0.0F};
  float volume_{0.0F};
  bool sensor_ok_{false};

  Step step_{Step::IDLE};
  int step_count_{0};
  std::chrono::steady_clock::time_point collection_start_;

  void publish_bool(
    const rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr & pub,
    bool value)
  {
    std_msgs::msg::Bool msg;
    msg.data = value;
    pub->publish(msg);
  }

  void publish_arm(const std::string & command)
  {
    std_msgs::msg::String msg;
    msg.data = command;
    arm_pub_->publish(msg);
  }

  // Stop the pump and valve, and send the requested arm command.
  void stop_collection(const std::string & arm_command)
  {
    publish_bool(pump_pub_, false);
    publish_bool(valve_pub_, false);
    publish_arm(arm_command);
  }

  void send_feedback(const std::string & state, float percentage)
  {
    if (!goal_handle_) {
      return;
    }

    auto feedback = std::make_shared<Collect::Feedback>();
    feedback->current_state = state;
    feedback->progress_percentage = percentage;
    goal_handle_->publish_feedback(feedback);

    RCLCPP_INFO(get_logger(), "%s: %.1f%%", state.c_str(), percentage);
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Collect::Goal> goal)
  {
    if (!active_ || goal_handle_) {
      RCLCPP_WARN(get_logger(), "Rejecting goal: controller busy or inactive");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->collection_duration <= 0.0F || goal->target_volume <= 0.0F) {
      RCLCPP_WARN(get_logger(), "Rejecting goal: values must be greater than zero");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_WARN(get_logger(), "Cancellation requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal)
  {
    goal_handle_ = goal;
    step_ = Step::PRECHECK;
    step_count_ = 0;
  }

  void abort_goal(const std::string & reason)
  {
    if (!goal_handle_) {
      return;
    }

    stop_collection("STOP");

    auto result = std::make_shared<Collect::Result>();
    result->success = false;
    result->collected_volume = volume_;
    result->message = reason;

    goal_handle_->abort(result);
    RCLCPP_ERROR(get_logger(), "%s", reason.c_str());

    goal_handle_.reset();
    step_ = Step::IDLE;
  }

  void cancel_goal()
  {
    stop_collection("STOP");

    auto result = std::make_shared<Collect::Result>();
    result->success = false;
    result->collected_volume = volume_;
    result->message = "Collection canceled";

    goal_handle_->canceled(result);
    RCLCPP_WARN(get_logger(), "Collection canceled");

    goal_handle_.reset();
    step_ = Step::IDLE;
  }

  void run_step()
  {
    if (!goal_handle_) {
      return;
    }

    // Check cancellation and lifecycle state before doing more work.
    if (goal_handle_->is_canceling()) {
      cancel_goal();
      return;
    }

    if (!active_) {
      abort_goal("Controller is not active");
      return;
    }

    const auto goal = goal_handle_->get_goal();

    switch (step_) {
      case Step::PRECHECK:
        if (!pressure_received_ || !volume_received_ || !sensor_received_) {
          abort_goal("Waiting for sensor data failed");
          return;
        }

        if (!sensor_ok_) {
          abort_goal("Sensor health check failed");
          return;
        }

        if (pressure_ > 2.0F) {
          abort_goal("Pressure is above the safety limit");
          return;
        }

        send_feedback("POSITIONING", 20.0F);
        publish_arm("DEPLOY");
        step_count_ = 0;
        step_ = Step::POSITIONING;
        break;

      case Step::POSITIONING:
        ++step_count_;

        if (step_count_ >= 10) {
          send_feedback("COLLECTING", 35.0F);
          publish_bool(valve_pub_, true);
          publish_bool(pump_pub_, true);

          collection_start_ = std::chrono::steady_clock::now();
          step_ = Step::COLLECTING;
        }
        break;

      case Step::COLLECTING:
      {
        if (!pressure_received_ || !volume_received_ || !sensor_received_) {
          abort_goal("Sensor data is unavailable");
          return;
        }

        if (!sensor_ok_) {
          abort_goal("Sensor failure during collection");
          return;
        }

        if (pressure_ > 2.0F) {
          abort_goal("Pressure limit exceeded");
          return;
        }

        float progress = 35.0F +
          55.0F * volume_ / goal->target_volume;
        progress = std::min(progress, 90.0F);
        send_feedback("COLLECTING", progress);

        const auto elapsed =
          std::chrono::steady_clock::now() - collection_start_;

        if (volume_ >= goal->target_volume) {
          // Target volume reached.
          publish_bool(pump_pub_, false);
          publish_bool(valve_pub_, false);
          publish_arm("HOME");

          step_count_ = 0;
          step_ = Step::RETRACTING;
        } else if (elapsed >= std::chrono::duration<float>(
                     goal->collection_duration)) {
          abort_goal("Collection timed out before reaching target volume");
          return;
        }
        break;
      }

      case Step::RETRACTING:
        ++step_count_;

        if (step_count_ >= 5) {
          auto result = std::make_shared<Collect::Result>();
          result->success = true;
          result->collected_volume = volume_;
          result->message = "Sediment collection completed";

          send_feedback("SUCCESS", 100.0F);
          goal_handle_->succeed(result);

          RCLCPP_INFO(
            get_logger(), "Collection completed: %.2f ml", volume_);

          goal_handle_.reset();
          step_ = Step::IDLE;
        }
        break;

      case Step::IDLE:
        break;
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SedimentController>();

  // A single-threaded executor keeps this beginner-friendly state machine
  // and its sensor callbacks running one at a time.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}