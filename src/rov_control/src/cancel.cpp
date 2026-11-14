#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rov_interfaces/action/collect_sediment.hpp"

using namespace std::chrono_literals;

class CancelCollectionClient : public rclcpp::Node
{
public:
  using Collect = rov_interfaces::action::CollectSediment;

  CancelCollectionClient() : Node("cancel_client")
  {
    client_ = rclcpp_action::create_client<Collect>(this, "collect_sediment");
  }

  void cancel()
  {
    if (!client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "Collection action server not available");
      return;
    }

    auto future = client_->async_cancel_all_goals();
    if (rclcpp::spin_until_future_complete(
        get_node_base_interface(), future, 5s) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Timed out waiting for cancel response");
      return;
    }

    RCLCPP_INFO(get_logger(), "Cancel-all request sent to collection action server");
  }

private:
  rclcpp_action::Client<Collect>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CancelCollectionClient>();
  node->cancel();
  rclcpp::shutdown();
  return 0;
}