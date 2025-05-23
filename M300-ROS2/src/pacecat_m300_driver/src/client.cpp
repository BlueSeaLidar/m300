#include "rclcpp/rclcpp.hpp"
#include "pacecat_m300_inter/srv/control.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("my_node")
    {
        // 创建客户端
        control_client_ = this->create_client<pacecat_m300_inter::srv::Control>("control_service");

        // 使节点在一定时间内可以等待服务的出现
        while (!control_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
    }

    rclcpp::Client<pacecat_m300_inter::srv::Control>::SharedFuture send_request(const std::string &func)
    {
        // 创建请求对象
        auto request = std::make_shared<pacecat_m300_inter::srv::Control::Request>();
        request->func = func;

        // 异步发送请求并返回 SharedFuture
        return control_client_->async_send_request(request);
    }

private:
    rclcpp::Client<pacecat_m300_inter::srv::Control>::SharedPtr control_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();

    // 发送请求并获取 Future
    auto future = node->send_request("your_function");

    // 等待响应
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Response received.");
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to receive response.");
    }

    rclcpp::shutdown();
    return 0;
}
