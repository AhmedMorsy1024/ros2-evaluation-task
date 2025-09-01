#ifndef GAZEBO_UTILS_CLIENT_HPP
#define GAZEBO_UTILS_CLIENT_HPP

#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <gazebo_msgs/srv/delete_entity.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <memory>
#include <chrono>
#include <functional>

/**
 * @class GazeboUtilsClient
 * @brief A utility class for interacting with Gazebo simulation services
 */
class GazeboUtilsClient
{
public:
    explicit GazeboUtilsClient(std::shared_ptr<rclcpp::Node> node);

    bool spawn_model(const std::string &model_name,
                    const std::string &xml,
                    const geometry_msgs::msg::Pose &pose);

    bool delete_model(const std::string &model_name);

    // Async interfaces (non-blocking)
    using SpawnFuture = rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedFuture;
    using DeleteFuture = rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedFuture;
    using SpawnCallback = std::function<void(SpawnFuture)>;
    using DeleteCallback = std::function<void(DeleteFuture)>;

    void spawn_model_async(const std::string &model_name,
                           const std::string &xml,
                           const geometry_msgs::msg::Pose &pose,
                           SpawnCallback cb);

    void delete_model_async(const std::string &model_name,
                            DeleteCallback cb);

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_client_;
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_client_;
    std::chrono::seconds service_timeout_;
};

#endif // GAZEBO_UTILS_CLIENT_HPP
