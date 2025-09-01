#include "gazebo_utils_client.hpp"

GazeboUtilsClient::GazeboUtilsClient(std::shared_ptr<rclcpp::Node> node)
    : node_(node), service_timeout_(5)
{
    spawn_client_ = node_->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
    delete_client_ = node_->create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");
    
    RCLCPP_INFO(node_->get_logger(), "GazeboUtilsClient initialized successfully");
}

bool GazeboUtilsClient::spawn_model(const std::string &model_name,
                                   const std::string &xml,
                                   const geometry_msgs::msg::Pose &pose)
{
    if (!spawn_client_->wait_for_service(service_timeout_)) {
        RCLCPP_ERROR(node_->get_logger(), 
                     "Spawn service not available after waiting %ld seconds", 
                     service_timeout_.count());
        return false;
    }

    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    request->name = model_name;
    request->xml = xml;
    request->robot_namespace = "";
    request->initial_pose = pose;
    request->reference_frame = "world";

    auto future = spawn_client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(node_, future, service_timeout_) == 
        rclcpp::FutureReturnCode::SUCCESS) 
    {
        auto response = future.get();
        
        if (response->success) {
            RCLCPP_INFO(node_->get_logger(), 
                       "Successfully spawned model '%s'", 
                       model_name.c_str());
            return true;
        } else {
            RCLCPP_ERROR(node_->get_logger(), 
                        "Failed to spawn model '%s': %s", 
                        model_name.c_str(), 
                        response->status_message.c_str());
            return false;
        }
    } else {
        RCLCPP_ERROR(node_->get_logger(), 
                    "Service call to spawn model '%s' timed out", 
                    model_name.c_str());
        return false;
    }
}

bool GazeboUtilsClient::delete_model(const std::string &model_name)
{
    if (!delete_client_->wait_for_service(service_timeout_)) {
        RCLCPP_ERROR(node_->get_logger(), 
                     "Delete service not available after waiting %ld seconds", 
                     service_timeout_.count());
        return false;
    }

    auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    request->name = model_name;

    auto future = delete_client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(node_, future, service_timeout_) == 
        rclcpp::FutureReturnCode::SUCCESS) 
    {
        auto response = future.get();
        
        if (response->success) {
            RCLCPP_INFO(node_->get_logger(), 
                       "Successfully deleted model '%s'", 
                       model_name.c_str());
            return true;
        } else {
            RCLCPP_WARN(node_->get_logger(), 
                       "Failed to delete model '%s': %s", 
                       model_name.c_str(), 
                       response->status_message.c_str());
            return false;
        }
    } else {
        RCLCPP_ERROR(node_->get_logger(), 
                    "Service call to delete model '%s' timed out", 
                    model_name.c_str());
        return false;
    }
}

void GazeboUtilsClient::spawn_model_async(const std::string &model_name,
                                          const std::string &xml,
                                          const geometry_msgs::msg::Pose &pose,
                                          SpawnCallback cb)
{
    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    request->name = model_name;
    request->xml = xml;
    request->robot_namespace = "";
    request->initial_pose = pose;
    request->reference_frame = "world";

    (void)spawn_client_->async_send_request(request, std::move(cb));
}

void GazeboUtilsClient::delete_model_async(const std::string &model_name,
                                           DeleteCallback cb)
{
    auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    request->name = model_name;

    (void)delete_client_->async_send_request(request, std::move(cb));
}
