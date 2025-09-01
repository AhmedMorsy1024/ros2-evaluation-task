#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "gazebo_utils_client.hpp"
#include <random>
#include <fstream>
#include <sstream>
#include <vector>
#include <memory>
#include <chrono>

/**
* @class ModelSpawnerNode
* @brief ROS2 node that manages spawning/deleting models in Gazebo
*/
class ModelSpawnerNode : public rclcpp::Node
{
public:
    ModelSpawnerNode() : Node("model_spawner_node"),
                        current_model_index_(0),
                        instance_counter_(0),
                        operation_in_progress_(false)
    {
        model_names_ = {
            "battery_9v_leader",
            "battery_energizer",
            "battery_varita",
            "lipo_battery"
        };

        loadModelSDFs();
        initializeRandomGenerators();
        RCLCPP_INFO(this->get_logger(), "ModelSpawnerNode initialized and ready");
    }

    void start()
    {
        auto node_ptr = this->rclcpp::Node::shared_from_this();
        gazebo_client_ = std::make_unique<GazeboUtilsClient>(node_ptr);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&ModelSpawnerNode::timerCallback, this)
        );

        RCLCPP_INFO(this->get_logger(), "ModelSpawnerNode started");
    }

private:
    void loadModelSDFs()
    {
        std::string package_path = ament_index_cpp::get_package_share_directory("ros2_eval_task");

        for (const auto& model_name : model_names_) {
            std::string sdf_path = package_path + "/models/" + model_name + "/model.sdf";

            std::ifstream file(sdf_path);
            if (file.is_open()) {
                std::stringstream buffer;
                buffer << file.rdbuf();
                model_sdfs_[model_name] = buffer.str();
                RCLCPP_INFO(this->get_logger(), "Loaded SDF for model: %s", model_name.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to load SDF for model: %s", model_name.c_str());
            }
        }
    }

    void initializeRandomGenerators()
    {
        random_engine_ = std::default_random_engine(
            std::chrono::system_clock::now().time_since_epoch().count()
        );
        x_distribution_ = std::uniform_real_distribution<double>(-0.21, 0.21);
        y_distribution_ = std::uniform_real_distribution<double>(-0.43, 0.43);
    }

    void timerCallback()
    {
        if (operation_in_progress_) {
            RCLCPP_WARN(this->get_logger(), "Previous operation still in progress, skipping this cycle");
            return;
        }

        std::string model_name = model_names_[current_model_index_];

        if (model_sdfs_.find(model_name) == model_sdfs_.end()) {
            RCLCPP_ERROR(this->get_logger(), "SDF not loaded for model: %s", model_name.c_str());
            current_model_index_ = (current_model_index_ + 1) % model_names_.size();
            return;
        }

        geometry_msgs::msg::Pose pose;
        pose.position.x = x_distribution_(random_engine_);
        pose.position.y = y_distribution_(random_engine_);
        pose.position.z = 1.1;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;

        operation_in_progress_ = true;

        RCLCPP_INFO(this->get_logger(), "Starting spawn cycle for: %s", model_name.c_str());

        std::string new_instance_name = model_name + "_instance_" + std::to_string(instance_counter_++);
        startSpawnSequence(model_name, new_instance_name, pose);
    }

    void startSpawnSequence(const std::string& model_base_name, const std::string& new_instance_name, const geometry_msgs::msg::Pose& pose)
    {
        auto sdf_xml = model_sdfs_[model_base_name];

        auto it = existing_model_instances_.find(model_base_name);
        if (it != existing_model_instances_.end()) {
            std::string old_instance_to_delete = it->second;
            RCLCPP_INFO(this->get_logger(), "Found existing model instance: %s, deleting it first", old_instance_to_delete.c_str());
            
            gazebo_client_->delete_model_async(old_instance_to_delete,
                [this, model_base_name, new_instance_name, pose, sdf_xml](GazeboUtilsClient::DeleteFuture future) {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "Successfully deleted existing model");
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Delete model failed: %s", response->status_message.c_str());
                    }
                    this->spawnNewModel(model_base_name, new_instance_name, sdf_xml, pose);
                }
            );
        } else {
            RCLCPP_INFO(this->get_logger(), "No existing model of type %s found, spawning directly", model_base_name.c_str());
            this->spawnNewModel(model_base_name, new_instance_name, sdf_xml, pose);
        }
    }

    void spawnNewModel(const std::string& model_base_name, const std::string& instance_name, const std::string& sdf_xml, const geometry_msgs::msg::Pose& pose)
    {
        gazebo_client_->spawn_model_async(instance_name, sdf_xml, pose,
            [this, model_base_name, instance_name, pose](GazeboUtilsClient::SpawnFuture future) {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(),
                        "Successfully spawned %s at position (%.3f, %.3f, %.3f)",
                        instance_name.c_str(), pose.position.x, pose.position.y, pose.position.z);

                    existing_model_instances_[model_base_name] = instance_name;
                    current_model_index_ = (current_model_index_ + 1) % model_names_.size();

                } else {
                    RCLCPP_ERROR(this->get_logger(),
                        "Failed to spawn model '%s': %s",
                        instance_name.c_str(), response->status_message.c_str());
                }

                operation_in_progress_ = false;
            }
        );
    }

    // Member variables
    std::unique_ptr<GazeboUtilsClient> gazebo_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::string> model_names_;
    std::map<std::string, std::string> model_sdfs_;
    std::map<std::string, std::string> existing_model_instances_;

    size_t current_model_index_;
    size_t instance_counter_;
    bool operation_in_progress_;

    std::default_random_engine random_engine_;
    std::uniform_real_distribution<double> x_distribution_;
    std::uniform_real_distribution<double> y_distribution_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ModelSpawnerNode>();
    node->start();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
