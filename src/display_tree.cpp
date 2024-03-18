// based on: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html

#include "rclcpp/rclcpp.hpp"
#include "simple_random_trees/tree.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

double deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}

class DisplayTree : public rclcpp::Node
{
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        for (const auto &param : parameters)
        {
            if (param.get_name() == "max_deg")
            {
                max_deg = param.as_double();
            }
            else if (param.get_name() == "max_dist")
            {
                max_dist = param.as_double();
            }
            else if (param.get_name() == "seed_size")
            {
                seed_size = param.as_int();
            }
            else
            {
                result.successful = false;
                result.reason = "Invalid parameter";
            }
        }
        return result;
    }

public:
    DisplayTree() : Node("display_tree"), count_(0)
    {
        this->declare_parameter("max_deg", max_deg);
        this->declare_parameter("max_dist", max_dist);
        this->declare_parameter("seed_size", seed_size);
        this->get_parameter("max_deg", max_deg);
        this->get_parameter("max_dist", max_dist);
        this->get_parameter("seed_size", seed_size);

        RCLCPP_INFO(this->get_logger(), "Starting display_tree");
        callback_handle_ = this->add_on_set_parameters_callback(std::bind(&DisplayTree::parametersCallback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("path_marker_topic", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&DisplayTree::timer_callback, this));
    }

private:
    std::vector<PointXYori> generateNewNode(PointXYori p, double max_deg, double max_dist)
    {
        std::vector<PointXYori> new_nodes;
        double random_num = (double)rand() / (RAND_MAX);
        double random_ori1 = (double)rand() / (RAND_MAX / max_deg);
        double random_dist1 = 1.0 + (double)rand() / (RAND_MAX / (max_dist - 1.0));
        if (random_num > 0.5)
        {
            double random_ori2 = (double)rand() / (RAND_MAX / max_deg) + max_deg / 2.0;
            random_ori1 = (double)rand() / (RAND_MAX / max_deg) - max_deg / 2.0;
            double random_dist2 = 1.0 + (double)rand() / (RAND_MAX / (max_dist - 1.0));
            PointXYori p2 = PointXYori(p.x + random_dist2 * cos(deg2rad(random_ori2)), p.y + random_dist2 * sin(deg2rad(random_ori2)), p.ori + random_ori2);
            new_nodes.push_back(p2);
        }
        PointXYori p1 = PointXYori(p.x + random_dist1 * cos(deg2rad(random_ori1)), p.y + random_dist1 * sin(deg2rad(random_ori1)), p.ori + random_ori1);
        new_nodes.push_back(p1);
        return new_nodes;
    }

    void timer_callback()
    {
        srand(time(NULL)); //initialize random
        auto message = visualization_msgs::msg::MarkerArray();
        visualization_msgs::msg::Marker line1_marker, text1_marker, node1_marker;
        line1_marker.ns = "line1";
        line1_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        line1_marker.action = visualization_msgs::msg::Marker::MODIFY;
        line1_marker.scale.x = 0.1; line1_marker.color.r = 0.2; line1_marker.color.g = 0.6; line1_marker.color.b = 0.8; line1_marker.color.a = 1.0; line1_marker.id = 1; 
        line1_marker.pose.position.x = 0.0; line1_marker.pose.position.y = 0.0; line1_marker.pose.position.z = 0.0; line1_marker.header.frame_id = "map";
        text1_marker.ns = "text1";
        text1_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text1_marker.action = visualization_msgs::msg::Marker::MODIFY;
        text1_marker.scale.x = 1.0; text1_marker.color.r = 1.0; text1_marker.color.g = 0.7; text1_marker.color.b = 0.05; text1_marker.color.a = 1.0; text1_marker.id = 2; text1_marker.scale.y = 1.0; text1_marker.scale.z = 1.0;
        text1_marker.pose.position.x = 0.0; text1_marker.pose.position.y = 0.0; text1_marker.pose.position.z = 0.0; text1_marker.header.frame_id = "map";
        node1_marker.ns = "node1";
        node1_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        node1_marker.action = visualization_msgs::msg::Marker::MODIFY;
        node1_marker.scale.x = 0.2; node1_marker.color.r = 0.96; node1_marker.color.g = 0.26; node1_marker.color.b = 0.21; node1_marker.color.a = 1.0; node1_marker.id = 3; node1_marker.scale.y = 0.2; node1_marker.scale.y = 0.2; 
        node1_marker.pose.position.x = 0.0; node1_marker.pose.position.y = 0.0; node1_marker.pose.position.z = 0.0; node1_marker.header.frame_id = "map";
        Tree tree;
        tree.modifyRoot(PointXYori(0.0, 0.5, 0.0));
        auto node_actual = tree.getRoot();
        if (seed_size > 20)
        {
            seed_size = 20;
            RCLCPP_WARN_STREAM(this->get_logger(), "Seed size too large, setting to 20");
        }
        for (int i = 0; i < seed_size; i++)
        {
            auto traj_end_points = tree.getLeaves(node_actual);
            RCLCPP_INFO_STREAM(this->get_logger(), "Number of leaves: " << traj_end_points.size() << " at iteration " << i);
            for (size_t t = 0; t < traj_end_points.size(); t++)
            {
                auto new_nodes = generateNewNode(traj_end_points[t]->pos, max_deg, 1.5);
                for (size_t j = 0; j < new_nodes.size(); j++)
                {
                    tree.addNode(traj_end_points[t], new_nodes[j]);
                }
            }
        }
        // tree.printTree(tree.getRoot());
        // RCLCPP_INFO_STREAM(this->get_logger(), "Tree size: " << tree.getSize(tree.getRoot()));
        text1_marker.text = "Tree size: " + std::to_string(tree.getSize(tree.getRoot()));

        tree.markerTree(tree.root, line1_marker);     // add tree to line marker, which can be visualized in rviz
        tree.markerNodeTree(tree.root, node1_marker); // add nodes to node marker, which can be visualized in rviz
        message.markers.push_back(text1_marker);
        message.markers.push_back(line1_marker);
        message.markers.push_back(node1_marker);
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    double max_deg = 45.0;
    double max_dist = 3.0;
    int seed_size = 8;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DisplayTree>());
    rclcpp::shutdown();
    return 0;
}