#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class RealSenseToMoveItNode : public rclcpp::Node {
public:
    RealSenseToMoveItNode() : Node("realsense_to_moveit_node") {
        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points", 1, std::bind(&RealSenseToMoveItNode::pointCloudCallback, this, std::placeholders::_1));
        
        // Changed from planning_scene to move_group, ensure the message type is appropriate
        // move_group_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("move_group", 1);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
        // Convert ROS point cloud to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        // Process the point cloud (e.g., filter it)
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 2.0);  // Filter out points further than 2 meters
        pass.filter(*cloud);

        // Create an OctoMap and insert the point cloud
        octomap::OcTree tree(0.005);  // 0.01 meters resolution
        for (auto& point : *cloud) {
            tree.updateNode(octomap::point3d(point.x, point.y, point.z), true);
        }
        tree.updateInnerOccupancy();

        // Prepare the OctoMap message
        octomap_msgs::msg::Octomap octomap_msg;
        octomap_msg.header.frame_id = cloud_msg->header.frame_id;
        octomap_msg.header.stamp = this->get_clock()->now();

        // Fill the OctoMap message
        octomap_msgs::binaryMapToMsg(tree, octomap_msg);

        // // Create a PlanningScene message
        // moveit_msgs::msg::PlanningScene planning_scene_msg;
        // planning_scene_msg.world.octomap.octomap = octomap_msg;
        // planning_scene_msg.world.octomap.header = octomap_msg.header;

        // // Publish the message, here to the move_group topic
        // move_group_pub_->publish(planning_scene_msg);

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        moveit_msgs::msg::PlanningScene planning_scene_msg;
        planning_scene_msg.world.octomap.octomap = octomap_msg;
        planning_scene_msg.world.octomap.header.frame_id = cloud_msg->header.frame_id;
        planning_scene_msg.is_diff = true;

        planning_scene_interface.applyPlanningScene(planning_scene_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    // rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr move_group_pub_; // Changed publisher's name and topic
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RealSenseToMoveItNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
