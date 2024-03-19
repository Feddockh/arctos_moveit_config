#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>



class RealSenseToMoveItNode : public rclcpp::Node {

    public:
        RealSenseToMoveItNode() : Node("realsense_to_moveit_node"), tfBuffer(std::make_shared<rclcpp::Clock>()), tfListener(tfBuffer) {

            // Create a subscription to the point cloud topic that uses the point cloud callback function
            point_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/camera/camera/depth/color/points", 1, std::bind(&RealSenseToMoveItNode::pointCloudCallback, this, std::placeholders::_1));

            // Create a shared pointer to the octomap tree
            tree = std::make_shared<octomap::OcTree>(resolution);
        }

    private:

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub; // Shared pointer to point cloud topic
        std::shared_ptr<octomap::OcTree> tree; // Shared pointer to the octomap tree
        tf2_ros::Buffer tfBuffer; // Transform buffer
        tf2_ros::TransformListener tfListener; // Transform listener
        double maxRange = 1.0;
        double resolution = 0.05;

        // Callback function for point cloud subscriber
        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {

            // Start a timer
            auto start = std::chrono::high_resolution_clock::now();

            // Transform the point cloud from the camera frame to the base link frame so that we can maintain points as we move the camera around
            try {
                geometry_msgs::msg::TransformStamped transform;
                transform = tfBuffer.lookupTransform("base_link", cloud_msg->header.frame_id, cloud_msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
                sensor_msgs::msg::PointCloud2 transformed_cloud;
                tf2::doTransform(*cloud_msg, transformed_cloud, transform);

                // Convert the transformed point cloud to PCL format for processing
                pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::fromROSMsg(transformed_cloud, *pcl_cloud);

                // Get the sensor origin relative to the base link frame
                const auto& t = transform.transform.translation;
                octomap::point3d sensorOrigin(t.x, t.y, t.z);

                // Parallel processing
                // const size_t numThreads = 1;
                const size_t numThreads = std::thread::hardware_concurrency();
                RCLCPP_INFO(this->get_logger(), "numThreads: %ld", numThreads);

                // Create a vector to store thread-local octree pointers
                std::vector<std::unique_ptr<octomap::OcTree>> localTrees(numThreads);
                for (auto& tree : localTrees) {
                    tree = std::make_unique<octomap::OcTree>(resolution);
                }

                size_t chunkSize = pcl_cloud->size() / numThreads;
                std::vector<std::thread> threads;

                for (size_t i = 0; i < numThreads; ++i) {
                    size_t startIdx = i * chunkSize;
                    size_t endIdx = (i + 1 == numThreads) ? pcl_cloud->size() : (i + 1) * chunkSize;

                    threads.emplace_back([&, startIdx, endIdx, i] {
                        for (size_t j = startIdx; j < endIdx; ++j) {
                            const pcl::PointXYZ& point = (*pcl_cloud)[j];
                                processPoint(localTrees[i].get(), point, sensorOrigin);
                        }
                    });
                }

                for (auto& thread : threads) {
                    thread.join();
                }

                // Merge thread-local trees into the main tree
                // octomap::OcTree tree(0.05);
                for (auto& localTree : localTrees) {
                    for (octomap::OcTree::leaf_iterator it = localTree->begin_leafs(), end = localTree->end_leafs(); it != end; ++it) {
                        if (localTree->isNodeOccupied(*it)) {
                            tree->updateNode(it.getKey(), true, true);
                        } else {
                            tree->updateNode(it.getKey(), false, true);
                        }
                    }
                }

                // Update the OctoMap with the new point cloud data
                // for (const auto& point : *pcl_cloud) {
                //     // tree->updateNode(octomap::point3d(point.x, point.y, point.z), true, true); // Insert occupied points (with lazy eval)
                //     // processPoint(point, sensorOrigin);
                // }

                // Update the tree structure (required by lazy eval)
                tree->updateInnerOccupancy();

            } catch (const tf2::TransformException &e) {
                RCLCPP_ERROR(this->get_logger(), "Could not transform point cloud: %s", e.what());
                return;
            }

            // Compute process time
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            RCLCPP_INFO(this->get_logger(), "Processing time: %ld ms", duration.count());

            // Prepare and publish the updated OctoMap
            octomap_msgs::msg::Octomap octomap_msg;
            octomap_msg.header.frame_id = "base_link";
            octomap_msg.header.stamp = this->get_clock()->now();
            octomap_msgs::binaryMapToMsg(*tree, octomap_msg);

            // Update the /move_group topic using the planning scene interface
            moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
            moveit_msgs::msg::PlanningScene planning_scene_msg;
            planning_scene_msg.world.octomap.octomap = octomap_msg;
            planning_scene_msg.world.octomap.header.frame_id = "base_link";
            planning_scene_msg.is_diff = true;
            planning_scene_interface.applyPlanningScene(planning_scene_msg);
        }

        void processPoint(octomap::OcTree *tree, const pcl::PointXYZ &point, const octomap::point3d &sensorOrigin) {

            // Create the surface point for the ray
            octomap::point3d surfacePoint(point.x, point.y, point.z);

            // Compute the direction
            octomap::point3d direction = surfacePoint - sensorOrigin;
            direction.normalize();

            // Compute the end point of the ray by extending to the maximum range
            octomap::point3d endPoint = sensorOrigin + direction * maxRange;

            // If the point is past the max range create an endpoint within the max range
            if (sensorOrigin.distance(surfacePoint) > maxRange) {

                // Update all nodes along the ray up to the end point with the unoccupied status
                octomap::KeyRay keyRay;
                if (tree->computeRayKeys(sensorOrigin, endPoint, keyRay)) {
                    for (auto key : keyRay) {
                        tree->updateNode(key, false, true);
                    }
                }

                // Update the end point with the unoccupied status
                tree->updateNode(surfacePoint, false, true);

            } else {

                // Update all nodes along the ray up to the surface point with the unoccupied status
                octomap::KeyRay keyRay;
                if (tree->computeRayKeys(sensorOrigin, surfacePoint, keyRay)) {
                    for (auto key : keyRay) {
                        tree->updateNode(key, false, true);
                    }
                }

                // Update the surface point with the occupied status
                tree->updateNode(surfacePoint, true, true);
            }
        }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RealSenseToMoveItNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
