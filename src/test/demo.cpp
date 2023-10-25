#include <iostream>
#include <vector>
#include <tuple>
#include <unistd.h>
#include <sys/stat.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav_msgs/msg/occupancy_grid.h>
#include <geometry_msgs/msg/point_stamped.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <tf/transform_datatypes.h>
// #include <tf/transform_broadcaster.h>
// #include <tf/tf.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

#include <ros2_viz_tools/ros2_viz_tools.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include "glog/logging.h"
#include "eigen3/Eigen/Dense"
#include "opencv2/core/core.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/opencv.hpp"
#include "path_optimizer_2/path_optimizer.hpp"
#include "tools/eigen2cv.hpp"
#include "data_struct/data_struct.hpp"
#include "tools/tools.hpp"
#include "data_struct/reference_path.hpp"
#include "tools/spline.h"
#include "config/planning_flags.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

class PlannerDemo : public rclcpp::Node{
    public:
        PlannerDemo() : Node("planner_demo"){
            // Initialize grid map from image.
            std::string image_dir = ament_index_cpp::get_package_share_directory("path_optimizer_2");

            std::string image_file = "gridmap.png";
            image_dir.append("/config/" + image_file);
            std::cout<< "map path: " << image_dir << std::endl;
            cv::Mat img_src = cv::imread(image_dir, CV_8UC1);
            double resolution = 0.2;  // in meter
            grid_map = std::make_shared<grid_map::GridMap>(std::vector<std::string>{"obstacle", "distance"});
            grid_map::GridMapCvConverter::initializeFromImage(img_src, resolution, *grid_map, grid_map::Position::Zero());
           
            grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(
                img_src, "obstacle", *grid_map, OCCUPY, FREE, 0.5);
            // Update distance layer.
            Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> binary =
                grid_map->get("obstacle").cast<unsigned char>();
            cv::distanceTransform(eigen2cv(binary), eigen2cv(grid_map->get("distance")),
                                CV_DIST_L2, CV_DIST_MASK_PRECISE);
            grid_map->get("distance") *= resolution;
            grid_map->setFrameId("/map");
        
            //  cv::imwrite("/home//map1.png", eigen2cv(grid_map.get("obstacle")));
            map_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/grid_map", 1);
            reference_sub = this->create_subscription<geometry_msgs::msg::PointStamped>
                                ("/clicked_point", 1, std::bind(&PlannerDemo::referenceCb, this, _1));
            start_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>
                                ("/initialpose", 1, std::bind(&PlannerDemo::startCb, this, _1));
            end_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>
                                ("/goal_pose", 1, std::bind(&PlannerDemo::goalCb, this, _1));
            timer_ = this->create_wall_timer(1000ms, std::bind(&PlannerDemo::initialization, this));

        }

        ~PlannerDemo(){}
    private:

        void initialization() {
            markers = std::make_shared<ros2_viz_tools::RosVizTools>(shared_from_this(), "markers");
            // replace timer with loop callback after initialization finished
            timer_ = this->create_wall_timer(300ms, std::bind(&PlannerDemo::timer_callback, this)); 
        }

        void timer_callback()
        {

            rclcpp::Time time = this->get_clock()->now();
            markers->clear();
            int id = 0;
            // Cancel at double click.
            if (reference_path_plot.size() >= 2) {
                const auto &p1 = reference_path_plot[reference_path_plot.size() - 2];
                const auto &p2 = reference_path_plot.back();
                if (distance(p1, p2) <= 0.001) {
                    reference_path_plot.clear();
                    reference_rcv = false;
                }
            }

            // Visualize reference path selected by mouse.
            visualization_msgs::msg::Marker reference_marker =
                markers->newSphereList(0.5, "reference point", id++, ros2_viz_tools::RED, marker_frame_id);
            for (size_t i = 0; i != reference_path_plot.size(); ++i) {
                geometry_msgs::msg::Point p;
                p.x = reference_path_plot[i].x;
                p.y = reference_path_plot[i].y;
                p.z = 1.0;
                reference_marker.points.push_back(p);
            }
            markers->append(reference_marker);
            // Visualize start and end point selected by mouse.
            geometry_msgs::msg::Vector3 scale;
            scale.x = 2.0;
            scale.y = 0.3;
            scale.z = 0.3;
            geometry_msgs::msg::Pose start_pose;
            start_pose.position.x = start_state.x;
            start_pose.position.y = start_state.y;
            start_pose.position.z = 1.0;
            geometry_msgs::msg::Quaternion start_quat = createQuaternionMsgFromYaw(start_state.heading);
            start_pose.orientation.x = start_quat.x;
            start_pose.orientation.y = start_quat.y;
            start_pose.orientation.z = start_quat.z;
            start_pose.orientation.w = start_quat.w;
            visualization_msgs::msg::Marker start_marker =
                markers->newArrow(scale, start_pose, "start point", id++, ros2_viz_tools::CYAN, marker_frame_id);
            markers->append(start_marker);
            geometry_msgs::msg::Pose end_pose;
            end_pose.position.x = end_state.x;
            end_pose.position.y = end_state.y;
            end_pose.position.z = 1.0;
            auto end_quat = createQuaternionMsgFromYaw(end_state.heading);
            end_pose.orientation.x = end_quat.x;
            end_pose.orientation.y = end_quat.y;
            end_pose.orientation.z = end_quat.z;
            end_pose.orientation.w = end_quat.w;
            visualization_msgs::msg::Marker end_marker =
                markers->newArrow(scale, end_pose, "end point", id++, ros2_viz_tools::CYAN, marker_frame_id);
            markers->append(end_marker);

            // Calculate.
            std::vector<PathOptimizationNS::SlState> result_path;
            std::vector<PathOptimizationNS::State> smoothed_reference_path, result_path_by_boxes;
            std::vector<std::vector<double>> a_star_display(3);
            bool opt_ok = false;
            
            if (reference_rcv && start_state_rcv && end_state_rcv) {
               
                PathOptimizationNS::PathOptimizer path_optimizer(start_state, end_state, *grid_map);
                opt_ok = path_optimizer.solve(reference_path_plot, &result_path);
                reference_path_opt = path_optimizer.getReferencePath();
                smoothed_reference_path.clear();
                if (!PathOptimizationNS::isEqual(reference_path_opt.getLength(), 0.0)) {
                    double s = 0.0;
                    while (s < reference_path_opt.getLength()) {
                        smoothed_reference_path.emplace_back(reference_path_opt.getXS()(s), reference_path_opt.getYS()(s));
                        s += 0.5;
                    }
                }
                if (opt_ok) {
                    std::cout << "ok!" << std::endl;
                }
            }

            // Visualize result path.
            ros2_viz_tools::ColorRGBA path_color;
            path_color.r = 0.063;
            path_color.g = 0.305;
            path_color.b = 0.545;
            if (!opt_ok) {
                path_color.r = 1.0;
                path_color.g = 0.0;
                path_color.b = 0.0;
            }
            visualization_msgs::msg::Marker result_marker =
                markers->newLineStrip(0.5, "optimized path", id++, path_color, marker_frame_id);
            for (size_t i = 0; i != result_path.size(); ++i) {
                geometry_msgs::msg::Point p;
                p.x = result_path[i].x;
                p.y = result_path[i].y;
                p.z = 1.0;
                result_marker.points.push_back(p);
                const auto k = result_path[i].k;
                path_color.a = std::min(fabs(k) / 0.15, 1.0);
                path_color.a = std::max((float)0.1, path_color.a);
                result_marker.colors.emplace_back(path_color);
            }
            markers->append(result_marker);

            // Visualize result path.
            visualization_msgs::msg::Marker result_boxes_marker =
                markers->newLineStrip(0.15, "optimized path by boxes", id++, ros2_viz_tools::BLACK, marker_frame_id);
            for (size_t i = 0; i != result_path_by_boxes.size(); ++i) {
                geometry_msgs::msg::Point p;
                p.x = result_path_by_boxes[i].x;
                p.y = result_path_by_boxes[i].y;
                p.z = 1.0;
                result_boxes_marker.points.push_back(p);
            }
            markers->append(result_boxes_marker);

            // Visualize smoothed reference path.
            visualization_msgs::msg::Marker smoothed_reference_marker =
                markers->newLineStrip(0.07,
                                    "smoothed reference path",
                                    id++,
                                    ros2_viz_tools::YELLOW,
                                    marker_frame_id);
            for (size_t i = 0; i != smoothed_reference_path.size(); ++i) {
                geometry_msgs::msg::Point p;
                p.x = smoothed_reference_path[i].x;
                p.y = smoothed_reference_path[i].y;
                p.z = 1.0;
                smoothed_reference_marker.points.push_back(p);
            }
            markers->append(smoothed_reference_marker);
            ros2_viz_tools::ColorRGBA vehicle_color = ros2_viz_tools::GRAY;
            vehicle_color.a = 0.5;
            visualization_msgs::msg::Marker vehicle_geometry_marker =
                markers->newLineList(0.03, "vehicle", id++, vehicle_color, marker_frame_id);
            // Visualize vehicle geometry.
            static const double length{FLAGS_car_length};
            static const double width{FLAGS_car_width};
            static const double rear_d{FLAGS_rear_length};
            static const double front_d{FLAGS_front_length};
            for (size_t i = 0; i != result_path.size(); ++i) {
                double heading = result_path[i].heading;
                PathOptimizationNS::State p1, p2, p3, p4;
                p1.x = front_d;
                p1.y = width / 2;
                p2.x = front_d;
                p2.y = -width / 2;
                p3.x = rear_d;
                p3.y = -width / 2;
                p4.x = rear_d;
                p4.y = width / 2;
                auto tmp_relto = result_path[i];
                tmp_relto.heading = heading;
                p1 = PathOptimizationNS::local2Global(tmp_relto, p1);
                p2 = PathOptimizationNS::local2Global(tmp_relto, p2);
                p3 = PathOptimizationNS::local2Global(tmp_relto, p3);
                p4 = PathOptimizationNS::local2Global(tmp_relto, p4);
                geometry_msgs::msg::Point pp1, pp2, pp3, pp4;
                pp1.x = p1.x;
                pp1.y = p1.y;
                pp1.z = 0.1;
                pp2.x = p2.x;
                pp2.y = p2.y;
                pp2.z = 0.1;
                pp3.x = p3.x;
                pp3.y = p3.y;
                pp3.z = 0.1;
                pp4.x = p4.x;
                pp4.y = p4.y;
                pp4.z = 0.1;
                vehicle_geometry_marker.points.push_back(pp1);
                vehicle_geometry_marker.points.push_back(pp2);
                vehicle_geometry_marker.points.push_back(pp2);
                vehicle_geometry_marker.points.push_back(pp3);
                vehicle_geometry_marker.points.push_back(pp3);
                vehicle_geometry_marker.points.push_back(pp4);
                vehicle_geometry_marker.points.push_back(pp4);
                vehicle_geometry_marker.points.push_back(pp1);
            }
            markers->append(vehicle_geometry_marker);
            visualization_msgs::msg::Marker block_state_marker =
            markers->newSphereList(0.45, "block state", id++, ros2_viz_tools::PINK, marker_frame_id);
            static std::vector<double> len_vec{FLAGS_rear_length, 0.0, FLAGS_front_length};
            auto block_ptr = reference_path_opt.isBlocked();
            if (block_ptr) {
                geometry_msgs::msg::Point p;
                p.x = block_ptr->front.x;
                p.y = block_ptr->front.y;
                p.z = 1.0;
                block_state_marker.points.emplace_back(p);
                p.x = block_ptr->rear.x;
                p.y = block_ptr->rear.y;
                block_state_marker.points.emplace_back(p);
            }
            markers->append(block_state_marker);

            // Plot bounds.
            visualization_msgs::msg::Marker front_bounds_marker =
                markers->newSphereList(0.25, "front bounds", id++, ros2_viz_tools::LIGHT_BLUE, marker_frame_id);
            for (const auto &bound : reference_path_opt.getBounds()) {
                const auto &front_bound = bound.front;
                geometry_msgs::msg::Point p;
                p.x = front_bound.x + front_bound.lb * cos(front_bound.heading + M_PI_2);
                p.y = front_bound.y + front_bound.lb * sin(front_bound.heading + M_PI_2);
                p.z = 1.0;
                front_bounds_marker.points.emplace_back(p);
                p.x = front_bound.x + front_bound.ub * cos(front_bound.heading + M_PI_2);
                p.y = front_bound.y + front_bound.ub * sin(front_bound.heading + M_PI_2);
                front_bounds_marker.points.emplace_back(p);
            }
            markers->append(front_bounds_marker);

            visualization_msgs::msg::Marker rear_bounds_marker =
                markers->newSphereList(0.25, "rear bounds", id++, ros2_viz_tools::LIME_GREEN, marker_frame_id);
            for (const auto &bound : reference_path_opt.getBounds()) {
                const auto &rear_bound = bound.rear;
                geometry_msgs::msg::Point p;
                p.x = rear_bound.x + rear_bound.lb * cos(rear_bound.heading + M_PI_2);
                p.y = rear_bound.y + rear_bound.lb * sin(rear_bound.heading + M_PI_2);
                p.z = 1.0;
                rear_bounds_marker.points.emplace_back(p);
                p.x = rear_bound.x + rear_bound.ub * cos(rear_bound.heading + M_PI_2);
                p.y = rear_bound.y + rear_bound.ub * sin(rear_bound.heading + M_PI_2);
                rear_bounds_marker.points.emplace_back(p);
            }
            markers->append(rear_bounds_marker);

            visualization_msgs::msg::Marker center_bounds_marker =
                markers->newSphereList(0.25, "center bounds", id++, ros2_viz_tools::CYAN, marker_frame_id);
            for (const auto &bound : reference_path_opt.getBounds()) {
                const auto &center_bounds = bound.center;
                geometry_msgs::msg::Point p;
                p.x = center_bounds.x + center_bounds.lb * cos(center_bounds.heading + M_PI_2);
                p.y = center_bounds.y + center_bounds.lb * sin(center_bounds.heading + M_PI_2);
                p.z = 1.0;
                center_bounds_marker.points.emplace_back(p);
                p.x = center_bounds.x + center_bounds.ub * cos(center_bounds.heading + M_PI_2);
                p.y = center_bounds.y + center_bounds.ub * sin(center_bounds.heading + M_PI_2);
                center_bounds_marker.points.emplace_back(p);
            }
            markers->append(center_bounds_marker);

            // Publish the grid_map.
            grid_map->setTimestamp(time.nanoseconds());
            nav_msgs::msg::OccupancyGrid message;
            grid_map::GridMapRosConverter::toOccupancyGrid(*grid_map, "obstacle", FREE, OCCUPY, message);
            map_publisher->publish(message);
            // Publish markers->
            markers->publish();
        }

        geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw){
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            return tf2::toMsg(q);
        }

        void referenceCb(const geometry_msgs::msg::PointStamped::ConstSharedPtr &p) {
            if (start_state_rcv && end_state_rcv) {
                reference_path_plot.clear();
            }
            PathOptimizationNS::State reference_point;
            reference_point.x = p->point.x;
            reference_point.y = p->point.y;
            reference_path_plot.emplace_back(reference_point);
            start_state_rcv = end_state_rcv = false;
            reference_rcv = reference_path_plot.size() >= 6;
            std::cout << "received a reference point" << std::endl;
        }

        void startCb(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &start) {
            start_state.x = start->pose.pose.position.x;
            start_state.y = start->pose.pose.position.y;
            tf2::Quaternion tf2_quat;
            tf2::fromMsg(start->pose.pose.orientation, tf2_quat);
            tf2::Matrix3x3 m(tf2_quat);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            start_state.heading = yaw;
            if (reference_rcv) {
                start_state_rcv = true;
            }
            std::cout << "get initial state." << std::endl;
        }

        void goalCb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &goal) {
            end_state.x = goal->pose.position.x;
            end_state.y = goal->pose.position.y;
            tf2::Quaternion tf2_quat;
            tf2::fromMsg(goal->pose.orientation, tf2_quat);
            tf2::Matrix3x3 m(tf2_quat);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            end_state.heading = yaw;
            if (reference_rcv) {
                end_state_rcv = true;
            }
            std::cout << "get the goal." << std::endl;
        }


        PathOptimizationNS::State start_state, end_state;
        std::vector<PathOptimizationNS::State> reference_path_plot;
        PathOptimizationNS::ReferencePath reference_path_opt;
        bool start_state_rcv = false, end_state_rcv = false, reference_rcv = false;
        std::shared_ptr<grid_map::GridMap> grid_map;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr reference_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr end_sub;
        // Markers initialization.
        std::shared_ptr<ros2_viz_tools::RosVizTools> markers;
        std::string marker_frame_id = "/map";
        rclcpp::TimerBase::SharedPtr timer_;
        // Add obstacle layer.
        unsigned char OCCUPY = 0;
        unsigned char FREE = 255;
};









int main(int argc, char **argv) {
    
    rclcpp::init(argc, argv);
    google::InitGoogleLogging("/home/developer/tmp");
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    std::string base_dir = ament_index_cpp::get_package_share_directory("path_optimizer_2");
    auto log_dir = base_dir + "/log";
    if (0 != access(log_dir.c_str(), 0)) {
        // if this folder not exist, create a new one.
        mkdir(log_dir.c_str(), 0777);
    }
    FLAGS_log_dir = log_dir;
    FLAGS_logbufsecs = 0;
    // FLAGS_max_log_size = 100;
    FLAGS_stop_logging_if_full_disk = true;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::executors::SingleThreadedExecutor executor;
    rclcpp::Node::SharedPtr demo_node = std::make_shared<PlannerDemo>();
    executor.add_node(demo_node);
    executor.spin();

    google::ShutdownGoogleLogging();
    return 0;
}