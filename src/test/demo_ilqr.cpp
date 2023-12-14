//
// Created by ljn on 20-2-4.
//

#include <iostream>
#include <vector>
#include <tuple>
#include <unistd.h>
#include <sys/stat.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav_msgs/msg/occupancy_grid.h>
#include <geometry_msgs/msg/point_stamped.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "glog/logging.h"
// #include <tf/transform_datatypes.h>
// #include <tf/transform_broadcaster.h>
// #include <tf/tf.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros2_viz_tools/ros2_viz_tools.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include "eigen3/Eigen/Dense"
#include "opencv2/core/core.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/opencv.hpp"
#include "eigen2cv.hpp"
#include "reference_line_processor.h"
#include "path/data_structure.h"
#include "path/path_problem_manager.h"
#include "solver/solver.h"
#include "path/gflags.h"
#include "path/tool.h"


// TODO: this file is a mess.

using namespace PathPlanning;
using std::placeholders::_1;
using namespace std::chrono_literals;


class PlannerDemoLQR : public rclcpp::Node{
    public:
        PlannerDemoLQR() : Node("planner_demo"){
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
                                ("/clicked_point", 1, std::bind(&PlannerDemoLQR::referenceCb, this, _1));
            start_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>
                                ("/initialpose", 1, std::bind(&PlannerDemoLQR::startCb, this, _1));
            end_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>
                                ("/goal_pose", 1, std::bind(&PlannerDemoLQR::goalCb, this, _1));
            timer_ = this->create_wall_timer(1000ms, std::bind(&PlannerDemoLQR::initialization, this));



        }

        ~PlannerDemoLQR(){}
    private:

        void initialization() {
            rclcpp::NodeOptions node_options;
            node_options.automatically_declare_parameters_from_overrides(true);
            markers = std::make_shared<ros2_viz_tools::RosVizTools>("markers", node_options);
            timer_ = this->create_wall_timer(300ms, std::bind(&PlannerDemoLQR::timer_callback, this)); 
        }

        void timer_callback()
        {



                rclcpp::Time time = this->get_clock()->now();
                markers->clear();
                int id = 0;
                // Cancel at double click.
                if (reference_points.size() >= 2) {
                    const auto &p1 = reference_points[reference_points.size() - 2];
                    const auto &p2 = reference_points.back();
                    if (std::sqrt(std::pow(p1.x-p2.x, 2) + std::pow(p1.y-p2.y, 2)) <= 0.001) {
                        reference_points.clear();
                        reference_rcv = false;
                    }
                }
                // Visualize reference path selected by mouse.
                visualization_msgs::msg::Marker reference_marker =
                    markers->newSphereList(0.5, "reference point", id++, ros2_viz_tools::RED, marker_frame_id);
                for (size_t i = 0; i != reference_points.size(); ++i) {
                    geometry_msgs::msg::Point p;
                    p.x = reference_points[i].x;
                    p.y = reference_points[i].y;
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
                geometry_msgs::msg::Quaternion start_quat = createQuaternionMsgFromYaw(start_state.theta);
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
                auto end_quat = createQuaternionMsgFromYaw(end_state.theta);
                end_pose.orientation.x = end_quat.x;
                end_pose.orientation.y = end_quat.y;
                end_pose.orientation.z = end_quat.z;
                end_pose.orientation.w = end_quat.w;
                visualization_msgs::msg::Marker end_marker =
                    markers->newArrow(scale, end_pose, "end point", id++, ros2_viz_tools::CYAN, marker_frame_id);
                markers->append(end_marker);

                // Calculate.
                // std::vector<PathOptimizationNS::SlState> result_path;
                // std::vector<PathOptimizationNS::State> smoothed_reference_path, result_path_by_boxes;
                std::vector<std::vector<double>> a_star_display(3);
                bool opt_ok = false;
                
                if (reference_rcv && start_state_rcv && end_state_rcv) {

                    Test::Map map(*grid_map);
                    Test::ReferenceLineProcessor reference_line_processor(reference_points, map, start_state);
                    std::shared_ptr<PathPlanning::ReferenceLine> ref_line_ptr = std::make_shared<PathPlanning::ReferenceLine>();
                    std::shared_ptr<PathPlanning::FreeSpace> free_space_ptr = std::make_shared<PathPlanning::FreeSpace>();
                    reference_line_processor.solve(ref_line_ptr, free_space_ptr);

                    visualization_msgs::msg::Marker boundary_points_marker =
                        markers->newSphereList(0.3, "boundary points", id++, ros2_viz_tools::BLUE, marker_frame_id);
                    for (const auto& pt : free_space_ptr->boundary_points()) {
                        geometry_msgs::msg::Point p;
                        p.x = pt.lb_xy.x;
                        p.y = pt.lb_xy.y;
                        p.z = 0.5;
                        boundary_points_marker.points.push_back(p);
                        p.x = pt.ub_xy.x;
                        p.y = pt.ub_xy.y;
                        p.z = 0.5;
                        boundary_points_marker.points.push_back(p);
                    }
                    markers->append(boundary_points_marker);

                    // solve
                    PathPlanning::PathProblemManager path_problem_manager;
                    path_problem_manager.formulate_path_problem(*free_space_ptr, *ref_line_ptr, start_state, end_state);
                    Solver::ILQRSolver<PathPlanning::N_PATH_STATE, PathPlanning::N_PATH_CONTROL> ilqr_solver(path_problem_manager);
                    const auto solve_status = ilqr_solver.solve();

                    visualization_msgs::msg::Marker init_path_marker =
                        markers->newLineStrip(0.3, "init path", id++, ros2_viz_tools::YELLOW, marker_frame_id);
                    const auto& init_path_raw = path_problem_manager.init_trajectory();
                    for (size_t i = 0; i != init_path_raw.size(); ++i) {
                        PathPlanning::SLPosition sl;
                        sl.s = init_path_raw[i].sample();
                        sl.l = init_path_raw[i].state()(PathPlanning::L_INDEX);
                        const auto xy = ref_line_ptr->get_xy_by_sl(sl);
                        geometry_msgs::msg::Point p;
                        p.x = xy.x;
                        p.y = xy.y;
                        p.z = 1.0;
                        init_path_marker.points.push_back(p);;
                    }
                    markers->append(init_path_marker);
                    ros2_viz_tools::ColorRGBA path_color;
                    path_color.r = 0.063;
                    path_color.g = 0.305;
                    path_color.b = 0.545;
                    if (solve_status != ILQRSolveStatus::SOLVED) {
                        path_color.r = 1.0;
                        path_color.g = 0.0;
                        path_color.b = 0.0;
                    }
                    visualization_msgs::msg::Marker result_marker =
                        markers->newLineStrip(FLAGS_vehicle_width, "optimized path", id++, path_color, marker_frame_id);
                    visualization_msgs::msg::Marker vehicle_geometry_marker =
                        markers->newLineList(0.02, "vehicle", id++, ros2_viz_tools::GRAY, marker_frame_id);
                    // Visualize vehicle geometry.
                    static const double length{FLAGS_vehicle_length};
                    static const double width{FLAGS_vehicle_width};
                    static const double rtc{FLAGS_rear_axle_to_center};
                    static const double rear_d{length / 2 - rtc};
                    static const double front_d{length - rear_d};
                    const auto& opt_path_raw = ilqr_solver.final_trajectory();
                    const auto result = PathProblemManager::transform_to_path_points(*ref_line_ptr, opt_path_raw);
                    for (size_t i = 0; i != result.size(); ++i) {
                        const auto path_point = result.at(i);
                        geometry_msgs::msg::Point p;
                        p.x = path_point.x;
                        p.y = path_point.y;
                        p.z = 1.0;
                        result_marker.points.push_back(p);
                        const auto k = path_point.kappa;
                        path_color.a = std::min(fabs(k) / 0.15, 1.0);
                        path_color.a = std::max((float)0.1, path_color.a);
                        result_marker.colors.emplace_back(path_color);
                        //
                        const double heading = path_point.theta;
                        PathPoint p1, p2, p3, p4;
                        p1.x = front_d;
                        p1.y = width / 2;
                        p2.x = front_d;
                        p2.y = -width / 2;
                        p3.x = -rear_d;
                        p3.y = -width / 2;
                        p4.x = -rear_d;
                        p4.y = width / 2;
                        p1 = local_to_global(path_point, p1);
                        p2 = local_to_global(path_point, p2);
                        p3 = local_to_global(path_point, p3);
                        p4 = local_to_global(path_point, p4);
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
                    markers->append(result_marker);
                    markers->append(vehicle_geometry_marker);
                }

                // Publish the grid_map.
                // grid_map.setTimestamp(time.toNSec());
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
                reference_points.clear();
            }
            PathPlanning::PathPoint point;
            point.x = p->point.x;
            point.y = p->point.y;
            reference_points.emplace_back(point);
            start_state_rcv = end_state_rcv = false;
            reference_rcv = reference_points.size() >= 6;
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

            start_state.theta = yaw;
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

            end_state.theta = yaw;
            if (reference_rcv) {
                end_state_rcv = true;
            }
            std::cout << "get the goal." << std::endl;
        }



        
        std::vector<PathPlanning::PathPoint> reference_points;
        PathPlanning::PathPoint start_state, end_state;
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
    rclcpp::Node::SharedPtr demo_node = std::make_shared<PlannerDemoLQR>();
    executor.add_node(demo_node);
    executor.spin();

    
    

    google::ShutdownGoogleLogging();
    return 0;
}


