/**
 * @file loop_closure_node.cpp
 * @author Patrick Hoffmann (pahoffmann@uni-osnabrueck.de)
 * @brief Loop Closure Node used to detect and apply loop closure to a tsdf based slam
 * @version 0.1
 * @date 2022-03-14
 *
 * @copyright Copyright (c) 2022
 *
 */

// ros related includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>

// c++ standard related includes
#include <sstream>
#include <iostream>

// external bibs
#include <highfive/H5File.hpp>

#include <loop_closure/params/loop_closure_params.h>

#include <loop_closure/map/global_map.h>
#include <loop_closure/map/local_map.h>
#include <loop_closure/util/colors.h>
#include <loop_closure/path/path.h>
#include <loop_closure/util/point.h>
#include <loop_closure/ray_tracer/ray_tracer.h>
#include <loop_closure/serialization/read_path_json.h>
#include <loop_closure/data_association/association_manager.h>
#include <loop_closure/visualization/ros_viewhelper.h>
#include <loop_closure/path/path_exploration.h>
#include <loop_closure/test/math_test.h>
#include <loop_closure/util/csv_wrapper.h>

// Configuration stuff //

LoopClosureParams params;

// ROS STUFF //
visualization_msgs::Marker ray_markers;
visualization_msgs::Marker bb_marker;
visualization_msgs::Marker tsdf_map;             // marker for the tsdf map (local)
visualization_msgs::Marker tsdf_map_full_before; // marker for the full tsdf map (global) (before map update)
visualization_msgs::Marker tsdf_map_full_after;  // marker for the full tsdf map (global) (after map update)

std::vector<visualization_msgs::Marker> loop_visualizations;
visualization_msgs::Marker path_marker;
visualization_msgs::Marker updated_path_marker;
visualization_msgs::Marker pose_marker;
visualization_msgs::Marker chunk_marker;
visualization_msgs::Marker tsdf_read_marker;
visualization_msgs::Marker bresenham_marker;

// used to store sinuglar updates between the path and the path after all the updates
std::vector<visualization_msgs::Marker> path_marker_updates;

// ros publishers declaration
ros::Publisher tsdf_before_publisher;
ros::Publisher tsdf_after_publisher;
ros::Publisher single_tsdf_publisher;
ros::Publisher pose_publisher;
ros::Publisher path_publisher;
ros::Publisher updated_path_publisher;
ros::Publisher ray_publisher;
ros::Publisher bb_publisher;
ros::Publisher chunk_publisher;
ros::Publisher bresenham_int_publisher;
ros::Publisher loop_pub;
ros::Publisher cloud_pub;

/// Map Stuff ///
std::shared_ptr<GlobalMap> global_map_ptr_;
std::shared_ptr<LocalMap> local_map_ptr_;

// eval
std::shared_ptr<CSVWrapper> csv_wrapper_ptr;

CSVWrapper::CSVObject *ray_trace_benchmark;
CSVWrapper::CSVRow ray_trace_benchmark_header;
CSVWrapper::CSVRow ray_trace_benchmark_raytracer;
CSVWrapper::CSVRow ray_trace_benchmark_bresenham;

/// Ray Tracer ///
RayTracer *ray_tracer;

/// Path ///
Path *path;
Path updated_path;

/// Association Manager, used to manage and keep track of the associations ///
AssociationManager *manager;

/**
 * @brief initializes the path and tries to find path positions, if none was specified
 *
 * @param path_method
 */
void initialize_benchmark()
{
    csv_wrapper_ptr.reset(new CSVWrapper(params.loop_closure.csv_save_path));

    global_map_ptr_ = std::make_shared<GlobalMap>(params.map.filename.string()); // create a global map, use it's attributes
    auto attribute_data = global_map_ptr_->get_attribute_data();

    local_map_ptr_ = std::make_shared<LocalMap>(attribute_data.get_map_size_x(),
                                                attribute_data.get_map_size_y(),
                                                attribute_data.get_map_size_z(),
                                                global_map_ptr_, true);

    // create raytracer from maps and params
    ray_tracer = new RayTracer(params, local_map_ptr_, global_map_ptr_);

    // init path and read from hdf5
    path = new Path();
    path->attach_raytracer(ray_tracer);

    // exit if path unavailable
    if (!global_map_ptr_->has_path())
    {
        std::cout << "Delivered map for bresenham does not have a path" << std::endl;
        exit(EXIT_SUCCESS);
    }
    else
    {
        std::cout << "Got path from file" << std::endl;
    }

    auto path_poses = global_map_ptr_->get_path();

    for (auto pose : path_poses)
    {
        path->add_pose(pose);
    }

    ray_trace_benchmark = csv_wrapper_ptr->create_object("ray_trace_benchmark");
    std::cout << "Created benchmark csv object" << std::endl;
}

/**
 * @brief Populate the ros publishers
 *
 */
void populate_publishers(ros::NodeHandle &n)
{
    tsdf_before_publisher = n.advertise<visualization_msgs::Marker>("tsdf_before", 1, true);
    tsdf_after_publisher = n.advertise<visualization_msgs::Marker>("tsdf_after", 1, true);
    single_tsdf_publisher = n.advertise<visualization_msgs::Marker>("single_tsdf", 1, true);
    path_publisher = n.advertise<visualization_msgs::Marker>("path", 1, true);
    updated_path_publisher = n.advertise<visualization_msgs::Marker>("path_updated", 1, true);
    ray_publisher = n.advertise<visualization_msgs::Marker>("rays", 100);
    bb_publisher = n.advertise<visualization_msgs::Marker>("bounding_box", 1, true);
    chunk_publisher = n.advertise<visualization_msgs::Marker>("chunk_poses", 1, true);
    bresenham_int_publisher = n.advertise<visualization_msgs::Marker>("bresenham_intersections", 1, true);
    cloud_pub = n.advertise<sensor_msgs::PointCloud2>("approx_cloud", 1, true);
}

void populate_markers()
{
    // create a marker for the updated map
    updated_path_marker = ROSViewhelper::initPathMarker(&updated_path);
    bresenham_marker = ray_tracer->get_bresenham_intersection_marker();
    pose_marker = ROSViewhelper::initPoseMarker(path->at(0));
    chunk_marker = ROSViewhelper::initPathExtractionVisualizion(global_map_ptr_, local_map_ptr_);
    bb_marker = ROSViewhelper::getBoundingBoxMarker(map_to_real(local_map_ptr_->get_size()), path->at(0));

    // tsdf
    auto gm_data = global_map_ptr_->get_full_data();
    tsdf_map_full_after = ROSViewhelper::marker_from_gm_read(gm_data);
}

/**
 * @brief Benchmark the ray_tracing method
 *
 */
void benchmark_ray_tracer()
{
    std::vector<int> hor_res = {2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048};
    std::vector<int> vert_res = {2, 4, 8, 16, 32, 64, 128};

    std::map<int, std::pair<int, int>> filtered_resolutions;

    for (auto hor : hor_res)
    {
        for (auto vert : vert_res)
        {
            int key = hor * vert;

            if (filtered_resolutions.find(key) == filtered_resolutions.end())
            {
                filtered_resolutions[key] = std::make_pair(hor, vert);
            }
        }
    }

    std::cout << "Found " << filtered_resolutions.size() << " distinct resolutions" << std::endl;

    for (auto key_value : filtered_resolutions)
    {
        auto pair = key_value.second;

        std::cout << "Tracing with: [" << pair.first << " | " << pair.second << "]" << std::endl;

        params.ray_tracer.hor_res = pair.first;
        params.ray_tracer.vert_res = pair.second;

        ray_tracer = new RayTracer(params, local_map_ptr_, global_map_ptr_);

        int count = 0;
        float acc_time = 0.0f;
        for (int i = 0; i < path->get_length(); i++)
        {
            ray_tracer->update_pose(path->at(i));
            auto tmp_association = new Association(*(path->at(i)), i, global_map_ptr_, params.loop_closure.json_dirname);
            ray_tracer->update_association(tmp_association);

            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            ray_tracer->start();
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

            float millisec = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1000.0f;

            tmp_association->clear_data();
            delete tmp_association;
            count++;
            acc_time += millisec;
        }

        acc_time /= count;

        std::cout << "Average time needed for [" << pair.first << " | " << pair.second << " ]:" << acc_time << "ms" << std::endl;
        ray_tracer->cleanup();
        delete ray_tracer;

        ray_trace_benchmark_header.add(std::to_string(key_value.first));
        ray_trace_benchmark_raytracer.add(std::to_string(acc_time));
    }

    ray_trace_benchmark->add_row(ray_trace_benchmark_header);
    ray_trace_benchmark->add_row(ray_trace_benchmark_raytracer);
}

void benchmark_bresenham()
{
    std::vector<int> hor_res = {2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048};
    std::vector<int> vert_res = {2, 4, 8, 16, 32, 64, 128};

    std::map<int, std::pair<int, int>> filtered_resolutions;

    for (auto hor : hor_res)
    {
        for (auto vert : vert_res)
        {
            int key = hor * vert;

            if (filtered_resolutions.find(key) == filtered_resolutions.end())
            {
                filtered_resolutions[key] = std::make_pair(hor, vert);
            }
        }
    }
    std::cout << "------------------------------------------" << std::endl;
    std::cout << "Start BRESENHAM" << std::endl;
    std::cout << "Found " << filtered_resolutions.size() << " distinct resolutions for bresenham" << std::endl;

    for (auto key_value : filtered_resolutions)
    {
        auto pair = key_value.second;

        std::cout << "Tracing with: [" << pair.first << " | " << pair.second << "]" << std::endl;

        params.ray_tracer.hor_res = pair.first;
        params.ray_tracer.vert_res = pair.second;

        ray_tracer = new RayTracer(params, local_map_ptr_, global_map_ptr_);

        int count = 0;
        float acc_time = 0.0f;
        for (int i = 0; i < path->get_length(); i++)
        {
            ray_tracer->update_pose(path->at(i));
            auto tmp_association = new Association(*(path->at(i)), i, global_map_ptr_, params.loop_closure.json_dirname);
            ray_tracer->update_association(tmp_association);

            std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
            ray_tracer->start_bresenham();
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

            float millisec = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1000.0f;

            tmp_association->clear_data();
            delete tmp_association;
            count++;
            acc_time += millisec;
        }

        ray_tracer->cleanup();
        delete ray_tracer;
        acc_time /= count;

        std::cout << "Average time needed for [" << pair.first << " | " << pair.second << " ]:" << acc_time << "ms" << std::endl;
        ray_trace_benchmark_bresenham.add(std::to_string(acc_time));
    }

    ray_trace_benchmark->add_row(ray_trace_benchmark_bresenham);
}

void benchmark_pcl_approximation()
{
    std::cout << __LINE__ << std::endl;
    auto marker = ROSViewhelper::initSinglePoseMarker(local_map_ptr_, path->at(0));
    std::cout << "There are " << marker.points.size() << " points in the single pose map" << std::endl;
    std::cout << __LINE__ << std::endl;

    auto cloud = ray_tracer->approximate_pointcloud(path->at(0));
    std::cout << "There are " << cloud->size() << " points in the cloud" << std::endl;

    std::cout << __LINE__ << std::endl;

    auto cloud_marker = ROSViewhelper::marker_from_pcl_pointcloud(cloud, "map");
    std::cout << __LINE__ << std::endl;

    single_tsdf_publisher.publish(marker);
    cloud_pub.publish(cloud_marker);
}
/**
 * @brief Main method
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "benchmark_node");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    // read params
    params = LoopClosureParams(nh);

    // define stuff for raytracer

    initialize_benchmark();

    // generate publishers
    populate_publishers(n);

    // start benchmark
    // benchmark_ray_tracer();
    // benchmark_bresenham();
    //benchmark_pcl_approximation();
    std::cout << __LINE__ << std::endl;
    auto marker = ROSViewhelper::initSinglePoseMarker(local_map_ptr_, path->at(0));
    std::cout << "There are " << marker.points.size() << " points in the single pose map" << std::endl;
    std::cout << __LINE__ << std::endl;

    auto cloud = ray_tracer->approximate_pointcloud(path->at(0));
    std::cout << "There are " << cloud->size() << " points in the cloud" << std::endl;

    auto cloud_marker = ROSViewhelper::marker_from_pcl_pointcloud(cloud, "map");

    single_tsdf_publisher.publish(marker);
    cloud_pub.publish(cloud_marker);

    // now populate the markers
    // populate_markers();

    // write all benchmarks
    csv_wrapper_ptr->write_all();

#ifdef DEBUG
    std::cout
        << "Before update size: " << tsdf_map_full_before.points.size() << std::endl;
    std::cout << "After update size: " << tsdf_map_full_after.points.size() << std::endl;
#endif

    // auto single_marker = ROSViewhelper::initPoseAssociationVisualization(global_map_ptr_, path->at(0), 0);

    // specify ros loop rate
    ros::Rate loop_rate(10);

    // some stuff doesnt need to be published every iteration...
    bb_publisher.publish(bb_marker);
    pose_publisher.publish(pose_marker);
    path_publisher.publish(path_marker);
    updated_path_publisher.publish(updated_path_marker);
    // tsdf_publisher.publish(tsdf_map);
    tsdf_before_publisher.publish(tsdf_map_full_before);
    tsdf_after_publisher.publish(tsdf_map_full_after);
    // tsdf_publisher.publish(single_marker);
    chunk_publisher.publish(chunk_marker);

    if (bresenham_marker.points.size() != 0)
    {
        bresenham_int_publisher.publish(bresenham_marker);
    }

    // publish path and loop detects
    for (auto marker : loop_visualizations)
    {
        loop_pub.publish(marker);
    }

    ros::spinOnce();

    // ros loop
    while (ros::ok())
    {
        // publish the individual messages
        bb_publisher.publish(bb_marker);
        ray_publisher.publish(ray_markers);
        //single_tsdf_publisher.publish(marker);
        cloud_pub.publish(cloud_marker);

        // more ros related stuff
        ros::spinOnce();

        loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}