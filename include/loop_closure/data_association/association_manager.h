#pragma once

/**
 * @file association_manager.h
 * @author Patrick Hoffmann (pahoffmann@uni-osnabrueck.de)
 * @brief Class used to manage (e.g. load and unload associations)
 * @version 0.1
 * @date 2022-03-14
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <map>
#include <iostream>
#include <cassert>
#include <ctime>
#include <chrono>
#include <regex>
//#include <filesystem>
#include <highfive/H5File.hpp>

#include <loop_closure/data_association/association.h>
#include <loop_closure/path/path.h>
#include <loop_closure/ray_tracer/ray_tracer.h>
#include <loop_closure/visualization/ros_viewhelper.h>

class AssociationManager
{
public:
  // enum which is used to determine which method is used to update the cells in the map
  enum UpdateMethod
  {
    MEAN,
    SINUS
  };

  /**
   * @brief Construct a new Association Manager object from a path. For every pose in the path, an association is created which is supposed
   *        to connect the tsdf data to a pose and make is serializable. creeeeeeepy.
   *
   * @param path
   * @param file_path
   */
  AssociationManager(Path *path, std::string file_path, RayTracer *tracer, std::shared_ptr<LocalMap> local_map_ptr_,
                     std::shared_ptr<GlobalMap> global_map_ptr_, Association::SerializationStrategy strat = Association::SerializationStrategy::HDF5);

  /**
   * @brief Destroy the Association Manager object
   * @todo TODO: implement this, this should remove all association data to stop polluting the hard drive
   */
  ~AssociationManager();

  /**
   * @brief Creates greedy associations using a ray-tracer, the current local map and the path
   *        Beginning from the last pose, we ray-trace and every tsdf cell which is touched by the ray, gets unrewokeable associated with it
   *        This greedy association is used to create a MVP and might not be the optimal way to tackle this problem, as cells get associated with
   *        poses, even if the artificial ray trace just crosses it at the border of the local map, or with a weird angle.
   *        Though, it is definetly the most easy way to tackle this problem.
   */
  void greedy_associations();

  /**
   * @brief This is just an idea at the moment, given the following: a ray is interupted, once it crosses a plane between the closest X poses
   *        This plane needs to be calculated for each pose "pair", with the average of the poses as location vector.
   *        This makes sure, that every tsdf cell gets only matched to the closest pose, inluding (hopefully) an acceleration in speed.
   *        Might be necessary to span a net of planes to limit each poses ranging, which somehow spans multiple boxes in which every position operates
   *        This seems pretty similiar to a 3D Voronoi Approach, i didnt even see that until now lol. nice.
   *        see: https://stackoverflow.com/questions/9227285/calculating-a-voronoi-diagram-for-planes-in-3d
   */
  void plane_limited_associations();

  /**
   * @brief This method ist supposed to update the localmap after finding a loop and getting the respective associations for each pose
   *        This means, this method needs the following parameters:
   *        1.) The new poses (only the updated ones) -> might just be the path
   *        2.) the start and end index of the respective poses (because only part of the path was updated)
   *        3.) A enum to define, which method should be used to update the cell positions
   *
   *        The algorithm will do the following:
   *
   *        1.) calc the pose differences for each of the respective poses (new - old)
   *            -> rotation and translation need to be considered
   *        2.) The algorithm will now rearrange the cells according to the pose differences, meaning:
   *            For every cell, all the poses which are associated with this cell need to be looked at.
   *            Each pose will shift the cell in a different way, according to its relocation.
   *            Many different methods might be used to update the cell positions:
   *            1.) Mean - weigh every pose the same, not taking into consideration the order of the poses, the distance between pose and cell and the
   *                angle of the ray
   *            2.) Sinus weighting: use a modified sinus/cosinus function to weigh according to order / distance / angle
   *            3.) TBD
   *       3.) The old cell positions will be removed (weight and value to default)
   *       4.) Things needed to consider:
   *           1.) Is there a need to fully copy parts of the local map to ensure there is no problem with overwriting stuff?
   *           2.) When can cells be resetted?
   *           3.) How can this be parallized in a useful way?
   *           4.) A copy of the localmap is necessary to ensure this can work properly
   *           5.) Old cells and new cells are updated in almost the same manner, ensuring we only need one run
   *
   * @return a ros visualization marker, which can be used to debug and see, if the data is read perfectly
   */
  visualization_msgs::Marker update_localmap(Path *new_path, int start_idx, int end_idx, UpdateMethod method = UpdateMethod::MEAN);

  /**
   * @brief a method, which reads all the association data and simply cleans the whole globalmap
   * @attention this method is simply used to test the associations and basically check if they are good2go
   *
   */
  void test_associations();

private:
  std::vector<Association> associations;
  Path *path;
  std::time_t time;
  std::string base_path;
  RayTracer *ray_tracer;
  std::shared_ptr<LocalMap> local_map_ptr;
  std::shared_ptr<GlobalMap> global_map_ptr;
  TSDFEntry default_entry;

  Vector3i l_map_size_half;
  Vector3i l_map_size;

  /**
   * @brief Create a serialization folder at the requested path using filesystem utils
   *
   * @param path
   */
  void create_serialization_folder(std::string path);

  /**
   * @brief calculates the pose differences between two paths
   *
   * @param before
   * @param after
   * @return std::vector<Matrix4f>
   */
  std::vector<Matrix4f> calculate_pose_differences(Path *new_path, int start_idx, int end_idx);

  /**
   * @brief
   *
   * @param previous_new_map
   * @param pose_differences
   * @param start_idx
   * @param end_idx
   */
  void fill_hashmap(
      boost::unordered_map<size_t, std::tuple<Vector3f, Vector3f, TSDFEntry, int>> &previous_new_map,
      std::vector<Matrix4f> &pose_differences, int start_idx, int end_idx);

  /**
   * @brief this method will fill a new hashmap of <Vector3i, TSDFEntry> which basically contains information on
   *        which
   *
   * @param previous_new_map
   * @param new_tsdf_map
   * @param method
   *
   */
  void filter_duplicate_tagret_cells(boost::unordered_map<size_t, std::tuple<Vector3f, Vector3f, TSDFEntry, int>> &previous_new_map,
                                     boost::unordered_map<size_t, std::tuple<Vector3i, TSDFEntry, int>> &new_tsdf_map,
                                     UpdateMethod &method);

  /**
   * @brief will calculate the bounding box of a number of given cells
   *
   * @param new_tsdf_map
   * @return std::pair<Vector3i, Vector3i>
   */
  std::pair<Vector3i, Vector3i> calculate_bounding_box(
      boost::unordered_map<size_t, std::tuple<Vector3i, TSDFEntry, int>> &new_tsdf_map);

  /**
   * @brief function, which will calculate the seperations to ensure a minimum number of shifts
   *
   * @return std::vector<std::pair<Vector3i, std::vector<std::pair<Vector3i, TSDFEntry>>>>
   */
  void calc_map_seperations(Vector3i bb_min, Vector3i bb_max,
                            std::vector<std::pair<Vector3i, std::vector<std::pair<Vector3i, TSDFEntry>>>> &map_seperations);

  /**
   * @brief fill the calculated map seperations with association data
   *
   * @param map_seperations
   * @param new_tsdf_map
   */
  void fill_map_seperations(std::vector<std::pair<Vector3i, std::vector<std::pair<Vector3i, TSDFEntry>>>> &map_seperations,
                            boost::unordered_map<size_t, std::tuple<Vector3i, TSDFEntry, int>> &new_tsdf_map);
};
