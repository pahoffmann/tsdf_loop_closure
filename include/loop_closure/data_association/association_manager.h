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

  /**
   * @brief will clear any data that is left in the manager, will also cleanup any association data in the hdf5
   * @warning will delete association data from global map
   * 
   */
  void cleanup();

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

  // different LEVELS of maps used to store data in

  // LEVEL 0: No data here, but simply the pose differences before and after the loop closure
  std::vector<Eigen::Matrix4f> pose_differences;

  // LEVEL 1: Map containing <old_cell_pose, new_cell_pose_accumulated, tsdf_entry, number of accumulations>
  //          Wheres the number of hits is determined by the poses, which have seen this
  boost::unordered_map<size_t, std::tuple<Vector3i, Vector3i, TSDFEntry, int>> level_one_data;

  // LEVEL 2: For different old cell poses, they may be mapped on the same new cell pose
  //          This is a critical step, as in this case the tsdf value of the new cell needs to be estimated
  //          This can be done in different version. To keep it easy we will stick to meaning them at first
  //          The final product is a map, which contains new tsdf entries for every cell which has to be updated
  //          The counter here is also used to accumulate tsdf values and weights:
  //          <new cell position, accumulated tsdf, accumulation counter>
  //          It is absolutely important, that this map also contains a reset of the old cell positions
  //          aka: <old_cell, default_tsdf_entry, 1>
  boost::unordered_map<size_t, std::tuple<Vector3i, TSDFEntry, int>> level_two_data;

  // LEVEL 3: The product of LEVEL 2 is a random collection of cells and its new TSDF values
  //          Problem: the bounding box surrounding each of these cells might be bigger, than the localmap
  //          Thus: the localmap needs to be shifted, if a cell that needs to be updated is currently ozt of bounds
  //          But: Shifting each time a cell is out of bounds, is really ineffecient, as this might lead to thousands of 
  //               shifts, with each shift taking hundreds of milliseconds or even seconds
  //               In the worst case, the map needs to be shifted after every single cell update.
  //               In that case no one would ever see a result of this algorithm, which keeps on shifting
  //               millions of times.
  //          Solution: To solve this problem the data needs to be structured to ensure a minimum number of shifts
  //                    To do this, the Bounding box of the data is determined and split into boxes of the size of the localmap
  //                    Afterwards, the data is inserted into it's belonging box/bucket
  //                    Like this, the space is seperated nearly optimally and the update process may begin by running over
  //                    each of these boxes/seperations, with empty boxes simply skipped to avoid useless shifts
  //          Data: <box_position, vector of <cell, new_tsdf_value>>
  std::vector<std::pair<Vector3i, std::vector<std::pair<Vector3i, TSDFEntry>>>> level_three_data;

  /**
   * @brief Create a serialization folder at the requested path using filesystem utils
   * @deprecated
   * @param path
   */
  void create_serialization_folder(std::string path);

  /**
   * @brief calculates the pose differences between two paths (LEVEL 0 data)
   *        currently pose differences for all poses are being calculated for simplicity
   *
   * @param before
   * @param after
   * @return std::vector<Matrix4f>
   */
  void calculate_pose_differences(Path *new_path);

  /**
   * @brief Generates LEVEL 1 data as described above
   *
   * @param previous_new_map
   * @param pose_differences
   * @param start_idx
   * @param end_idx
   */
  void generate_level_one_data(int start_idx, int end_idx);

  /**
   * @brief generates data of LEVEL 2 as described above
   *
   * @param method defines the way, the level 2 data should be generated, right now it is mean update only
   *
   */
  void generate_level_two_data(UpdateMethod method);

  /**
   * @brief will calculate the bounding box of a number of given cells, used to generate LEVEL 3 data
   * 
   * @return std::pair<Vector3i, Vector3i>
   */
  std::pair<Vector3i, Vector3i> calculate_level_three_bounding_box();

  /**
   * @brief function used to prepare an array for LEVEL 3 data
   * 
   * @param bb_min Minimum cell of the bounding box (min of x, y and z)
   * @param bb_max Maximum cell of the bounding box (max of x, y and z)
   * 
   * @return std::vector<std::pair<Vector3i, std::vector<std::pair<Vector3i, TSDFEntry>>>>
   */
  void prepare_level_three_data(Vector3i bb_min, Vector3i bb_max);

  /**
   * @brief generates data of LEVEL 3 as described above (actually fills the previously generated array with LEVEL 3 data)
   */
  void generate_level_three_data();

  /**
   * @brief updates the localmap with level three data
   * 
   */
  void update_localmap_level_three();

  /**
   * @brief calculates a new cell position using a the associated pose and the difference between the old and new pose position
   * 
   * @param old_cell 
   * @param transform 
   * @param old_pose 
   * @return Vector3i 
   */
  Vector3i calculate_new_cell_position(Vector3i &old_cell, Eigen::Matrix4f &transform, Pose *old_pose);
};
