#include <loop_closure/path/path.h>

Path::Path(Path &other)
{
    this->ray_tracer = other.ray_tracer;

    for (int i = 0; i < other.get_length(); i++)
    {
        // copy pose data
        Pose *pose = other.at(i);
        Pose tmp(*pose);

        this->add_pose(tmp);
    }
}

void Path::fromJSON(std::string filename)
{
    PATH::json_to_path(filename, poses);
}

/**
 * @todo TODO: test this code piece
 */
std::pair<int, int> Path::find_loop_greedy(int start_idx, float max_dist, float min_traveled, bool check_visibility)
{
    // error identification
    if (start_idx > get_length())
    {
        return std::make_pair(-1, -1);
    }

    if (check_visibility && ray_tracer == NULL)
    {
        throw std::logic_error("[RayTracer] Must attach a ray tracer to the path, before trying visibility checks");
    }

    float dist_traveled = 0.0f;

    // a vector storing the distance from the start pose to the pose with index (i - start_idx)
    std::vector<float> distance_vec(get_length());

    float dist = 0.0f;

    const Pose *current = at(start_idx);
    const Pose *next;

    for (int i = start_idx; i < get_length(); i++)
    {
        next = at(i);

        float dist_cur_next = (next->pos - current->pos).norm();

        // std::cout << "distance between " << *current << " and " << *next << ": " << dist_cur_next << std::endl;

        if (i > 0)
        {
            distance_vec[i - start_idx] = distance_vec[(i - start_idx) - 1] + dist_cur_next;
        }
        else
        {
            distance_vec[i - start_idx] = dist_cur_next;
        }

        current = next;

        // std::cout << "Distance [" << i << "] :" << distance_vec[i - start_idx] << std::endl;
    }

    // now that we have calculated the distances, lezgo

    // store the indices of the loop closure candidates
    int index_i = -1;
    int index_j = -1;

    for (int i = start_idx; i < get_length(); i++)
    {
        for (int j = i + 1; j < get_length(); j++)
        {
            float distance = distance_vec[j] - distance_vec[i];

            // std::cout << "Current distance in loop closure detection" << distance << std::endl;

            // not enough distance traveled
            if (distance < min_traveled)
            {
                // std::cout << "[Path - Find loop (greedy)] skipping, because distance travled is not enough" << std::endl;
                continue;
            }
            // calc the direct "air distance" between the two candidates
            float direct_distance = (at(j)->pos - at(i)->pos).norm();

            // std::cout << "Direct dist: " << direct_distance << std::endl;

            // if this distance is bigger than the one specified, we also continue with the next candidates
            if (direct_distance > max_dist)
            {
                continue;
            }

            std::cout << "Found LC Candidate!" << std::endl;

            // if we reach this point we have (as it seems) a candidate. now we need to check the visibility criteria (e.g. : is pos i visible from pos j ?)
            // this should usually be the case.
            // only do this check if specified by the method
            if (check_visibility && !ray_tracer->is_visible(*(at(i)), *(at(j))))
            {
                std::cout << "Candidate did not pass visibility check" << std::endl;
                continue;
            }
            else
            {
                std::cout << "Candidade passed visibility check: " << i << " | " << j << std::endl;
                index_i = i;
                index_j = j;

                return std::make_pair(index_i, index_j);
            }
        }
    }

    return std::make_pair(index_i, index_j);
}

std::pair<int, int> Path::find_loop_kd_min_dist(int start_idx, float max_dist, float min_traveled, bool check_visibility = false)
{
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_path;
    pcl::PointCloud<PointType>::Ptr pose_cloud;

    kdtree_path.reset(new pcl::KdTreeFLANN<PointType>());
    pose_cloud.reset(new pcl::PointCloud<PointType>());

    // fill pose cloud
    for (int i = 0; i < get_length(); i++)
    {
        PointType point;
        Pose *current = at(i);

        point.x = current->pos.x();
        point.y = current->pos.y();
        point.z = current->pos.z();

        pose_cloud->push_back(point);
    }

    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;

    kdtree_path->setInputCloud(pose_cloud);

    int end_idx = -1;

    // iterate over the path, find first lc with given criteria
    for (int i = start_idx; i < get_length(); i++)
    {
        // do radius search around current pose
        kdtree_path->radiusSearch(pose_cloud->at(i), max_dist, pointSearchIndLoop, pointSearchSqDisLoop, 0);

        for (int j = 0; j < (int)pointSearchIndLoop.size(); j++)
        {
            int id = pointSearchIndLoop[j];

            float distance = get_distance_between_path_poses(i, id);

            if (distance > min_traveled)
            {
                end_idx = id;
                start_idx = i;

                return std::make_pair(start_idx, end_idx);
            }
        }
    }

    // no loop found: return error value

    return std::make_pair(-1, -1);
}

std::vector<std::pair<int, int>> Path::find_loop_kd_min_dist_backwards(int idx, float max_dist, float min_traveled, bool check_visibility)
{
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_path;
    pcl::PointCloud<PointType>::Ptr pose_cloud;

    kdtree_path.reset(new pcl::KdTreeFLANN<PointType>());
    pose_cloud.reset(new pcl::PointCloud<PointType>());

    // fill pose cloud
    for (int i = 0; i < get_length(); i++)
    {
        PointType point;
        Pose *current = at(i);

        point.x = current->pos.x();
        point.y = current->pos.y();
        point.z = current->pos.z();

        pose_cloud->push_back(point);
    }

    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;

    kdtree_path->setInputCloud(pose_cloud);

    int end_idx = idx;
    int start_idx = -1;

    // do radius search around current pose
    kdtree_path->radiusSearch(pose_cloud->at(end_idx), max_dist, pointSearchIndLoop, pointSearchSqDisLoop, 0); // TODO: parameterize

    std::vector<std::pair<int, int>> lc_candidates;

    for (int i = 0; i < (int)pointSearchIndLoop.size(); i++)
    {
        int id = pointSearchIndLoop[i];

        float distance = get_distance_between_path_poses(id, end_idx);

        if (distance > min_traveled)
        {
            start_idx = id;
            lc_candidates.push_back(std::make_pair(start_idx, end_idx));

            std::cout << "Distance for candidate: [" << start_idx << " | " << end_idx << "] : " << distance << "| Min: " << min_traveled << std::endl; 
        }

        // TODO: limit number of pairs
    }

    // if the number is too small redo with bigger params (as there might be something going on)
    // if (lc_candidates.size() < 10)
    // {
    //     lc_candidates.clear();

    //     // do radius search around current pose
    //     //kdtree_path->radiusSearch(pose_cloud->at(end_idx), max_dist * 2, pointSearchIndLoop, pointSearchSqDisLoop, 0); // TODO: parameterize

    //     std::vector<std::pair<int, int>> lc_candidates;

    //     for (int i = 0; i < (int)pointSearchIndLoop.size(); i++)
    //     {
    //         int id = pointSearchIndLoop[i];

    //         float distance = get_distance_between_path_poses(id, end_idx);

    //         if (distance > (min_traveled * 1.7f) && lc_candidates.size() < 10)
    //         {
    //             start_idx = id;
    //             lc_candidates.push_back(std::make_pair(start_idx, end_idx));
    //         }

    //         // TODO: limit number of pairs
    //     }
    // }

    // no loop found: return error value

    return lc_candidates;
}

void Path::blur(int start_idx, int end_idx, double radius)
{
    if (start_idx < 0 || end_idx > get_length() - 1 || start_idx > end_idx)
    {
        throw std::logic_error("[Path] - blur(): indexing error");
    }

    // now blur every point in the path
    for (int i = start_idx; i <= end_idx; i++)
    {
        double x = generateRandomNumber(-1.0f * radius, radius);
        double y = generateRandomNumber(-1.0f * radius, radius);
        double z = generateRandomNumber(-1.0f * radius, radius);

        // add the random vec defined by radius to the current pos
        at(i)->add(Vector3f(x, y, z));
    }

    // now the current path has been blurred
}

Path Path::blur_ret(int start_idx, int end_idx, double radius)
{
    if (start_idx < 0 || end_idx > get_length() - 1 || start_idx > end_idx)
    {
        throw std::logic_error("[Path] - blur(): indexing error");
    }

    // copy path
    Path path(*this);

    // now blur every point in the path between the indexes
    for (int i = start_idx; i < end_idx; i++)
    {
        double x = generateRandomNumber(-1.0f * radius, radius);
        double y = generateRandomNumber(-1.0f * radius, radius);
        double z = generateRandomNumber(-1.0f * radius, radius);

        // add the random vec defined by radius to the current pos
        path.at(i)->add(Vector3f(x, y, z));
    }

    // now the current path has been blurred

    return path;
}

Path Path::rotate_ret(float roll_deg, float pitch_deg, float yaw_deg, Pose *rotation_pose)
{
    Vector3f rotate_point;

    // calculate center vec if NULL is passed to function
    if (rotation_pose == NULL)
    {
        Vector3f tmp = Vector3f::Zero();

        for (auto pose : poses)
        {
            tmp = tmp + pose.pos;
        }

        tmp = tmp / get_length();

        rotate_point = tmp;
    }
    else
    {
        rotate_point = rotation_pose->pos;
    }

    // vector to rotate around identified

    // calc rotation matrix

    // create eigen quaternion from euler
    Eigen::Quaternionf q;
    auto rollAngle = Eigen::AngleAxisf(roll_deg * (M_PI / 180.0f), Eigen::Vector3f::UnitX());
    auto pitchAngle = Eigen::AngleAxisf(pitch_deg * (M_PI / 180.0f), Eigen::Vector3f::UnitY());
    auto yawAngle = Eigen::AngleAxisf(yaw_deg * (M_PI / 180.0f), Eigen::Vector3f::UnitZ());

    q = yawAngle * pitchAngle * rollAngle;
    q.normalize();

    auto rot_mat = q.toRotationMatrix();

    Path path(*this);

    // rotate every point
    for (int i = 0; i < path.get_length(); i++)
    {
        auto pos = path.at(i);
        Vector3f rotated = rot_mat * (pos->pos - rotate_point) + rotate_point;
        Eigen::Matrix3f pose_rotation = rot_mat * pos->rotationMatrixFromQuaternion();
        Eigen::Quaternionf quat(pose_rotation);

        pos->pos = rotated;
        pos->quat = quat;
    }

    return path;
}

Path Path::translate_ret(Vector3f translation_vector, int start_idx, int end_idx)
{
    if (start_idx < 0 || end_idx > get_length() - 1 || start_idx > end_idx)
    {
        throw std::logic_error("[Path] - blur(): indexing error");
    }

    Path tmp(*this);

    for (int i = start_idx; i <= end_idx; i++)
    {
        tmp.at(i)->add(translation_vector);
    }

    return tmp;
}

Vector3f Path::get_centroid()
{
    Vector3f accumulated = Vector3f::Zero();

    for (auto pose : poses)
    {
        accumulated += pose.pos;
    }

    accumulated /= poses.size();

    return accumulated;
}

float Path::get_distance_between_path_poses(int idx1, int idx2)
{
    // guard clause
    if (idx1 < 0 || idx1 > get_length() - 1 || idx2 < 0 || idx2 > get_length() - 1 || idx1 > idx2)
    {
        return -1.0f;
    }

    // distance is 0 for equal path indices
    if (idx1 == idx2)
    {
        return 0;
    }

    float dist = 0.0f;

    Pose *current;
    Pose *next;

    for (int i = idx1; i < idx2 - 1; i++)
    {
        current = at(i);
        next = at(i + 1);
        dist += (next->pos - current->pos).norm();
    }

    return dist;
}
