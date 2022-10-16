#include <loop_closure/util/point.h>
#include <boost/filesystem.hpp>
#include <Eigen/Dense>
#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <omp.h>

#include <boost/range/iterator_range.hpp>

#include <highfive/H5File.hpp>

namespace CoordSysTransform
{
    /**
     * @brief Builds a transformation matrix from an array of 16 values
     *
     * @param alignxf
     * @return Eigen::Matrix4f
     */
    static Eigen::Matrix4f buildTransformation(float *alignxf)
    {
        Eigen::Matrix3f rotation;
        Eigen::Vector4f translation;

        rotation << alignxf[0], alignxf[4], alignxf[8],
            alignxf[1], alignxf[5], alignxf[9],
            alignxf[2], alignxf[6], alignxf[10];

        translation << alignxf[12], alignxf[13], alignxf[14], 1.0;

        Eigen::Matrix4f transformation;
        transformation.setIdentity();
        transformation.template block<3, 3>(0, 0) = rotation;
        transformation.template rightCols<1>() = translation;

        return transformation;
    }

    /**
     * @brief Get the Transformation from pose -> will transform from righthand to lefthandside
     *
     * @param pose
     * @return Eigen::Matrix4f
     */
    static Eigen::Matrix4f getTransformationFromPose(const boost::filesystem::path &pose)
    {
        std::ifstream poseIn(pose.c_str());
        if (poseIn.good())
        {
            float rPosTheta[3];
            float rPos[3];
            float rPosInitial[3];
            float alignxf[16];

            poseIn >> rPosInitial[0] >> rPosInitial[1] >> rPosInitial[2];
            poseIn >> rPosTheta[0] >> rPosTheta[1] >> rPosTheta[2];

            rPos[0] = rPosInitial[2] / 100.0f;
            rPos[1] = -rPosInitial[0] / 100.0f;
            rPos[2] = rPosInitial[1] / 100.0f;

            // float x_axis_angle = 360.0f - rPosTheta[2];
            // float y_axis_angle = 360.0f - rPosTheta[0]; // oder : rPosTheta[0];
            // float z_axis_angle = 360.0f - rPosTheta[1];

            // in interval [-180, 180]
            float x_axis_angle = - rPosTheta[2];
            float y_axis_angle = - rPosTheta[0]; // oder : rPosTheta[0];
            float z_axis_angle = - rPosTheta[1];

            // rPosTheta[0] *= 0.0174533;
            // rPosTheta[1] *= 0.0174533;
            // rPosTheta[2] *= 0.0174533;

            rPosTheta[0] = x_axis_angle * 0.0174533;
            rPosTheta[1] = y_axis_angle * 0.0174533;
            rPosTheta[2] = z_axis_angle * 0.0174533;

            float sx = sin(rPosTheta[0]);
            float cx = cos(rPosTheta[0]);
            float sy = sin(rPosTheta[1]);
            float cy = cos(rPosTheta[1]);
            float sz = sin(rPosTheta[2]);
            float cz = cos(rPosTheta[2]);

            alignxf[0] = cy * cz;
            alignxf[1] = sx * sy * cz + cx * sz;
            alignxf[2] = -cx * sy * cz + sx * sz;
            alignxf[3] = 0.0;
            alignxf[4] = -cy * sz;
            alignxf[5] = -sx * sy * sz + cx * cz;
            alignxf[6] = cx * sy * sz + sx * cz;
            alignxf[7] = 0.0;
            alignxf[8] = sy;
            alignxf[9] = -sx * cy;
            alignxf[10] = cx * cy;

            alignxf[11] = 0.0;

            alignxf[12] = rPos[0];
            alignxf[13] = rPos[1];
            alignxf[14] = rPos[2];
            alignxf[15] = 1;

            return buildTransformation(alignxf);
        }
        else
        {
            return Matrix4f::Identity();
        }
    }

    /**
     * @brief function which converts a pointcloud from righthand to lefthand sided
     *
     * @param pcl
     */
    static void leftToRightHandSided(pcl::PointCloud<PointType>::Ptr cloud)
    {
        // Get point channel
        size_t size = cloud->size();

#pragma omp parallel for
        for (size_t i = 0; i < size; i++)
        {
            PointType &point = cloud->at(i);
            float x = point.x;
            float y = point.y;
            float z = point.z;

            point.x = z / 100.0f;
            point.y = -x / 100.0f;
            point.z = y / 100.0f;
        }
    }

    /**
     * @brief will transform a pcl cloud to a hdf5 compatible 1d vector
     *
     * @param cloud
     * @return std::vector<float>
     */
    static std::vector<float> cloud_to_hdf5_compatible_vector(pcl::PointCloud<PointType>::Ptr cloud)
    {
        std::vector<float> hdf5_scan_vec(cloud->size() * 3);

        for (auto point : cloud->points)
        {
            hdf5_scan_vec.push_back(point.x);
            hdf5_scan_vec.push_back(point.y);
            hdf5_scan_vec.push_back(point.z);
        }

        return hdf5_scan_vec;
    }

    /**
     * @brief reads a scan file and instantly transforms the points to a right hand sided coordinate system
     *
     * @param path
     * @return pcl::PointCloud<PointType>::Ptr
     */
    static pcl::PointCloud<PointType>::Ptr read_scan_file_and_transform(boost::filesystem::path &path)
    {
        std::ifstream scanIn(path.c_str());

        pcl::PointCloud<PointType>::Ptr cloud;

        cloud.reset(new pcl::PointCloud<PointType>());

        if (scanIn.good())
        {

            float x, y, z;

            while (scanIn >> x >> y >> z)
            {
                PointType point;

                // instantly transform to right hand sided coordinate system
                point.x = z / 100.0f;
                point.y = -x / 100.0f;
                point.z = y / 100.0f;

                cloud->push_back(point);
            }
        }

        return cloud;
    }

    /**
     * @brief reads a scan file and instantly transforms the points to a right hand sided coordinate system
     *
     * @param path
     * @return pcl::PointCloud<PointType>::Ptr
     */
    static pcl::PointCloud<PointType>::Ptr read_scan_file(boost::filesystem::path &path)
    {
        std::ifstream scanIn(path.c_str());

        pcl::PointCloud<PointType>::Ptr cloud;

        cloud.reset(new pcl::PointCloud<PointType>());

        if (scanIn.good())
        {

            float x, y, z;

            while (scanIn >> x >> y >> z)
            {
                PointType point;

                // instantly transform to right hand sided coordinate system
                point.x = x / 100.0f;
                point.y = y / 100.0f;
                point.z = z / 100.0f;

                cloud->push_back(point);
            }
        }

        return cloud;
    }

    /**
     * @brief reads a path and transforms all the data inside into the ros coordinate system
     *
     * @param path
     */
    static std::pair<std::vector<boost::filesystem::path>, std::vector<boost::filesystem::path>> get_filenames(const boost::filesystem::path &path)
    {
        // check if it is a directory
        if (!boost::filesystem::is_directory(path))
        {
            throw std::invalid_argument("[Coordinate Transformation] The delivered path is not a directory");
        }

        // create directory if not yet exists to save transformed data:
        if (!boost::filesystem::exists(boost::filesystem::path(path.string() + "/transformed")))
        {
            boost::filesystem::create_directory(boost::filesystem::path(path.string() + "/transformed"));
        }

        boost::filesystem::directory_iterator lastFile;
        std::vector<boost::filesystem::path> scan_files;
        std::vector<boost::filesystem::path> pose_files;

        // First, look for .3d files
        for (boost::filesystem::directory_iterator it(path); it != lastFile; it++)
        {
            boost::filesystem::path p = it->path();
            if (p.extension().string() == ".3d")
            {
                // Check for naming convention "scanxxx.3d"
                int num = 0;
                if (sscanf(p.filename().string().c_str(), "scan%3d", &num))
                {
                    scan_files.push_back(p);
                }
            }
            else if (p.extension().string() == ".pose")
            {
                pose_files.push_back(p);
            }
        }

        std::cout << "Found " << scan_files.size() << " scan files" << std::endl;
        std::cout << "Found " << pose_files.size() << " pose files" << std::endl;

        // check if we got the same number of stuff
        if (scan_files.size() != pose_files.size())
        {
            throw std::invalid_argument("Number of scan files and pose files differ... aborting...");
        }

        return std::make_pair(scan_files, pose_files);

        // auto h5_file_name = boost::filesystem::path(path.string() + "/transformed/transformed.h5");
        // HighFive::File file_{h5_file_name.string(), HighFive::File::OpenOrCreate | HighFive::File::Truncate};

        // int num_iterations = scan_files.size();

        // // create group for poses and scans
        // auto scan_group = file_.createGroup("/scans");
        // auto pose_group = file_.createGroup("/poses");

        // // create attributes
        // scan_group.createAttribute("num_scans", num_iterations);
        // pose_group.createAttribute("num_poses", num_iterations);

        // // iterate over the scans and data, transform them and save them in the hdf5

        // for (int i = 0; i < num_iterations; i++)
        // {
        //     std::string dataset_name = std::to_string(i);

        //     pcl::PointCloud::Ptr
        // }
    }

}