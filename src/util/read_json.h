#pragma once

/**
 * @file read_json.h
 * @author Patrick Hoffmann (pahoffmann@uni-osnabrueck.de)
 * @brief
 * @version 0.1
 * @date 2022-03-09
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <jsoncpp/json/json.h>
#include "point.h"
#include "path.h"
#include <fstream>
#include <string>

namespace PATH
{
    /**
     * @brief reads a file into a json::Value object
     *
     * @param filename
     * @return Json::Value
     */
    static Json::Value filename_to_json(std::string filename)
    {
        Json::Value json;
        std::ifstream poses_file(filename, std::ifstream::binary);
        poses_file >> json;
        return json;
    }

    /**
     * @brief Reads a Json formatted path into the path object
     *
     * @param filename   name of the JSON file
     * @param path       the path obj
     */
    static void json_to_path(std::string filename, std::vector<Pose> &path)
    {
        // obtain json object from filename
        auto json = filename_to_json(filename);

        // print description
        std::string description = json.get("description", "UTF-8").asString();
        printf("%s", description.c_str()); // conversion to c string needed for printf

        const Json::Value poses = json["poses"];



        for (Json::ValueConstIterator it = poses.begin(); it != poses.end(); ++it)
        {
            const Json::Value pose = (*it);

            std::cout << pose << std::endl;

            path.push_back(poseFromEuler(
                (*it)["x"].asFloat(),
                (*it)["y"].asFloat(),
                (*it)["z"].asFloat(),
                (*it)["roll"].asFloat(),
                (*it)["pitch"].asFloat(),
                (*it)["yaw"].asFloat()
            ));
        }
    }
}
