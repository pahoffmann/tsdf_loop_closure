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
#include "association.h"
#include "../path/path.h"


class AssociationManager
{
private:
    std::vector<Association> associations;
    Path *path;
    std::string base_path;
public:
    AssociationManager(Path *path, std::string file_path);
    ~AssociationManager();
};


