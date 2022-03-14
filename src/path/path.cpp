#include "path.h"

/**
 * @brief constructor, currently empty
 *
 */
Path::Path(/* args */)
{
}

void Path::fromJSON(std::string filename)
{
    PATH::json_to_path(filename, poses);
}