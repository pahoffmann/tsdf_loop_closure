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

int Path::find_next_loop(int start_idx)
{
    return 0;
}