#include <loop_closure/util/csv_wrapper.h>

CSVWrapper::CSVWrapper(boost::filesystem::path dir, char sep_char)
{
    this->seperator = sep_char;
    this->save_dir = dir;
}

void CSVWrapper::write_all()
{
    auto t = std::time(0);
    std::tm *now = std::localtime(&t);

    std::stringstream ss;
    ss << now->tm_mday << "-" << now->tm_mon << "-" << now->tm_year << "--" << now->tm_hour << "-" << now->tm_min << "-" << now->tm_sec;

    std::string time_string = ss.str();


    boost::filesystem::path base_path = save_dir / boost::filesystem::path(time_string);
    boost::filesystem::create_directory(base_path);

    std::cout << "Creating CSV in directory: " << base_path.string() << std::endl;

    for(auto it = csv_objects.begin(); it != csv_objects.end(); ++it)
    {
        auto o = (*it).second;
        std::string file_path = o.name + ".csv";

        std::ofstream file;

        file.open((base_path.string() + "/" + file_path).c_str(), std::ios::out | std::ios::trunc);
        
        if(!file.is_open())
        {
            std::cout << "Could not create file with path: " << base_path.string() + "/" + file_path << std::endl;
            continue;
        }

        file << o.header.to_string();
        file << std::endl;

        for(auto row : o.rows)
        {
            file << row.to_string();
            file << std::endl;
        }

        file.close();
    }

}
