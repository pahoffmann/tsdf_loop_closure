#pragma once

#include <loop_closure/util/point.h>

#include <vector>
#include <chrono>
#include <ctime>
#include <boost/unordered_map.hpp>

class CSVWrapper
{
public:
    struct CSVRow
    {
        std::vector<std::string> data;

        void add(std::string entry)
        {
            data.push_back(entry);
        }

        /**
         * @brief gets the row as a string containing the data seperated by a given seperator
         * 
         * @param seperator 
         * @return std::string 
         */
        std::string to_string(char seperator = ',')
        {
            std::stringstream ss;

            for(int i = 0; i < data.size(); i++)
            {
                ss << data[i];

                if(i < data.size() - 1)
                {
                    ss << seperator;
                }
            }

            return ss.str();
        }
    };

    struct CSVObject
    {
        std::string name;
        CSVRow header;
        std::vector<CSVRow> rows;

        CSVObject(std::string name)
        {
            this->name = name;
        }

        CSVObject()
        {

        }

        void add_row(std::vector<std::string> row_data)
        {
            CSVRow row;
            for(auto s : row_data)
            {
                row.add(s);
            }

            this->rows.push_back(row);
        }

        void add_row(CSVRow row)
        {
            this->rows.push_back(row);
        }

        void set_header(std::vector<std::string> header_data)
        {
            for(auto s : header_data)
            {
                this->header.add(s);
            }
        }

        void set_header(CSVRow header_data)
        {
            this->header = header_data;
        }
    };

    CSVWrapper(boost::filesystem::path dir, char sep_char = ',');
    ~CSVWrapper() = default;

    inline CSVObject* create_object(std::string name)
    {
        if(csv_objects.find(name) != csv_objects.end())
        {
            return &(csv_objects[name]);
        }

        CSVObject object(name);

        csv_objects[name] = object;

        return &csv_objects[name];
    }

    /**
     * @brief writes all csv objects to the specified location
     * 
     */
    void write_all();

private:
    
    boost::unordered_map<std::string, CSVObject> csv_objects;
    //std::vector<CSVObject> csv_objects;
    boost::filesystem::path save_dir;
    char seperator;
};
