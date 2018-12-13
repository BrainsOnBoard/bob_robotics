#pragma once

// BoB robotics includes
#include "assert.h"

// Third-party includes
#include "../third_party/path.h"

// Standard C++ includes
#include <algorithm>
#include <fstream>
#include <initializer_list>
#include <string>
#include <tuple>
#include <vector>

namespace BoBRobotics {
class CSVFile
{
public:
    CSVFile(const filesystem::path &filepath)
      : m_FileStream(filepath.str())
    {
        BOB_ASSERT(m_FileStream.good());
    }

    template<typename ElementType>
    auto readColumns(std::initializer_list<std::string> columnNames)
    {
        // Move stream back to the start of the file
        m_FileStream.clear();
        m_FileStream.seekg(std::ios::beg);

        // Get the first row (= column nmames)
        std::string line;
        BOB_ASSERT(std::getline(m_FileStream, line));

        // Split the first row at commas
        std::vector<std::string> fields;
        std::string field;
        for (std::stringstream ss{ line }; std::getline(ss, field, ','); fields.push_back(field))
            ;

        // Find the headings corresponding to the ones in columnNames
        std::vector<size_t> columnIndexes;
        columnIndexes.reserve(columnNames.size());
        for (auto &column : columnNames) {
            const auto elem = std::find(fields.cbegin(), fields.cend(), column);
            BOB_ASSERT(elem != fields.cend());
            columnIndexes.push_back(std::distance(fields.cbegin(), elem));
        }

        std::vector<std::vector<ElementType>> ret(columnNames.size());
        const size_t maxColumn = *std::max_element(columnIndexes.cbegin(), columnIndexes.cend());
        while (std::getline(m_FileStream, line)) {
            // Split next row
            fields.clear();
            for (std::stringstream ss{ line }; fields.size() <= maxColumn && std::getline(ss, field, ','); fields.push_back(field))
                ;

            // Parse the values for the columns we're interested in and push into vector
            for (size_t i = 0; i < columnIndexes.size(); i++) {
                ElementType val;
                std::stringstream ssconvert{ fields[columnIndexes[i]] };
                ssconvert >> val;
                ret[i].push_back(val);
            }
        }

        // Return a vector of vectors (column * row)
        return ret;
    }

private:
    std::ifstream m_FileStream;
}; // CSVFile
}