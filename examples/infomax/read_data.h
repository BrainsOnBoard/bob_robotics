#pragma once

// Eigen
#include <Eigen/Core>

// Third-party includes
#include "third_party/path.h"

// Standard C includes
#include <cstdint>

// Standard C++ includes
#include <array>
#include <fstream>
#include <stdexcept>

namespace {
using namespace Eigen;
using namespace std::literals;

// Read a matrix of type T from a binary file
template<typename T = double>
auto
readData(const filesystem::path &filepath)
{
    // Open file
    std::ifstream is(filepath.str(), std::ios::binary);
    if (!is.good()) {
        throw std::runtime_error("Could not open "s + filepath.str());
    }

    // The matrix size is encoded as 2 x int32_t
    int32_t size[2];
    is.read(reinterpret_cast<char *>(&size), sizeof(size));

    // Create data array and fill it
    Matrix<T, Dynamic, Dynamic> data(size[0], size[1]);
    is.read(reinterpret_cast<char *>(data.data()), sizeof(T) * data.size());

    return data;
}
}