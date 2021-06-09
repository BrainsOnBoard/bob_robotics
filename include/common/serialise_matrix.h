#pragma once

// BoB robotics includes
#include "macros.h"

// Third-party includes
#include "third_party/path.h"

// Eigen
#include <Eigen/Core>

// Standard C++ includes
#include <fstream>

namespace BoBRobotics {
//! Read a matrix of type T from a binary file
template<class T>
auto
readMatrix(const filesystem::path &filepath)
{
    BOB_ASSERT(filepath.is_file());

    // Open file
    std::ifstream ifs;
    ifs.exceptions(std::ios::badbit | std::ios::failbit | std::ios::eofbit);
    ifs.open(filepath.str(), std::ios::in | std::ios::binary);

    // The matrix size is encoded as 2 x int32_t
    int32_t size[2];
    ifs.read(reinterpret_cast<char *>(&size), sizeof(size));

    // Create data array and fill it
    using namespace Eigen;
    Matrix<T, Dynamic, Dynamic> data(size[0], size[1]);
    ifs.read(reinterpret_cast<char *>(data.data()), sizeof(T) * data.size());

    return data;
}

template<class T>
void
writeMatrix(const filesystem::path &filepath, const T &matrix)
{
    std::ofstream ofs;
    ofs.exceptions(std::ios::badbit | std::ios::failbit);
    ofs.open(filepath.str(), std::ios::out | std::ios::binary);

    const int size[2]{ (int) matrix.rows(), (int) matrix.cols() };
    ofs.write(reinterpret_cast<const char *>(size), sizeof(size));
    ofs.write(reinterpret_cast<const char *>(matrix.data()), matrix.size() * sizeof(typename T::Scalar));
}
} // BoBRobotics
