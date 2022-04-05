#include "common.h"

// BoB robotics includes
#include "navigation/image_database.h"

namespace py = pybind11;
using namespace py::literals;

namespace BoBRobotics {
namespace Navigation {
void
addDatabaseClass(py::module_ &m)
{
    py::class_<ImageDatabase>(m, "DatabaseInternal")
            .def(py::init<const std::string &>(),
                 "database_path"_a)
            .def("__len__", &ImageDatabase::size)
            .def("get_entries", &ImageDatabase::getEntries)
            .def("read_images",
                 [](ImageDatabase &db) { return db.readImages(); });
}
} // Navigation
} // BoBRobotics
