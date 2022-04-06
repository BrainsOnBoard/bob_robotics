#include "common.h"

// BoB robotics includes
#include "navigation/image_database.h"

namespace py = pybind11;
using namespace py::literals;
using namespace ranges;

namespace {
auto
toIntRange(const py::iterable &iterable)
{
    const auto toInt = [](const py::handle &handle) { return handle.cast<int>(); };
    return views::transform(iterable, toInt);
}

std::vector<cv::Mat>
readImages(const BoBRobotics::Navigation::ImageDatabase &db,
           const std::experimental::optional<py::iterable> &entries,
           bool greyscale)
{
    std::vector<cv::Mat> images;
    auto loadImage = [&](size_t i, cv::Mat fr) {
        images[i] = std::move(fr);

        // Check for Ctrl+C etc.
        BOB_ASSERT(!PyErr_CheckSignals());
    };

    if (!entries) {
        // ...load all images
        images.resize(db.size());
        db.forEachImage(loadImage);
    } else {
        // ...load only specific images
        images.resize(py::len(entries.value()));

        // Convert Python iterable into range of ints
        db.forEachImage(loadImage, toIntRange(entries.value()), greyscale);
    }

    return images;
}

void
truncateDatabase(BoBRobotics::Navigation::ImageDatabase &db,
                const py::iterable &entriesToKeep)
{
    return db.truncate(toIntRange(entriesToKeep));
}
} // anonymous namespace

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
            .def("needs_unwrapping", &ImageDatabase::needsUnwrapping)
            .def("read_images", &readImages, "entries"_a = std::experimental::nullopt, "greyscale"_a = true)
            .def("_truncate", &truncateDatabase, "entries_to_keep"_a);
}
} // Navigation
} // BoBRobotics
