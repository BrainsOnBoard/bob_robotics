#include "common.h"

// BoB robotics includes
#include "navigation/image_database.h"

namespace py = pybind11;
using namespace py::literals;
using namespace ranges;

namespace {
std::vector<cv::Mat>
readImages(const BoBRobotics::Navigation::ImageDatabase &db,
           const std::experimental::optional<py::iterable> &entries,
           bool greyscale)
{
    std::vector<cv::Mat> images;
    auto loadImage = [&](size_t i, cv::Mat fr) {
        images[i] = std::move(fr);

        // Check for Ctrl+C etc.
        checkPythonErrors();
    };

    if (!entries) {
        // ...load all images
        images.resize(db.size());
        db.forEachImage(loadImage);
    } else {
        // ...load only specific images
        images.resize(py::len(entries.value()));

        // Convert Python iterable into range of ints
        db.forEachImage(loadImage, toRange<int>(entries.value()), greyscale);
    }

    return images;
}

void
unwrapDatabase(const BoBRobotics::Navigation::ImageDatabase &database, const cv::Size &unwrapRes, std::experimental::optional<std::string> path, size_t frameSkip, bool greyscale) {
    if (!path) {
        path.emplace((database.getPath().parent_path() / ("unwrapped_" + database.getName())).str());
    }
    database.unwrap(*path, unwrapRes, frameSkip, greyscale);
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
            .def("unwrap", &unwrapDatabase,
                 "size"_a,
                 "destination"_a = std::experimental::nullopt,
                 "frame_skip"_a = 1,
                 "greyscale"_a = false);
}
} // Navigation
} // BoBRobotics
