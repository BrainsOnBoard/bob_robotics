// BoB robotics includes
#include "common/main.h"
#include "navigation/infomax.h"
#include "navigation/perfect_memory.h"
#include "navigation/image_database.h"

// Third-party includes
#include "pybind11/pybind11.h"

// Python includes
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>

// Standard C++ includes
#include <stdexcept>

namespace py = pybind11;
using namespace BoBRobotics::Navigation;
using namespace units::angle;

namespace pybind11 {
namespace detail {
    template <> struct type_caster<cv::Size> {
    public:
        PYBIND11_TYPE_CASTER(cv::Size, _("cvSize"));

        // For converting from a Python tuple to a cv::Mat
        bool load(handle src, bool) {
            return PyArg_ParseTuple(src.ptr(), "ii", &value.width, &value.height);
        }
    };

    template <> struct type_caster<cv::Mat> {
    public:
        PYBIND11_TYPE_CASTER(cv::Mat, _("cvMat"));

        bool load(handle src, bool) {
            auto *obj = src.ptr();
            if (!PyArray_Check(obj)) {
                return false;
            }

            auto *arr = reinterpret_cast<PyArrayObject *>(obj);

            /*
             * It would be totally trivial to add other types, but we're only
             * interested in uint8-type images for now.
             */
            int cvType;
            switch (PyArray_TYPE(arr)) {
            case NPY_UINT8:
                cvType = CV_8U;
                break;
            default:
                throw std::runtime_error{ "Only uint8-type arrays are allowed" };
            }

            /*
             * Note that we are taking a reference to arr's data, rather than
             * copying it. We only support 2D (i.e. greyscale) images.
             */
            BOB_ASSERT(PyArray_NDIM(arr) == 2);
            const cv::Size size{ (int) PyArray_DIM(arr, 1), (int) PyArray_DIM(arr, 0) };
            value = cv::Mat(size, cvType, PyArray_DATA(arr));

            return true;
        }
    };
}
} // namespace pybind11::detail

template<class T>
class PyAlgoWrapper {
public:
    PyAlgoWrapper(const cv::Size &size)
      : m_Algo(size)
    {}

    double getHeading(const cv::Mat &img) const
    {
        const degree_t angle = std::get<0>(m_Algo.getHeading(getResizedMat(img)));
        return angle.value();
    }

    auto test(const cv::Mat &img) const
    {
        return m_Algo.test(getResizedMat(img));
    }

    void train(const cv::Mat &img)
    {
        m_Algo.train(getResizedMat(img));
    }

    void trainRoute(const char *dbPath)
    {
        const ImageDatabase database{ dbPath };
        BOB_ASSERT(!database.empty());

        for (const auto &entry : database) {
            train(entry.load());
        }
    }

private:
    T m_Algo;
    mutable cv::Mat m_Resized;

    const auto &getResizedMat(const cv::Mat &img) const
    {
        const auto size = m_Algo.getUnwrapResolution();
        if (img.size() != size) {
            cv::resize(img, m_Resized, size);
            return m_Resized;
        }

        // No resizing needed
        return img;
    }
};

template<class Algo>
void addAlgo(py::handle scope, const char *name)
{
    using T = PyAlgoWrapper<Algo>;
    py::class_<T>(scope, name)
            .def(py::init<const cv::Size &>())
            .def("get_heading", &T::getHeading)
            .def("test", &T::test)
            .def("train", &T::train)
            .def("train_route", &T::trainRoute);
}

PYBIND11_MODULE(_navigation, m) {
    // Initialise plog
    BoBRobotics::initLogging();

    // Initialise numpy
    if (_import_array() < 0) {
        throw std::runtime_error{ "numpy.core.multiarray failed to import" };
    }

    // Add various algorithms as Python classes
    addAlgo<PerfectMemoryRotater<>>(m, "PerfectMemory");
    addAlgo<InfoMaxRotater<>>(m, "InfoMax");
}
