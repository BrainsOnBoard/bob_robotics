// BoB robotics includes
#include "common/main.h"
#include "navigation/image_database.h"
#include "navigation/infomax.h"
#include "navigation/perfect_memory.h"

// Third-party includes
#include "plog/Log.h"
#include "third_party/path.h"
#include "third_party/units.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Python includes
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>

// Standard C++ includes
#include <stdexcept>
#include <string>

using namespace BoBRobotics::Navigation;
using namespace units::angle;

template<class T>
struct PyAlgoWrapper
{
    void destroy()
    {
        // Invoke destructor explicitly
        if (constructed)
            algo.~T();

        Py_TYPE(this)->tp_free(reinterpret_cast<PyObject *>(this));
        LOGD << "PerfectMemory object deallocated";
    }

    cv::Mat bufToMat(const Py_buffer &buf)
    {
        if (buf.itemsize != 1) {
            throw std::runtime_error("Image must be uint8");
        }

        /*
         * buf.shape seems not to be defined when using numpy arrays, so we can't
         * check the height and width explicitly
         */
        const auto sz = algo.getUnwrapResolution();
        if (buf.len != (sz.height * sz.width)) {
            throw std::runtime_error("Image is either not greyscale or the wrong size");
        }

        return cv::Mat{ sz, CV_8UC1, buf.buf };
    }

    PyObject *getHeading(PyObject *args)
    {
        Py_buffer buf;
        if (!PyArg_ParseTuple(args, "y*", &buf))
            return nullptr;

        try {
            const degree_t heading = std::get<0>(algo.getHeading(bufToMat(buf)));
            return PyFloat_FromDouble(heading.value());
        } catch (std::exception &e) {
            PyErr_SetString(PyExc_RuntimeError, e.what());
            return nullptr;
        }
    }

    PyObject *test(PyObject *args)
    {
        Py_buffer buf;
        if (!PyArg_ParseTuple(args, "y*", &buf))
            return nullptr;

        try {
            const float result = algo.test(bufToMat(buf));
            return PyFloat_FromDouble(result);
        } catch (std::exception &e) {
            PyErr_SetString(PyExc_RuntimeError, e.what());
            return nullptr;
        }
    }

    PyObject *train(PyObject *args)
    {
        Py_buffer buf;
        if (!PyArg_ParseTuple(args, "y*", &buf))
            return nullptr;

        try {
            algo.train(bufToMat(buf));
        } catch (std::exception &e) {
            PyErr_SetString(PyExc_RuntimeError, e.what());
            return nullptr;
        }

        Py_RETURN_NONE;
    }

    PyObject *trainRoute(PyObject *args)
    {
        const char *dbPath;
        if (!PyArg_ParseTuple(args, "s", &dbPath))
            return nullptr;

        try {
            const ImageDatabase database{ dbPath };
            BOB_ASSERT(!database.empty());

            cv::Mat img;
            for (const auto &entry : database) {
                // Resize images if necessary
                img = entry.load();
                if (img.size() != algo.getUnwrapResolution()) {
                    cv::resize(img, img, algo.getUnwrapResolution());
                }

                // Train underlying algorithm
                algo.train(img);
            }
        } catch (std::exception &e) {
            PyErr_SetString(PyExc_RuntimeError, e.what());
            return nullptr;
        }

        Py_RETURN_NONE;
    }

    static PyObject* construct(PyTypeObject *type, PyObject *args, PyObject * /*kwds*/)
    {
        int width, height;
        if (!PyArg_ParseTuple(args, "ii", &width, &height))
            return nullptr;

        // Allocate memory
        PyObject *self = type->tp_alloc(type, 0);
        if (!self)
            return nullptr;

        // Construct algo object in place
        try {
            auto &wrapper = *reinterpret_cast<PyAlgoWrapper<T> *>(self);
            new (&wrapper.algo) T({ width, height });

            // Extra flag needed in case there is an error thrown in the constructor
            wrapper.constructed = true;
        } catch (std::exception &e) {
            type->tp_free(self);
            PyErr_SetString(PyExc_RuntimeError, e.what());
            return nullptr;
        }

        return self;
    }

    static PyTypeObject getType(const std::string &name)
    {
        static PyMethodDef methods[] = {
            { "train", (PyCFunction) &PyAlgoWrapper<T>::train, METH_VARARGS, "Train with a single image" },
            { "train_route", (PyCFunction) &PyAlgoWrapper<T>::trainRoute, METH_VARARGS, "Train with an image database" },
            { "test", (PyCFunction) &PyAlgoWrapper<T>::test, METH_VARARGS, "Get output value for single test image" },
            { "get_heading", (PyCFunction) &PyAlgoWrapper<T>::getHeading, METH_VARARGS, "Get the heading estimate for a given image" },
            {}
        };

        const std::string fullName = "navigation." + name;
        return {
            // clang-format off
            PyVarObject_HEAD_INIT(&PyType_Type, 0)
            // clang-format on
            fullName.c_str(),                        /* tp_name */
            sizeof(PyAlgoWrapper<T>),                /* tp_basicsize */
            0,                                       /* tp_itemsize */
            (destructor) &PyAlgoWrapper<T>::destroy, /* tp_dealloc */
            0,                                       /* tp_print */
            0,                                       /* tp_getattr */
            0,                                       /* tp_setattr */
            0,                                       /* tp_reserved */
            0,                                       /* tp_repr */
            0,                                       /* tp_as_number */
            0,                                       /* tp_as_sequence */
            0,                                       /* tp_as_mapping */
            0,                                       /* tp_hash */
            0,                                       /* tp_call */
            0,                                       /* tp_str */
            0,                                       /* tp_getattro */
            0,                                       /* tp_setattro */
            0,                                       /* tp_as_buffer */
            Py_TPFLAGS_DEFAULT,                      /* tp_flags */
            0,                                       /* tp_doc */
            0,                                       /* tp_traverse */
            0,                                       /* tp_clear */
            0,                                       /* tp_richcompare */
            0,                                       /* tp_weaklistoffset */
            0,                                       /* tp_iter */
            0,                                       /* tp_iternext */
            methods,                                 /* tp_methods */
            0,                                       /* tp_members */
            0,                                       /* tp_getset */
            0,                                       /* tp_base */
            0,                                       /* tp_dict */
            0,                                       /* tp_descr_get */
            0,                                       /* tp_descr_set */
            0,                                       /* tp_dictoffset */
            0,                                       /* tp_init */
            0,                                       /* tp_alloc */
            (newfunc) &construct,                    /* tp_new */
        };
    }
    static PyTypeObject Type;

    PyObject_HEAD;
    T algo;
    bool constructed;
};

template<class Algo>
void addAlgo(PyObject *module, const std::string &name)
{
    static PyTypeObject algoType = PyAlgoWrapper<Algo>::getType(name);
    if (PyType_Ready(&algoType) < 0)
        throw std::exception{};

    Py_INCREF(&algoType);
    if (PyModule_AddObject(module, name.c_str(), (PyObject *) &algoType) < 0) {
        Py_DECREF(&algoType);
        throw std::exception{};
    }
}

static struct PyModuleDef ModuleDefinitions
{
    // clang-format off
    PyModuleDef_HEAD_INIT,
    "navigation",
    "A Python wrapper for the BoB robotics navigation module",
    -1,
    // clang-format on
};

PyMODINIT_FUNC
PyInit__navigation(void)
{
    import_array();             // init numpy
    BoBRobotics::initLogging(); // init plog

    PyObject *module = PyModule_Create(&ModuleDefinitions);
    if (!module)
        return nullptr;

    try {
        addAlgo<PerfectMemoryRotater<>>(module, "PerfectMemory");
    } catch (std::exception &) {
        Py_DECREF(module);
    }

    return module;
}
