// BoB robotics includes
#include "common/main.h"
#include "navigation/image_database.h"
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
#include <exception>

#ifdef _WIN32
#define DLL_EXPORT extern "C" __declspec(dllexport)
#else
#define DLL_EXPORT
#endif

using namespace BoBRobotics::Navigation;
using namespace units::angle;

struct PyPerfectMemory
{
    PyObject_HEAD;
    PerfectMemoryRotater<> pm;
};

DLL_EXPORT PyObject *
PerfectMemory_new(PyTypeObject *type, PyObject *args, PyObject * /*kwds*/)
{
    int width, height;
    if (!PyArg_ParseTuple(args, "ii", &width, &height))
        return nullptr;

    // Allocate memory
    PyObject *self = type->tp_alloc(type, 0);
    if (!self)
        return nullptr;

    // Construct in place; assume no exceptions thrown
    auto *pm = &reinterpret_cast<PyPerfectMemory *>(self)->pm;
    new (pm) PerfectMemoryRotater<>({ width, height });

    return self;
}

DLL_EXPORT void
PerfectMemory_dealloc(PyPerfectMemory *self)
{
    // Invoke destructor explicitly
    self->pm.~PerfectMemoryRotater<>();

    Py_TYPE(self)->tp_free(reinterpret_cast<PyObject *>(self));
    LOGD << "PerfectMemory object deallocated";
}

static cv::Mat bufToMat(const PyPerfectMemory &self, const Py_buffer &buf)
{
    if (buf.itemsize != 1) {
        throw std::runtime_error("Image must be uint8");
    }

    /*
     * buf.shape seems not to be defined when using numpy arrays, so we can't
     * check the height and width explicitly
     */
    const auto sz = self.pm.getUnwrapResolution();
    if (buf.len != (sz.height * sz.width)) {
        throw std::runtime_error("Image is either not greyscale or the wrong size");
    }

    return cv::Mat{ sz, CV_8UC1, buf.buf };
}

DLL_EXPORT PyObject *
PerfectMemory_train(PyPerfectMemory *self, PyObject *args)
{
    Py_buffer buf;
    if (!PyArg_ParseTuple(args, "y*", &buf))
        return nullptr;

    try {
        self->pm.train(bufToMat(*self, buf));
    } catch (std::exception &e) {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return nullptr;
    }

    Py_RETURN_NONE;
}

DLL_EXPORT PyObject *
PerfectMemory_trainRoute(PyPerfectMemory *self, PyObject *args)
{
    const char *dbPath;
    if (!PyArg_ParseTuple(args, "s", &dbPath))
        return nullptr;

    try {
        const ImageDatabase database{ dbPath };
        BOB_ASSERT(!database.empty());

        cv::Mat img;
        for (const auto &entry : database) {
            img = entry.load();
            if (img.size() != self->pm.getUnwrapResolution()) {
                cv::resize(img, img, self->pm.getUnwrapResolution());
            }

            self->pm.train(img);
        }
    } catch (std::exception &e) {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return nullptr;
    }

    Py_RETURN_NONE;
}

DLL_EXPORT PyObject *
PerfectMemory_test(PyPerfectMemory *self, PyObject *args)
{
    Py_buffer buf;
    if (!PyArg_ParseTuple(args, "y*", &buf))
        return nullptr;

    try {
        const float result = self->pm.test(bufToMat(*self, buf));
        return PyFloat_FromDouble(result);
    } catch (std::exception &e) {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return nullptr;
    }
}

DLL_EXPORT PyObject *
PerfectMemory_getHeading(PyPerfectMemory *self, PyObject *args)
{
    Py_buffer buf;
    if (!PyArg_ParseTuple(args, "y*", &buf))
        return nullptr;

    try {
        const degree_t heading = std::get<0>(self->pm.getHeading(bufToMat(*self, buf)));
        return PyFloat_FromDouble(heading.value());
    } catch (std::exception &e) {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return nullptr;
    }
}

static PyMethodDef PerfectMemoryMethods[] = {
    { "train", (PyCFunction) PerfectMemory_train, METH_VARARGS, "Train with a single image" },
    { "train_route", (PyCFunction) PerfectMemory_trainRoute, METH_VARARGS, "Train with an image database" },
    { "test", (PyCFunction) PerfectMemory_test, METH_VARARGS, "Get output value for single test image" },
    { "get_heading", (PyCFunction) PerfectMemory_getHeading, METH_VARARGS, "Get the heading estimate for a given image"},
    {}
};

static PyTypeObject PyPerfectMemoryType = {
    // clang-format off
    PyVarObject_HEAD_INIT(&PyType_Type, 0)
    // clang-format on
    "navigation.PerfectMemory",         /* tp_name */
    sizeof(PyPerfectMemory),            /* tp_basicsize */
    0,                                  /* tp_itemsize */
    (destructor) PerfectMemory_dealloc, /* tp_dealloc */
    0,                                  /* tp_print */
    0,                                  /* tp_getattr */
    0,                                  /* tp_setattr */
    0,                                  /* tp_reserved */
    0,                                  /* tp_repr */
    0,                                  /* tp_as_number */
    0,                                  /* tp_as_sequence */
    0,                                  /* tp_as_mapping */
    0,                                  /* tp_hash */
    0,                                  /* tp_call */
    0,                                  /* tp_str */
    0,                                  /* tp_getattro */
    0,                                  /* tp_setattro */
    0,                                  /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT,                 /* tp_flags */
    0,                                  /* tp_doc */
    0,                                  /* tp_traverse */
    0,                                  /* tp_clear */
    0,                                  /* tp_richcompare */
    0,                                  /* tp_weaklistoffset */
    0,                                  /* tp_iter */
    0,                                  /* tp_iternext */
    PerfectMemoryMethods,               /* tp_methods */
    0,                                  /* tp_members */
    0,                                  /* tp_getset */
    0,                                  /* tp_base */
    0,                                  /* tp_dict */
    0,                                  /* tp_descr_get */
    0,                                  /* tp_descr_set */
    0,                                  /* tp_dictoffset */
    0,                                  /* tp_init */
    0,                                  /* tp_alloc */
    (newfunc) PerfectMemory_new,        /* tp_new */
};

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

    if (PyType_Ready(&PyPerfectMemoryType) < 0)
        return nullptr;

    PyObject *pModule = PyModule_Create(&ModuleDefinitions);
    if (!pModule)
        return nullptr;
    PyModule_AddStringConstant(pModule, "bob_robotics_path", BOB_ROBOTICS_PATH);

    Py_INCREF(&PyPerfectMemoryType);
    if (PyModule_AddObject(pModule, "PerfectMemory", (PyObject *) &PyPerfectMemoryType) < 0) {
        Py_DECREF(&PyPerfectMemoryType);
        Py_DECREF(pModule);
    }

    return pModule;
}
