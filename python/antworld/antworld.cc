// BoB robotics includes
#include "antworld/agent.h"
#include "common/main.h"

// Third-party includes
#include "plog/Log.h"
#include "third_party/path.h"

// Python includes
#include <Python.h>
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>

using namespace BoBRobotics;

struct AgentObjectData
{
    AgentObjectData(const cv::Size &renderSize)
      : window{ AntWorld::AntAgent::initialiseWindow(renderSize) }
      , renderer(256, 0.001, 1000.0, 360_deg)
      , agent(*window, renderer, renderSize)
    {}

    std::unique_ptr<sf::Window> window;
    AntWorld::Renderer renderer;
    AntWorld::AntAgent agent;
};

struct AgentObject
{
    PyObject_HEAD
            AgentObjectData *members;
};

static PyObject *
Agent_new(PyTypeObject *type, PyObject *args, PyObject * /*kwds*/)
{
    int width, height;
    if (!PyArg_ParseTuple(args, "ii", &width, &height))
        return nullptr;

    PyObject *self = type->tp_alloc(type, 0);
    if (!self)
        return nullptr;
    try {
        auto data = new AgentObjectData({ width, height });
        reinterpret_cast<AgentObject *>(self)->members = data;
    } catch (std::exception &e) {
        Py_DECREF(self);
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return nullptr;
    }

    // /*
    //  * I feel like we shouldn't have to manually increment the refcount here,
    //  * but if we don't then we get a segfault on exit ¯\_(ツ)_/¯
    //  */
    // Py_INCREF(self);
    return self;
}

static void
Agent_dealloc(AgentObject *self)
{
    if (self->members) {
        delete self->members;
    }
    Py_TYPE(self)->tp_free(reinterpret_cast<PyObject *>(self));
    LOGD << "Agent object deallocated";
}

static PyObject *
Agent_load_world(AgentObject *self, PyObject *args)
{
    char *filepath_c;
    if (!PyArg_ParseTuple(args, "s", &filepath_c)) {
        return nullptr;
    }

    auto &world = self->members->renderer.getWorld();
    try {
        const filesystem::path filepath = filepath_c;
        const auto ext = filepath.extension();
        if (ext == "bin") {
            // Load with default world and ground colours
            world.load(filepath, { 0.0f, 1.0f, 0.0f }, { 0.898f, 0.718f, 0.353f });
        } else if (ext == "obj") {
            world.loadObj(filepath);
        } else {
            throw std::runtime_error{ "Unknown file type" };
        }
    } catch (std::exception &e) {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return nullptr;
    }

    // Return world limits as a tuple of tuples: (xlim, ylim, zlim) = ...
    const auto &worldMin = world.getMaxBound();
    const auto &worldMax = world.getMinBound();
    auto xlim = PyTuple_New(2);
    PyTuple_SET_ITEM(xlim, 1, PyFloat_FromDouble(worldMin.x().value()));
    PyTuple_SET_ITEM(xlim, 0, PyFloat_FromDouble(worldMax.x().value()));
    auto ylim = PyTuple_New(2);
    PyTuple_SET_ITEM(ylim, 1, PyFloat_FromDouble(worldMin.y().value()));
    PyTuple_SET_ITEM(ylim, 0, PyFloat_FromDouble(worldMax.y().value()));
    auto zlim = PyTuple_New(2);
    PyTuple_SET_ITEM(zlim, 1, PyFloat_FromDouble(worldMin.z().value()));
    PyTuple_SET_ITEM(zlim, 0, PyFloat_FromDouble(worldMax.z().value()));

    auto worldBounds = PyTuple_New(3);
    PyTuple_SET_ITEM(worldBounds, 0, xlim);
    PyTuple_SET_ITEM(worldBounds, 1, ylim);
    PyTuple_SET_ITEM(worldBounds, 2, zlim);
    return worldBounds;
}

static PyObject *
Agent_read_frame(AgentObject *self, PyObject *)
{
    const auto size = self->members->agent.getOutputSize();

    // Allocate new numpy array
    const npy_intp dims[3] = { size.height, size.width, 3 };
    auto array = PyArray_SimpleNew(3, dims, NPY_UINT8);
    if (!array)
        return nullptr;

    try {
        // A cv::Mat wrapper for the allocated data
        auto data = PyArray_DATA(reinterpret_cast<PyArrayObject *>(array));
        cv::Mat frame{ size.height, size.width, CV_8UC3, data };
        self->members->agent.readFrameSync(frame);
        BOB_ASSERT(frame.type() == CV_8UC3);
    } catch (std::exception &e) {
        Py_DECREF(array);
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return nullptr;
    }

    // array is now populated with image data
    return array;
}

static PyObject *
Agent_read_frame_greyscale(AgentObject *self, PyObject *)
{
    const auto size = self->members->agent.getOutputSize();

    // Allocate new numpy array
    const npy_intp dims[2] = { size.height, size.width };
    auto array = PyArray_SimpleNew(2, dims, NPY_UINT8);
    if (!array)
        return nullptr;

    try {
        // A cv::Mat wrapper for the allocated data
        auto data = PyArray_DATA(reinterpret_cast<PyArrayObject *>(array));
        cv::Mat frame{ size.height, size.width, CV_8UC1, data };
        self->members->agent.readGreyscaleFrameSync(frame);
        BOB_ASSERT(frame.type() == CV_8UC1);
    } catch (std::exception &e) {
        Py_DECREF(array);
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return nullptr;
    }

    // array is now populated with image data
    return array;
}

static PyObject *
Agent_set_position(AgentObject *self, PyObject *args)
{
    using namespace units::length;

    double x, y, z;
    if (!PyArg_ParseTuple(args, "ddd", &x, &y, &z)) {
        return nullptr;
    }

    // Assume no exceptions thrown
    self->members->agent.setPosition(meter_t{ x }, meter_t{ y }, meter_t{ z });

    Py_RETURN_NONE;
}

static PyObject *
Agent_set_attitude(AgentObject *self, PyObject *args)
{
    using namespace units::angle;

    double yaw, pitch, roll;
    if (!PyArg_ParseTuple(args, "ddd", &yaw, &pitch, &roll)) {
        return nullptr;
    }

    // Assume no exceptions thrown
    self->members->agent.setAttitude(degree_t{ yaw }, degree_t{ pitch }, degree_t{ roll });

    Py_RETURN_NONE;
}

static PyObject *
Agent_update_display(AgentObject *self, PyObject *)
{
    self->members->agent.update();
    Py_RETURN_NONE;
}

static PyMethodDef Agent_methods[] = {
    { "load_world", (PyCFunction) Agent_load_world, METH_VARARGS,
	  "Load a 3D world from a .obj or .bin file" },
	{ "read_frame_greyscale", (PyCFunction) Agent_read_frame_greyscale, METH_NOARGS,
	  "Read the current view as a numpy array" },
	{ "set_position", (PyCFunction) Agent_set_position, METH_VARARGS,
	  "Set the agent's current position" },
	{ "set_attitude", (PyCFunction) Agent_set_attitude, METH_VARARGS,
	  "Set the agent's current attitude" },
	{ "update_display", (PyCFunction) Agent_update_display, METH_NOARGS,
	  "Render to the current window (you don't need to call this explicitly if calling read_frame)" },
    {}
};

static PyTypeObject AgentType = {
    PyVarObject_HEAD_INIT(&PyType_Type, 0)
    "antworld.Agent",           /* tp_name */
    sizeof(AgentObject),        /* tp_basicsize */
    0,                          /* tp_itemsize */
    (destructor) Agent_dealloc, /* tp_dealloc */
    0,                          /* tp_print */
    0,                          /* tp_getattr */
    0,                          /* tp_setattr */
    0,                          /* tp_reserved */
    0,                          /* tp_repr */
    0,                          /* tp_as_number */
    0,                          /* tp_as_sequence */
    0,                          /* tp_as_mapping */
    0,                          /* tp_hash */
    0,                          /* tp_call */
    0,                          /* tp_str */
    0,                          /* tp_getattro */
    0,                          /* tp_setattro */
    0,                          /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT,         /* tp_flags */
    0,                          /* tp_doc */
    0,                          /* tp_traverse */
    0,                          /* tp_clear */
    0,                          /* tp_richcompare */
    0,                          /* tp_weaklistoffset */
    0,                          /* tp_iter */
    0,                          /* tp_iternext */
    Agent_methods,              /* tp_methods */
    0,                          /* tp_members */
    0,                          /* tp_getset */
    0,                          /* tp_base */
    0,                          /* tp_dict */
    0,                          /* tp_descr_get */
    0,                          /* tp_descr_set */
    0,                          /* tp_dictoffset */
    0,                          /* tp_init */
    0,                          /* tp_alloc */
    (newfunc) Agent_new,        /* tp_new */
};

static struct PyModuleDef ModuleDefinitions
{
    PyModuleDef_HEAD_INIT,
	"antworld",
	"A Python wrapper for the BoB robotics ant world module",
	-1,
};

PyMODINIT_FUNC
PyInit_antworld(void)
{
    import_array();             // init numpy
    BoBRobotics::initLogging(); // init plog

    if (PyType_Ready(&AgentType) < 0)
        return nullptr;

    PyObject *pModule = PyModule_Create(&ModuleDefinitions);
    if (!pModule)
        return nullptr;
    PyModule_AddStringConstant(pModule, "bob_robotics_path", BOB_ROBOTICS_PATH);

    Py_INCREF(&AgentType);
    if (PyModule_AddObject(pModule, "Agent", (PyObject *) &AgentType) < 0) {
        Py_DECREF(&AgentType);
        Py_DECREF(pModule);
    }

    return pModule;
}
