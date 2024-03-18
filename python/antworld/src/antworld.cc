// BoB robotics includes
#include "antworld/agent.h"
#include "common/main.h"

// Third-party includes
#include "plog/Log.h"
#include "third_party/path.h"

// Python includes
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>

#ifdef _WIN32
#define DLL_EXPORT extern "C" __declspec(dllexport)
#else
#define DLL_EXPORT
#endif

using namespace BoBRobotics;
using degree_t = units::angle::degree_t;

struct AgentObjectData
{
    AgentObjectData(const cv::Size &renderSize, GLsizei cubemapSize, 
                    double nearClip, double farClip,
                    degree_t horizontalFOV, degree_t verticalFOV)
      : window{ AntWorld::AntAgent::initialiseWindow(renderSize) }
      , renderer(cubemapSize, nearClip, farClip, horizontalFOV, verticalFOV)
      , agent(*window, renderer, renderSize)
    {}

    std::unique_ptr<sf::Window> window;
    AntWorld::Renderer renderer;
    AntWorld::AntAgent agent;
};

struct AgentObject
{
    // clang-format off
    PyObject_HEAD
    AgentObjectData *members;
    // clang-format on
};

DLL_EXPORT PyObject *
Agent_new(PyTypeObject *type, PyObject *args, PyObject *kwargs)
{
    
    static char *kwlist[] = {"width", "height", "cubemap_size", 
                             "near_clip", "far_clip", 
                             "horizontal_fov", "vertical_fov", NULL};
    
    // Read renderer settings from kwargs
    GLsizei cubemapSize = 256;
    double nearClip = 0.001;
    double farClip = 1000.0;
    double horizontalFOV = 360.0;
    double verticalFOV = 75.0;
    int width, height;
    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "ii|Idddd", kwlist, 
                                     &width, &height, &cubemapSize,
                                     &nearClip, &farClip, &horizontalFOV, &verticalFOV)) 
    {
        return nullptr;
    }

    PyObject *self = type->tp_alloc(type, 0);
    if (!self)
        return nullptr;
    try {
        auto data = new AgentObjectData({ width, height }, cubemapSize,
                                        nearClip, farClip, 
                                        degree_t{horizontalFOV}, degree_t{verticalFOV});
        reinterpret_cast<AgentObject *>(self)->members = data;
    } catch (std::exception &e) {
        Py_DECREF(self);
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return nullptr;
    }

    return self;
}

DLL_EXPORT void
Agent_dealloc(AgentObject *self)
{
    delete self->members;

    Py_TYPE(self)->tp_free(reinterpret_cast<PyObject *>(self));
    LOGD << "Agent object deallocated";
}

DLL_EXPORT PyObject *
Agent_load_world(AgentObject *self, PyObject *args)
{
    char *filepath_c;
    int clear = 1;
    if (!PyArg_ParseTuple(args, "s|p", &filepath_c, &clear)) {
        return nullptr;
    }

    auto &world = self->members->renderer.getWorld();
    try {
        const filesystem::path filepath = filepath_c;
        const auto ext = filepath.extension();
        if (ext == "bin") {
            // Load with default world and ground colours
            world.load(filepath, { 0.0f, 1.0f, 0.0f },
                       { 0.898f, 0.718f, 0.353f }, clear);
        } else if (ext == "obj") {
            world.loadObj(filepath, 1.0f, -1, GL_RGB, clear);
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

DLL_EXPORT PyObject *
Agent_read_frame(AgentObject *self, PyObject *)
{
    const auto size = self->members->agent.getOutputSize();

    // Allocate new numpy array
    npy_intp dims[3] = { size.height, size.width, 3 };
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

DLL_EXPORT PyObject *
Agent_read_frame_greyscale(AgentObject *self, PyObject *)
{
    const auto size = self->members->agent.getOutputSize();

    // Allocate new numpy array
    npy_intp dims[2] = { size.height, size.width };
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

DLL_EXPORT PyObject *
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

DLL_EXPORT PyObject *
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

DLL_EXPORT PyObject *
Agent_set_fog(AgentObject *self, PyObject *args, PyObject *kwargs)
{
    static char *kwlist[] = {"mode", "colour", "start", "end", "density", NULL};

    // Read renderer settings from kwargs
    char *mode = nullptr;
    PyObject *colour = nullptr;
    GLfloat start = 0.0f;
    GLfloat end = 0.0f;
    GLfloat density = 1.0f;
    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "s|Offf", kwlist, 
                                     &mode, &colour, &start, &end, &density)) 
    {
        return nullptr;
    }

    // If colour was passed
    if(colour) {
        // Parse colour 
        GLfloat colourArray[4]{0.75f, 0.75f, 0.75f, 1.0f};
        if (!PyArg_ParseTuple(colour, "fff|f", &colourArray[0], &colourArray[1], 
            &colourArray[2], &colourArray[3])) 
        {
            return nullptr;
        }
        
        // Set fog colour
        glFogfv (GL_FOG_COLOR, colourArray);
    }

    // Set fog start, end and density
    glFogf(GL_FOG_START, start);
    glFogf(GL_FOG_END, end);
    glFogf(GL_FOG_DENSITY, density);
    
    // If fog is disabled, turn it off
    if(strcmp(mode, "disabled") == 0) {
        glDisable(GL_FOG);
    }
    // Otherwise
    else {
        // Enable
        glEnable(GL_FOG);

        // Convert mode string to GL enumeration
        if(strcmp(mode, "linear") == 0) {
            glFogi(GL_FOG_MODE, GL_LINEAR);
        }
        else if(strcmp(mode, "exp") == 0) {
            glFogi(GL_FOG_MODE, GL_EXP);
        }
        else if(strcmp(mode, "exp2") == 0) {
            glFogi(GL_FOG_MODE, GL_EXP2);
        }
        else {
            return nullptr;
        }
    }
    Py_RETURN_NONE;
}

DLL_EXPORT PyObject *
Agent_set_clear_colour(AgentObject *self, PyObject *args)
{
    // Parse colour 
    GLfloat r = 0.75f;
    GLfloat g = 0.75f;
    GLfloat b = 0.75f;
    GLfloat a = 1.0f;
    if(!PyArg_ParseTuple(args, "fff|f", &r, &g, &b, &a)) 
    {
        return nullptr;
    }

    glClearColor(r, g, b, a);
    
    Py_RETURN_NONE;
}

DLL_EXPORT PyObject *
Agent_display(AgentObject *self, PyObject *)
{
    self->members->agent.display();
    Py_RETURN_NONE;
}

static PyMethodDef Agent_methods[] = {
    // clang-format off
    { "load_world", (PyCFunction) Agent_load_world, METH_VARARGS,
      "Load a 3D world from a .obj or .bin file" },
    { "read_frame", (PyCFunction) Agent_read_frame, METH_NOARGS,
      "Read the current view in colour as a numpy array" },
    { "read_frame_greyscale", (PyCFunction) Agent_read_frame_greyscale, METH_NOARGS,
      "Read the current view in greyscale as a numpy array" },
    { "set_position", (PyCFunction) Agent_set_position, METH_VARARGS,
      "Set the agent's current position" },
    { "set_attitude", (PyCFunction) Agent_set_attitude, METH_VARARGS,
      "Set the agent's current attitude" },
    { "set_fog", (PyCFunction) Agent_set_fog, METH_VARARGS | METH_KEYWORDS,
      "Configure fog settings for rendering" },
    { "set_clear_colour", (PyCFunction) Agent_set_clear_colour, METH_VARARGS,
      "Configure clear colour settings for rendering" },
    { "display", (PyCFunction) Agent_display, METH_NOARGS,
      "Render to the current window (you don't need to call this explicitly if calling read_frame)" },
    {}
    // clang-format on
};

static PyTypeObject AgentType = {
    // clang-format off
    PyVarObject_HEAD_INIT(&PyType_Type, 0)
    // clang-format on
    "antworld.Agent",           /* tp_name */
    sizeof(AgentObject),        /* tp_basicsize */
    0,                          /* tp_itemsize */
    (destructor) Agent_dealloc, /* tp_dealloc */
    0,                          /* tp_print */
    nullptr,                    /* tp_getattr */
    nullptr,                    /* tp_setattr */
    nullptr,                    /* tp_reserved */
    nullptr,                    /* tp_repr */
    nullptr,                    /* tp_as_number */
    nullptr,                    /* tp_as_sequence */
    nullptr,                    /* tp_as_mapping */
    nullptr,                    /* tp_hash */
    nullptr,                    /* tp_call */
    nullptr,                    /* tp_str */
    nullptr,                    /* tp_getattro */
    nullptr,                    /* tp_setattro */
    nullptr,                    /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT,         /* tp_flags */
    nullptr,                    /* tp_doc */
    nullptr,                    /* tp_traverse */
    nullptr,                    /* tp_clear */
    nullptr,                    /* tp_richcompare */
    0,                          /* tp_weaklistoffset */
    nullptr,                    /* tp_iter */
    nullptr,                    /* tp_iternext */
    Agent_methods,              /* tp_methods */
    nullptr,                    /* tp_members */
    nullptr,                    /* tp_getset */
    nullptr,                    /* tp_base */
    nullptr,                    /* tp_dict */
    nullptr,                    /* tp_descr_get */
    nullptr,                    /* tp_descr_set */
    0,                          /* tp_dictoffset */
    nullptr,                    /* tp_init */
    nullptr,                    /* tp_alloc */
    (newfunc) Agent_new,        /* tp_new */
    // NOLINTNEXTLINE
};

static struct PyModuleDef ModuleDefinitions
{
    // clang-format off
    PyModuleDef_HEAD_INIT,
    "antworld",
    "A Python wrapper for the BoB robotics ant world module",
    -1,
    // clang-format on
    // NOLINTNEXTLINE
};

PyMODINIT_FUNC
PyInit__antworld(void) // NOLINT
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
