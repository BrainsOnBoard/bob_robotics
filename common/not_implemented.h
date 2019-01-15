#pragma once

// Standard C++ includes
#include <sstream>
#include <stdexcept>

//! A macro which produces a function body that throws at runtime
#define BOB_NOT_IMPLEMENTED(funcdef)                                                                                          \
    funcdef                                                                                                                   \
    {                                                                                                                         \
        std::stringstream ss;                                                                                                      \
        ss << "The function " << __FUNCTION__ << " is not implemented for this class (" << __FILE__ << " at " << __LINE__ << ")"; \
        throw std::runtime_error(ss.str());                                                                                   \
    }
