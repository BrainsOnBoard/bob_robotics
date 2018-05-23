#pragma once

// C++ includes
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

// local includes
#include "socket.h"

namespace GeNNRobotics {
namespace Net {
using CommandHandler = void (*)(std::vector<std::string> &command, void *userData);
typedef std::pair<CommandHandler, void *> HandlerItem;

class Node
{
public:
    virtual Socket *getSocket() const = 0;
    void addHandler(std::string command, CommandHandler handler, void *userData = nullptr)
    {
        m_Handlers.emplace(command, HandlerItem(handler, userData));
    }

protected:
    bool tryRunHandler(std::vector<std::string> &command)
    {
        HandlerItem handler;
        try {
            handler = m_Handlers.at(command[0]);
        } catch (std::out_of_range &) {
            return false;
        }

        handler.first(command, handler.second);
        return true;
    }

private:
    std::map<std::string, HandlerItem> m_Handlers;

}; // Node
} // Net
} // GeNNRobotics
