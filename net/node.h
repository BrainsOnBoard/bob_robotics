#pragma once

// C++ includes
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// local includes
#include "socket.h"

namespace GeNNRobotics {
namespace Net {
using Command = std::vector<std::string>;    
using CommandHandler = void (*)(Command &command,
                                void *userData);
using HandlerItem = std::pair<CommandHandler, void *>;

class Node
{
public:
    virtual Socket *getSocket() const = 0;

    void addHandler(std::string command,
                    CommandHandler handler,
                    void *userData = nullptr)
    {
        m_Handlers.emplace(command, HandlerItem(handler, userData));
    }

    void run()
    {
        while (true) {
            Socket *sock = getSocket();
            auto command = sock->readCommand();
            if (!parseCommand(command)) {
                break;
            }
        }
    }

protected:
    virtual bool parseCommand(Command &command) = 0;

    bool tryRunHandler(Command &command)
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
