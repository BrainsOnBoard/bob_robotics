/*
 * An abstract class inherited by Server and Client classes.
 */

#pragma once

// C++ includes
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// GeNN robotics includes
#include "../common/threadable.h"

// local includes
#include "socket.h"

namespace GeNNRobotics {
namespace Net {
using CommandHandler = void (*)(Command &command,
                                void *userData);
using HandlerItem = std::pair<CommandHandler, void *>;

class Node : public Threadable
{
public:
    virtual Socket *getSocket() const = 0;

    /*
     * Add a handler for a specified type of command (e.g. if it's an IMG command,
     * it should be handled by Video::NetSource).
     */
    void addHandler(std::string command,
                    CommandHandler handler,
                    void *userData = nullptr)
    {
        m_Handlers.emplace(command, HandlerItem(handler, userData));
    }

    /*
     * Repeatedly read and parse commands from the socket until stopped.
     */
    void run() override
    {
        while (m_DoRun) {
            Socket *sock = getSocket();
            auto command = sock->readCommand();
            if (!parseCommand(command)) {
                break;
            }
        }
    }

protected:
    virtual bool parseCommand(Command &command)
    {
        if (command[0] == "BYE") {
            return false;
        }
        if (command[0] == "HEY") {
            return true;
        }
        if (tryRunHandler(command)) {
            return true;
        } else {
            throw bad_command_error();
        }
    }

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
