/*
 * An abstract class inherited by Server and Client classes.
 */

#pragma once

// C++ includes
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

// GeNN robotics includes
#include "../common/threadable.h"

// local includes
#include "socket.h"

namespace GeNNRobotics {
namespace Net {
class Node; // forward declaration

class Handler
{
public:
    virtual std::string getHandledCommandName() const = 0;
    virtual void onCommandReceived(Node &node, Command &command) = 0;
    virtual void onConnected(Node &node) {}
};

class Node : public Threadable
{
public:
    virtual Socket *getSocket() const = 0;

    /*
     * Add a handler for a specified type of command (e.g. if it's an IMG command,
     * it should be handled by Video::NetSource).
     */
    void addHandler(Handler &handler)
    {
        m_Handlers.emplace(handler.getHandledCommandName(), handler);
        if (m_IsConnected) {
            handler.onConnected(*this);
        }
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
    bool m_IsConnected = false;

    void notifyHandlers()
    {
        m_IsConnected = true;
        for (auto handler : m_Handlers) {
            handler.second.onConnected(*this);
        }
    }

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
        try {
            Handler &handler = m_Handlers.at(command[0]);
            handler.onCommandReceived(std::ref(*this), command);
            return true;
        } catch (std::out_of_range &) {
            return false;
        }
    }

private:
    std::map<std::string, Handler &> m_Handlers;

}; // Node
} // Net
} // GeNNRobotics
