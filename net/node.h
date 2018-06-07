/*
 * An abstract class inherited by Server and Client classes.
 */

#pragma once

// C++ includes
#include <atomic>
#include <functional>
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

using CommandHandler = std::function<void(Node &, const Command &)>;
using ConnectedHandler = std::function<void(Node &)>;

class Node : public Threadable
{
public:
    virtual Socket *getSocket() const = 0;

    /*
     * Add a handler for a specified type of command (e.g. if it's an IMG command,
     * it should be handled by Video::NetSource).
     */
    void addCommandHandler(const std::string commandName, const CommandHandler handler)
    {
        m_CommandHandlers.emplace(commandName, handler);
    }

    void addConnectedHandler(const ConnectedHandler handler)
    {
        m_ConnectedHandlers.push_back(handler);
        if (m_IsConnected) {
            handler(*this);
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
    
    bool isConnected() const{ return m_IsConnected; }

protected:
    std::atomic<bool> m_IsConnected{false};

    void notifyConnectedHandlers()
    {
        m_IsConnected = true;
        for (auto handler : m_ConnectedHandlers) {
            handler(*this);
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
            CommandHandler &handler = m_CommandHandlers.at(command[0]);
            handler(*this, command);
            return true;
        } catch (std::out_of_range &) {
            return false;
        }
    }

private:
    std::map<std::string, CommandHandler> m_CommandHandlers;
    std::vector<ConnectedHandler> m_ConnectedHandlers;

}; // Node
} // Net
} // GeNNRobotics
