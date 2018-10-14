#pragma once

// Standard C++ includes
#include <atomic>
#include <functional>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

// BoB robotics includes
#include "../common/threadable.h"

// Local includes
#include "socket.h"

namespace BoBRobotics {
namespace Net {
class Node; // forward declaration

//! A callback function to handle incoming commands over the network
using CommandHandler = std::function<void(Node &, const Command &)>;

//! A callback function which is notified when a connection is made
using ConnectedHandler = std::function<void(Node &)>;

//----------------------------------------------------------------------------
// BoBRobotics::Net::Node
//----------------------------------------------------------------------------
//! An abstract class representing a network connection, inherited by Server and Client classes
class Node : public Threadable
{
public:
    //! Gets the socket currently associated with this connection
    virtual Socket *getSocket() const = 0;

    /*!
     * \brief Add a handler for a specified type of command
     *
     * e.g. if it's an IMG command, it should be handled by Video::NetSource.
     */
    void addCommandHandler(const std::string commandName, const CommandHandler handler)
    {
        m_CommandHandlers.emplace(commandName, handler);
    }

    //! Add a handler which is signalled when a connection is successfully made
    void addConnectedHandler(const ConnectedHandler handler)
    {
        m_ConnectedHandlers.push_back(handler);
        if (m_IsConnected) {
            handler(*this);
        }
    }

    //! Return true if this Node is currently connected
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
            throw BadCommandError();
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

    virtual void runInternal() override
    {
        while (isRunning()) {
            Socket *sock = getSocket();
            auto command = sock->readCommand();
            if (!parseCommand(command)) {
                break;
            }
        }
    }

private:
    std::map<std::string, CommandHandler> m_CommandHandlers;
    std::vector<ConnectedHandler> m_ConnectedHandlers;

}; // Node
} // Net
} // BoBRobotics
