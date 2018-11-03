#pragma once

// BoB robotics includes
#include "../common/threadable.h"
#include "socket.h"

// Standard C++ includes
#include <atomic>
#include <functional>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace BoBRobotics {
namespace Net {
//! Provides a thread-safe interface for writing to Sockets
class SocketWriter {
public:
    SocketWriter(Socket &socket, std::mutex &mutex)
      : m_Socket(socket)
      , m_Mutex(mutex)
    {
        m_Mutex.lock();
    }

    ~SocketWriter()
    {
        m_Mutex.unlock();
    }

    // Object is non-copyable
    SocketWriter(const SocketWriter &) = delete;
    void operator=(const SocketWriter &) = delete;
    SocketWriter(SocketWriter &&) = default;
    SocketWriter &operator=(SocketWriter &&) = default;

    //! Send data via the Socket
    template<typename... Args>
    void send(Args&&... args)
    {
        m_Socket.send(std::forward<Args>(args)...);
    }

private:
    Socket &m_Socket;
    std::mutex &m_Mutex;
};

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
    Node()
    {
        // Make sure Windows networking is initialised
        OS::Net::WindowsNetworking::initialise();
    }

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

    //! Read from this Node's Socket into buffer
    void read(void *buffer, size_t len)
    {
        getSocket()->read(buffer, len);
    }

    //! Return a transaction object for writing to this Node's Socket
    SocketWriter getSocketWriter()
    {
        return SocketWriter(*getSocket(), m_SendMutex);
    }

protected:
    std::atomic<bool> m_IsConnected{ false };

    //! Gets the socket currently associated with this connection
    virtual Socket *getSocket() = 0;

    void notifyConnectedHandlers()
    {
        m_IsConnected = true;
        for (auto handler : m_ConnectedHandlers) {
            handler(*this);
        }
    }

    virtual void runInternal() override
    {
        Command command;
        do {
            command = getSocket()->readCommand();
        } while (isRunning() && parseCommand(command));
    }

    void disconnect()
    {
        stop();
        Socket *sock = getSocket();
        if (sock && sock->valid()) {
            sock->send("BYE\n");
            sock->close();
        }
    }

private:
    std::map<std::string, CommandHandler> m_CommandHandlers;
    std::mutex m_SendMutex;
    std::vector<ConnectedHandler> m_ConnectedHandlers;

    bool parseCommand(Command &command)
    {
        if (command[0] == "BYE") {
            getSocket()->close();
            return false;
        }
        if (command[0] == "HEY") {
            return true;
        }
        if (tryRunHandler(command)) {
            return true;
        }

        throw BadCommandError();
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

}; // Node
} // Net
} // BoBRobotics
