#pragma once

// BoB robotics includes
#include "../common/threadable.h"
#include "socket.h"

// Standard C++ includes
#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace BoBRobotics {
namespace Net {

class Connection; // forward declaration

//! A callback function to handle incoming commands over the network
using CommandHandler = std::function<void(Connection &, const Command &)>;

//! A callback function which is notified when a connection is made
using ConnectedHandler = std::function<void(Connection &)>;

//----------------------------------------------------------------------------
// BoBRobotics::Net::Connection
//----------------------------------------------------------------------------
//! An abstract class representing a network connection, inherited by Server and Client classes
class Connection : public Threadable
{
public:
    //! Provides a thread-safe interface for writing to Sockets
    class SocketWriter
    {
    public:
        SocketWriter(Connection &connection)
          : m_Connection(connection)
        {
            m_Connection.m_SendMutex->lock();
        }

        ~SocketWriter()
        {
            m_Connection.m_SendMutex->unlock();
        }

        // Object is non-copyable
        SocketWriter(const SocketWriter &) = delete;
        void operator=(const SocketWriter &) = delete;
        SocketWriter(SocketWriter &&) = default;
        SocketWriter &operator=(SocketWriter &&) = default;

        //! Send data via the Socket
        template<typename... Args>
        void send(Args &&... args)
        {
            m_Connection.m_Socket.send(std::forward<Args>(args)...);
        }

    private:
        Connection &m_Connection;
    };

    template<typename... Ts>
    Connection(Ts&&... args)
      : m_SendMutex(std::make_unique<std::mutex>())
      , m_Socket(std::forward<Ts>(args)...)
    {}

    virtual ~Connection()
    {
        stop();

        if (m_Socket.isOpen()) {
            m_Socket.send("BYE\n");
        }
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

    //! Read from this Connection's Socket into buffer
    void read(void *buffer, size_t len)
    {
        m_Socket.read(buffer, len);
    }

    //! Return a transaction object for writing to this Connection's Socket
    SocketWriter getSocketWriter()
    {
        return SocketWriter(*this);
    }

    // Object is non-copyable
    Connection(const Connection &) = delete;
    void operator=(const Connection &) = delete;
    Connection(Connection &&old) = default;
    Connection &operator=(Connection &&) = default;

protected:
    Socket &getSocket() { return m_Socket; }

    virtual void runInternal() override
    {
        Command command;
        do {
            command = m_Socket.readCommand();
        } while (isRunning() && parseCommand(command));
    }

private:
    std::map<std::string, CommandHandler> m_CommandHandlers;
    std::unique_ptr<std::mutex> m_SendMutex;
    Socket m_Socket;

    bool parseCommand(Command &command)
    {
        if (command[0] == "BYE") {
            m_Socket.close();
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

}; // Connection
} // Net
} // BoBRobotics
