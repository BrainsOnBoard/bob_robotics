#pragma once

// BoB robotics includes
#include "common/thread.h"
#include "socket.h"

// Standard C++ includes
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
        SocketWriter(Connection &connection);
        ~SocketWriter();

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

    static constexpr size_t DefaultBufferSize = 1024 * 8; //! Default buffer size, in bytes
    static constexpr int DefaultListenPort = 2000;        //! Default listening port

    template<typename... Ts>
    Connection(Ts&&... args)
      : m_Buffer(DefaultBufferSize)
      , m_Socket(std::forward<Ts>(args)...)
      , m_SendMutex(std::make_unique<std::mutex>())
      , m_CommandHandlersMutex(std::make_unique<std::mutex>())
    {}

    virtual ~Connection() override;

    bool isOpen() const;

    /*!
     * \brief Add a handler for a specified type of command
     *
     * e.g. if it's an IMG command, it should be handled by Video::NetSource.
     * Set to nullptr to disable and ignore these commands.
     */
    void setCommandHandler(const std::string &commandName, const CommandHandler handler);

    //! Read a specified number of bytes into a buffer
    void read(void *buffer, size_t length);

    //! Return a transaction object for writing to this Connection's Socket
    SocketWriter getSocketWriter();

    std::string readNextCommand();

    // Object is non-copyable
    Connection(const Connection &) = delete;
    void operator=(const Connection &) = delete;
    Connection(Connection &&old) = default;
    Connection &operator=(Connection &&) = default;

protected:
    Socket &getSocket();
    virtual void runInternal() override;

private:
    std::map<std::string, CommandHandler> m_CommandHandlers;
    std::vector<char> m_Buffer;
    Socket m_Socket;
    std::unique_ptr<std::mutex> m_SendMutex, m_CommandHandlersMutex;
    size_t m_BufferStart = 0, m_BufferBytes = 0;

    bool parseCommand(Command &command);

    //! Read a plaintext command, splitting it into separate words
    Command readCommand();

    //! Read a single line in, stopping at a newline char
    std::string readLine();

    // Debit the byte store by specified amount
    void debitBytes(const size_t nbytes);

}; // Connection
} // Net
} // BoBRobotics
