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

    static constexpr size_t DefaultBufferSize = 1024 * 8; //! Default buffer size, in bytes
    static constexpr int DefaultListenPort = 2000;        //! Default listening port

    template<typename... Ts>
    Connection(Ts&&... args)
      : m_Buffer(DefaultBufferSize)
      , m_Socket(std::forward<Ts>(args)...)
      , m_SendMutex(std::make_unique<std::mutex>())
      , m_CommandHandlersMutex(std::make_unique<std::mutex>())
    {}

    virtual ~Connection() override
    {
        close();
    }

    void close()
    {
        if (m_Socket.isOpen()) {
            getSocketWriter().send("BYE\n");
            m_Socket.close();
        }

        // Wait for thread to terminate
        stop();
    }

    bool isOpen() const { return m_Socket.isOpen(); }

    /*!
     * \brief Add a handler for a specified type of command
     *
     * e.g. if it's an IMG command, it should be handled by Video::NetSource.
     * Set to nullptr to disable and ignore these commands.
     */
    void setCommandHandler(const std::string &commandName, const CommandHandler handler)
    {
        std::lock_guard<std::mutex> guard(*m_CommandHandlersMutex);
        m_CommandHandlers.emplace(commandName, handler);
    }

    //! Read a specified number of bytes into a buffer
    void read(void *buffer, size_t length)
    {
        // initially, copy over any leftover bytes in m_Buffer
        auto cbuffer = reinterpret_cast<char *>(buffer);
        if (m_BufferBytes > 0) {
            size_t tocopy = std::min(length, m_BufferBytes);
            std::copy_n(&m_Buffer[m_BufferStart], tocopy, cbuffer);
            cbuffer += tocopy;
            length -= tocopy;
            debitBytes(tocopy);
        }

        // keep reading from socket until we have enough bytes
        while (length > 0) {
            size_t nbytes = m_Socket.read(cbuffer, length);
            cbuffer += nbytes;
            length -= nbytes;
        }
    }

    //! Return a transaction object for writing to this Connection's Socket
    SocketWriter getSocketWriter()
    {
        return SocketWriter(*this);
    }

    std::string readNextCommand()
    {
        Command command = readCommand();
        parseCommand(command);
        return command[0];
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
            command = readCommand();
        } while (isRunning() && parseCommand(command));
    }

private:
    std::map<std::string, CommandHandler> m_CommandHandlers;
    std::vector<char> m_Buffer;
    Socket m_Socket;
    std::unique_ptr<std::mutex> m_SendMutex;
    std::unique_ptr<std::mutex> m_CommandHandlersMutex;
    size_t m_BufferStart = 0;
    size_t m_BufferBytes = 0;

    bool parseCommand(Command &command)
    {
        if (command[0] == "BYE") {
            m_Socket.close();
            return false;
        }
        if (command[0] == "HEY") {
            return true;
        }

        try {
            std::lock_guard<std::mutex> guard(*m_CommandHandlersMutex);
            CommandHandler &handler = m_CommandHandlers.at(command[0]);

            // handler will be nullptr if it has been removed
            if (handler) {
                handler(*this, command);
            }
            return true;
        } catch (std::out_of_range &) {
            throw BadCommandError();
        }
    }

    //! Read a plaintext command, splitting it into separate words
    Command readCommand()
    {
        std::string line = readLine();
        std::istringstream iss(line);
        Command results(std::istream_iterator<std::string>{ iss },
                        std::istream_iterator<std::string>());
        return results;
    }

        //! Read a single line in, stopping at a newline char
    std::string readLine()
    {
        std::ostringstream oss;
        while (true) {
            if (m_BufferBytes == 0) {
                m_BufferBytes += m_Socket.read(&m_Buffer[m_BufferStart],
                                               DefaultBufferSize - m_BufferStart);
            }

            // look for newline char
            for (size_t i = 0; i < m_BufferBytes; i++) {
                char &c = m_Buffer[m_BufferStart + i];
                if (c == '\n') {
                    c = '\0';
                    oss << std::string(&m_Buffer[m_BufferStart]);
                    debitBytes(i + 1);

                    std::string outstring = oss.str();
#ifdef TRACE_NET
                    std::cout << "<<< " << outstring << std::endl;
#endif
                    return outstring;
                }
            }

            // if newline is not present, append the text we received and try
            // another read
            oss << std::string(&m_Buffer[m_BufferStart], m_BufferBytes);
            debitBytes(m_BufferBytes);
        }
    }

    // Debit the byte store by specified amount
    void debitBytes(const size_t nbytes)
    {
        m_BufferStart += nbytes;
        if (m_BufferStart == DefaultBufferSize) {
            m_BufferStart = 0;
        }
        m_BufferBytes -= nbytes;
    }

}; // Connection
} // Net
} // BoBRobotics
