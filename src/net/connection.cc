// BoB robotics includes
#include "common/logging.h"
#include "net/connection.h"

// Standard C++ includes
#include <iterator>
#include <sstream>

namespace BoBRobotics {
namespace Net {

Connection::SocketWriter::SocketWriter(Connection &connection)
    : m_Connection(connection)
{
    m_Connection.m_SendMutex->lock();
}

Connection::SocketWriter::~SocketWriter()
{
    m_Connection.m_SendMutex->unlock();
}

Connection::~Connection()
{
    if (m_Socket.isOpen()) {
        m_Socket.send("BYE\n");
        m_Socket.close();
    }

    // Wait for thread to terminate
    LOG_DEBUG << "Waiting for connection to close...";
    stop();
    LOG_DEBUG << "Connection closed";
}

bool Connection::isOpen() const { return m_Socket.isOpen(); }

void Connection::setCommandHandler(const std::string &commandName, const CommandHandler handler)
{
    std::lock_guard<std::mutex> guard(*m_CommandHandlersMutex);
    m_CommandHandlers.emplace(commandName, handler);
}

void Connection::read(void *buffer, size_t length)
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

Connection::SocketWriter Connection::getSocketWriter()
{
    return SocketWriter(*this);
}

std::string Connection::readNextCommand()
{
    Command command = readCommand();
    parseCommand(command);
    return command[0];
}

Socket &Connection::getSocket() { return m_Socket; }

void Connection::runInternal()
{
    Command command;
    do {
        command = readCommand();
    } while (isRunning() && parseCommand(command));
}

bool Connection::parseCommand(Command &command)
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

Command Connection::readCommand()
{
    std::string line = readLine();
    std::istringstream iss(line);
    Command results(std::istream_iterator<std::string>{ iss },
                    std::istream_iterator<std::string>());
    return results;
}

std::string Connection::readLine()
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
                LOG_VERBOSE << "<<< " << outstring;

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
void Connection::debitBytes(const size_t nbytes)
{
    m_BufferStart += nbytes;
    if (m_BufferStart == DefaultBufferSize) {
        m_BufferStart = 0;
    }
    m_BufferBytes -= nbytes;
}

} // Net
} // BoBRobotics
