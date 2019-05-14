#pragma once

// C++ includes
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// Extra 'nix includes
#ifndef WIN32
#include <unistd.h>
#include <fcntl.h>
#endif

#include <math.h>

// OpenCV
#include <opencv2/opencv.hpp>

// BoBRobotics includes
#include "video/input.h"
#include "robots/omni2D.h"
#include "robots/tank.h"
// standardise sockets across platforms
#include "os/net.h"
#include "common/threadable.h"

namespace BoBRobotics {
namespace Net {

#define RESP_DATA_NUMS 31
#define RESP_DATA_SPIKES 32
#define RESP_DATA_IMPULSES 33
#define RESP_HELLO 41
#define RESP_RECVD 42
#define RESP_ABORT 43
#define RESP_FINISHED 44
#define AM_SOURCE 45
#define AM_TARGET 46
#define NOT_SET 99

enum dataTypes {
    ANALOG,
    EVENT,
    IMPULSE
};



class SpineML_Connection : public Threadable
{

public:
	int socket;
	unsigned char conn_type;
	std::string name;
	int size;

	SpineML_Connection()
	: socket(0)
	{
		m_Input = NULL;
		m_OutputOmni2D = NULL;
		m_OutputTank = NULL;
	}

	~SpineML_Connection()
	{
		close(socket);
	}

    void runInternal() override
    {

		while (isRunning()) {
			//std::cout << "Here! " << socket << std::endl;

			if (conn_type == AM_SOURCE) {

				//std::cout << "here src " << m_Input << std::endl;

			    // poll the camera until we get a new frame
			    while (!update()) {
			        std::this_thread::sleep_for(std::chrono::milliseconds(5));
			    }
				//std::cout << "send" << std::endl;

			} else if (conn_type == AM_TARGET) {
				//std::cout << "here targ " << m_Input << std::endl;

				//std::cout << "recv" << std::endl;
				int n = 0;
				switch (m_RobotType) {
					case TANK:
						{
							double data[2];
							int loop = 0;
							//while (loop < 100 || n < 1) {
							n = read(socket, data,sizeof(double)*2);
							int sendVal = RESP_RECVD;
							n = write(socket, &sendVal,1);
								//loop++;
							//}
							if (loop == 100) {
								// disconnect
								stop();
								close(socket);
								socket = 0;
							}
							// left wheel, right wheel
							m_OutputTank->tank(data[0], data[1]);
						}
						break;
					case OMNI:
						{
							double data[3];
							int loop = 0;
							//while (loop < 100 || n < 1) {
							n = read(socket, data,sizeof(double)*3);
							int sendVal = RESP_RECVD;
							n = write(socket, &sendVal,1);
								//loop++;
							//}
							if (loop == 100) {
								// disconnect
								stop();
								close(socket);
								socket = 0;
							}
							// f/b, l/r, turn
							std::cout << "Data! " << data[0] << " " << data[1] << " " << data[2] << std::endl;
							m_OutputOmni2D->omni2D(data[0], data[1], data[2]);
						}
						break;
					case NO_ROBOT:
						break;
				}
			} else {

			}
		}
    }

	bool update()
    {
        cv::Mat frame;
        bool newFrame = readFrame(frame);

		// send new frame
		if (newFrame) {
			int n = 0;
			// send data
			int sent_bytes = 0;
			int loop = 0;
			// convert frame to DOUBLE
			frame.convertTo(frame, CV_64F);
			while (loop < 100 && sent_bytes < sizeof(double)*size) {
				n = write(socket, frame.ptr<double>(0)+sent_bytes,sizeof(double)*size);
				if (n > 0) sent_bytes += n;
				++loop;
			}
			if (loop == 100) {
				// disconnect
				stop();
				socket = 0;
				std::cout << "disconnect" << std::endl;
			}
			std::cout << "sent data " << size << std::endl;
			// get reply
			unsigned char returnVal;
			loop = 0;
			while (loop < 100 && n < 1) {
				n = read(socket, &returnVal,1);
				++loop;
			}
			std::cout << "got reply " << size << std::endl;
			if (loop == 100) {
				// disconnect
				stop();
				socket = 0;
			}
		}

        return newFrame;
    }

	void setInput(BoBRobotics::Video::Input &input) {
		std::cout << "Setting input..." << &input << std::endl;
		m_Input = &input;
	}

	void setRobot(BoBRobotics::Robots::Omni2D &robot) {
		m_RobotType = OMNI;
		m_OutputOmni2D = &robot;
	}

	void setRobot(BoBRobotics::Robots::Tank &robot) {
		m_RobotType = TANK;
		m_OutputTank = &robot;
	}

private:
	BoBRobotics::Video::Input *m_Input;
	BoBRobotics::Robots::Omni2D *m_OutputOmni2D;
	BoBRobotics::Robots::Tank *m_OutputTank;

	cv::Mat m_Frame;

	enum RobotType {
		NO_ROBOT,
		TANK,
		OMNI
	};

	RobotType m_RobotType;

    virtual bool readFrame(cv::Mat &frame)
    {
        if (!m_Input->readFrame(m_Frame)) {
            return false;
        }

        // unwrap frame if required
       /* if (m_Unwrapper && m_ShowUnwrapped) {
            m_Unwrapper->unwrap(m_Frame, m_Unwrapped);
            frame = m_Unwrapped;
        } else {*/
            frame = m_Frame;
        //}
        return true;
    }

};

class SpineML_Network : public Threadable
{
public:

    SpineML_Network()
      : SpineML_Network(0)
    {}

	SpineML_Network(int port)
	: m_ListenSocket(NULL),
	m_NumConnections(0)
	{

		m_Port = port;

		// set up the networking code needed to talk to the model:
		if (!setupSockets()) {
			// error
			return;
		}
	}

    virtual ~SpineML_Network()
    {
		close(m_ListenSocket);
	}

/*
 * Keep accepting connections up to m_NumConnections
 */
void runInternal() override
{
    // Start listening
    if (listen(m_ListenSocket, 10)) {
        throw std::runtime_error("Error (" + std::to_string(errno) + "): Could not listen");
    }

    // for incoming connection
    sockaddr_in addr;
    socklen_t addrlen = sizeof(addr);

	int n = 0;
	unsigned char returnVal;
	unsigned char sendVal;

    // loop until stopped
    while (isRunning()) {
		if (m_Input.socket == 0 || m_Output.socket == 0) {
		    // wait for incoming TCP connection
		    std::cout << "Waiting for incoming connection..." << std::endl;
			unsigned char conn_type = 0;
			int new_socket = accept(m_ListenSocket, (sockaddr *) &addr, &addrlen);
			// work out what type of connection we have and fill struct
			n = read(new_socket, &returnVal,1);
			if (n < 0) goto err;
			if (returnVal == AM_SOURCE) {
				// he's a source so we're a target
				conn_type = AM_TARGET;
				m_Output.conn_type = AM_TARGET;
				m_Output.socket = new_socket;
			} else if (returnVal == AM_TARGET) {
				// he's a target so we're a source
				conn_type = AM_SOURCE;
				m_Input.conn_type = AM_SOURCE;
				m_Input.socket = new_socket;
			} else {
				throw std::runtime_error("Error handshaking: bad data " + std::to_string(returnVal));
			}
			sendVal = RESP_HELLO;
			n = write(new_socket, &sendVal,1);
			if (n < 0) {
				throw std::runtime_error("Error writing in handshake");
			}

			// data type
			n = read(new_socket, &returnVal,1);
			if (n < 0 || returnVal != RESP_DATA_NUMS) {
				throw std::runtime_error("Error getting datatype (must be Analog currently)");
			}
			sendVal = RESP_RECVD;
			n = write(new_socket, &sendVal,1);
			if (n < 0) {
				throw std::runtime_error("Error writing in datatype");
			}

			// data size
			if (conn_type == AM_TARGET) {
				n = read(new_socket, (char *) &(m_Output.size),sizeof(int));
			} else {
				n = read(new_socket, (char *) &(m_Input.size),sizeof(int));
			}
			if (n < 0) {
				throw std::runtime_error("Error getting data size");
			}
			sendVal = RESP_RECVD;
			n = write(new_socket, &sendVal,1);
			if (n < 0) {
				throw std::runtime_error("Error writing in data size");
			}

			// name
			int name_size = 0;
			n = read(new_socket, (char *) &(name_size),sizeof(int));
			if (n < 0) {
				throw std::runtime_error("Error getting name size");
			}
			if (conn_type == AM_TARGET) {
				m_Input.name.resize(name_size);
				n = read(new_socket, (char *) &(m_Output.name[0]),name_size);
			} else {
				m_Output.name.resize(name_size);
				n = read(new_socket, (char *) &(m_Input.name[0]),name_size);
			}
			if (n < 0) {
				throw std::runtime_error("Error getting name");
			}
			sendVal = RESP_RECVD;
			n = write(new_socket, &sendVal,1);
			if (n < 0) {
				throw std::runtime_error("Error writing in name");
			}

		} else {
			// we have all the required connections...
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
	return;
err:
     throw std::runtime_error("Error (" + std::to_string(errno) + "): Could not read from socket");

}

void setInput(BoBRobotics::Video::Input &input) {
	m_Input.setInput(input);
}

void setRobot(BoBRobotics::Robots::Omni2D &robot) {
	m_Output.setRobot(robot);
}

void setRobot(BoBRobotics::Robots::Tank &robot) {
	m_Output.setRobot(robot);
}

bool haveAllConnections() {
	if (m_Input.socket != 0 && m_Output.socket != 0) {
		return true;
	} else {
		return false;
	}
}

void runConnections()
{
	m_Input.runInBackground();
	m_Output.runInBackground();
}

void stopConnections()
{
	m_Input.stop();
	m_Output.stop();
}

private:
	bool setupSockets()
	{
		struct sockaddr_in addr;

		m_ListenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (m_ListenSocket == INVALID_SOCKET) {
		    goto error;
		}

		memset(&addr, 0, sizeof(addr));
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = htonl(INADDR_ANY);
		addr.sin_port = htons(m_Port);

		if (bind(m_ListenSocket, (const sockaddr *) &addr, (int) sizeof(addr))) {
		    goto error;
		}

		return true;

error:
		std::cerr << "Error (" << errno << "): SpineML TCP " << m_Port
		          << std::endl;
		exit(1);
	}

	int m_ListenSocket;
	int m_Port;
	int m_NumConnections;
	SpineML_Connection m_Input;
	SpineML_Connection m_Output;

}; // SpineML_Network
} // Net
} // BoBRobotics
