// C++ includes
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// net includes
#include "../../os/net.h"

// Extra 'nix includes
#ifndef WIN32
#include <unistd.h>
#include <fcntl.h>
#endif

int m_Socket;
int m_Port;

int m_Send_Socket;
int m_Send_Port;

bool readFrame()
{

	unsigned char buffer[72*19];

	if (!m_Socket) return false;

	// get the most recent UDP frame (grayscale for now)
	while (recv(m_Socket, buffer, 72*19, 0) > 0) {
		// fill in the outFrame
		//std::cout << (int) buffer[0] << std::endl;        
		// ping the buffer onwards!
		send(m_Send_Socket, buffer, 72*19, 0);
		
	} 
	return true;
}


bool setupSockets()
{
	int which_conn = 0;
	{
	struct sockaddr_in addr;

	m_Socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (m_Socket == INVALID_SOCKET) {
		goto error;
	}

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons(m_Port);

	if (bind(m_Socket, (const sockaddr *) &addr, (int) sizeof(addr))) {
		goto error;
	}

	// non-blocking socket
	fcntl(m_Socket, F_SETFL, O_NONBLOCK); 
	}
	{
	which_conn = 1;
	struct sockaddr_in addr;

	m_Send_Socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (m_Send_Socket == INVALID_SOCKET) {
		goto error;
	}

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = inet_addr("192.168.1.5");
	addr.sin_port = htons(m_Send_Port);

	if (bind(m_Send_Socket, (const sockaddr *) &addr, (int) sizeof(addr))) {
		goto error;
	}

	struct in_addr interface_addr;
	inet_pton(AF_INET, "192.168.1.5", &(interface_addr));
	setsockopt (m_Send_Socket, IPPROTO_IP, IP_UNICAST_IF, &interface_addr, sizeof(interface_addr));
	setsockopt (m_Send_Socket, IPPROTO_IP, IP_MULTICAST_IF, &interface_addr, sizeof(interface_addr));

	// non-blocking socket
	fcntl(m_Send_Socket, F_SETFL, O_NONBLOCK); 

	struct sockaddr_in remoteaddr;
	remoteaddr.sin_family = AF_INET;
	remoteaddr.sin_addr.s_addr = inet_addr("10.0.0.2");
	remoteaddr.sin_port = htons(m_Send_Port);
	connect(m_Send_Socket, (struct sockaddr *)&remoteaddr, sizeof(remoteaddr));
	}

	return true;

error:
	int port = which_conn > 0 ? m_Send_Port : m_Port; 
	std::cerr << "Error (" << errno << "): RPi Cam UDP " << port
			  << std::endl;
	exit(1);
}

struct cmds {
	double roll;
	double pitch;
	double yaw;
	double throttle;
	bool have_new;
};

cmds getMotorCommands() 
{
	double buffer[4];
	
	cmds commands;
	commands.have_new = false;

	while (recv(m_Send_Socket, (char *) buffer, 4*sizeof(double), 0) > 0) {
		commands.roll = buffer[0];
		commands.pitch = buffer[1];
		commands.yaw = buffer[2];
		commands.throttle = buffer[3];
		commands.have_new = true;
		//std::cout << "m" << std::endl;
	}

	return commands;
} 

