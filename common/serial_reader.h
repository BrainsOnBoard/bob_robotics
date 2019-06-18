// USB-reader
#pragma once

#include <string>
#include "termios.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <fstream>
#include <unistd.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <chrono>
#include <thread>
#include <errno.h>

#define BAUDRATE B9600
 

class SerialReader {
    private:
    int fileDescriptor;
    bool connected = false;
   
    struct termios settings;


    public:
    SerialReader() : fileDescriptor(-1) {}
    ~SerialReader() { close(fileDescriptor); }

    //! reads the serial port (USB) and returns the string from it
    std::string readData() {
        try {
            if (fileDescriptor < 0) {
                throw "You are not connected to the device";
            }
        
            std::string serialString;
            int res=0;
            size_t received = 0;
            const size_t size = 1024;
            char buf[size];

            int bytesInBuffer = 0;
            // Get the number of bytes waiting in the serial buffer
            ioctl(fileDescriptor, FIONREAD, &bytesInBuffer);
            
            while (bytesInBuffer > 0) { 
                res = read(fileDescriptor, buf, size);   
                ioctl(fileDescriptor, FIONREAD, &bytesInBuffer);       
                buf[res] = 0; /* set end of string */      
                serialString += buf;
                received += res;  
            }
            tcflush(fileDescriptor, TCIFLUSH);
            return serialString;
        }
        catch(const char *error) {
            std::cout << "[SerialReader:readData()] : " << error << std::endl;
        }
        return "";

    }


    void connect(const char *serial_device_path) {
    
        try {        
            fileDescriptor= open(serial_device_path, O_RDWR | O_NOCTTY);
            if (fileDescriptor < 0) {
                throw "device not found";
            }    
            connected = true;

            /* 
            BAUDRATE: Set bps rate.
            CS8     : 8n1 (8bit,no parity,1 stopbit)
            CLOCAL  : local connection, no modem contol
            CREAD   : enable receiving characters */
            settings.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
            //IGNPAR  : ignore bytes with parity errors
            settings.c_iflag = IGNPAR;
            /*  Raw output  */
            settings.c_oflag = (OPOST | ONLCR);
            //ICANON  : enable canonical input
            settings.c_lflag = ICANON;

            /* now clean the modem line and activate the settings for the port */
            tcflush(fileDescriptor, TCIFLUSH);
            tcsetattr(fileDescriptor,TCSANOW,&settings);
        }
        catch(const char *s) {
            std::cout << "[SerialReader:connect()]: " << s <<  std::endl;
        }
    }

    bool isConnected() const { return connected; }


};