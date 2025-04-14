#ifndef SERIAL_H
#define SERIAL_H

#include <iostream>
#include <windows.h>
#include <stdio.h>
#include <string>

#define MAX_PORTS 20

class SERIAL{
	
	private:
	
	int BAUD_RATE;        // Baud rate for communication
	char workingPort[20]; // Buffer for storing the working COM port
	HANDLE hSerial;       // Handle to the serial port
	DWORD bytesRead = 0;  // Stores number of bytes read
	
	
	public:
	
	
	SERIAL(int rate);
	// rate = baud rate 
	
	
	bool CheckCOMPort(const char* portName);
	//loops to all the available ports
	
	bool FindWorkingCOMPort();
	//loops to the working port and stores it in a buffer
	
	void read(char* outBuffer, size_t bufferSize, bool& flag);
	//handles all the data transfers 
	void readLine(char* outBuffer, size_t bufferSize, bool& flag);
	
	
	void begin(int interval = 50, int total = 50, int multiplier = 10);
	/*
	interval	=	Maximum time (in milliseconds) between receiving two characters in ms.
	total		=	Constant time-out value (in milliseconds) for read operations in ms.
	multiplier	=	Multiplier for write operation time-out, based on bytes written in ms.
	*/
	
	void stop(); // self explinatory
	
};
#endif