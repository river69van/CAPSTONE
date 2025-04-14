#include "serial.h"
#include <cstdio> 
#include <iostream>

SERIAL::SERIAL(int rate){
	BAUD_RATE = rate;
}

bool SERIAL::CheckCOMPort(const char* portName) {
    HANDLE tempHandle = CreateFile(portName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (tempHandle != INVALID_HANDLE_VALUE) {
        CloseHandle(tempHandle);
        return true;
    }
    return false;
}

bool SERIAL::FindWorkingCOMPort() {
    for (int i = 1; i <= MAX_PORTS; i++) {
        snprintf(workingPort, sizeof(workingPort), "\\\\.\\COM%d", i);
        if (CheckCOMPort(workingPort)) {
            return true;
        }
    }
    MessageBox(NULL, "No active COM port found!", "Error", MB_ICONERROR);
    return false;
}

void SERIAL::begin(int interval, int total, int multiplier) {
    if (hSerial != INVALID_HANDLE_VALUE) {
        CloseHandle(hSerial);
    }

    hSerial = CreateFile(workingPort, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (hSerial == INVALID_HANDLE_VALUE) {
        MessageBox(NULL, "Error: Cannot open COM port!", "Error", MB_ICONERROR);
        return;
    }

    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(hSerial, &dcbSerialParams)) {
        MessageBox(NULL, "Failed to get serial port state!", "Error", MB_ICONERROR);
        CloseHandle(hSerial);
        return;
    }

    dcbSerialParams.BaudRate = BAUD_RATE;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity   = NOPARITY;

    SetCommState(hSerial, &dcbSerialParams);

    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = interval;
    timeouts.ReadTotalTimeoutConstant = total;
    timeouts.ReadTotalTimeoutMultiplier = multiplier;
    SetCommTimeouts(hSerial, &timeouts);
}

void SERIAL::read(char* outBuffer, size_t bufferSize, bool& flag) {
    if (hSerial == INVALID_HANDLE_VALUE) {
		
		
		//message box for an error if the serial port can't be opened 
        int notDetected = MessageBox(NULL, "Serial port not open!", "Error", MB_OK | MB_ICONERROR);
		
		if(notDetected == IDOK || notDetected == IDCANCEL){// to check if the message box is pressed in anyway
			flag = false; // for external trigger for the serial loop to recieve 
		}
        outBuffer[0] = '\0';
        return;
    }
	//memset(outBuffer, 0, bufferSize);
	 

    if (ReadFile(hSerial, outBuffer, bufferSize - 1, &bytesRead, NULL)) {
        outBuffer[bytesRead] = '\0';
    } else {
		
		//message box if the serial port is removed
       int buttonOkremovedPort = MessageBox(NULL, "Error reading from serial port!", "Error", MB_OK | MB_ICONERROR);
		if(buttonOkremovedPort == IDOK || buttonOkremovedPort == IDCANCEL){ // to check if the message box is pressed in anyway
			flag = false;
		}
        outBuffer[0] = '\0';
    }
	
	
	/*
	 while (true) {  // Keep reading until newline is found
        if (ReadFile(hSerial, outBuffer, bufferSize - 1, &bytesRead, NULL)) {
            outBuffer[bytesRead] = '\0';
            if (strchr(outBuffer, '\n')) break;  // Stop if newline is received
        } else {
            MessageBox(NULL, "Error reading from serial port!", "Error", MB_OK | MB_ICONERROR);
            flag = false;
            return;
        }
    }
	*/
}

void SERIAL::readLine(char* outBuffer, size_t bufferSize, bool& flag) {
    if (hSerial == INVALID_HANDLE_VALUE) {
        MessageBox(NULL, "Serial port not open!", "Error", MB_OK | MB_ICONERROR);
        flag = false;
        outBuffer[0] = '\0';
        return;
    }

    char tempBuffer[256]; // Temporary buffer
    DWORD bytesRead;
    size_t index = 0;

    while (index < bufferSize - 1) {
        char ch;
        DWORD bytesReadSingle;
        
        // Read one character at a time
        if (!ReadFile(hSerial, &ch, 1, &bytesReadSingle, NULL) || bytesReadSingle == 0) {
            break;  // Stop if no more data
        }

        if (ch == '\n') {  // Newline detected, message complete
            break;
        }

        tempBuffer[index++] = ch;
    }

    tempBuffer[index] = '\0';  // Null-terminate the string
    strncpy(outBuffer, tempBuffer, bufferSize);
}


void SERIAL::stop() {
    if (hSerial != INVALID_HANDLE_VALUE) {
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
    }
}