#pragma once

#include "windows.h"

class CSerialPort
{
public:

	CSerialPort(void);
	~CSerialPort(void);

	// prototypes
	void OpenSerial( void );
	void Send( char* data );
	void CloseSerial( void );

protected:

	HANDLE SerialPort;
	int Error;
	unsigned long dwNumberOfBytesWritten;
};
