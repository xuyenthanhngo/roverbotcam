#include "StdAfx.h"
#include "SerialPort.h"

CSerialPort::CSerialPort(void)
{
}

CSerialPort::~CSerialPort(void)
{
}

void CSerialPort::OpenSerial( void ) { // Setup a serial COM port connection

//	SerialPort = CreateFile(TEXT("COM16"), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, NULL, 0);
	SerialPort = CreateFile(TEXT("\\\\.\\COM16"), GENERIC_WRITE, NULL, NULL, OPEN_EXISTING, NULL, NULL);

	// Validate the COM port connection
	if (SerialPort==INVALID_HANDLE_VALUE) {
		Error=GetLastError();
		printf("\nError opening serial port!\nError code: %i\n\n", Error);
		system("PAUSE");
	}
	else
		printf("\nSerial port was successfully established!\n\n");

	// Verify port settings
	DCB dcbConfig;

	if(GetCommState(SerialPort, &dcbConfig)) {
		dcbConfig.BaudRate = 9600;
		dcbConfig.ByteSize = 8;
		dcbConfig.Parity = NOPARITY;
		dcbConfig.StopBits = ONESTOPBIT;
		dcbConfig.fBinary = TRUE;
		dcbConfig.fParity = TRUE;
	
		SetCommState(SerialPort, &dcbConfig);
	}
}

void CSerialPort::Send( char *data ) {

	// Send data out the COM port
	if( WriteFile( SerialPort, (LPVOID)data, 1, &dwNumberOfBytesWritten, NULL) == 0 ) {
		Error=GetLastError();
		printf("\nWrite error!\nError code: %i\n\n", Error);
	}
}

void CSerialPort::CloseSerial( void ) {

	if(SerialPort != INVALID_HANDLE_VALUE)
		CloseHandle(SerialPort);
}
