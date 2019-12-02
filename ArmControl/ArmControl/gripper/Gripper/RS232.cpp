#include "RS232.h"

using namespace std;

static HANDLE  hComm[30];

int init_rs232(int com, int baud, int parity, int databits, int stopbits) //初始化comport
{
	HANDLE hCom = INVALID_HANDLE_VALUE;
	DCB dcb;
	COMMTIMEOUTS timeouts;

	if (com == 1)
		hCom = CreateFile(_T("COM1"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 2)
		hCom = CreateFile(_T("COM2"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 3)
		hCom = CreateFile(_T("COM3"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 4)
		hCom = CreateFile(_T("COM4"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 5)
		hCom = CreateFile(_T("COM5"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 6)
		hCom = CreateFile(_T("COM6"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 7)
		hCom = CreateFile(_T("COM7"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 8)
		hCom = CreateFile(_T("COM8"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 9)
		hCom = CreateFile(_T("COM9"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 10)
		hCom = CreateFile(_T("\\\\.\\COM10"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 11)
		hCom = CreateFile(_T("\\\\.\\COM11"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 12)
		hCom = CreateFile(_T("\\\\.\\COM12"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 13)
		hCom = CreateFile(_T("\\\\.\\COM13"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 14)
		hCom = CreateFile(_T("\\\\.\\COM14"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 15)
		hCom = CreateFile(_T("\\\\.\\COM15"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 16)
		hCom = CreateFile(_T("\\\\.\\COM16"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 17)
		hCom = CreateFile(_T("\\\\.\\COM17"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 18)
		hCom = CreateFile(_T("\\\\.\\COM18"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 19)
		hCom = CreateFile(_T("\\\\.\\COM19"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 20)
		hCom = CreateFile(_T("\\\\.\\COM20"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 21)
		hCom = CreateFile(_T("\\\\.\\COM21"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 22)
		hCom = CreateFile(_T("\\\\.\\COM22"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 23)
		hCom = CreateFile(_T("\\\\.\\COM23"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 24)
		hCom = CreateFile(_T("\\\\.\\COM24"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 25)
		hCom = CreateFile(_T("\\\\.\\COM25"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 26)
		hCom = CreateFile(_T("\\\\.\\COM26"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 27)
		hCom = CreateFile(_T("\\\\.\\COM27"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 28)
		hCom = CreateFile(_T("\\\\.\\COM28"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 29)
		hCom = CreateFile(_T("\\\\.\\COM29"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	else if (com == 30)
		hCom = CreateFile(_T("\\\\.\\COM30"), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	if (hCom == INVALID_HANDLE_VALUE)  {
		printf("RS232 error : CreateFile COM%d", com);
		//            exit(-1);
	}
	hComm[com - 1] = hCom;

	SetupComm(hCom, 1024, 1024);              // setup size of input/output buffer

	FillMemory(&dcb, sizeof(dcb), 0);
	dcb.DCBlength = sizeof(dcb);
	dcb.BaudRate = baud;
	dcb.fBinary = TRUE;
	//      dcb.fDtrControl = DTR_CONTROL_ENABLE;
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	//      dcb.fRtsControl = RTS_CONTROL_TOGGLE; // the RTS line is high if chars are available for transmission
	dcb.fRtsControl = RTS_CONTROL_DISABLE;
	dcb.ByteSize = databits;
	dcb.Parity = parity;
	dcb.StopBits = stopbits;
	if (!SetCommState(hCom, &dcb)) {
		printf("RS232 error : SetCommState");
		exit(-1);
	}
	timeouts.ReadIntervalTimeout = MAXDWORD;
	timeouts.ReadTotalTimeoutMultiplier = MAXDWORD;
	//      timeouts.ReadIntervalTimeout = 1;
	//      timeouts.ReadTotalTimeoutMultiplier = 1;
	timeouts.ReadTotalTimeoutConstant = 10;         // in milliseconds
	timeouts.WriteTotalTimeoutMultiplier = 0;
	timeouts.WriteTotalTimeoutConstant = 0;
	//      timeouts.WriteTotalTimeoutMultiplier = 1;
	//      timeouts.WriteTotalTimeoutConstant = 10;
	if (!SetCommTimeouts(hCom, &timeouts)) {
		printf("RS232 error : SetCommTimeouts");
		exit(-1);
	}

	if (!PurgeComm(hCom, PURGE_TXCLEAR | PURGE_RXCLEAR)) {
		printf("RS232 error : PurgeComm");
		exit(-1);
	}

	printf("initial the RS232\n");
	return(0);
}

void reset_rs232(int com)
{
	CloseHandle(hComm[com - 1]);
}

void out_com(int com, char c)    //送出一個字元到comport
{
	DWORD chars;

	--com;
	WriteFile(hComm[com], &c, 1, &chars, NULL);
}

void out_coms(int com, const char *s, int len)   //送出一個字串到comport

{
	DWORD chars;

	--com;
	WriteFile(hComm[com], s, len, &chars, NULL);
}



int in_com(int com, char *ch, int milisec)    //接收一個字元      milisec  等待的時間
{
	int timeout;
	DWORD chars;

	--com;
	for (timeout = milisec; timeout >= 0; timeout--) {
		ReadFile(hComm[com], ch, 1, &chars, NULL);
		if (chars == 1)
			return(1);
	}
	*ch = 0;
	return (0);
}

int in_coms(int com, char *s, int milisec, int len) //milisec = 10;  len = sizeof(s);
{
	int timeout;
	DWORD chars;

	--com;
	for (timeout = milisec; timeout >= 0; timeout--) {
		ReadFile(hComm[com], s, len, &chars, NULL);
		if (chars > 0)
			s[chars] = 0x00;
		return(chars);
	}
	*s = 0;
	return (0);
}