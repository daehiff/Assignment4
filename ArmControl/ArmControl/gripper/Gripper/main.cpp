#include "Gripper.h"
#include <windows.h>
#include "process.h"
#include "RS232.h"

using namespace std;

int main(){
	init_rs232(COM, 9600, 0, 8, 0); //com, baud rate, parity, databits, stopbits

	GripperTurnOn();

	GripperClose();

	GripperOpen();

	
	system("pause");
	return 0;
}