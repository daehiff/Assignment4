#include "Gripper.h"
#include <iostream> 
using namespace std;

void GripperTurnOn(){
	cout << "Turn on the gripper." << endl;
	char s;
	s = 0xAA;
	out_coms(COM,  &s , 1);
	Sleep(9000);
}

void GripperOpen(){
	cout << "Open the gripper." << endl;
	out_coms(COM,"OPEN", 4);
	Sleep(3000);
}

void GripperClose(){
	cout << "Close the gripper." << endl;
	out_coms(COM, "CLOSE", 5);
	Sleep(3000);
}