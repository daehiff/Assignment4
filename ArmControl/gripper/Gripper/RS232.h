#ifndef RS232_H
#define RS232_H
#include <tchar.h>
#pragma once

#include <tchar.h>
#pragma once
#include <iostream>
#include <fstream>
#include <windows.h>
#include <commctrl.h>
#include <cstring>
#include <ctime>
using namespace std;

#define COM 2

int init_rs232(int, int, int, int, int);
void reset_rs232(int);
void out_com(int, char);
void out_coms(int, const char*, int);
int in_com(int, char*, int);
int in_coms(int, char*, int, int);

const int LCOM = 1;
const int RCOM = 2;
#endif