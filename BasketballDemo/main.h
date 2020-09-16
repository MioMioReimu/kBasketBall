#pragma once
#ifndef MAIN_H
#define MAIN_H
#include <windows.h>

#include <stdio.h>
#include <math.h>
#include <iostream>
#include <ctime>
#include <cstdlib>
#include<string.h>
#include <fstream>
#include <vector>

#include <gl\gl.h>										
#include <gl\glu.h>										
#include <gl\glaux.h>
#include<GL\glut.h>
#include<GL\GLEXT.H>

//DirectX Graphics 头文件
#include<d3d9.h>
#include<d3dx9.h>
#include<dinput.h>
//#define DIRECTINPUT_VERSION 0x0800
#pragma comment(lib, "dxguid.lib") 
#pragma comment(lib, "dinput8.lib") 
#pragma comment(lib,"Winmm.lib")

#include"Physics.h"

//释放对象宏
#define SafeRelease(pObject) \
	if(pObject!=NULL){pObject->Release();pObject=NULL;}
using namespace std;
using namespace Physics;
#define SCREEN_DEPTH 16
#endif