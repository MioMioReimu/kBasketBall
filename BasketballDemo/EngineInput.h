#pragma once
#include"main.h"

#define MouseBufferSize 10
#define KeyBufferSize 20
static LPDIRECTINPUT8 DirectInput=NULL;
BOOL CALLBACK EnumJSCallback(LPCDIDEVICEINSTANCE,LPVOID);

class Input
{
private:
	LPDIRECTINPUTDEVICE8 KeyboardDevice;//键盘设备指针
	LPDIRECTINPUTDEVICE8 MouseDevice;//鼠标设备指针
	DIDEVICEOBJECTDATA MouseBuffer[MouseBufferSize];//鼠标缓冲区
	DIDEVICEOBJECTDATA KeyBuffer[KeyBufferSize];
	DIMOUSESTATE MouseState;	//鼠标状态
public:
	unsigned char KeyList[256];			//键盘状态表
	long totalMouseMoveX,totalMouseMoveY,totalMouseMoveZ;//鼠标总移动量
	int lastMouseMoveX,lastMouseMoveY,lastMouseMoveZ;//鼠标上次移动量
public:
	Input();
	~Input();
	bool InitInput(HINSTANCE hInst,HWND hWnd);
	bool UpdateKeyboard();
	bool UpdateMouse();
	bool KeyDown(int key)
	{
		return (KeyList[key]&0x80)?true:false;
	}
	bool KeyUp(int key)
	{
		return (KeyList[key]&0x80)?false:true;
	}
	bool KeyPressed(int key);
	long GetMouseTotalMoveX()
	{
		return totalMouseMoveX;
	}
	long GetMouseTotalMoveY()
	{
		return totalMouseMoveY;
	}
	void GetMouseLastMove(int &dx,int &dy)
	{
		dx=lastMouseMoveX;dy=lastMouseMoveY;
	};
	int GetWheelTotalMove()
	{
		return totalMouseMoveZ;
	}
	int GetWheelLastMove()
	{
		return lastMouseMoveZ;
	}
	bool ButtonPressed(int button);	//上次更新到这次更新的时间内某个键是否被按过
	bool ButtonDown(int button);		//这次更新的时候某个键是否被按下
	bool ButtonUp(int button);			//这次更新的时候某个键是否未被按下
private:
	bool CreateDirectInput(HINSTANCE hInst);
	bool CreateKeyboard(HWND hWnd);
	bool CreateMouse(HWND hWnd);
	void InputRelease();
};