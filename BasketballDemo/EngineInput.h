#pragma once
#include"main.h"

#define MouseBufferSize 10
#define KeyBufferSize 20
static LPDIRECTINPUT8 DirectInput=NULL;
BOOL CALLBACK EnumJSCallback(LPCDIDEVICEINSTANCE,LPVOID);

class Input
{
private:
	LPDIRECTINPUTDEVICE8 KeyboardDevice;//�����豸ָ��
	LPDIRECTINPUTDEVICE8 MouseDevice;//����豸ָ��
	DIDEVICEOBJECTDATA MouseBuffer[MouseBufferSize];//��껺����
	DIDEVICEOBJECTDATA KeyBuffer[KeyBufferSize];
	DIMOUSESTATE MouseState;	//���״̬
public:
	unsigned char KeyList[256];			//����״̬��
	long totalMouseMoveX,totalMouseMoveY,totalMouseMoveZ;//������ƶ���
	int lastMouseMoveX,lastMouseMoveY,lastMouseMoveZ;//����ϴ��ƶ���
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
	bool ButtonPressed(int button);	//�ϴθ��µ���θ��µ�ʱ����ĳ�����Ƿ񱻰���
	bool ButtonDown(int button);		//��θ��µ�ʱ��ĳ�����Ƿ񱻰���
	bool ButtonUp(int button);			//��θ��µ�ʱ��ĳ�����Ƿ�δ������
private:
	bool CreateDirectInput(HINSTANCE hInst);
	bool CreateKeyboard(HWND hWnd);
	bool CreateMouse(HWND hWnd);
	void InputRelease();
};