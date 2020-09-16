#include"EngineInput.h"
void Input::InputRelease()
{
	if(KeyboardDevice){
		KeyboardDevice->Unacquire();
	}
	if(MouseDevice)
	{
		MouseDevice->Unacquire();
	}
	SafeRelease(KeyboardDevice);
	SafeRelease(MouseDevice);
	SafeRelease(DirectInput);
}
Input::Input()
{
	this->KeyboardDevice=MouseDevice=NULL;
	totalMouseMoveX=totalMouseMoveY=totalMouseMoveZ=0;
	lastMouseMoveX=lastMouseMoveY=lastMouseMoveZ=0;
	
}
Input::~Input()
{
	InputRelease();
}
bool Input::CreateDirectInput(HINSTANCE hInst)
{
	// ����IDirectInput �ӿڶ���
	if(FAILED(DirectInput8Create(hInst,DIRECTINPUT_VERSION,IID_IDirectInput8,(void**)&DirectInput,NULL)))
	{
		MessageBox(NULL,L"����IDirectInput8 �ӿڶ���ʧ�ܡ�",L"����",
			MB_OK|MB_ICONINFORMATION);
		return false;
	}
	return true;
}
bool Input::CreateKeyboard(HWND hWnd)
{
	//�������������豸
	if(FAILED(DirectInput->CreateDevice(GUID_SysKeyboard,
		&KeyboardDevice,NULL)))
	{
		MessageBox(NULL,L"���������������ʧ�ܡ�",L"����",
			MB_OK|MB_ICONINFORMATION);
		return false;
	}

	//���ü��������豸�����ݸ�ʽ
	if(FAILED(KeyboardDevice->SetDataFormat(&c_dfDIKeyboard)))
	{
		MessageBox(NULL,L"���ü��������豸�����ݶ�ȡ���ݸ�ʽʧ��.",L"����",MB_OK|MB_ICONINFORMATION);
		InputRelease();
		return false;
	}

	//���ü��������豸��Э������
	if(FAILED(KeyboardDevice->SetCooperativeLevel(hWnd,DISCL_FOREGROUND|DISCL_NONEXCLUSIVE)))
	{
		MessageBox(NULL,L"����Э������ʧ��.",L"����",MB_OK|MB_ICONINFORMATION);
		InputRelease();
		return false;
	}
	//���ü��̻�����
	::DIPROPDWORD dipRop;
	dipRop.diph.dwHeaderSize=sizeof(::DIPROPHEADER);
	dipRop.diph.dwSize=sizeof(DIPROPDWORD);
	dipRop.diph.dwObj=0;
	dipRop.diph.dwHow=DIPH_DEVICE;
	dipRop.dwData=KeyBufferSize;
	if(FAILED(KeyboardDevice->SetProperty(DIPROP_BUFFERSIZE,&(dipRop.diph))))
	{
		MessageBox(NULL,L"���ü��������豸������ʧ��",L"����",MB_OK|MB_ICONINFORMATION);
		InputRelease();
		return false;
	}
	//��ü��������豸�ķ���Ȩ
	if(FAILED(KeyboardDevice->Acquire()))
	{
		MessageBox(NULL,L"��ü��������豸�ķ���Ȩʧ��.",L"����",MB_OK|MB_ICONINFORMATION);
		InputRelease();
		return false;
	}

	//���̻���������
	ZeroMemory(KeyList,sizeof(char)*256);
	return true;
}
bool Input::CreateMouse(HWND hWnd)
{
	//������������豸
	if(FAILED(DirectInput->CreateDevice(GUID_SysMouse,&MouseDevice,NULL)))
	{
		MessageBox(NULL,L"������������豸����ʧ��.",L"����",MB_OK|MB_ICONINFORMATION);
		InputRelease();
		return false;
	}
	//��������豸�����ݸ�ʽ
	if(FAILED(MouseDevice->SetDataFormat(&c_dfDIMouse)))
	{
		MessageBox(NULL,L"�����������ݶ�ȡ��ʽʧ��.",L"����",MB_OK|MB_ICONINFORMATION);
		InputRelease();
		return false;
	}

	//��������豸��Э������
	if(FAILED(MouseDevice->SetCooperativeLevel(hWnd,DISCL_NONEXCLUSIVE|DISCL_FOREGROUND)))
	{
		MessageBox(NULL,L"��������豸��Э������ʧ��.",L"����",MB_OK|MB_ICONINFORMATION);
		InputRelease();
		return false;
	}

	//��������豸�����ԣ�ʹ�û����������ݣ�
	DIPROPDWORD dipROPWORD;
	dipROPWORD.diph.dwSize=sizeof(DIPROPDWORD);
	dipROPWORD.diph.dwHeaderSize=sizeof(DIPROPHEADER);
	dipROPWORD.diph.dwObj=0;
	dipROPWORD.diph.dwHow=DIPH_DEVICE;
	dipROPWORD.dwData=MouseBufferSize;		//������껺������С
	if(FAILED(MouseDevice->SetProperty(DIPROP_BUFFERSIZE,&dipROPWORD.diph)))
	{
		MessageBox(NULL,L"��������豸����ʧ��.",L"����",MB_OK|MB_ICONINFORMATION);
		InputRelease();
		return false;
	}

	//�������豸�ķ���Ȩ
	if(FAILED(MouseDevice->Acquire()))
	{
		MessageBox(NULL,L"ȡ�����ķ���Ȩʧ��.",L"����",MB_OK|MB_ICONINFORMATION);
		InputRelease();
		return false;
	}

	return true;
}
bool Input::InitInput(HINSTANCE hInst,HWND hWnd)
{
	bool temp=true;
	temp=CreateDirectInput(hInst);
	temp=CreateKeyboard(hWnd);
	temp=CreateMouse(hWnd);
	return temp;
}


bool Input::UpdateKeyboard()
{
	DWORD dwReadNum=KeyBufferSize;
	ZeroMemory(KeyBuffer,sizeof(DIDEVICEOBJECTDATA)*KeyBufferSize);


	if(FAILED(KeyboardDevice->GetDeviceState(sizeof(KeyList),(LPVOID)KeyList)))
	{
		KeyboardDevice->Acquire();
		if(FAILED(KeyboardDevice->GetDeviceState(sizeof(KeyList),(LPVOID)KeyList)))
			return false;
	}
	if(DIERR_INPUTLOST==KeyboardDevice->GetDeviceData(sizeof(DIDEVICEOBJECTDATA),KeyBuffer,&dwReadNum,0))
	{
		KeyboardDevice->Acquire();
		if(DIERR_INPUTLOST==KeyboardDevice->GetDeviceData(sizeof(DIDEVICEOBJECTDATA),KeyBuffer,&dwReadNum,0))
			return false;
	}
	return true;
}
bool Input::UpdateMouse()
{
	DWORD dwReadNum=MouseBufferSize;
	// ÿ�ζ�ȡ����ǰ��һ��Ҫʹ��껺��������
	ZeroMemory(MouseBuffer,sizeof(DIDEVICEOBJECTDATA)*MouseBufferSize);
	this->lastMouseMoveX=lastMouseMoveY=lastMouseMoveZ=0;
	if(FAILED(MouseDevice->Poll()))
			if(FAILED(MouseDevice->Acquire()))
				return false;
	if(DIERR_INPUTLOST==MouseDevice->GetDeviceData(
		sizeof(DIDEVICEOBJECTDATA),MouseBuffer,&dwReadNum,0))
		return false;
	//��ʱ��ȡ�����Ϣ
	if(DIERR_INPUTLOST==MouseDevice->GetDeviceState(sizeof(DIMOUSESTATE),&this->MouseState))
		return false;
	//ѭ����ȡ�������
	for(unsigned int i=0;i<dwReadNum;i++)
	{
		if(MouseBuffer[i].dwOfs==DIMOFS_X)
		{
			totalMouseMoveX+=MouseBuffer[i].dwData;
			this->lastMouseMoveX+=MouseBuffer[i].dwData;
		}
		if(MouseBuffer[i].dwOfs==DIMOFS_Y)
		{
			totalMouseMoveY+=MouseBuffer[i].dwData;
			this->lastMouseMoveY+=MouseBuffer[i].dwData;
		}
		if(MouseBuffer[i].dwOfs==DIMOFS_Z)
		{
			totalMouseMoveZ+=MouseBuffer[i].dwData;
			this->lastMouseMoveZ+=MouseBuffer[i].dwData;
		}
	}
	
	return true;
}
bool Input::ButtonPressed(int button)
{
	if(button>=3||button<0)return false;
	for(int i=0;i<MouseBufferSize;i++)
	{
		if((MouseBuffer[i].dwOfs==DIMOFS_BUTTON0+button)&&(MouseBuffer[i].dwData&0x80))
			return true;
	}
	return false;
}
bool Input::ButtonDown(int button)
{
	if(button>2||button<0)return false;
	return (bool)(this->MouseState.rgbButtons[button]&0x80);
}
bool Input::ButtonUp(int button)
{
	if(button>2||button<0)return false;
	return !(this->MouseState.rgbButtons[button]&0x80);
}

bool Input::KeyPressed(int key)
{
	for(int i=0;i<KeyBufferSize;i++)
	{
		if((KeyBuffer[i].dwOfs==key)&&(KeyBuffer[i].dwData&0x80))
			return true;
	}
	return false;
}
