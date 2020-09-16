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
	// 创建IDirectInput 接口对象
	if(FAILED(DirectInput8Create(hInst,DIRECTINPUT_VERSION,IID_IDirectInput8,(void**)&DirectInput,NULL)))
	{
		MessageBox(NULL,L"建立IDirectInput8 接口对象失败。",L"警告",
			MB_OK|MB_ICONINFORMATION);
		return false;
	}
	return true;
}
bool Input::CreateKeyboard(HWND hWnd)
{
	//创建键盘输入设备
	if(FAILED(DirectInput->CreateDevice(GUID_SysKeyboard,
		&KeyboardDevice,NULL)))
	{
		MessageBox(NULL,L"建立键盘输入对象失败。",L"警告",
			MB_OK|MB_ICONINFORMATION);
		return false;
	}

	//设置键盘输入设备的数据格式
	if(FAILED(KeyboardDevice->SetDataFormat(&c_dfDIKeyboard)))
	{
		MessageBox(NULL,L"设置键盘输入设备的数据读取数据格式失败.",L"警告",MB_OK|MB_ICONINFORMATION);
		InputRelease();
		return false;
	}

	//设置键盘输入设备的协调级别
	if(FAILED(KeyboardDevice->SetCooperativeLevel(hWnd,DISCL_FOREGROUND|DISCL_NONEXCLUSIVE)))
	{
		MessageBox(NULL,L"设置协调级别失败.",L"警告",MB_OK|MB_ICONINFORMATION);
		InputRelease();
		return false;
	}
	//设置键盘缓冲区
	::DIPROPDWORD dipRop;
	dipRop.diph.dwHeaderSize=sizeof(::DIPROPHEADER);
	dipRop.diph.dwSize=sizeof(DIPROPDWORD);
	dipRop.diph.dwObj=0;
	dipRop.diph.dwHow=DIPH_DEVICE;
	dipRop.dwData=KeyBufferSize;
	if(FAILED(KeyboardDevice->SetProperty(DIPROP_BUFFERSIZE,&(dipRop.diph))))
	{
		MessageBox(NULL,L"设置键盘输入设备缓冲区失败",L"错误",MB_OK|MB_ICONINFORMATION);
		InputRelease();
		return false;
	}
	//获得键盘输入设备的访问权
	if(FAILED(KeyboardDevice->Acquire()))
	{
		MessageBox(NULL,L"获得键盘输入设备的访问权失败.",L"警告",MB_OK|MB_ICONINFORMATION);
		InputRelease();
		return false;
	}

	//键盘缓冲区清零
	ZeroMemory(KeyList,sizeof(char)*256);
	return true;
}
bool Input::CreateMouse(HWND hWnd)
{
	//创建鼠标输入设备
	if(FAILED(DirectInput->CreateDevice(GUID_SysMouse,&MouseDevice,NULL)))
	{
		MessageBox(NULL,L"建立鼠标输入设备对象失败.",L"警告",MB_OK|MB_ICONINFORMATION);
		InputRelease();
		return false;
	}
	//设置鼠标设备的数据格式
	if(FAILED(MouseDevice->SetDataFormat(&c_dfDIMouse)))
	{
		MessageBox(NULL,L"设置鼠标的数据读取格式失败.",L"警告",MB_OK|MB_ICONINFORMATION);
		InputRelease();
		return false;
	}

	//设置鼠标设备的协调级别
	if(FAILED(MouseDevice->SetCooperativeLevel(hWnd,DISCL_NONEXCLUSIVE|DISCL_FOREGROUND)))
	{
		MessageBox(NULL,L"设置鼠标设备的协调级别失败.",L"警告",MB_OK|MB_ICONINFORMATION);
		InputRelease();
		return false;
	}

	//设置鼠标设备的属性（使用缓冲区读数据）
	DIPROPDWORD dipROPWORD;
	dipROPWORD.diph.dwSize=sizeof(DIPROPDWORD);
	dipROPWORD.diph.dwHeaderSize=sizeof(DIPROPHEADER);
	dipROPWORD.diph.dwObj=0;
	dipROPWORD.diph.dwHow=DIPH_DEVICE;
	dipROPWORD.dwData=MouseBufferSize;		//设置鼠标缓冲区大小
	if(FAILED(MouseDevice->SetProperty(DIPROP_BUFFERSIZE,&dipROPWORD.diph)))
	{
		MessageBox(NULL,L"设置鼠标设备属性失败.",L"警告",MB_OK|MB_ICONINFORMATION);
		InputRelease();
		return false;
	}

	//获得鼠标设备的访问权
	if(FAILED(MouseDevice->Acquire()))
	{
		MessageBox(NULL,L"取得鼠标的访问权失败.",L"警告",MB_OK|MB_ICONINFORMATION);
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
	// 每次读取数据前，一定要使鼠标缓冲区清零
	ZeroMemory(MouseBuffer,sizeof(DIDEVICEOBJECTDATA)*MouseBufferSize);
	this->lastMouseMoveX=lastMouseMoveY=lastMouseMoveZ=0;
	if(FAILED(MouseDevice->Poll()))
			if(FAILED(MouseDevice->Acquire()))
				return false;
	if(DIERR_INPUTLOST==MouseDevice->GetDeviceData(
		sizeof(DIDEVICEOBJECTDATA),MouseBuffer,&dwReadNum,0))
		return false;
	//即时读取鼠标信息
	if(DIERR_INPUTLOST==MouseDevice->GetDeviceState(sizeof(DIMOUSESTATE),&this->MouseState))
		return false;
	//循环读取鼠标数据
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
