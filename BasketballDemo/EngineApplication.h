#pragma once
#include"main.h"
#include"EngineInput.h"
#include"EngineWorld.h"
#include"EngineCamera.h"
//这个类封装了很多Win32程序的细节
//包括消息处理函数，窗口的注册和创建，openGL的初始化
class Application
{
public:
	///<Win32程序信息
	//窗口句柄 创建了窗口之后需要把窗口句柄存下来供DXInput初始化和其他函数使用
	HWND hWnd;		
	//实例句柄
	HINSTANCE hInst;
	//窗口名字
	LPCWSTR WndName;
	//是否全屏
	bool bFullScreen;
	//窗口大小
	int width;	
	int height;
	///Win32程序信息>

	///<程序数据
	//DX输入系统
	Input InputSystem;
	//场景类
	Scene* pScene;
	//摄像机类
	Camera* pCamera;
	///程序数据>
public:
	Application(){};
	Application(HINSTANCE hInst,LPCWSTR WndName,int width,int height,bool bFullScreen=false)
		:WndName(WndName),hInst(hInst),width(width),height(height),bFullScreen(bFullScreen){};//设定Win32一些信息
	~Application(){};
	//初始化窗口 OpenGL设备环境 程序数据 和输入系统 在其中会调用CreateWnd InitOpenGL函数 SetLight函数
	void InitApplication();
	//渲染场景 该参数是离上次调用该更新函数的时间差 
	void Render(float duration=0.016667);
	//更新所有信息 在其中会调用Render KeyProc MouseProc
	void Update(float duration=0.016667);
	//键盘状态数据更新和处理函数
	void KeyProc();		
	//设置光源
	void SetLight();		
	//windows消息处理函数 这个一般不需要修改
	static LRESULT CALLBACK WndProc(HWND hWnd,UINT message,WPARAM wParam,LPARAM lParam);
private:
	//创建窗口 在InitApplication中被调用
	bool CreateWnd(DWORD dwStyle);
	//初始化OpenGL设备环境
	void InitOpenGL();	
	//设置像素设备格式 在InitOpenGL中被调用
	bool SetupPixelFormat();
	//注册窗口类 在CreateWnd中被调用
	ATOM RegWndClass();
};

