#include"main.h"
#include"EngineApplication.h"
#include"EngineInput.h"
float lastFrame;
float thisFrame;
LARGE_INTEGER clockBegin,clockEnd,clockFreq;
float GetFrameInterval()
{
	QueryPerformanceCounter(&clockEnd);
	float frameInterval=((float)(clockEnd.QuadPart-clockBegin.QuadPart))/(float)clockFreq.QuadPart;
	clockBegin=clockEnd;
	return frameInterval;
}
int APIENTRY WinMain(HINSTANCE hInstance,HINSTANCE hPrevInstance,
	LPSTR lpCmdLine,int nCmdShow)
{
	HWND hWnd;
	MSG msg;
	LPCWSTR WinTitle=L"Game";
	Application GameEngine_App(hInstance,WinTitle,800,600,false);
	GameEngine_App.InitApplication();
	hWnd=GameEngine_App.hWnd;
	GetMessage(&msg,NULL,NULL,NULL);   // initialize the struct msg

	//do other initializing code
	QueryPerformanceFrequency(&clockFreq);
	QueryPerformanceCounter(&clockEnd);

	//显式地使用消息循环体，进行输入处理
	while(msg.message!=WM_QUIT)
	{
		if(PeekMessage(&msg,NULL,0,0,PM_REMOVE))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		else
		{
			GameEngine_App.Update(GetFrameInterval());
		}
	}

	// unregister Window Class
	UnregisterClass(WinTitle,hInstance);
	return true;
}