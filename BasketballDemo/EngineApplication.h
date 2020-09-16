#pragma once
#include"main.h"
#include"EngineInput.h"
#include"EngineWorld.h"
#include"EngineCamera.h"
//������װ�˺ܶ�Win32�����ϸ��
//������Ϣ�����������ڵ�ע��ʹ�����openGL�ĳ�ʼ��
class Application
{
public:
	///<Win32������Ϣ
	//���ھ�� �����˴���֮����Ҫ�Ѵ��ھ����������DXInput��ʼ������������ʹ��
	HWND hWnd;		
	//ʵ�����
	HINSTANCE hInst;
	//��������
	LPCWSTR WndName;
	//�Ƿ�ȫ��
	bool bFullScreen;
	//���ڴ�С
	int width;	
	int height;
	///Win32������Ϣ>

	///<��������
	//DX����ϵͳ
	Input InputSystem;
	//������
	Scene* pScene;
	//�������
	Camera* pCamera;
	///��������>
public:
	Application(){};
	Application(HINSTANCE hInst,LPCWSTR WndName,int width,int height,bool bFullScreen=false)
		:WndName(WndName),hInst(hInst),width(width),height(height),bFullScreen(bFullScreen){};//�趨Win32һЩ��Ϣ
	~Application(){};
	//��ʼ������ OpenGL�豸���� �������� ������ϵͳ �����л����CreateWnd InitOpenGL���� SetLight����
	void InitApplication();
	//��Ⱦ���� �ò��������ϴε��øø��º�����ʱ��� 
	void Render(float duration=0.016667);
	//����������Ϣ �����л����Render KeyProc MouseProc
	void Update(float duration=0.016667);
	//����״̬���ݸ��ºʹ�����
	void KeyProc();		
	//���ù�Դ
	void SetLight();		
	//windows��Ϣ������ ���һ�㲻��Ҫ�޸�
	static LRESULT CALLBACK WndProc(HWND hWnd,UINT message,WPARAM wParam,LPARAM lParam);
private:
	//�������� ��InitApplication�б�����
	bool CreateWnd(DWORD dwStyle);
	//��ʼ��OpenGL�豸����
	void InitOpenGL();	
	//���������豸��ʽ ��InitOpenGL�б�����
	bool SetupPixelFormat();
	//ע�ᴰ���� ��CreateWnd�б�����
	ATOM RegWndClass();
};

