#include "EngineApplication.h"

//创建窗口函数
bool Application::CreateWnd(DWORD dwStyle)
{
	if(!dwStyle)
		dwStyle=WS_OVERLAPPEDWINDOW|WS_CLIPSIBLINGS|WS_CLIPCHILDREN;
	this->WndName=WndName;
		RegWndClass();

	//创建窗口
	hWnd=CreateWindow(WndName,WndName,/*WS_SYSMENU|WS_CAPTION|WS_VISIBLE*/WS_OVERLAPPEDWINDOW,
		0,0,width,height,NULL,NULL,hInst,NULL);
	if(!hWnd)
		return false;

	ShowWindow(hWnd,SW_SHOWNORMAL); //显示窗口
	UpdateWindow(hWnd);
	SetFocus(hWnd);
	return true;
}

//注册窗口类
ATOM Application::RegWndClass()
{
	WNDCLASSEX WndClassEx;//窗口类型
	WndClassEx.cbSize=sizeof(WNDCLASSEX);
	WndClassEx.style=CS_HREDRAW|CS_VREDRAW;
	WndClassEx.lpfnWndProc=(WNDPROC)WndProc;
	WndClassEx.cbClsExtra=0;
	WndClassEx.cbWndExtra=0;
	WndClassEx.hInstance=hInst;
	WndClassEx.hIcon=0;
	WndClassEx.hCursor=LoadCursor(NULL,IDC_ARROW);
	WndClassEx.hbrBackground=(HBRUSH)GetStockObject(BLACK_BRUSH);
	WndClassEx.lpszClassName=WndName;		// define WNDCLASSNAME "MyGame"
	WndClassEx.lpszMenuName=0;
	WndClassEx.hIconSm=NULL;
	return RegisterClassEx(&WndClassEx);
}

// 消息回调函数
LRESULT CALLBACK Application::WndProc(HWND hWnd,UINT message,WPARAM wParam,LPARAM lParam)
{
	switch(message)
	{
	case WM_DESTROY:
		PostQuitMessage(0);
		break;
	case WM_SIZE:
		//窗口大小发生变化时更新视口和透视矩阵
		{
		int height=HIWORD(lParam);
		int width=LOWORD(lParam);
		if(height==0)
			height=1;
		glViewport(0,0,width,height);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();

		gluPerspective(45,(float)width/(float)height,0.1,1000.0f);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		break;
		}
	default:
		return DefWindowProc(hWnd,message,wParam,lParam);
	};
	return 0;
}

void Application::InitOpenGL()
{
	HDC hDC=::GetDC(hWnd);
	if (!SetupPixelFormat())
		PostQuitMessage (0);

	HGLRC hRC = wglCreateContext(hDC);
	wglMakeCurrent(hDC, hRC);

	if (height==0)										
		height=1;										

	glViewport(0,0,width,height);						

	glMatrixMode(GL_PROJECTION);		
	glLoadIdentity();						
	gluPerspective(45.0f,(GLfloat)width/(GLfloat)height, .1f ,1000.0f);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT,GL_NICEST);
	glHint(GL_POINT_SMOOTH_HINT,GL_NICEST);
	glHint(GL_GENERATE_MIPMAP_HINT,GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT,GL_NICEST);
	glMatrixMode(GL_MODELVIEW);						
	glLoadIdentity();
}
bool Application::SetupPixelFormat()
{
	PIXELFORMATDESCRIPTOR pfd;
	int pixelformat;
	HDC hDC=GetDC(hWnd);
	pfd.nSize = sizeof(PIXELFORMATDESCRIPTOR);
	pfd.nVersion = 1;

	pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
	pfd.dwLayerMask = PFD_MAIN_PLANE;
	pfd.iPixelType = PFD_TYPE_RGBA;
	pfd.cColorBits = SCREEN_DEPTH;
	pfd.cDepthBits = SCREEN_DEPTH;
	pfd.cAccumBits = 0;
	pfd.cStencilBits = 0;

	if ( (pixelformat = ChoosePixelFormat(hDC, &pfd)) == FALSE )
	{
		MessageBox(NULL, L"ChoosePixelFormat failed", L"Error", MB_OK);
		return FALSE;
	}

	if (SetPixelFormat(hDC, pixelformat, &pfd) == FALSE)
	{
		MessageBox(NULL, L"SetPixelFormat failed", L"Error", MB_OK);
		return FALSE;
	}
	return TRUE;			
}

void Application::InitApplication()
{
	//创建窗口
	this->CreateWnd(0);
	//初始化openGL
	this->InitOpenGL();
	//初始化DXInput
	this->InputSystem.InitInput(hInst,hWnd);
	//新建一个摄像机
	this->pCamera=new Camera(&InputSystem);
	//新建一个场景
	this->pScene=new Scene;
	//初始化场景
	pScene->InitScene("lanqiuchang.3ds");
	//初始化灯光
	this->SetLight();
}

void Application::Render(float duration)//渲染场景 该参数是离上次调用该更新函数的时间差
{
	glEnable(GL_DEPTH_TEST);
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	//设置视角和更新视角
	
	this->pCamera->UpdateCamera();
	//gluLookAt(100,200,100,0,0,0,0,1,0);
	this->SetLight();
	this->pScene->Render(duration);
	SwapBuffers(GetDC(hWnd));
}
void Application::SetLight()		//set Light
{
	glMatrixMode(GL_MODELVIEW);
	//enable the lights and lighting
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glEnable(GL_LIGHT2);
	glEnable(GL_LIGHT3);
	glEnable(GL_LIGHT4);
	glEnable(GL_LIGHT5);

	float pointlight_ambient[4]={0.784,0.784,0.784,1.0};		//point light ambient  between brown and white
	float pointlight_diffuse[4]={0.784,0.784,0.784,1.0};		//point light diffuse	 between brown and white
	float pointlight_specular[4]={0.902,0.902,0.902,1.0};		//point light specular  nearly white
	float pointlight_constAttenuation=1;		//point light const attenuation
	float pointlight_linearAttenuation=0.03;		//linear
	float pointlight_quadAttenuation=0.0;		//quadratic attenuation

	float spotlight_ambient[4]={0.9255,0.9059,0.6706,1.0};
	float spotlight_diffuse[4]={0.9255,0.9059,0.6706,1.0};
	float spotlight_specular[4]={0.9098,0.9059,0.8118,1.0};
	float spotlight_constAttenuation=1;
	float spotlight_linearAttenuation=0.03;
	//float spotlight_quadAttenuation=0.0;
	float spotlight_exponent=0.5;	//exponent of spotlight ; used to cotrol the concentration ratio of the spot light
	float spotlight_cutoff=60;		//half of the taper angle of the spotlight

	float globalAmbient[4]={0.1,0.1,0.1,1.0};//global ambient

	//position of the lights in the world coordinate
	float pointlight0_pos[4]={0,40,70,1.0};
	float pointlight1_pos[4]={0,40,-70,1.0};
	float spotlight0_pos[4]={85,40,145,1.0};
	float spotlight1_pos[4]={85,40,-145,1.0};
	float spotlight2_pos[4]={-85,40,145,1.0};
	float spotlight3_pos[4]={-85,40,-145,1.0};

	//direction of the spot lights in the world coordinate 
	float spotlight0_direc[3]={-45,-40,-70};
	float spotlight1_direc[3]={-45,-40,70};
	float spotlight2_direc[3]={45,-40,-70};
	float spotlight3_direc[3]={45,-40,70};


	//set position
	glLightfv(GL_LIGHT0,GL_POSITION,pointlight0_pos);
	glLightfv(GL_LIGHT1,GL_POSITION,pointlight1_pos);
	glLightfv(GL_LIGHT2,GL_POSITION,spotlight0_pos);
	glLightfv(GL_LIGHT3,GL_POSITION,spotlight1_pos);
	glLightfv(GL_LIGHT4,GL_POSITION,spotlight2_pos);
	glLightfv(GL_LIGHT5,GL_POSITION,spotlight3_pos);

	//set ambient
	glLightfv(GL_LIGHT0,GL_AMBIENT,pointlight_ambient);
	glLightfv(GL_LIGHT1,GL_AMBIENT,pointlight_ambient);
	glLightfv(GL_LIGHT2,GL_AMBIENT,spotlight_ambient);
	glLightfv(GL_LIGHT3,GL_AMBIENT,spotlight_ambient);
	glLightfv(GL_LIGHT4,GL_AMBIENT,spotlight_ambient);
	glLightfv(GL_LIGHT5,GL_AMBIENT,spotlight_ambient);
	
	//set diffuse
	glLightfv(GL_LIGHT0,GL_DIFFUSE,pointlight_diffuse);
	glLightfv(GL_LIGHT1,GL_DIFFUSE,pointlight_diffuse);
	glLightfv(GL_LIGHT2,GL_DIFFUSE,spotlight_diffuse);
	glLightfv(GL_LIGHT3,GL_DIFFUSE,spotlight_diffuse);
	glLightfv(GL_LIGHT4,GL_DIFFUSE,spotlight_diffuse);
	glLightfv(GL_LIGHT5,GL_DIFFUSE,spotlight_diffuse);

	//set specular
	glLightfv(GL_LIGHT0,GL_SPECULAR,pointlight_specular);
	glLightfv(GL_LIGHT1,GL_SPECULAR,pointlight_specular);
	glLightfv(GL_LIGHT2,GL_SPECULAR,spotlight_specular);
	glLightfv(GL_LIGHT3,GL_SPECULAR,spotlight_specular);
	glLightfv(GL_LIGHT4,GL_SPECULAR,spotlight_specular);
	glLightfv(GL_LIGHT5,GL_SPECULAR,spotlight_specular);

	//set spot light direction
	glLightfv(GL_LIGHT2,GL_SPOT_DIRECTION,spotlight0_direc);
	glLightfv(GL_LIGHT3,GL_SPOT_DIRECTION,spotlight1_direc);
	glLightfv(GL_LIGHT4,GL_SPOT_DIRECTION,spotlight2_direc);
	glLightfv(GL_LIGHT5,GL_SPOT_DIRECTION,spotlight3_direc);

	//set spot light exponent
	glLightf(GL_LIGHT2,GL_SPOT_EXPONENT,spotlight_exponent);
	glLightf(GL_LIGHT3,GL_SPOT_EXPONENT,spotlight_exponent);
	glLightf(GL_LIGHT4,GL_SPOT_EXPONENT,spotlight_exponent);
	glLightf(GL_LIGHT5,GL_SPOT_EXPONENT,spotlight_exponent);

	//set spot light cutoff angle
	glLightf(GL_LIGHT2,GL_SPOT_EXPONENT,spotlight_cutoff);
	glLightf(GL_LIGHT3,GL_SPOT_EXPONENT,spotlight_cutoff);
	glLightf(GL_LIGHT4,GL_SPOT_EXPONENT,spotlight_cutoff);
	glLightf(GL_LIGHT5,GL_SPOT_EXPONENT,spotlight_cutoff);

	//set light constant attenuation
	glLightf(GL_LIGHT0,GL_CONSTANT_ATTENUATION,pointlight_constAttenuation);
	glLightf(GL_LIGHT1,GL_CONSTANT_ATTENUATION,pointlight_constAttenuation);
	glLightf(GL_LIGHT2,GL_CONSTANT_ATTENUATION,spotlight_constAttenuation);
	glLightf(GL_LIGHT3,GL_CONSTANT_ATTENUATION,spotlight_constAttenuation);
	glLightf(GL_LIGHT4,GL_CONSTANT_ATTENUATION,spotlight_constAttenuation);
	glLightf(GL_LIGHT5,GL_CONSTANT_ATTENUATION,spotlight_constAttenuation);

	//set light linear attenuation
	glLightf(GL_LIGHT0,GL_LINEAR_ATTENUATION,pointlight_linearAttenuation);
	glLightf(GL_LIGHT1,GL_LINEAR_ATTENUATION,pointlight_linearAttenuation);
	glLightf(GL_LIGHT2,GL_LINEAR_ATTENUATION,spotlight_linearAttenuation);
	glLightf(GL_LIGHT3,GL_LINEAR_ATTENUATION,spotlight_linearAttenuation);
	glLightf(GL_LIGHT4,GL_LINEAR_ATTENUATION,spotlight_linearAttenuation);
	glLightf(GL_LIGHT5,GL_LINEAR_ATTENUATION,spotlight_linearAttenuation);

	//the light quadratic attenuation is defaulted

	//the global ambient
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT,globalAmbient);

	//shade model
	glClearColor(0,0,0,0);
	glShadeModel(GL_SMOOTH);
	//set the light veiw point (0,0,0) in the view coordinate
	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER,GL_TRUE);
	//if we use two side lighting
	//glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE);
	//delay the specular color until the texture is completed
	glLightModeli(GL_LIGHT_MODEL_COLOR_CONTROL,GL_SEPARATE_SPECULAR_COLOR);
	//glLightModeli(GL_LIGHT_MODEL_COLOR_CONTROL,GL_SEPARATE_SPECULAR_COLOR);

}
void Application::Update(float duration)//更新所有信息 在其中会调用Render KeyProc MouseProc
{
	KeyProc();
	Render(duration);
}
void Application::KeyProc()		//键盘状态数据更新和处理函数
{
	InputSystem.UpdateKeyboard();
	if(InputSystem.KeyPressed(DIK_N))
		this->pScene->AddBall();
	if(InputSystem.KeyPressed(DIK_M))
		this->pScene->AddBall1();
}