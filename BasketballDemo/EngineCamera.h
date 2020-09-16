#include"main.h"
#include"EngineInput.h"
class Camera
	//该摄像机的坐标系为左手坐标系 与openGL 中默认的摄像机坐标系相同
{
protected:
	Input *pInput;
	float delta_x,delta_y,delta_z;//绕x y z 轴右手螺旋方向旋转的角度	即欧拉角的a b r  航空器中的 pitch yaw roll 假设飞机的头朝向z
	Vector3 cameraPos;			//摄像机在原默认摄像机坐标系中的位置
	Vector3 lastLocalMove;	//上次更新摄像机时摄像机的在随体坐标系中的位移向量
	Matrix3 rotationMatrix;	//摄像机随体坐标系到 默认坐标系的变换矩阵
	Matrix4 viewMatrix;			//摄像机的视角矩阵
	Quaternion orientation;	//摄像机随体坐标系经过一系列旋转之后在开始默认的摄像机坐标系中的方位
	float GLmatrix[16];
public:
	Camera(Input *pInput):pInput(pInput)
	{
		delta_x=0.0;
		delta_y=0.0;
		delta_z=0.0;
		cameraPos.SetData(0,28,0);
	};
	void UpdateViewMatrix();
	void UpdateKey();
	void UpdateMouse();
	void EularAngularToQuaternion();
	void EularAngularToMatrix();
	void UpdateCamera();
};