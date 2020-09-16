#include"main.h"
#include"EngineInput.h"
class Camera
	//�������������ϵΪ��������ϵ ��openGL ��Ĭ�ϵ����������ϵ��ͬ
{
protected:
	Input *pInput;
	float delta_x,delta_y,delta_z;//��x y z ����������������ת�ĽǶ�	��ŷ���ǵ�a b r  �������е� pitch yaw roll ����ɻ���ͷ����z
	Vector3 cameraPos;			//�������ԭĬ�����������ϵ�е�λ��
	Vector3 lastLocalMove;	//�ϴθ��������ʱ�����������������ϵ�е�λ������
	Matrix3 rotationMatrix;	//�������������ϵ�� Ĭ������ϵ�ı任����
	Matrix4 viewMatrix;			//��������ӽǾ���
	Quaternion orientation;	//�������������ϵ����һϵ����ת֮���ڿ�ʼĬ�ϵ����������ϵ�еķ�λ
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