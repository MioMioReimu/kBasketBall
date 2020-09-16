#pragma once
#include"main.h"
using namespace Physics;
using namespace std;
#include"3DS.h"
#include"EngineInput.h"
//�ýṹ����Դ�������ģ�ͺ���������ļ���ģ�͹�������
struct Actor
{
	//char objectName[255];

	//��Դ�������������
	unsigned objectIndex;
	//��object��������ϵ�����������е�body����������ϵ�ı任����
	Matrix4 objectTobody;
	//���������м���ģ��ָ��
	RigidBody *body;
};
//�����б�����
typedef vector<Actor> ActorListType;

//��Դ�� 3DS�ļ�������ģ�����ݱ����ڸ�����
//�������һ����������ɲ���֮һ
class Resource
{
public:
	Model resourceModel;
	int *drawList;

public:
	//����3DS�ļ���ʼ��һ����Դ
	bool InitResource(char *FileName);
	//��������
	void LoadTexture();
	//
	void LoadMesh();
	//������������������
	void DrawObject(unsigned objectIndex);
};

//������һ����������Ⱦ 
//һ��������һ����Դһ�������б��һ������������
class Scene
{
public:
	//��Դ
	Resource *pResource;
	//�����б�
	ActorListType ActorList;
	//������������
	Simulation simulate;
public:
	//����3DS�ļ���ʼ��һ������
	void InitScene(char *FileName);
	~Scene();
	//�����body��pPrimitive��Ӧnew����
	void AddActor(RigidBody *body,unsigned int objectIndex,CollisionPrimitive *pPrimitive);
	//�����actor�����ָ��bodyҲ��ǳ���� �����Ҫnew����
	void AddActor(Actor& actor);
	//�ڳ���������һ����
	void AddBall();
	//����һ����
	void AddBall1();
	//�Գ���������Ⱦ
	void Render(float duration=0.016667);
};