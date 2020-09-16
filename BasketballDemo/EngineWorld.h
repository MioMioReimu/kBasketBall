#pragma once
#include"main.h"
using namespace Physics;
using namespace std;
#include"3DS.h"
#include"EngineInput.h"
//该结构将资源类里面的模型和物理引擎的计算模型关联起来
struct Actor
{
	//char objectName[255];

	//资源类中物体的索引
	unsigned objectIndex;
	//从object自身坐标系到物理引擎中的body的随体坐标系的变换矩阵
	Matrix4 objectTobody;
	//物理引擎中计算模型指针
	RigidBody *body;
};
//物体列表类型
typedef vector<Actor> ActorListType;

//资源类 3DS文件读入后的模型数据保存在该类中
//这个类是一个场景的组成部分之一
class Resource
{
public:
	Model resourceModel;
	int *drawList;

public:
	//根据3DS文件初始化一个资源
	bool InitResource(char *FileName);
	//加载纹理
	void LoadTexture();
	//
	void LoadMesh();
	//根据索引画出该物体
	void DrawObject(unsigned objectIndex);
};

//场景类一个场景的渲染 
//一个场景有一个资源一个物体列表和一个物理引擎类
class Scene
{
public:
	//资源
	Resource *pResource;
	//物体列表
	ActorListType ActorList;
	//物理引擎主类
	Simulation simulate;
public:
	//根据3DS文件初始化一个场景
	void InitScene(char *FileName);
	~Scene();
	//这里的body和pPrimitive均应new出来
	void AddActor(RigidBody *body,unsigned int objectIndex,CollisionPrimitive *pPrimitive);
	//这里的actor里面的指针body也是浅复制 因此需要new出来
	void AddActor(Actor& actor);
	//在场景中增加一个球
	void AddBall();
	//增加一个球
	void AddBall1();
	//对场景进行渲染
	void Render(float duration=0.016667);
};