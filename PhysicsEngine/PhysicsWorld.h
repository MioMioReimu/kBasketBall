#pragma once
//#include"PhysicsCommon.h"
//#include"PhysicsCollisionCoarse.h"
//#include"PhysicsCollisionFine.h"
//#include"PhysicsForceGenerate.h"
namespace Physics
{
	
	template class DLL_PHYSICS allocator<RigidBody*>;
	template class DLL_PHYSICS allocator<CollisionPlane*>;
	template class DLL_PHYSICS vector<RigidBody*,allocator<RigidBody*>>;
	template class DLL_PHYSICS vector<CollisionPlane*,allocator<CollisionPlane*>>;
	typedef vector<RigidBody*> BodyListType;//物体数组
	//typedef vector<PotentialContact> PotentialContactListType;//已经在别的地方定义了
	//typedef vector<Contact> ContactListType;//已经在别的地方定义了
	typedef vector<CollisionPlane*> PlaneListType;//碰撞平面数组
	class DLL_PHYSICS Simulation
	{
	public:
		BodyListType BodyList;	//这个数据需要用户使用的时候传进来
		PlaneListType PlaneList;//这个数据需要用户指定 场景中的平面
	protected:
		ContactResolver contactSolution;//碰撞解决器
		PotentialContactListType potentialContactList;//可能碰撞对数组
		ContactListType contactList;//碰撞列表
		
	public:
		~Simulation();
		void PushBackBody(RigidBody*body);//
		bool PopBackBody();//把body链表中的末尾body删除 
		void ClearBodyList();//清除所有的物体 
		void PushBackPlane(CollisionPlane*plane);
		bool PopBackPlane();
		void ClearPlaneList();
		
		void RunSimulation(real duration);//运行模拟
		
	protected:
		void GetPotentialContactList();//更新物体可能碰撞链表
		void GetContactList();//更新物体碰撞链表和物体与平面碰撞链表 两个链表在一起的
		void IntegrateContact(real duration);//处理物体
	};
}