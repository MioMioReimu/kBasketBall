#pragma once
//#include"PhysicsRigidBody.h"
namespace Physics
{
	class DLL_PHYSICS ForceRegistry;
	class DLL_PHYSICS ForceGenerator;
		//力与刚体关联类型
	struct DLL_PHYSICS ForceRegistration		
		{
			RigidBody *body;
			ForceGenerator *forceGen;
		};
		//力与刚体关联表 类型
	template class DLL_PHYSICS allocator<ForceRegistration>;
	template class DLL_PHYSICS vector<ForceRegistration,allocator<ForceRegistration>>;
	typedef std::vector<ForceRegistration> Registry;

		//力生成器 任何类型的力 均继承自该类 该类为一个虚类
		//需要继承并实现updateForce函数
	class DLL_PHYSICS ForceGenerator
	{
	public:
		virtual void updateForce(RigidBody* body,real duration)=0;
	};

		//力注册类
		//所有物体受到的力的关系表存在该类，并由该类更新物体受到的力
	class DLL_PHYSICS ForceRegistry
	{
	public:
		Registry registrations;
	public:
		void add(RigidBody *body,ForceGenerator *forceGen);//向表中加入一个关系
		void remove(RigidBody *body,ForceGenerator *forceGen);//从表中删除一个关系
		void clear();//清除表中所有关系
		void updateForces(real duration);//更新表中物体的受力
	};
		//重力类
	class DLL_PHYSICS Gravity:public ForceGenerator
	{
		Vector3 gravity;
	public:
		Gravity(const Vector3 &gravity);
		void updateForce(RigidBody *body,real duration);
	};
}