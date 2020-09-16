#pragma once
//#include"PhysicsRigidBody.h"
namespace Physics
{
	class DLL_PHYSICS ForceRegistry;
	class DLL_PHYSICS ForceGenerator;
		//��������������
	struct DLL_PHYSICS ForceRegistration		
		{
			RigidBody *body;
			ForceGenerator *forceGen;
		};
		//������������ ����
	template class DLL_PHYSICS allocator<ForceRegistration>;
	template class DLL_PHYSICS vector<ForceRegistration,allocator<ForceRegistration>>;
	typedef std::vector<ForceRegistration> Registry;

		//�������� �κ����͵��� ���̳��Ը��� ����Ϊһ������
		//��Ҫ�̳в�ʵ��updateForce����
	class DLL_PHYSICS ForceGenerator
	{
	public:
		virtual void updateForce(RigidBody* body,real duration)=0;
	};

		//��ע����
		//���������ܵ������Ĺ�ϵ����ڸ��࣬���ɸ�����������ܵ�����
	class DLL_PHYSICS ForceRegistry
	{
	public:
		Registry registrations;
	public:
		void add(RigidBody *body,ForceGenerator *forceGen);//����м���һ����ϵ
		void remove(RigidBody *body,ForceGenerator *forceGen);//�ӱ���ɾ��һ����ϵ
		void clear();//����������й�ϵ
		void updateForces(real duration);//���±������������
	};
		//������
	class DLL_PHYSICS Gravity:public ForceGenerator
	{
		Vector3 gravity;
	public:
		Gravity(const Vector3 &gravity);
		void updateForce(RigidBody *body,real duration);
	};
}