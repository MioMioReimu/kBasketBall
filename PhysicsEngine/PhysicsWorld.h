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
	typedef vector<RigidBody*> BodyListType;//��������
	//typedef vector<PotentialContact> PotentialContactListType;//�Ѿ��ڱ�ĵط�������
	//typedef vector<Contact> ContactListType;//�Ѿ��ڱ�ĵط�������
	typedef vector<CollisionPlane*> PlaneListType;//��ײƽ������
	class DLL_PHYSICS Simulation
	{
	public:
		BodyListType BodyList;	//���������Ҫ�û�ʹ�õ�ʱ�򴫽���
		PlaneListType PlaneList;//���������Ҫ�û�ָ�� �����е�ƽ��
	protected:
		ContactResolver contactSolution;//��ײ�����
		PotentialContactListType potentialContactList;//������ײ������
		ContactListType contactList;//��ײ�б�
		
	public:
		~Simulation();
		void PushBackBody(RigidBody*body);//
		bool PopBackBody();//��body�����е�ĩβbodyɾ�� 
		void ClearBodyList();//������е����� 
		void PushBackPlane(CollisionPlane*plane);
		bool PopBackPlane();
		void ClearPlaneList();
		
		void RunSimulation(real duration);//����ģ��
		
	protected:
		void GetPotentialContactList();//�������������ײ����
		void GetContactList();//����������ײ�����������ƽ����ײ���� ����������һ���
		void IntegrateContact(real duration);//��������
	};
}