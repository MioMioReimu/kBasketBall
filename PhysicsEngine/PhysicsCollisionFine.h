#pragma once
//#include"PhysicsContact.h"
//#include"PhysicsCollisionCoarse.h"
namespace Physics
{
	class DLL_PHYSICS IntersectionTests;
	class DLL_PHYSICS CollisionDetector;

	//��ײ������ 
	class DLL_PHYSICS CollisionPrimitive
	{
	public:
		friend IntersectionTests;
		friend CollisionDetector;
		RigidBody *body;	//�û���������body
		Matrix4 offset;		//������������ϵ�����������ϵ�ı任����
		void CalculateInternals()
		{
			transform=body->GetTransform()*offset;
		}
		Vector3 GetAxis(unsigned index)const
		{
			return transform.GetAxisVector(index);//�õ��任�������������һ�����ڵõ�����������,���������ĵ�λ��
		}
		const Matrix4& GetTransform()const
		{
			return transform;
		}
	public:
		CollisionPrimitive(RigidBody*body,const Matrix4& offset):body(body),offset(offset)
		{}
		CollisionPrimitive(RigidBody *body):body(body){};
		CollisionPrimitive(){body=NULL;}
	public:
		Matrix4 transform;//��������ϵ������ı任����
	};

	//����ײ�����ڴ���һ�����Ե�����ĸ���
	class DLL_PHYSICS CollisionSphere: public CollisionPrimitive
	{
	public:
		real radius;
		CollisionSphere(real radius,RigidBody*body):radius(radius),CollisionPrimitive(body)
		{}
		CollisionSphere(real radius,RigidBody *body,const Matrix4 &m):
		radius(radius),CollisionPrimitive(body,m)
		{}
		CollisionSphere(){radius=0;body=NULL;}
	};
	//����ײ�����ڴ���һ�����Ե���Բ���ĸ��� ���������
	class DLL_PHYSICS CollisionCircle:public CollisionPrimitive
	{
	public:
		real radius;
		Vector3 normal;//Բ����������ϵ�еĵ�λ����
		CollisionCircle(real radius,const Vector3& normal,RigidBody*body):
		radius(radius),normal(normal),CollisionPrimitive(body)
		{}
		CollisionCircle(real radius,const Vector3 & normal,RigidBody *body,const Matrix4 &m):
		radius(radius),normal(normal),CollisionPrimitive(body,m)
		{}
		CollisionCircle(){radius=0;body=NULL;}
	};
	//����ײ�����ڴ���һ�����Ե���Բ����ĸ���
	class DLL_PHYSICS CollisionCylinder:public CollisionPrimitive
	{
	public:
		real radius;
		Vector3 normal;//Բ��������������ϵ�еķ����� �䳤�ȱ�ʾԲ����ĳ���
	};

	//ƽ�治��һ��primitive �������ڴ����κθ��壬������������ײ���粻���ļ�����
	//������ڴ���ǽ�� ����Ȳ����ƶ�������
	class DLL_PHYSICS CollisionPlane
	{
	public:

		Vector3 direction;	//ƽ�淨��
		real offset;	//ƽ���λ��
		real friction;
		real restitution;
	public:
		CollisionPlane(){friction=0.0;restitution=1.0;offset=0;direction.SetData(0,1,0);};
		CollisionPlane(real friction,real restitution,real offset,const Vector3 &normal):
		friction(friction),restitution(restitution),offset(offset),direction(normal)
		{}
	};
	//OBB��Χ��
	//����һ�����Ե���������ĸ���
	class DLL_PHYSICS CollisionBox:public CollisionPrimitive
	{
	public:
		Vector3 halfSize;//������ĳ���ߵ�һ��
		CollisionBox(const Vector3& halfSize,RigidBody* body,const Matrix4& offset):halfSize(halfSize),CollisionPrimitive(body,offset)
		{}
		CollisionBox(const Vector3& halfSize,RigidBody* body):halfSize(halfSize),CollisionPrimitive(body)
		{};
		CollisionBox(){this->body=NULL;}
	};
	//���������������ײ�������ײ���� ���Ƿ���ײ�������ײ�������
	//���������ĺ���ȫ�Ǿ�̬����
	class DLL_PHYSICS IntersectionTests
	{
	public:
		static bool SphereAndHalfSpace(const CollisionSphere &sphere,const CollisionPlane &plane);
		static bool SphereAndSphere(const CollisionSphere &one,const CollisionSphere &two);
		static real TransformToAxis(const CollisionBox &box,const Vector3 &axis);
		static bool OverlapOnAxis(const CollisionBox &one,const CollisionBox &two,const Vector3 &axis,const Vector3 &toCentre);
		static bool BoxAndBox(const CollisionBox &one,const CollisionBox &two);
		static bool BoxAndHalfSpace(const CollisionBox& box,const CollisionPlane &plane);
	};
	//typedef vector<Contact> ContactListType;
	//���ڴ洢��ײ����Ϣ
	/*struct CollisionData
	{
		Contact *contactArray;//������ײ����
		Contact *contacts;//��ǰ��ײ
		int contactsLeft;//ʣ�¿����ɵ���ײ��Ŀ
		unsigned contactCount;//��ײ����Ŀ
		real friction;//Ħ��ϵ��
		real restitution;//�ָ�ϵ��
		real tolerance;

		bool HasMoreContacts()
		{
			return contactsLeft>0;
		}

		void Reset(unsigned maxContacts)
		{
			contactsLeft=maxContacts;
			contactCount=0;
			contacts=contactArray;
		}
		void addContacts(unsigned count)
		{
			contactsLeft-=count;
			contactCount+=count;
			//�ѵ�ǰ��ײ����ָ����ǰ�ƶ�
			contacts+=count;
		}
	};*/
	
	class DLL_PHYSICS CollisionDetector//��ײ����� �������ײ��Ϣ
	{
	public:
		static unsigned SphereAndHalfSpace(const CollisionSphere &sphere,const CollisionPlane &plane,ContactListType*contactList);//
		static unsigned SphereAndTruePlane(const CollisionSphere &sphere,const CollisionPlane &plane,ContactListType*contactList);//
		static unsigned SphereAndSphere(const CollisionSphere& one,const CollisionSphere& two,ContactListType*contactList);//
		static unsigned BoxAndHalfSpace(const CollisionBox &box,const CollisionPlane &plane,ContactListType*contactList);//
		static unsigned BoxAndBox(const CollisionBox &one,const CollisionBox &two,ContactListType *contactList);//
		static unsigned BoxAndPoint(const CollisionBox &box,const Vector3 &point,ContactListType*contactList);//
		static unsigned BoxAndSphere(const CollisionBox &box,const CollisionSphere &sphere,ContactListType*contactList);//
		
		static unsigned SphereAndCircle(const CollisionSphere& sphere,const CollisionCircle& circle,ContactListType *contactList);
		static unsigned SphereAndCylinder(const CollisionSphere& sphere,const CollisionCircle& circle,ContactListType *contactList);
	};
}