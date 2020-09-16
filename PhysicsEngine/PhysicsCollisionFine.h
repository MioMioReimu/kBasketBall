#pragma once
//#include"PhysicsContact.h"
//#include"PhysicsCollisionCoarse.h"
namespace Physics
{
	class DLL_PHYSICS IntersectionTests;
	class DLL_PHYSICS CollisionDetector;

	//碰撞基物体 
	class DLL_PHYSICS CollisionPrimitive
	{
	public:
		friend IntersectionTests;
		friend CollisionDetector;
		RigidBody *body;	//该基物体代表的body
		Matrix4 offset;		//这个基体的坐标系到物体的坐标系的变换矩阵
		void CalculateInternals()
		{
			transform=body->GetTransform()*offset;
		}
		Vector3 GetAxis(unsigned index)const
		{
			return transform.GetAxisVector(index);//得到变换矩阵的列向量，一般用于得到第四列向量,即物体中心的位置
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
		Matrix4 transform;//自身坐标系到世界的变换矩阵
	};

	//在碰撞中用于代表一个可以当做球的刚体
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
	//在碰撞中用于代表一个可以当做圆环的刚体 比如篮球框
	class DLL_PHYSICS CollisionCircle:public CollisionPrimitive
	{
	public:
		real radius;
		Vector3 normal;//圆的自身坐标系中的单位法线
		CollisionCircle(real radius,const Vector3& normal,RigidBody*body):
		radius(radius),normal(normal),CollisionPrimitive(body)
		{}
		CollisionCircle(real radius,const Vector3 & normal,RigidBody *body,const Matrix4 &m):
		radius(radius),normal(normal),CollisionPrimitive(body,m)
		{}
		CollisionCircle(){radius=0;body=NULL;}
	};
	//在碰撞中用于代表一个可以当做圆柱体的刚体
	class DLL_PHYSICS CollisionCylinder:public CollisionPrimitive
	{
	public:
		real radius;
		Vector3 normal;//圆柱体在自身坐标系中的法向量 其长度表示圆柱体的长度
	};

	//平面不是一个primitive 它不用于代表任何刚体，它用于物体碰撞世界不动的几何体
	//这个用于代表墙壁 地面等不能移动的物体
	class DLL_PHYSICS CollisionPlane
	{
	public:

		Vector3 direction;	//平面法线
		real offset;	//平面的位置
		real friction;
		real restitution;
	public:
		CollisionPlane(){friction=0.0;restitution=1.0;offset=0;direction.SetData(0,1,0);};
		CollisionPlane(real friction,real restitution,real offset,const Vector3 &normal):
		friction(friction),restitution(restitution),offset(offset),direction(normal)
		{}
	};
	//OBB包围盒
	//代表一个可以当做长方体的刚体
	class DLL_PHYSICS CollisionBox:public CollisionPrimitive
	{
	public:
		Vector3 halfSize;//长方体的长宽高的一半
		CollisionBox(const Vector3& halfSize,RigidBody* body,const Matrix4& offset):halfSize(halfSize),CollisionPrimitive(body,offset)
		{}
		CollisionBox(const Vector3& halfSize,RigidBody* body):halfSize(halfSize),CollisionPrimitive(body)
		{};
		CollisionBox(){this->body=NULL;}
	};
	//这个类里面用于碰撞基体的碰撞测试 即是否碰撞不检测碰撞穿刺深度
	//这个类里面的函数全是静态函数
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
	//用于存储碰撞的信息
	/*struct CollisionData
	{
		Contact *contactArray;//可能碰撞数组
		Contact *contacts;//当前碰撞
		int contactsLeft;//剩下可容纳的碰撞数目
		unsigned contactCount;//碰撞的数目
		real friction;//摩擦系数
		real restitution;//恢复系数
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
			//把当前碰撞处理指针往前移动
			contacts+=count;
		}
	};*/
	
	class DLL_PHYSICS CollisionDetector//碰撞检测器 会输出碰撞信息
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