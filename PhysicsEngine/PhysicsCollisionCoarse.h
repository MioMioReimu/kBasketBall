#pragma once
//#include"PhysicsForceGenerate.h"
//#include"PhysicsWorld.h"
namespace Physics
{
	class DLL_PHYSICS RigidBody;
	//粗糙包围球
	class DLL_PHYSICS BoundingSphere
	{
	public:
		Vector3 centre;
		real radius;
	public:
		//默认构造函数
		BoundingSphere():centre(){radius=(real)0.0f;};
		BoundingSphere(const Vector3 &centre,real radius);	//根据球心坐标和半径构造一个包容体
		BoundingSphere(const BoundingSphere &one,const BoundingSphere &two);	//根据两个包容体构造一个包容它们两个的包容体
		bool OverLaps(const BoundingSphere& other)const;		//判断两个包容体是否碰撞
		real GetGrowth(const BoundingSphere& other)const;	//本包容体和other合并后的包容体与本包容体体积的差别
		real GetSize()const;//返回这个包容体的体积
	};

	//可能碰撞对
	struct DLL_PHYSICS PotentialContact
	{
		RigidBody *body[2];
		int ContactType;
	};
	//可能碰撞对的数组
	template class DLL_PHYSICS allocator<PotentialContact>;
	template class DLL_PHYSICS vector<PotentialContact,allocator<PotentialContact>>;
	typedef vector<PotentialContact> PotentialContactListType;
	//BoundingVolume树节点类
	class DLL_PHYSICS BVHNode
	{
	public:
		BVHNode *children[2];			//这个节点的孩子节点
		BoundingSphere volume;//这个节点的包容体
		RigidBody *body;					//这个节点的物体，只有叶子节点才可以有。其他为NULL
		BVHNode *parent;			//这个节点的父节点，根节点为NULL
	public:
		BVHNode(BVHNode *parent,const BoundingSphere &volume,	//构造函数
			RigidBody* body=NULL):parent(parent),volume(volume),body(body)
		{
			children[0]=children[1]=NULL;
		}
		bool IsLeaf()const		//判断该节点是否死叶子节点
		{
			return (body!=NULL);
		}

	public:
		unsigned GetPotentialContacts(PotentialContactListType* p)const;	//检查这个节点及其所有的子节点，把它们写到给的contacts数组里面去，上限为limit
		void Insert(RigidBody *body,const BoundingSphere &volume);	//插入给定的物体，用给定的包容体到该层，
		~BVHNode()	;	//删除这个节点及其它的子节点和物体指针
		bool Overlaps(const BVHNode*other)const;	//判断这两个节点的包容体是否碰撞
		unsigned GetPotentialContactsWith(const BVHNode*other,
			PotentialContactListType* p)const;	//
		void RecalculateBoundingVolume(bool recurse=true);//对于非叶子节点，这个函数可以更新它包容体，基于其子节点的包容体（当其子节点发生变化时调用）
	};
	/*
	template<typename BoundingVolumeClass>
	class BVHNode
	{
	public:
			//子节点 叶子节点为NULL
		BVHNode *children[2];
			//包容体
		BoundingVolumeClass volume;
			//非叶子节点为NULL 叶子节点包含的物体
		RigidBody *body;
			//父节点 root节点为NULL
		BVHNode* parent;
	public:
		BVHNode(BVHNode *parent,const BoundingVolumeClass &volume,RigidBody* body=NULL)
		:parent(parent),volume(volume),body(body)
		{
			children[0]=children[1]=NULL;
		}

		bool IsLeaf()const
		{
			return (body!=NULL);
		}
		//
		unsigned GetPotentialContacts(PotentialContact *contacts,unsigned limit)const
		{
			if(isLeaf()||limit==0)return 0;
			return children[0]->GetPotentialContactsWith(children[1],contacts,limit);
		}
		void Insert(RigidBody* body,const BoundingVolumeClass &volume);
		~BVHNode();
	protected:
		bool OverLaps(const BVHNode<BoundingVolumeClass> *other)const;
		unsigned GetPotentialContactsWith(const BVHNode<BoundingVolumeClass>*other,PotentialContacts* contacts,unsigned limit)const;
		void RecalculateBoundingVoluem(bool recurse=true);
	};*/
}