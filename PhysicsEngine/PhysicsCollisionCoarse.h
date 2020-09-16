#pragma once
//#include"PhysicsForceGenerate.h"
//#include"PhysicsWorld.h"
namespace Physics
{
	class DLL_PHYSICS RigidBody;
	//�ֲڰ�Χ��
	class DLL_PHYSICS BoundingSphere
	{
	public:
		Vector3 centre;
		real radius;
	public:
		//Ĭ�Ϲ��캯��
		BoundingSphere():centre(){radius=(real)0.0f;};
		BoundingSphere(const Vector3 &centre,real radius);	//������������Ͱ뾶����һ��������
		BoundingSphere(const BoundingSphere &one,const BoundingSphere &two);	//�������������幹��һ���������������İ�����
		bool OverLaps(const BoundingSphere& other)const;		//�ж������������Ƿ���ײ
		real GetGrowth(const BoundingSphere& other)const;	//���������other�ϲ���İ������뱾����������Ĳ��
		real GetSize()const;//�����������������
	};

	//������ײ��
	struct DLL_PHYSICS PotentialContact
	{
		RigidBody *body[2];
		int ContactType;
	};
	//������ײ�Ե�����
	template class DLL_PHYSICS allocator<PotentialContact>;
	template class DLL_PHYSICS vector<PotentialContact,allocator<PotentialContact>>;
	typedef vector<PotentialContact> PotentialContactListType;
	//BoundingVolume���ڵ���
	class DLL_PHYSICS BVHNode
	{
	public:
		BVHNode *children[2];			//����ڵ�ĺ��ӽڵ�
		BoundingSphere volume;//����ڵ�İ�����
		RigidBody *body;					//����ڵ�����壬ֻ��Ҷ�ӽڵ�ſ����С�����ΪNULL
		BVHNode *parent;			//����ڵ�ĸ��ڵ㣬���ڵ�ΪNULL
	public:
		BVHNode(BVHNode *parent,const BoundingSphere &volume,	//���캯��
			RigidBody* body=NULL):parent(parent),volume(volume),body(body)
		{
			children[0]=children[1]=NULL;
		}
		bool IsLeaf()const		//�жϸýڵ��Ƿ���Ҷ�ӽڵ�
		{
			return (body!=NULL);
		}

	public:
		unsigned GetPotentialContacts(PotentialContactListType* p)const;	//�������ڵ㼰�����е��ӽڵ㣬������д������contacts��������ȥ������Ϊlimit
		void Insert(RigidBody *body,const BoundingSphere &volume);	//������������壬�ø����İ����嵽�ò㣬
		~BVHNode()	;	//ɾ������ڵ㼰�������ӽڵ������ָ��
		bool Overlaps(const BVHNode*other)const;	//�ж��������ڵ�İ������Ƿ���ײ
		unsigned GetPotentialContactsWith(const BVHNode*other,
			PotentialContactListType* p)const;	//
		void RecalculateBoundingVolume(bool recurse=true);//���ڷ�Ҷ�ӽڵ㣬����������Ը����������壬�������ӽڵ�İ����壨�����ӽڵ㷢���仯ʱ���ã�
	};
	/*
	template<typename BoundingVolumeClass>
	class BVHNode
	{
	public:
			//�ӽڵ� Ҷ�ӽڵ�ΪNULL
		BVHNode *children[2];
			//������
		BoundingVolumeClass volume;
			//��Ҷ�ӽڵ�ΪNULL Ҷ�ӽڵ����������
		RigidBody *body;
			//���ڵ� root�ڵ�ΪNULL
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