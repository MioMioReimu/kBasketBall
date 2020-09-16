#define DLL_PHYSICS_EXIST
#include"Physics.h"
namespace Physics
{
	BoundingSphere::BoundingSphere(const Vector3 &centre,real radius)	//根据球心坐标和半径构造一个包容体
	{
		this->centre=centre;
		this->radius=radius;
	}
	BoundingSphere::BoundingSphere(const BoundingSphere &one,const BoundingSphere &two)	//根据两个包容体构造一个包容它们两个的包容体
	{
		Vector3 temp=two.centre-one.centre;
		real d=temp.Magnitude();
		real radiusDiff=real_abs(two.radius-one.radius);
		if(radiusDiff>=d)	//如果大球包容了小球
		{
			if(one.radius>two.radius)
			{
				centre=one.centre;
				radius=one.radius;
			}
			else
			{
				centre=two.centre;
				radius=two.radius;
			}
		}
		else
		{
			radius=(d+one.radius+two.radius)/2;	//半径=（r1+r2+球心距离）/2
			centre=one.centre+temp*((radius-one.radius)/(d));//球心坐标=球心1坐标+方向向量*（半径-球1半径）
		}
	}
	bool BoundingSphere::OverLaps(const BoundingSphere& other)const		//判断两个包容体是否碰撞
	{
		//assert((this->centre.data[0]!=other.centre.data[0])&&(this->centre.data[1]!=other.centre.data[1])&&(this->centre.data[2]!=other.centre.data[2]));
		real temp=(this->centre-other.centre).Magnitude();
		//assert(temp>=0.001f);
		if(temp<real_abs(this->radius+other.radius))
			return true;
		return false;
	}
	real BoundingSphere::GetGrowth(const BoundingSphere& other)const	//
	{
		BoundingSphere temp((*this),other);
		return temp.radius*temp.radius-radius*radius;
		//return real_abs(this->GetSize()-temp.GetSize());
	}
	real BoundingSphere::GetSize()const//返回这个包容体的体积
	{
		return this->radius*this->radius*this->radius*(real)4.0*(real)REAL_PI/(real)3.0;
	}

	/*template<typename BoundingVolumeClass>
	bool BVHNode<BoundingVolumeClass>::OverLaps(const BVHNode<BoundingVolumeClass> *other)const
	{
		return volume.overlaps(other->volume);
	}
	template<typename BoundingVolumeClass>
	unsigned BVHNode<BoundingVolumeClass>::GetPotentialContactsWith(const BVHNode<BoundingVolumeClass>*other,PotentialContacts* contacts,unsigned limit)const;
	template<typename BoundingVolumeClass>
	void BVHNode<BoundingVolumeClass>::RecalculateBoundingVoluem(bool recurse=true);
	*/
	unsigned BVHNode::GetPotentialContacts(PotentialContactListType* p)const	//检查这个节点及其所有的子节点，把它们写到给的contacts数组里面去，上限为limit
	{
		if(IsLeaf())return 0;
		return this->children[0]->GetPotentialContactsWith(children[1],p);
	}
	void BVHNode::Insert(RigidBody *otherBody,const BoundingSphere &otherVolume)	//插入给定的物体，用给定的包容体到该层，
	{
		if(IsLeaf())
		{
			children[0]=new BVHNode(this,volume,body);
			children[1]=new BVHNode(this,otherVolume,otherBody);
			this->body=NULL;
			this->RecalculateBoundingVolume();
		}
		else
		{
			if(children[0]->volume.GetGrowth(otherVolume)<children[1]->volume.GetGrowth(otherVolume))
				children[0]->Insert(otherBody,otherVolume);
			else
				children[1]->Insert(otherBody,otherVolume);
		}
	}
	BVHNode::~BVHNode()	//删除这个节点及其它的子节点和物体指针
	{
		if(parent)//如果不是根节点，析构该节点之后其父节点则只有一个孩子，需要把这个孩子即该节点的兄弟 上移
		{
			BVHNode* sibling;
			if(parent->children[0]==this)sibling=parent->children[1];
			else sibling=parent->children[0];

			parent->volume=sibling->volume;
			parent->body=sibling->body;
			parent->children[0]=sibling->children[0];
			parent->children[1]=sibling->children[1];

			sibling->parent=NULL;
			sibling->body=NULL;
			sibling->children[0]=NULL;
			sibling->children[1]=NULL;
			delete sibling;

			parent->RecalculateBoundingVolume(true);
		}

		if(children[0])				//递归析构其孩子
		{
			children[0]->parent=NULL;
			delete children[0];
		};
		if(children[1])				//递归析构其孩子
		{
			children[1]->parent=NULL;
			delete children[1];
		}
	}
	bool BVHNode::Overlaps(const BVHNode*other)const//判断这两个节点的包容体是否碰撞
	{
		return this->volume.OverLaps(other->volume);
	}
	unsigned BVHNode::GetPotentialContactsWith(const BVHNode*other,
		PotentialContactListType* p)const	//
	{
		//如果两个不相交或者limit达到上限
		//if((!Overlaps(other))&&(this->IsLeaf())&&(other->IsLeaf()))return 0;
			//如果两个都是叶子节点
		if(IsLeaf()&&other->IsLeaf())
		{
			if(!Overlaps(other))return 0;
			PotentialContact temp;
			temp.body[0]=this->body;
			temp.body[1]=other->body;
			temp.ContactType=this->body->PrimitiveType+other->body->PrimitiveType;
			p->push_back(temp);
			return 1;
		}
			//如果other是叶子节点 或者 this不是叶子节点且this的包容体比other大
		if(other->IsLeaf()||(!IsLeaf()&&volume.GetSize()>=other->volume.GetSize()))
		{
			
			children[0]->GetPotentialContactsWith(children[1],p);
			if(!Overlaps(other))return 0;
			children[0]->GetPotentialContactsWith(other,p);
			children[1]->GetPotentialContactsWith(other,p);
			return 1;
		}
		if(this->IsLeaf()||(!IsLeaf()&&volume.GetSize()<other->volume.GetSize()))
		{
			
			other->children[0]->GetPotentialContactsWith(other->children[1],p);
			if(!Overlaps(other))return 0;
			this->GetPotentialContactsWith(other->children[0],p);
			this->GetPotentialContactsWith(other->children[1],p);
			return 1;
		}
	}
	void BVHNode::RecalculateBoundingVolume(bool recurse)//对于非叶子节点，这个函数可以更新它包容体，基于其子节点的包容体（当其子节点发生变化时调用）
	{
		if(this->IsLeaf())return;
		this->volume=BoundingSphere(children[0]->volume,children[1]->volume);
		if(parent)parent->RecalculateBoundingVolume(true);
	}
}