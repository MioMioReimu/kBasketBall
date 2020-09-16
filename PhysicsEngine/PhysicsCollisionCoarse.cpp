#define DLL_PHYSICS_EXIST
#include"Physics.h"
namespace Physics
{
	BoundingSphere::BoundingSphere(const Vector3 &centre,real radius)	//������������Ͱ뾶����һ��������
	{
		this->centre=centre;
		this->radius=radius;
	}
	BoundingSphere::BoundingSphere(const BoundingSphere &one,const BoundingSphere &two)	//�������������幹��һ���������������İ�����
	{
		Vector3 temp=two.centre-one.centre;
		real d=temp.Magnitude();
		real radiusDiff=real_abs(two.radius-one.radius);
		if(radiusDiff>=d)	//������������С��
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
			radius=(d+one.radius+two.radius)/2;	//�뾶=��r1+r2+���ľ��룩/2
			centre=one.centre+temp*((radius-one.radius)/(d));//��������=����1����+��������*���뾶-��1�뾶��
		}
	}
	bool BoundingSphere::OverLaps(const BoundingSphere& other)const		//�ж������������Ƿ���ײ
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
	real BoundingSphere::GetSize()const//�����������������
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
	unsigned BVHNode::GetPotentialContacts(PotentialContactListType* p)const	//�������ڵ㼰�����е��ӽڵ㣬������д������contacts��������ȥ������Ϊlimit
	{
		if(IsLeaf())return 0;
		return this->children[0]->GetPotentialContactsWith(children[1],p);
	}
	void BVHNode::Insert(RigidBody *otherBody,const BoundingSphere &otherVolume)	//������������壬�ø����İ����嵽�ò㣬
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
	BVHNode::~BVHNode()	//ɾ������ڵ㼰�������ӽڵ������ָ��
	{
		if(parent)//������Ǹ��ڵ㣬�����ýڵ�֮���丸�ڵ���ֻ��һ�����ӣ���Ҫ��������Ӽ��ýڵ���ֵ� ����
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

		if(children[0])				//�ݹ������亢��
		{
			children[0]->parent=NULL;
			delete children[0];
		};
		if(children[1])				//�ݹ������亢��
		{
			children[1]->parent=NULL;
			delete children[1];
		}
	}
	bool BVHNode::Overlaps(const BVHNode*other)const//�ж��������ڵ�İ������Ƿ���ײ
	{
		return this->volume.OverLaps(other->volume);
	}
	unsigned BVHNode::GetPotentialContactsWith(const BVHNode*other,
		PotentialContactListType* p)const	//
	{
		//����������ཻ����limit�ﵽ����
		//if((!Overlaps(other))&&(this->IsLeaf())&&(other->IsLeaf()))return 0;
			//�����������Ҷ�ӽڵ�
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
			//���other��Ҷ�ӽڵ� ���� this����Ҷ�ӽڵ���this�İ������other��
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
	void BVHNode::RecalculateBoundingVolume(bool recurse)//���ڷ�Ҷ�ӽڵ㣬����������Ը����������壬�������ӽڵ�İ����壨�����ӽڵ㷢���仯ʱ���ã�
	{
		if(this->IsLeaf())return;
		this->volume=BoundingSphere(children[0]->volume,children[1]->volume);
		if(parent)parent->RecalculateBoundingVolume(true);
	}
}