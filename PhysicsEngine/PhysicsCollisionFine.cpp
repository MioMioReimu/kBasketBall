#define DLL_PHYSICS_EXIST
#include"Physics.h"
namespace Physics
{
	bool IntersectionTests::SphereAndHalfSpace(const CollisionSphere &sphere,const CollisionPlane &plane)
	{
		//���ĵ�ƽ��ľ���=
		real ballDistance=plane.direction*sphere.GetAxis(3)-sphere.radius;

		return ballDistance<=plane.offset;
	}
	bool IntersectionTests::SphereAndSphere(const CollisionSphere &one,const CollisionSphere &two)
	{
		Vector3 ballCenterDistance=one.GetAxis(3)-two.GetAxis(3);
		return ballCenterDistance.SquareMagnitude()<(one.radius+two.radius)*(one.radius+two.radius);
	}
	real IntersectionTests::TransformToAxis(const CollisionBox &box,const Vector3 &axis)
	{
		return box.halfSize.data[0]*real_abs(axis*box.GetAxis(0))+
			box.halfSize.data[1]*real_abs(axis*box.GetAxis(1))+
			box.halfSize.data[2]*real_abs(axis*box.GetAxis(2));
	}

	bool IntersectionTests::OverlapOnAxis(const CollisionBox &one,const CollisionBox &two,const Vector3 &axis,const Vector3 &toCentre)
	{
		//��box��halfSizeͶӰ������
		real oneProject=TransformToAxis(one,axis);
		real twoProject=TransformToAxis(two,axis);

		//������box����������ͶӰ������
		real distance=real_abs(toCentre*axis);
		return distance<oneProject+twoProject;
	}
	bool IntersectionTests::BoxAndBox(const CollisionBox &one,const CollisionBox &two)
	{
#define TEST_BOXOVERLAP(axis) OverlapOnAxis(one,two,(axis),toCentre)
		Vector3 toCentre=two.GetAxis(3)-one.GetAxis(3);
		bool temp=(
			TEST_BOXOVERLAP(one.GetAxis(0))&&
			TEST_BOXOVERLAP(one.GetAxis(1))&&
			TEST_BOXOVERLAP(one.GetAxis(2))&&

			TEST_BOXOVERLAP(two.GetAxis(0))&&
			TEST_BOXOVERLAP(two.GetAxis(1))&&
			TEST_BOXOVERLAP(two.GetAxis(2))&&

			TEST_BOXOVERLAP(one.GetAxis(0) % two.GetAxis(0)) &&
			TEST_BOXOVERLAP(one.GetAxis(0) % two.GetAxis(1)) &&
			TEST_BOXOVERLAP(one.GetAxis(0) % two.GetAxis(2)) &&
			TEST_BOXOVERLAP(one.GetAxis(1) % two.GetAxis(0)) &&
			TEST_BOXOVERLAP(one.GetAxis(1) % two.GetAxis(1)) &&
			TEST_BOXOVERLAP(one.GetAxis(1) % two.GetAxis(2)) &&
			TEST_BOXOVERLAP(one.GetAxis(2) % two.GetAxis(0)) &&
			TEST_BOXOVERLAP(one.GetAxis(2) % two.GetAxis(1)) &&
			TEST_BOXOVERLAP(one.GetAxis(2) % two.GetAxis(2))
			);
#undef TEST_BOXOVERLAP
		return temp;
	}

	bool IntersectionTests::BoxAndHalfSpace(const CollisionBox& box,const CollisionPlane &plane)
	{
		//������ĶԽ�����ƽ�淨�����ϵ�ͶӰ
		real projectRadius=IntersectionTests::TransformToAxis(box,plane.direction);
		//
		real boxDistance=plane.direction*box.GetAxis(3)-projectRadius-plane.offset;
		return boxDistance<=real(0.0);
	}

	unsigned CollisionDetector::SphereAndHalfSpace(const CollisionSphere &sphere,const CollisionPlane &plane,ContactListType*contactList)
	{
		if(!sphere.body->canMove)return 0;
		
		Vector3 position=sphere.GetAxis(3);
		real ballDistance=plane.direction*position-sphere.radius-plane.offset;
		if(ballDistance>=0)
		return 0;
		Contact contact ;
		contact.contactNormal=plane.direction;
		contact.penetration=-ballDistance;
		contact.contactPoint=position-plane.direction*(ballDistance+sphere.radius);
		contact.SetBodyData(sphere.body,NULL,(plane.friction+sphere.body->friction)/(real)2.0,(plane.restitution+sphere.body->restitution)/(real)2.0);

		contactList->push_back(contact);
		return 1;
	}
	unsigned CollisionDetector::SphereAndTruePlane(const CollisionSphere &sphere,const CollisionPlane &plane,ContactListType*contactList)
	{
		if(!sphere.body->canMove)return 0;
		Vector3 position=sphere.GetAxis(3);//�õ���������λ��
		real CentreDistance=plane.direction*position-plane.offset;//�õ����ĵ�ƽ��ľ���
		if(CentreDistance*CentreDistance>sphere.radius*sphere.radius)return 0;//������ĵ�ƽ��ľ��������İ뾶�򲻷�����ײ

		Contact contact;
		if(CentreDistance>=0)
			//���������ƽ�淨������ͬ��
		{
			contact.contactNormal=plane.direction;
			contact.penetration=sphere.radius-CentreDistance;
		}
		else
			//���������ƽ�淨���������
		{
			contact.contactNormal=plane.direction*-1;
			contact.penetration=sphere.radius+CentreDistance;
		}
		contact.contactPoint=position-plane.direction*CentreDistance;//������ײ��
		contact.SetBodyData(sphere.body,NULL,(plane.friction+sphere.body->friction)/(real)2.0,(plane.restitution+sphere.body->restitution)/(real)2.0);

		contactList->push_back(contact);
		return 1;
	}
	unsigned CollisionDetector::SphereAndSphere(const CollisionSphere& one,const CollisionSphere& two,ContactListType*contactList)
	{
		if((!one.body->canMove)&&(!two.body->canMove))return 0;//����������嶼�ǹ̶����򷵻�
		Vector3 CentreOne=one.GetAxis(3);//�õ������������λ��
		Vector3 CentreTwo=two.GetAxis(3);
		Vector3 CentreDistance=CentreOne-CentreTwo;//���������������
		real CentreDisMagn=CentreDistance.Magnitude();//
		if(CentreDisMagn<=0.0||CentreDisMagn>=one.radius+two.radius)return 0;//�ж��Ƿ��ཻ
		//if(CentreDisMagn==0.0)CentreDisMagn=

		Contact contact;
		contact.contactNormal=CentreDistance*(((real)1.0)/CentreDisMagn);
		contact.penetration=(one.radius+two.radius-CentreDisMagn);
		contact.contactPoint=CentreOne+CentreDistance*(real)0.5;
		if(!one.body->canMove)
			contact.SetBodyData(NULL,two.body,(one.body->friction+two.body->friction)/(real)2.0
			,(one.body->restitution+two.body->restitution)/(real)2.0);
		else if(!two.body->canMove)
			contact.SetBodyData(one.body,NULL,(one.body->friction+two.body->friction)/(real)2.0
			,(one.body->restitution+two.body->restitution)/(real)2.0);
		else
			contact.SetBodyData(one.body,two.body,(one.body->friction+two.body->friction)/(real)2.0
			,(one.body->restitution+two.body->restitution)/(real)2.0);
		contactList->push_back(contact);
		//data->addContacts(1);

		return 1;
	}
	unsigned CollisionDetector::BoxAndHalfSpace(const CollisionBox &box,const CollisionPlane &plane,ContactListType*contactList)
	{
		if(!box.body->canMove)return 0;//��������ǹ̶����򷵻�
		if(!IntersectionTests::BoxAndHalfSpace(box,plane))return 0;
		static real mults[8][3]={{1,1,1},{-1,1,1},{1,-1,1},{-1,-1,1},{1,1,-1},{-1,1,-1},{1,-1,-1},{-1,-1,-1}};
		Contact contact;
		unsigned contactsUsed=0;
		for(unsigned i=0;i<8;i++)
		{
			Vector3 vertexPos(mults[i][0],mults[i][1],mults[i][2]);
			vertexPos/=box.halfSize;
			vertexPos=box.transform.Transform(vertexPos);
			real vertexDistance=vertexPos*plane.direction;
			if(vertexDistance<=plane.offset)
			{
				//��ײ����Ϊbox�Ķ���
				contact.contactPoint=vertexPos;
				//�����ײ����Ϊbox�Ķ���Ͷ��������ߵ�ƽ��ĵ� ���е�
				//contact->contactPoint=vertexPos+plane.direction*(((real)0.5)*(plane.offset-vertexDistance));
				contact.contactNormal=plane.direction;
				contact.penetration=plane.offset-vertexDistance;
				contact.SetBodyData(box.body,NULL,(plane.friction+box.body->friction)/(real)2.0,(plane.restitution+box.body->restitution)/(real)2.0);
				contactList->push_back(contact);
				//contact++;
				//contactsUsed++;
				//if(contactsUsed==data->contactsLeft) 
				//{
				//	data->addContacts(contactsUsed);
				//	return contactsUsed;
				//}
			}
		}
		//data->addContacts(contactsUsed);
		return contactsUsed;
	}
	unsigned CollisionDetector::BoxAndSphere(const CollisionBox &box,const CollisionSphere &sphere,ContactListType*contactList)
	{
		if((!box.body->canMove)&&(!sphere.body->canMove))return 0;
		//if(data->contactsLeft<=0)return 0;
		Vector3 centre=sphere.GetAxis(3);
		Vector3 relatedCentre=box.transform.TransformInverse(centre);
		//���ݷ����ᶨ�� �������ĺͰ�Χ�е��ľ���ֱ����������ϵķ����� ����һ�����ϴ��� ��뾶+��Χ�а�Խ�������Ӧ�����ϵ�ͶӰ����
		//����Ͱ�Χ�в��ཻ
		//ע��3��������������ཻ ��������Ͱ�Χ���ཻ
		//������һ���������ϵ�ͶӰ���㲻�ཻ ����Ͱ�Χ�в��ཻ
		if(real_abs(relatedCentre.data[0])-sphere.radius>box.halfSize.data[0]||
			real_abs(relatedCentre.data[1])-sphere.radius>box.halfSize.data[1]||
			real_abs(relatedCentre.data[2])-sphere.radius>box.halfSize.data[2])
			return 0;
		Vector3 closestPt(0,0,0);
		real distance;
		//�����Χ�����ĸ��������������
		for(int i=0;i<3;i++)
		{
			distance=relatedCentre.data[i];
			if(distance>box.halfSize.data[i])distance=box.halfSize.data[i];
			if(distance<-box.halfSize.data[i])distance=-box.halfSize.data[i];
			closestPt.data[i]=distance;
		}
		distance=(closestPt-relatedCentre).SquareMagnitude();
		if(distance>sphere.radius*sphere.radius) return 0;//�����뾶��������㵽���ĵľ������ཻ
		Vector3 closestPtWorld=box.transform.Transform(closestPt);

		//����contact��Ϣ
		Contact contact;
		contact.contactNormal=(closestPtWorld-centre);
		contact.contactNormal.Normalize();
		contact.contactPoint=closestPtWorld;
		contact.penetration=sphere.radius-real_sqrt(distance);
		if(!box.body->canMove)
			contact.SetBodyData(NULL,sphere.body,(box.body->friction+sphere.body->friction)/(real)2.0
			,(box.body->restitution+sphere.body->restitution)/(real)2.0);
		else if(!sphere.body->canMove)
			contact.SetBodyData(box.body,NULL,(box.body->friction+sphere.body->friction)/(real)2.0
			,(box.body->restitution+sphere.body->restitution)/(real)2.0);
		else
			contact.SetBodyData(box.body,sphere.body,(box.body->friction+sphere.body->friction)/(real)2.0
			,(box.body->restitution+sphere.body->restitution)/(real)2.0);
		contactList->push_back(contact);

		//data->addContacts(1);
		return 1;
	}

	static inline real PenetrationOnAxis(const CollisionBox &one,const CollisionBox &two,const Vector3 &axis, const Vector3 &toCentre)//�����������box��ĳ�����ϵ�contact���
		//Ҫ��axis��Box������*box�ı任���������ϵ��ͬ
	{
		//box�İ�Խ��������ϵ�ͶӰ
		real oneProject=IntersectionTests::TransformToAxis(one,axis);
		real twoProject=IntersectionTests::TransformToAxis(two,axis);

		//box�������������ϵ�ͶӰ
		real distance=real_abs(toCentre*axis);

		//������������ϵ�contact���
		return oneProject+twoProject-distance;
	}

	static inline bool TryAxis(const CollisionBox &one,const CollisionBox &two,const Vector3 &axis,const Vector3& toCentre,unsigned index,
		real & smallestPenetration,unsigned &smallestCase)
	{
		real penetration=PenetrationOnAxis(one,two,axis,toCentre);
		if(penetration<0)return false;
		if(penetration<smallestPenetration)
		{
			smallestPenetration=penetration;
			smallestCase=index;
		}
		return true;
	}

#define CHECK_OVERLAP(axis,index)\
	if(!TryAxis(one,two,(axis),toCentre,(index),pen,best)) return 0;

	void FillPointFaceBoxBox(const CollisionBox &one,const CollisionBox &two,const Vector3 &toCentre,ContactListType*contactList,unsigned best,real pen)
		//�����������box2����Ķ����box1��ײ
		//���ڵ����ײ�͵����ײ���ʺ�С �������迼��
	{
		Contact contact;
		Vector3 normal=one.GetAxis(best);//ȷ�����Ǹ�����ײ
		if(one.GetAxis(best)*toCentre>0)
			normal=normal*-1.0f;

		Vector3 vertex=two.halfSize;
		if(two.GetAxis(0)*normal<0)vertex.data[0]=-vertex.data[0];
		if(two.GetAxis(1)*normal<0)vertex.data[0]=-vertex.data[0];
		if(two.GetAxis(2)*normal<0)vertex.data[0]=-vertex.data[0];

		contact.contactNormal=normal;
		contact.contactPoint=two.GetTransform().Transform(vertex);
		contact.penetration=pen;
		if(!one.body->canMove)
			contact.SetBodyData(NULL,two.body,(one.body->friction+two.body->friction)/(real)2.0
			,(one.body->restitution+two.body->restitution)/(real)2.0);
		else if(!two.body->canMove)
			contact.SetBodyData(one.body,NULL,(one.body->friction+two.body->friction)/(real)2.0
			,(one.body->restitution+two.body->restitution)/(real)2.0);
		else
			contact.SetBodyData(one.body,two.body,(one.body->friction+two.body->friction)/(real)2.0
			,(one.body->restitution+two.body->restitution)/(real)2.0);
		contactList->push_back(contact);
	};

	static inline Vector3 contactPoint(const Vector3 &pOne,const Vector3 &dOne,real oneSize,
		const Vector3 &pTwo,const Vector3 &dTwo,real twoSize,bool useOne)
	{
		Vector3 toSt,cOne,cTwo;
		real dpStaOne,dpStaTwo,dpOneTwo,smOne,smTwo;
		real denom,mua,mub;

		smOne=dOne.SquareMagnitude();
		smTwo=dTwo.SquareMagnitude();
		dpOneTwo=dTwo*dOne;

		toSt=pOne-pTwo;
		dpStaOne=dOne*toSt;
		dpStaTwo=dTwo*toSt;

		denom=smOne*smTwo-dpOneTwo*dpOneTwo;

		//
		if(real_abs(denom)<0.0001f)
			return useOne?pOne:pTwo;

		mua=(dpOneTwo*dpStaTwo -smTwo*dpStaOne)/denom;
		mub=(smOne*dpStaTwo - dpOneTwo*dpStaOne)/denom;

		if(mua>oneSize||mua<-oneSize||mub>twoSize||mub<-twoSize)
			return useOne?pOne:pTwo;
		else
		{
			cOne=pOne+dOne*mua;
			cTwo=pTwo+dTwo*mub;
			return cOne*0.5+cTwo*0.5;
		}
	}

	unsigned CollisionDetector::BoxAndBox(const CollisionBox &one,const CollisionBox &two,ContactListType *contactList)
	{
		if((!one.body->canMove)&&(!two.body->canMove))return 0;//����������嶼�ǹ̶����򷵻�
		//
		Vector3 toCentre=two.GetAxis(3)-one.GetAxis(3);

		//
		real pen=REAL_MAX;
		unsigned best=0xffffff;

		//
		CHECK_OVERLAP(one.GetAxis(0),0);
		CHECK_OVERLAP(one.GetAxis(1),1);
		CHECK_OVERLAP(one.GetAxis(2),2);

		CHECK_OVERLAP(two.GetAxis(0),3);
		CHECK_OVERLAP(two.GetAxis(1),4);
		CHECK_OVERLAP(two.GetAxis(2),5);

		unsigned bestSingleAxis=best;

		CHECK_OVERLAP(one.GetAxis(0)%two.GetAxis(0),6);
		CHECK_OVERLAP(one.GetAxis(0)%two.GetAxis(1),7);
		CHECK_OVERLAP(one.GetAxis(0)%two.GetAxis(2),8);
		CHECK_OVERLAP(one.GetAxis(1)%two.GetAxis(0),9);
		CHECK_OVERLAP(one.GetAxis(1)%two.GetAxis(1),10);
		CHECK_OVERLAP(one.GetAxis(1)%two.GetAxis(2),11);
		CHECK_OVERLAP(one.GetAxis(2)%two.GetAxis(0),12);
		CHECK_OVERLAP(one.GetAxis(2)%two.GetAxis(1),13);
		CHECK_OVERLAP(one.GetAxis(2)%two.GetAxis(2),14);

		//
		if(best==0xffffff)return 0;

		//��������֪��
		if(best<3)
		{
			FillPointFaceBoxBox(one,two,toCentre,contactList,best,pen);
			return 1;
		}
		else if(best<6)
		{
			FillPointFaceBoxBox(two,one,toCentre*-1,contactList,best-3,pen);
			return 1;
		}
		else
		{
			//��һ����߻��߱߱���ײ 
			//�ҵ���������
			best-=6;
			unsigned oneAxisIndex=best/3;
			unsigned twoAxisIndex=best%3;
			Vector3 oneAxis=one.GetAxis(oneAxisIndex);
			Vector3 twoAxis=two.GetAxis(twoAxisIndex);
			Vector3 axis=oneAxis%twoAxis;
			axis.Normalize();

			if(axis*toCentre>0) axis=axis*-1.0f;

			Vector3 ptOnOneEdge=one.halfSize;
			Vector3 ptOnTwoEdge=two.halfSize;
			for(unsigned i=0;i<3;i++)
			{
				if(i==oneAxisIndex) ptOnOneEdge[i]=0;
				else if(one.GetAxis(i)*axis>0)ptOnOneEdge[i]=-ptOnOneEdge[i];

				if(i==twoAxisIndex) ptOnTwoEdge[i]=0;
				else if(two.GetAxis(i)*axis<0)ptOnTwoEdge[i]=-ptOnTwoEdge[i];
			}

			//
			ptOnOneEdge=one.transform.Transform(ptOnOneEdge);
			ptOnOneEdge=one.transform.Transform(ptOnTwoEdge);

			//
			Vector3 vertex=contactPoint(ptOnOneEdge,oneAxis,one.halfSize[oneAxisIndex],
				ptOnTwoEdge,twoAxis,two.halfSize[twoAxisIndex],bestSingleAxis>2);

			//
			Contact contact;

			contact.contactNormal=axis;
			contact.contactPoint=vertex;
			contact.penetration=pen;
			if(!one.body->canMove)
				contact.SetBodyData(NULL,two.body,(one.body->friction+two.body->friction)/(real)2.0
				,(one.body->restitution+two.body->restitution)/(real)2.0);
			else if(!two.body->canMove)
				contact.SetBodyData(one.body,NULL,(one.body->friction+two.body->friction)/(real)2.0
				,(one.body->restitution+two.body->restitution)/(real)2.0);
			else
				contact.SetBodyData(one.body,two.body,(one.body->friction+two.body->friction)/(real)2.0
				,(one.body->restitution+two.body->restitution)/(real)2.0);
			contactList->push_back(contact);
			return 1;
		}
		return 0;
	}

	unsigned CollisionDetector::BoxAndPoint(const CollisionBox &box,const Vector3 &point,ContactListType*contactList)
	{
		//����������ϵ�еĵ�任����Χ�е�����ϵ��
		Vector3 relatedPt=box.transform.TransformInverse(point);

		Vector3 normal;

		//
		real min_depth=box.halfSize[0]-real_abs(relatedPt[0]);
		if(min_depth<0)return 0;
		normal=box.GetAxis(0)*(real)((relatedPt[0])?-1:1);

		real depth=box.halfSize[1]-real_abs(relatedPt[1]);
		if(depth<0)return 0;
		else if(depth<min_depth)
		{
			min_depth=depth;
			normal=box.GetAxis(1)*(real)((relatedPt[1])?-1:1);
		}

		depth=box.halfSize[2]-real_abs(relatedPt[2]);
		if(depth<0)return 0;
		else if(depth<min_depth)
		{
			min_depth=depth;
			normal=box.GetAxis(2)*(real)((relatedPt[1])?-1:1);
		}

		//
		Contact contact;
		contact.contactNormal=normal;
		contact.contactPoint=point;
		contact.penetration=min_depth;

		//
		contact.SetBodyData(box.body,NULL,box.body->friction,box.body->restitution);


		return 1;
	}

	unsigned CollisionDetector::SphereAndCircle(const CollisionSphere& sphere,const CollisionCircle& circle,ContactListType *contactList)
	{
		if((!sphere.body->canMove)&&(!circle.body->canMove))return 0;//����������嶼�ǹ̶����򷵻�
		Vector3 SphereCentre=sphere.GetAxis(3);
		Vector3 CircleCentre=circle.GetAxis(3);
		Vector3 normal=circle.transform.TransformDirection(circle.normal);
		real mag=(SphereCentre-CircleCentre)*normal;//���ĵ�Բƽ��ľ���
		if(real_abs(mag)>=sphere.radius)return 0;//���������뾶 ���ཻ

		Vector3 Circle1Centre=SphereCentre-normal*mag;//������ĵ�Բƽ��Ľ���Բ��
		real radius1=sqrtf(sphere.radius*sphere.radius-mag*mag);//�������İ뾶
		assert(radius1<circle.radius);//���� ����ԲС��Բ
		Vector3 circleVector=Circle1Centre-CircleCentre;//Բ�ĵ�����Բ�ĵ�����
		real circleDistance=circleVector.Magnitude();//�õ�Բ�ļ�ľ���
		circleVector.Normalize();//Բ��������һ��
		
		
		if(circleDistance>=(radius1+circle.radius)||circle.radius>=(radius1+circleDistance))return 0;//�������Բ��Բ����������� ���򷵻�
		Vector3 NearPoint=circleVector*(circle.radius-circleDistance)+Circle1Centre;//���Բ�ϵ�����ԲԲ������ĵ�

		Contact contact;
		contact.contactPoint=NearPoint;//�����
		NearPoint=SphereCentre-NearPoint;//����
		contact.penetration=NearPoint.Magnitude()-sphere.radius;//������С���
		NearPoint.Normalize();
		contact.contactNormal=NearPoint;//����
		contact.SetBodyData(sphere.body,NULL,(sphere.body->friction+circle.body->friction)/(real)2.0
			,(sphere.body->restitution+circle.body->restitution)/(real)2.0);
		contactList->push_back(contact);
		return 1;
	}
}