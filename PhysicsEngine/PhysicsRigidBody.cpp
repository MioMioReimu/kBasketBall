#define DLL_PHYSICS_EXIST
#include"Physics.h"
namespace Physics
{
	void RigidBody::SetImportAttributes(real mass,real damping,const Vector3& p,const Vector3 &v,const Vector3 &r,const Matrix3& inertiaTensor,const Vector3 & acc,real friction
		,real restitution,const BoundingSphere& volume,int PrimitiveType,bool canMove,bool canSleep)
	{
		this->SetMass(mass);
		this->SetDamping(damping,damping);
		this->SetPosition(p);
		this->SetVelocity(v);
		this->SetRotation(r);
		this->SetInertiaTensor(inertiaTensor);
		this->SetAcceleration(acc);
		this->friction=friction;
		this->restitution=restitution;
		this->Volume=volume;
		this->PrimitiveType=PrimitiveType;
		this->canMove=canMove;
		this->canSleep=canSleep;
	}
	RigidBody::RigidBody()
	{
		this->inverseMass=0;
		this->motion=3;
		//this->cannotSleeptimes=CANNOTSLEEPTIMES;
		this->friction=0;
		this->restitution=1;
		this->PrimitiveType=0;
		this->canMove=true;
		this->SetAwake(true);

	}
	void RigidBody::SetNotMoveBody(const Vector3 &p,real friction,real restitution,const BoundingSphere& volume,
		int PrimitiveType,bool canSleep)
	{
		this->SetMass(REAL_MAX);
		this->friction=friction;
		this->restitution=restitution;
		this->Volume=volume;
		this->PrimitiveType=PrimitiveType;
		this->canSleep=canSleep;
		this->canMove=false;
		this->SetAwake(true);
		this->SetPosition(p);
	}
	void RigidBody::SetPrimitive(CollisionPrimitive *p)
	{
		this->pPrimitive=p;
	}
	void RigidBody::SetPrimitiveType(int PrimitiveType)
	{
		this->PrimitiveType=PrimitiveType;
	}
	void RigidBody::SetVolume(const BoundingSphere& volume)
	{
		this->Volume=volume;
	}
	void RigidBody::SetCanMove(bool CanMove)
	{
		this->canMove=CanMove;
	}
	//�����ı任
	void RigidBody::TransformInverseInertiaTensorWorld(Matrix3 & iitWorld,const Quaternion &q,const Matrix3 &iitBody,const Matrix4 &rotmat)	
	{
		real t4 = rotmat.data[0]*iitBody.data[0]+
			rotmat.data[1]*iitBody.data[3]+
			rotmat.data[2]*iitBody.data[6];
		real t9 = rotmat.data[0]*iitBody.data[1]+
			rotmat.data[1]*iitBody.data[4]+
			rotmat.data[2]*iitBody.data[7];
		real t14 = rotmat.data[0]*iitBody.data[2]+
			rotmat.data[1]*iitBody.data[5]+
			rotmat.data[2]*iitBody.data[8];
		real t28 = rotmat.data[4]*iitBody.data[0]+
			rotmat.data[5]*iitBody.data[3]+
			rotmat.data[6]*iitBody.data[6];
		real t33 = rotmat.data[4]*iitBody.data[1]+
			rotmat.data[5]*iitBody.data[4]+
			rotmat.data[6]*iitBody.data[7];
		real t38 = rotmat.data[4]*iitBody.data[2]+
			rotmat.data[5]*iitBody.data[5]+
			rotmat.data[6]*iitBody.data[8];
		real t52 = rotmat.data[8]*iitBody.data[0]+
			rotmat.data[9]*iitBody.data[3]+
			rotmat.data[10]*iitBody.data[6];
		real t57 = rotmat.data[8]*iitBody.data[1]+
			rotmat.data[9]*iitBody.data[4]+
			rotmat.data[10]*iitBody.data[7];
		real t62 = rotmat.data[8]*iitBody.data[2]+
			rotmat.data[9]*iitBody.data[5]+
			rotmat.data[10]*iitBody.data[8];

		iitWorld.data[0] = t4*rotmat.data[0]+
			t9*rotmat.data[1]+
			t14*rotmat.data[2];
		iitWorld.data[1] = t4*rotmat.data[4]+
			t9*rotmat.data[5]+
			t14*rotmat.data[6];
		iitWorld.data[2] = t4*rotmat.data[8]+
			t9*rotmat.data[9]+
			t14*rotmat.data[10];
		iitWorld.data[3] = t28*rotmat.data[0]+
			t33*rotmat.data[1]+
			t38*rotmat.data[2];
		iitWorld.data[4] = t28*rotmat.data[4]+
			t33*rotmat.data[5]+
			t38*rotmat.data[6];
		iitWorld.data[5] = t28*rotmat.data[8]+
			t33*rotmat.data[9]+
			t38*rotmat.data[10];
		iitWorld.data[6] = t52*rotmat.data[0]+
			t57*rotmat.data[1]+
			t62*rotmat.data[2];
		iitWorld.data[7] = t52*rotmat.data[4]+
			t57*rotmat.data[5]+
			t62*rotmat.data[6];
		iitWorld.data[8] = t52*rotmat.data[8]+
			t57*rotmat.data[9]+
			t62*rotmat.data[10];
	}
	//����ɱ���֪����������� ����任���� �����������
	void RigidBody::CalculateDerivedData()
	{
		orientation.Normalize();
		this->CalculateTransformMatrix(this->transformMatrix,position,orientation);
		this->TransformInverseInertiaTensorWorld(this->inverseInertiaTensorWorld,orientation,this->inverseInertiaTensor,transformMatrix);
	}

	void RigidBody::AddWorldForceAtWorldCenter(const Vector3 & worldForce)//Ϊ��������һ��������������ϵ�������ڸ������ĵ���
	{
		this->forceAccum+=worldForce;
		isAwake=true;
		this->cannotSleeptimes=CANNOTSLEEPTIMES;
	}
	void RigidBody::AddWorldForceAtWorldPoint(const Vector3 &worldForce, const Vector3 &worldPoint)	//Ϊ��������һ��������������ϵ�������ڸ������
	{
		Vector3 pt=worldPoint-position;
		forceAccum+=worldForce;
		torqueAccum+=pt%worldForce;
		isAwake=true;
		this->cannotSleeptimes=CANNOTSLEEPTIMES;
	}
	void RigidBody::AddWorldForceAtLocalPoint(const Vector3 &worldForce,const Vector3 &localPoint)	//Ϊ��������һ�����ڸ�������ϵ�������ڸ������
	{
		Vector3 worldPoint=this->GetPointInWorldSpace(localPoint);
		AddWorldForceAtWorldPoint(worldForce,worldPoint);
	}
	void RigidBody::AddLocalForceAtLocalPoint(const Vector3 &localForce,const Vector3 &localPoint)
	{
		Vector3 worldPoint=this->GetPointInWorldSpace(localPoint);
		Vector3 worldForce=this->GetDirectionInWorldSpace(localForce);
		AddWorldForceAtWorldPoint(worldForce,worldPoint);
	}
	void RigidBody::AddLocalForceAtWorldPoint(const Vector3 &localForce,const Vector3 &worldPoint)
	{
		Vector3 worldForce=this->GetDirectionInWorldSpace(localForce);
		AddWorldForceAtWorldPoint(worldForce,worldPoint);
	}
	void RigidBody::AddWorldTorque(const Vector3 &worldTorque)	//Ϊ��������һ��������������ϵ������
	{
		this->torqueAccum+=worldTorque;
		isAwake=true;
		this->cannotSleeptimes=CANNOTSLEEPTIMES;
	}

	void RigidBody::ClearAccumulators()					//�����������
	{
		forceAccum.Clear();
		torqueAccum.Clear();
	}
	void RigidBody::Integrate(real duration)		//����֡�ʸ��������״̬
	{
		CalculateDerivedData();
		if((!isAwake)||(!canMove))return;
		lastFrameAcceleration=acceleration;
		lastFrameAcceleration.AddScaledVector(forceAccum,inverseMass);
		Vector3 angularAcceleration=inverseInertiaTensorWorld.Transform(torqueAccum);
		velocity.AddScaledVector(lastFrameAcceleration,duration);
		rotation.AddScaledVector(angularAcceleration,duration);
		velocity*=real_pow(linearDamping,duration);
		rotation*=real_pow(angularDamping,duration);
		position.AddScaledVector(velocity,duration);
		orientation.AddScaledVector(rotation,duration);
		CalculateDerivedData();
		ClearAccumulators();
		this->Volume.centre=this->GetPosition();
		if(canSleep)
		{
			real currentMotion = velocity.ScalarProduct(velocity) +
				rotation.ScalarProduct(rotation);

			real bias = real_pow((real)0.5, duration);
			motion = bias*motion + (1-bias)*currentMotion;

			if ((motion < sleepEpsilon)&&(cannotSleeptimes<0)) SetAwake(false);
			else if (motion > 10 * sleepEpsilon) motion = 10 * sleepEpsilon;
			cannotSleeptimes--;
		}
		//assert(this->velocity.Magnitude()<=200);
	}

	void RigidBody::SetMass(const real mass)		//��������
	{
		assert(mass!=0);
		this->inverseMass=((real)1.0)/mass;
	}
	real RigidBody::GetMass() const			//�õ�����
	{
		if(inverseMass==0)
			return REAL_MAX;
		else
			return ((real)1.0)/inverseMass;
	}
	void RigidBody::SetInverseMass(const real inverseMass)	//���������ĵ���
	{
		this->inverseMass=inverseMass;
	}
	real RigidBody::GetInverseMass() const		//�õ������ĵ���
	{
		return inverseMass;
	}
	bool RigidBody::HasFiniteMass() const		//�Ƿ������������
	{
		return inverseMass>=(real)0.0;
	}

	void RigidBody::SetInertiaTensor(const Matrix3 &inertiaTensor)//������������ϵ�Ĺ�������
	{
		Matrix3 temp;
		temp.SetInverse(inertiaTensor);
		if(CheckInverseInertiaTensor(temp))
			this->inverseInertiaTensor=temp;
	}
	void RigidBody::GetInertiaTensor(Matrix3 *inertiaTensor)const// �õ���������ϵ�Ĺ������� ����ڲ���ָ��
	{
		inertiaTensor->SetInverse(this->inverseInertiaTensor);
	}
	Matrix3 RigidBody::GetInertiaTensor() const//�õ���������ϵ�Ĺ������� ����Ƿ���ֵ
	{
		return this->inverseInertiaTensor.GetInverse();
	}
	void RigidBody::GetInertiaTensorWorld(Matrix3 *inertiaTensorWorld) const//�õ���������ϵ�Ĺ������� ����ڲ���ָ��
	{
		inertiaTensorWorld->SetInverse(this->inverseInertiaTensor);
	}
	Matrix3 RigidBody::GetInertiaTensorWorld() const//�õ���������ϵ�Ĺ������� ����Ƿ���ֵ
	{
		return this->inverseInertiaTensor.GetInverse();
	}
	void RigidBody::SetInverseInertiaTensor(const Matrix3 &inverseInertiaTensor)//������������ϵ�Ĺ�����������
	{
		if(CheckInverseInertiaTensor(inverseInertiaTensor))
			this->inverseInertiaTensor=inverseInertiaTensor;
	}
	void RigidBody::GetInverseInertiaTensor(Matrix3 *inverseInertiaTensor)//�õ���������ϵ�Ĺ����������� ����ڲ���ָ��
	{
		*inverseInertiaTensor=this->inverseInertiaTensor;
	}
	Matrix3 RigidBody::GetInverseInertiaTensor()const//�õ���������ϵ�Ĺ����������棬����Ƿ���ֵ
	{
		return inverseInertiaTensor;
	}
	void RigidBody::GetInverseInertiaTensorWorld(Matrix3 *inverseInertiaTensorWorld)const//�õ���������ϵ�Ĺ����������� ����ڲ���ָ��
	{
		*inverseInertiaTensorWorld=this-> inverseInertiaTensorWorld;
	}
	Matrix3 RigidBody::GetInverseInertiaTensorWorld()const//�õ���������ϵ�Ĺ����������� ����Ƿ���ֵ
	{
		return inverseInertiaTensorWorld;
	}

	void RigidBody::SetDamping(const real linearDamping,const real angularDamping)//���������������ת����
	{
		this->linearDamping=linearDamping;
		this->angularDamping=angularDamping;
	}
	void RigidBody::SetLinearDamping(const real linearDamping)//������������
	{
		this->linearDamping=linearDamping;
	}
	real RigidBody::GetLinearDamping()const//�õ���������
	{
		return linearDamping;
	}
	void RigidBody::SetAngularDamping(const real angularDamping)//������ת����
	{
		this->angularDamping=angularDamping;
	}
	real RigidBody::GetAngularDamping()const//�õ���ת����
	{
		return angularDamping;
	}

	void RigidBody::SetPosition(const Vector3 &p)//����������������������ϵ�е�λ��
	{
		position=p;
	}
	void RigidBody::SetPosition(const real x,const real y, const real z)//����������������������ϵ�е�λ��
	{
		position.data[0]=x;
		position.data[1]=y;
		position.data[2]=z;
	}
	void RigidBody::GetPosition(Vector3 *p)const//�õ�������������������ϵ�е�λ��
	{
		*p=this->position;
	}
	Vector3 RigidBody::GetPosition()const//�õ�������������������ϵ�е�λ��
	{
		return position;
	}

	void RigidBody::SetOrientation(const Quaternion &orientation)//���ø�������������ϵ�еķ���
	{
		this->orientation=orientation;
		this->orientation.Normalize();
	}
	void RigidBody::SetOrientation(const real r,const real i,const real j,const real k)//���ø�������������ϵ�еķ���
	{
		orientation.data[0]=r;
		orientation.data[1]=i;
		orientation.data[2]=j;
		orientation.data[3]=k;
		orientation.Normalize();
	}
	void RigidBody::GetOrientation(Quaternion *orientation)const//�õ���������������ϵ�еķ��� ��������ڲ�������Ԫ��ָ��
	{
		*orientation=this->orientation;
	}
	Quaternion RigidBody::GetOrientation()const//�õ���������������ϵ�еķ��� �������һ����Ԫ��
	{
		return orientation;
	}
	void RigidBody::GetOrientation(Matrix3 *matrix)const//�õ���������������ϵ�еķ��� ���������Ԫ����Matrix3��ʽ ����Ԫ�����Եȼ�ת��Ϊ3*3�ľ�����ʽ��
	{
		GetOrientation(matrix->data);
	}
	void RigidBody::GetOrientation(real matrix[9]) const//�õ���������������ϵ�еķ��� ������س���Ϊ9��������ʽ��ʵ�ʾ���matrix3�ı���
	{
		matrix[0] = transformMatrix.data[0];
		matrix[1] = transformMatrix.data[1];
		matrix[2] = transformMatrix.data[2];

		matrix[3] = transformMatrix.data[4];
		matrix[4] = transformMatrix.data[5];
		matrix[5] = transformMatrix.data[6];

		matrix[6] = transformMatrix.data[8];
		matrix[7] = transformMatrix.data[9];
		matrix[8] = transformMatrix.data[10];
	}

	void RigidBody::GetTransform(Matrix4 *transform)const//�õ�Matrix4��ʽ�ı任���󣬽�������ڲ���ָ��
	{
		for(int i=0;i<12;i++)
			transform->data[i]=this->transformMatrix.data[i];
	}
	void RigidBody::GetTransform(real matrix[16])const//�õ�һ������Ϊ16��������ʽ�ı任���󣬽�������ڲ���
	{
		for(int i=0;i<12;i++)
			matrix[i]=this->transformMatrix.data[i];
		matrix[12]=matrix[13]=matrix[14]=0;
		matrix[15]=1;
	}
	void RigidBody::GetGLTransform(real matrix[16]) const//�õ�һ��OpenGL�и�ʽ�ı任����ת������float���ͣ�
	{
		matrix[0] = (real)transformMatrix.data[0];
		matrix[1] = (real)transformMatrix.data[4];
		matrix[2] = (real)transformMatrix.data[8];
		matrix[3] = 0;

		matrix[4] = (real)transformMatrix.data[1];
		matrix[5] = (real)transformMatrix.data[5];
		matrix[6] = (real)transformMatrix.data[9];
		matrix[7] = 0;

		matrix[8] = (real)transformMatrix.data[2];
		matrix[9] = (real)transformMatrix.data[6];
		matrix[10] = (real)transformMatrix.data[10];
		matrix[11] = 0;

		matrix[12] = (real)transformMatrix.data[3];
		matrix[13] = (real)transformMatrix.data[7];
		matrix[14] = (real)transformMatrix.data[11];
		matrix[15] = 1;
	}
	real * RigidBody::GetGLTransformReturn(real matrix[16])const
	{
		matrix[0] = (real)transformMatrix.data[0];
		matrix[1] = (real)transformMatrix.data[4];
		matrix[2] = (real)transformMatrix.data[8];
		matrix[3] = 0;

		matrix[4] = (real)transformMatrix.data[1];
		matrix[5] = (real)transformMatrix.data[5];
		matrix[6] = (real)transformMatrix.data[9];
		matrix[7] = 0;

		matrix[8] = (real)transformMatrix.data[2];
		matrix[9] = (real)transformMatrix.data[6];
		matrix[10] = (real)transformMatrix.data[10];
		matrix[11] = 0;

		matrix[12] = (real)transformMatrix.data[3];
		matrix[13] = (real)transformMatrix.data[7];
		matrix[14] = (real)transformMatrix.data[11];
		matrix[15] = 1;
		return matrix;
	}
	Matrix4 RigidBody::GetTransform()const//����һ��Matrix4��ʽ�ı任����
	{
		return this->transformMatrix;
	}

	Vector3 RigidBody::GetPointInLocalSpace(const Vector3 &worldp)const//�Ѳ����е���������ת���ɸ�������ϵ������
	{
		return this->transformMatrix.TransformInverse(worldp);
		//return this->orientation.TransformInversePointReturn(worldp,position);
	}
	Vector3 RigidBody::GetPointInWorldSpace(const Vector3 &localp)const//�ɲ����еĸ�������ϵ������ת������������
	{
		return this->transformMatrix.Transform(localp);
		//return this->orientation.TransformPointReturn(localp,position);
	}
	Vector3 RigidBody::GetDirectionInLocalSpace(const Vector3 &worldD)const//�Ѳ����е���������ϵ�еķ�������ת���ɸ�������ϵ�е�
	{
		return this->transformMatrix.TransformInverseDirection(worldD);
		//return this->orientation.TransformInverseVectorReturn(worldD);
	}
	Vector3 RigidBody::GetDirectionInWorldSpace(const Vector3 &localD)const//�Ѳ����еĸ�������ϵ�еķ�������ת������������ϵ�е�
	{
		return this->transformMatrix.TransformDirection(localD);
		//return this->orientation.TransformVectorReturn(localD);
	}

	void RigidBody::SetVelocity(const Vector3 &v)//���ø�����������������ϵ�е����ٶ�
	{
		this->velocity=v;
	}
	void RigidBody::SetVelocity(const real x,const real y,const real z)//���ø������ĵ����ٶ�
	{
		velocity.data[0]=x;
		velocity.data[1]=y;
		velocity.data[2]=z;
	}
	void RigidBody::GetVelocity(Vector3 *v)const//�õ���������ٶ� ��������ڲ���ָ��
	{
		*v=velocity;
	}
	Vector3 RigidBody::GetVelocity()const//���ظ�������ٶ�
	{
		return velocity;
	}
	void RigidBody::AddVelocity(const Vector3 &deltaVelocity)//Ϊ�����������ٶ�deltav
	{
		velocity+=deltaVelocity;
	}

	void RigidBody::SetRotation(const Vector3 &rotation)//���ø���Ľ��ٶ�
	{
		this->rotation=rotation;
	}
	void RigidBody::SetRotation(const real x, const real y, const real z)//���ø���Ľ��ٶ�
	{
		rotation.data[0]=x;
		rotation.data[1]=y;
		rotation.data[2]=z;
	}
	void RigidBody::GetRotation(Vector3 *rotation) const//�õ�����Ľ��ٶȣ����������ָ��
	{
		*rotation=this->rotation;
	}
	Vector3 RigidBody::GetRotation() const//���ظ���Ľ��ٶ�
	{
		return rotation;
	}
	void RigidBody::AddRotation(const Vector3 &deltaRotation)//Ϊ�������ӽ��ٶ�deltar
	{
		rotation+=deltaRotation;
	}

	bool RigidBody::GetAwake() const//���ظ����awake״̬
	{
		return this->isAwake;
	}
	void RigidBody::SetAwake(const bool awake)//���ø����awake״̬���������˯��������û���ٶȵ�
	{
		if(awake)
		{
			isAwake=true;
			this->cannotSleeptimes=CANNOTSLEEPTIMES;
			motion=sleepEpsilon*((real)10);	//���������һ���ƶ��ۼӣ�������µ�ʱ��˯����
		}
		else
		{
			isAwake=false;
			velocity.Clear();
			rotation.Clear();
		}
	}
	bool RigidBody::GetCanSleep() const//���ظ����Ƿ���sleep
	{
		return this->canSleep;
	}
	void RigidBody::SetCanSleep(const bool canSleep)//���ø����Ƿ���sleep
	{
		this->canSleep=canSleep;
		if(!canSleep &&!isAwake)SetAwake();
	}

	void RigidBody::GetLastFrameAcceleration(Vector3 *linearAcceleration) const//�õ�������һ֡�ļ��ٶ�
	{
		*linearAcceleration=this->lastFrameAcceleration;
	}
	Vector3 RigidBody::GetLastFrameAcceleration() const//�õ�������һ֡�ļ��ٶ�
	{
		return this->lastFrameAcceleration;
	}

	void RigidBody::SetAcceleration(const Vector3 &acceleration)//���ü��ٶ�
	{
		this->acceleration=acceleration;
	}
	void RigidBody::SetAcceleration(const real x, const real y, const real z)//���ü��ٶ�
	{
		this->acceleration.data[0]=x;
		this->acceleration.data[1]=y;
		this->acceleration.data[2]=z;
	}
	void RigidBody::GetAcceleration(Vector3 *acceleration) const//�õ����ٶ�
	{
		*acceleration=this->acceleration;
	}
	Vector3 RigidBody::GetAcceleration() const//�õ����ٶ�
	{
		return this->acceleration;
	}

	Vector3 RigidBody::GetVelocityAtLocalPoint(const Vector3 &localp)const
	{
		Vector3 r=this->GetPointInWorldSpace(localp)-position;
		return this->rotation%r+this->velocity;
	}
	Vector3 RigidBody::GetVelocityAtWorldPoint(const Vector3 &worldp)const
	{
		return this->rotation%(worldp-position)+this->velocity;
	}
	void RigidBody::GetVelocityAtLocalPoint(const Vector3 &localp,Vector3 &velocityp)const
	{
		Vector3 r=this->GetPointInWorldSpace(localp)-position;
		velocityp= this->rotation%r+this->velocity;
	}
	void RigidBody::GetVelocityAtWorldPoint(const Vector3 &worldp,Vector3 &velocityp)const
	{
		velocityp=this->rotation%(worldp-position)+this->velocity;
	}
}
