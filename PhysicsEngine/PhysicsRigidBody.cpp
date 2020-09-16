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
	//张量的变换
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
	//计算可被已知量计算出的量 例如变换矩阵 世界惯性张量
	void RigidBody::CalculateDerivedData()
	{
		orientation.Normalize();
		this->CalculateTransformMatrix(this->transformMatrix,position,orientation);
		this->TransformInverseInertiaTensorWorld(this->inverseInertiaTensorWorld,orientation,this->inverseInertiaTensor,transformMatrix);
	}

	void RigidBody::AddWorldForceAtWorldCenter(const Vector3 & worldForce)//为刚体增加一个基于世界坐标系的作用于刚体中心的力
	{
		this->forceAccum+=worldForce;
		isAwake=true;
		this->cannotSleeptimes=CANNOTSLEEPTIMES;
	}
	void RigidBody::AddWorldForceAtWorldPoint(const Vector3 &worldForce, const Vector3 &worldPoint)	//为刚体增加一个基于世界坐标系的作用于刚体的力
	{
		Vector3 pt=worldPoint-position;
		forceAccum+=worldForce;
		torqueAccum+=pt%worldForce;
		isAwake=true;
		this->cannotSleeptimes=CANNOTSLEEPTIMES;
	}
	void RigidBody::AddWorldForceAtLocalPoint(const Vector3 &worldForce,const Vector3 &localPoint)	//为刚体增加一个基于刚体坐标系的作用于刚体的力
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
	void RigidBody::AddWorldTorque(const Vector3 &worldTorque)	//为刚体增加一个基于世界坐标系的力矩
	{
		this->torqueAccum+=worldTorque;
		isAwake=true;
		this->cannotSleeptimes=CANNOTSLEEPTIMES;
	}

	void RigidBody::ClearAccumulators()					//清除力和力矩
	{
		forceAccum.Clear();
		torqueAccum.Clear();
	}
	void RigidBody::Integrate(real duration)		//基于帧率更新物体的状态
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

	void RigidBody::SetMass(const real mass)		//设置质量
	{
		assert(mass!=0);
		this->inverseMass=((real)1.0)/mass;
	}
	real RigidBody::GetMass() const			//得到质量
	{
		if(inverseMass==0)
			return REAL_MAX;
		else
			return ((real)1.0)/inverseMass;
	}
	void RigidBody::SetInverseMass(const real inverseMass)	//设置质量的倒数
	{
		this->inverseMass=inverseMass;
	}
	real RigidBody::GetInverseMass() const		//得到质量的倒数
	{
		return inverseMass;
	}
	bool RigidBody::HasFiniteMass() const		//是否具有有限质量
	{
		return inverseMass>=(real)0.0;
	}

	void RigidBody::SetInertiaTensor(const Matrix3 &inertiaTensor)//设置自身坐标系的惯性张量
	{
		Matrix3 temp;
		temp.SetInverse(inertiaTensor);
		if(CheckInverseInertiaTensor(temp))
			this->inverseInertiaTensor=temp;
	}
	void RigidBody::GetInertiaTensor(Matrix3 *inertiaTensor)const// 得到自身坐标系的惯性张量 结果在参数指针
	{
		inertiaTensor->SetInverse(this->inverseInertiaTensor);
	}
	Matrix3 RigidBody::GetInertiaTensor() const//得到自身坐标系的惯性张量 结果是返回值
	{
		return this->inverseInertiaTensor.GetInverse();
	}
	void RigidBody::GetInertiaTensorWorld(Matrix3 *inertiaTensorWorld) const//得到世界坐标系的惯性张量 结果在参数指针
	{
		inertiaTensorWorld->SetInverse(this->inverseInertiaTensor);
	}
	Matrix3 RigidBody::GetInertiaTensorWorld() const//得到世界坐标系的惯性张量 结果是返回值
	{
		return this->inverseInertiaTensor.GetInverse();
	}
	void RigidBody::SetInverseInertiaTensor(const Matrix3 &inverseInertiaTensor)//设置自身坐标系的惯性张量的逆
	{
		if(CheckInverseInertiaTensor(inverseInertiaTensor))
			this->inverseInertiaTensor=inverseInertiaTensor;
	}
	void RigidBody::GetInverseInertiaTensor(Matrix3 *inverseInertiaTensor)//得到自身坐标系的惯性张量的逆 结果在参数指针
	{
		*inverseInertiaTensor=this->inverseInertiaTensor;
	}
	Matrix3 RigidBody::GetInverseInertiaTensor()const//得到自身坐标系的惯性张量的逆，结果是返回值
	{
		return inverseInertiaTensor;
	}
	void RigidBody::GetInverseInertiaTensorWorld(Matrix3 *inverseInertiaTensorWorld)const//得到世界坐标系的惯性张量的逆 结果在参数指针
	{
		*inverseInertiaTensorWorld=this-> inverseInertiaTensorWorld;
	}
	Matrix3 RigidBody::GetInverseInertiaTensorWorld()const//得到世界坐标系的惯性张量的逆 结果是返回值
	{
		return inverseInertiaTensorWorld;
	}

	void RigidBody::SetDamping(const real linearDamping,const real angularDamping)//设置线性阻尼和旋转阻尼
	{
		this->linearDamping=linearDamping;
		this->angularDamping=angularDamping;
	}
	void RigidBody::SetLinearDamping(const real linearDamping)//设置线性阻尼
	{
		this->linearDamping=linearDamping;
	}
	real RigidBody::GetLinearDamping()const//得到线性阻尼
	{
		return linearDamping;
	}
	void RigidBody::SetAngularDamping(const real angularDamping)//设置旋转阻尼
	{
		this->angularDamping=angularDamping;
	}
	real RigidBody::GetAngularDamping()const//得到旋转阻尼
	{
		return angularDamping;
	}

	void RigidBody::SetPosition(const Vector3 &p)//设置物体中心在世界坐标系中的位置
	{
		position=p;
	}
	void RigidBody::SetPosition(const real x,const real y, const real z)//设置物体中心在世界坐标系中的位置
	{
		position.data[0]=x;
		position.data[1]=y;
		position.data[2]=z;
	}
	void RigidBody::GetPosition(Vector3 *p)const//得到物体中心在世界坐标系中的位置
	{
		*p=this->position;
	}
	Vector3 RigidBody::GetPosition()const//得到物体中心在世界坐标系中的位置
	{
		return position;
	}

	void RigidBody::SetOrientation(const Quaternion &orientation)//设置刚体在世界坐标系中的方向
	{
		this->orientation=orientation;
		this->orientation.Normalize();
	}
	void RigidBody::SetOrientation(const real r,const real i,const real j,const real k)//设置刚体在世界坐标系中的方向
	{
		orientation.data[0]=r;
		orientation.data[1]=i;
		orientation.data[2]=j;
		orientation.data[3]=k;
		orientation.Normalize();
	}
	void RigidBody::GetOrientation(Quaternion *orientation)const//得到刚体在世界坐标系中的方向 结果返回在参数的四元数指针
	{
		*orientation=this->orientation;
	}
	Quaternion RigidBody::GetOrientation()const//得到刚体在世界坐标系中的方向 结果返回一个四元数
	{
		return orientation;
	}
	void RigidBody::GetOrientation(Matrix3 *matrix)const//得到刚体在世界坐标系中的方向 结果返回四元数的Matrix3形式 （四元数可以等价转换为3*3的矩阵形式）
	{
		GetOrientation(matrix->data);
	}
	void RigidBody::GetOrientation(real matrix[9]) const//得到刚体在世界坐标系中的方向 结果返回长度为9的数组形式，实际就是matrix3的变形
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

	void RigidBody::GetTransform(Matrix4 *transform)const//得到Matrix4形式的变换矩阵，结果返回在参数指针
	{
		for(int i=0;i<12;i++)
			transform->data[i]=this->transformMatrix.data[i];
	}
	void RigidBody::GetTransform(real matrix[16])const//得到一个长度为16的数组形式的变换矩阵，结果返回在参数
	{
		for(int i=0;i<12;i++)
			matrix[i]=this->transformMatrix.data[i];
		matrix[12]=matrix[13]=matrix[14]=0;
		matrix[15]=1;
	}
	void RigidBody::GetGLTransform(real matrix[16]) const//得到一个OpenGL中格式的变换矩阵（转置且是float类型）
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
	Matrix4 RigidBody::GetTransform()const//返回一个Matrix4形式的变换矩阵
	{
		return this->transformMatrix;
	}

	Vector3 RigidBody::GetPointInLocalSpace(const Vector3 &worldp)const//把参数中的世界坐标转换成刚体坐标系的坐标
	{
		return this->transformMatrix.TransformInverse(worldp);
		//return this->orientation.TransformInversePointReturn(worldp,position);
	}
	Vector3 RigidBody::GetPointInWorldSpace(const Vector3 &localp)const//吧参数中的刚体坐标系的坐标转换成世界坐标
	{
		return this->transformMatrix.Transform(localp);
		//return this->orientation.TransformPointReturn(localp,position);
	}
	Vector3 RigidBody::GetDirectionInLocalSpace(const Vector3 &worldD)const//把参数中的世界坐标系中的方向向量转换成刚体坐标系中的
	{
		return this->transformMatrix.TransformInverseDirection(worldD);
		//return this->orientation.TransformInverseVectorReturn(worldD);
	}
	Vector3 RigidBody::GetDirectionInWorldSpace(const Vector3 &localD)const//把参数中的刚体坐标系中的方向向量转换成世界坐标系中的
	{
		return this->transformMatrix.TransformDirection(localD);
		//return this->orientation.TransformVectorReturn(localD);
	}

	void RigidBody::SetVelocity(const Vector3 &v)//设置刚体中心在世界坐标系中的线速度
	{
		this->velocity=v;
	}
	void RigidBody::SetVelocity(const real x,const real y,const real z)//设置刚体中心的线速度
	{
		velocity.data[0]=x;
		velocity.data[1]=y;
		velocity.data[2]=z;
	}
	void RigidBody::GetVelocity(Vector3 *v)const//得到刚体的线速度 结果返回在参数指针
	{
		*v=velocity;
	}
	Vector3 RigidBody::GetVelocity()const//返回刚体的线速度
	{
		return velocity;
	}
	void RigidBody::AddVelocity(const Vector3 &deltaVelocity)//为刚体增加线速度deltav
	{
		velocity+=deltaVelocity;
	}

	void RigidBody::SetRotation(const Vector3 &rotation)//设置刚体的角速度
	{
		this->rotation=rotation;
	}
	void RigidBody::SetRotation(const real x, const real y, const real z)//设置刚体的角速度
	{
		rotation.data[0]=x;
		rotation.data[1]=y;
		rotation.data[2]=z;
	}
	void RigidBody::GetRotation(Vector3 *rotation) const//得到刚体的角速度，结果返回在指针
	{
		*rotation=this->rotation;
	}
	Vector3 RigidBody::GetRotation() const//返回刚体的角速度
	{
		return rotation;
	}
	void RigidBody::AddRotation(const Vector3 &deltaRotation)//为刚体增加角速度deltar
	{
		rotation+=deltaRotation;
	}

	bool RigidBody::GetAwake() const//返回刚体的awake状态
	{
		return this->isAwake;
	}
	void RigidBody::SetAwake(const bool awake)//设置刚体的awake状态，如果刚体睡眠则它是没有速度的
	{
		if(awake)
		{
			isAwake=true;
			this->cannotSleeptimes=CANNOTSLEEPTIMES;
			motion=sleepEpsilon*((real)10);	//给物体加上一点移动累加，以免更新的时候睡眠了
		}
		else
		{
			isAwake=false;
			velocity.Clear();
			rotation.Clear();
		}
	}
	bool RigidBody::GetCanSleep() const//返回刚体是否能sleep
	{
		return this->canSleep;
	}
	void RigidBody::SetCanSleep(const bool canSleep)//设置刚体是否能sleep
	{
		this->canSleep=canSleep;
		if(!canSleep &&!isAwake)SetAwake();
	}

	void RigidBody::GetLastFrameAcceleration(Vector3 *linearAcceleration) const//得到刚体上一帧的加速度
	{
		*linearAcceleration=this->lastFrameAcceleration;
	}
	Vector3 RigidBody::GetLastFrameAcceleration() const//得到刚体上一帧的加速度
	{
		return this->lastFrameAcceleration;
	}

	void RigidBody::SetAcceleration(const Vector3 &acceleration)//设置加速度
	{
		this->acceleration=acceleration;
	}
	void RigidBody::SetAcceleration(const real x, const real y, const real z)//设置加速度
	{
		this->acceleration.data[0]=x;
		this->acceleration.data[1]=y;
		this->acceleration.data[2]=z;
	}
	void RigidBody::GetAcceleration(Vector3 *acceleration) const//得到加速度
	{
		*acceleration=this->acceleration;
	}
	Vector3 RigidBody::GetAcceleration() const//得到加速度
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
