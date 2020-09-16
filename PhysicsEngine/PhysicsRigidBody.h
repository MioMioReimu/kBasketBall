#pragma once
//#include"PhysicsCommon.h"
//#include"PhysicsMath.h"
namespace Physics
{
	class DLL_PHYSICS CollisionPrimitive;
	class DLL_PHYSICS BoundingSphere;
	//enum PrimitiveEnum {Plane,Sphere,HalfPlane,Box,Circle};
	class DLL_PHYSICS RigidBody
	{
		//该刚体随体坐标系是相对于刚体固定不变的，也就是说刚体中某个质点无论刚体怎么旋转其在随体坐标系中的坐标不变
	public:
		real inverseMass;		//该物体质量的倒数
		real linearDamping;	//该物体运动的线性阻尼
		real angularDamping;	//该物体运动的旋转阻尼
		Vector3 position;		//该物体中心在世界坐标系中的位置
		Quaternion orientation;		//该物体随体坐标系在世界坐标系中方位
		Vector3 velocity;			//该物体中心在世界坐标系中的线速度
		Vector3 rotation;			//该物体中心在世界坐标系的角速度
		Matrix4 transformMatrix;	//随体坐标系到世界坐标系的变换矩阵
		Matrix3 inverseInertiaTensor;	//该物体在随体坐标系中的惯性张量矩阵的逆矩阵
		Matrix3 inverseInertiaTensorWorld;	//该物体在世界坐标系中的惯性张量的逆
		Vector3 forceAccum;		//该物体受到的合力，力是基于世界坐标系
		Vector3 torqueAccum;		//该物体受到的合力矩，力矩也是基于世界坐标系描述的
		Vector3 acceleration;		//该物体的恒定加速度，比如重力 非恒定的力 用ForceGenerator
		Vector3 lastFrameAcceleration;			//该物体上一帧的加速度
		real friction;			//该物体的摩擦系数
		real restitution;		//该物体的恢复系数
		CollisionPrimitive *pPrimitive;//该物体的包围体类指针
		BoundingSphere Volume;		//该物体的粗糙包围球
		int PrimitiveType;				//该物体的包围体类型 仅SPHERE CIRCLE PLANE BOX
		bool canMove;					//该物体是否能移动
		real motion;					//存储着物体运动量，当运动量很小时可以把物体置于睡眠
		bool isAwake;				//该物体可以进入睡眠状态而不受到更新函数或者碰撞的影响
		bool canSleep;				//有些物体永远不能进入睡眠状态，比如受控制的物体
		long cannotSleeptimes;		//剩下的不可睡眠的次数
	public:
		//设置一些刚体重要的属性
		RigidBody();
		void SetImportAttributes(real mass,real damping,const Vector3& p,const Vector3 &v,const Vector3 &r,const Matrix3& inertiaTensor,const Vector3 & acc,real friction
			,real restitution,const BoundingSphere& volume,int PrimitiveType,bool canMove,bool canSleep);
		void SetNotMoveBody(const Vector3 &p,real friction,real restitution,const BoundingSphere& volume,int PrimitiveType,bool canSleep);
	public:
		static void CalculateTransformMatrix(Matrix4 &transformMatrix,const Vector3 &position,const Quaternion &orientation)	//计算变换矩阵
		{
			transformMatrix.SetOrientationAndPos(orientation,position);
		};
		static inline bool CheckInverseInertiaTensor(const Matrix3 &iitWorld)		//检查惯性张量的逆
		{return 1; };
		static void TransformInverseInertiaTensorWorld(Matrix3 & iitWorld,const Quaternion &q,const Matrix3 &iitBody,const Matrix4 &rotmat);	//张量的变换
		void CalculateDerivedData();				//计算可被已知量计算出的量 例如变换矩阵 世界惯性张量
		
		void AddWorldForceAtWorldCenter(const Vector3 & worldForce);//为刚体增加一个基于世界坐标系的作用于刚体中心的力
		void AddWorldForceAtWorldPoint(const Vector3 &worldForce, const Vector3 &worldPoint);	//为刚体增加一个基于世界坐标系的作用于刚体的力
		void AddWorldForceAtLocalPoint(const Vector3 &worldForce,const Vector3 &localPoint);	//为刚体增加一个基于刚体坐标系的作用于刚体的力
		void AddLocalForceAtLocalPoint(const Vector3 &localForce,const Vector3 &localPoint);
		void AddLocalForceAtWorldPoint(const Vector3 &localForce,const Vector3 &worldPoint);
		void AddWorldTorque(const Vector3 &worldTorque);	//为刚体增加一个基于世界坐标系的力矩
		
		void ClearAccumulators();					//清除力和力矩
		void Integrate(real duration);		//基于帧率更新物体的状态

		void SetMass(const real mass);		//设置质量
		real GetMass() const;			//得到质量
		void SetInverseMass(const real inverseMass);	//设置质量的倒数
		real GetInverseMass() const;		//得到质量的倒数
		bool HasFiniteMass() const;		//是否具有有限质量

		void SetInertiaTensor(const Matrix3 &inertiaTensor);		//设置自身坐标系的惯性张量
		void GetInertiaTensor(Matrix3 *inertiaTensor)const;// 得到自身坐标系的惯性张量 结果在参数指针
		Matrix3 GetInertiaTensor() const;//得到自身坐标系的惯性张量 结果是返回值
		void GetInertiaTensorWorld(Matrix3 *inertiaTensorWorld) const;//得到世界坐标系的惯性张量 结果在参数指针
		Matrix3 GetInertiaTensorWorld() const;//得到世界坐标系的惯性张量 结果是返回值
		void SetInverseInertiaTensor(const Matrix3 &inverseInertiaTensor);//设置自身坐标系的惯性张量的逆
		void GetInverseInertiaTensor(Matrix3 *inverseInertiaTensor);//得到自身坐标系的惯性张量的逆 结果在参数指针
		Matrix3 GetInverseInertiaTensor()const;//得到自身坐标系的惯性张量的逆，结果是返回值
		void GetInverseInertiaTensorWorld(Matrix3 *inverseInertiaTensorWorld)const;//得到世界坐标系的惯性张量的逆 结果在参数指针
		Matrix3 GetInverseInertiaTensorWorld()const;//得到世界坐标系的惯性张量的逆 结果是返回值

		void SetDamping(const real linearDamping,const real angularDamping);//设置线性阻尼和旋转阻尼
		void SetLinearDamping(const real linearDamping);//设置线性阻尼
		real GetLinearDamping()const;//得到线性阻尼
		void SetAngularDamping(const real angularDamping);//设置旋转阻尼
		real GetAngularDamping()const;//得到旋转阻尼

		void SetPosition(const Vector3 &p);//设置物体中心在世界坐标系中的位置
		void SetPosition(const real x,const real y, const real z);//设置物体中心在世界坐标系中的位置
		void GetPosition(Vector3 *p)const;//得到物体中心在世界坐标系中的位置
		Vector3 GetPosition()const;//得到物体中心在世界坐标系中的位置

		void SetOrientation(const Quaternion &orientation);//设置刚体在世界坐标系中的方向
		void SetOrientation(const real r,const real i,const real j,const real k);//设置刚体在世界坐标系中的方向
		void GetOrientation(Quaternion *orientation)const;//得到刚体在世界坐标系中的方向 结果返回在参数的四元数指针
		Quaternion GetOrientation()const;//得到刚体在世界坐标系中的方向 结果返回一个四元数
		void GetOrientation(Matrix3 *matrix)const;//得到刚体在世界坐标系中的方向 结果返回四元数的Matrix3形式 （四元数可以等价转换为3*3的矩阵形式）
		void GetOrientation(real matrix[9]) const;//得到刚体在世界坐标系中的方向 结果返回长度为9的数组形式，实际就是matrix3的变形

		void GetTransform(Matrix4 *transform)const;//得到Matrix4形式的变换矩阵，结果返回在参数指针
		void GetTransform(real matrix[16])const;//得到一个长度为16的数组形式的变换矩阵，结果返回在参数
		void GetGLTransform(real matrix[16]) const;//得到一个OpenGL中格式的变换矩阵（转置且是float类型）
		Matrix4 GetTransform()const;//返回一个Matrix4形式的变换矩阵
		real * GetGLTransformReturn(real matrix[16])const;

		Vector3 GetPointInLocalSpace(const Vector3 &worldp)const;//把参数中的世界坐标转换成刚体坐标系的坐标
		Vector3 GetPointInWorldSpace(const Vector3 &localp)const;//吧参数中的刚体坐标系的坐标转换成世界坐标
		Vector3 GetDirectionInLocalSpace(const Vector3 &worldD)const;//把参数中的世界坐标系中的方向向量转换成刚体坐标系中的
		Vector3 GetDirectionInWorldSpace(const Vector3 &localD)const;//把参数中的刚体坐标系中的方向向量转换成世界坐标系中的

		void SetVelocity(const Vector3 &v);//设置刚体中心在世界坐标系中的线速度
		void SetVelocity(const real x,const real y,const real z);//设置刚体中心的线速度
		void GetVelocity(Vector3 *v)const;//得到刚体的线速度 结果返回在参数指针
		Vector3 GetVelocity()const;//返回刚体的线速度
		void AddVelocity(const Vector3 &deltaVelocity);//为刚体增加线速度deltav

		void SetRotation(const Vector3 &rotation);//设置刚体的角速度
		void SetRotation(const real x, const real y, const real z);//设置刚体的角速度
		void GetRotation(Vector3 *rotation) const;//得到刚体的角速度，结果返回在指针
		Vector3 GetRotation() const;//返回刚体的角速度
		void AddRotation(const Vector3 &deltaRotation);//为刚体增加角速度deltar

		bool GetAwake() const;//返回刚体的awake状态
		void SetAwake(const bool awake=true);//设置刚体的awake状态，如果刚体睡眠则它是没有速度的
		bool GetCanSleep() const;//返回刚体是否能sleep
		void SetCanSleep(const bool canSleep=true);//设置刚体是否能sleep

		void GetLastFrameAcceleration(Vector3 *linearAcceleration) const;//得到刚体上一帧的加速度
		Vector3 GetLastFrameAcceleration() const;//得到刚体上一帧的加速度
		
		void SetAcceleration(const Vector3 &acceleration);//设置加速度
		void SetAcceleration(const real x, const real y, const real z);//设置加速度
		void GetAcceleration(Vector3 *acceleration) const;//得到加速度
		Vector3 GetAcceleration() const;//得到加速度

		//得到刚体上一点的速度 
		Vector3 GetVelocityAtLocalPoint(const Vector3 &localp)const;
		Vector3 GetVelocityAtWorldPoint(const Vector3 &worldp)const;
		void GetVelocityAtLocalPoint(const Vector3 &localp,Vector3 &velocity)const;
		void GetVelocityAtWorldPoint(const Vector3 &worldp,Vector3 &velocity)const;

		void SetPrimitiveType(int PrimitiveType);
		void SetVolume(const BoundingSphere& volume);
		void SetCanMove(bool CanMove=true);
		void SetPrimitive(CollisionPrimitive *p);
	};
};