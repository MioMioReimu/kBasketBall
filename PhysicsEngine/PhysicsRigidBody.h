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
		//�ø�����������ϵ������ڸ���̶�����ģ�Ҳ����˵������ĳ���ʵ����۸�����ô��ת������������ϵ�е����겻��
	public:
		real inverseMass;		//�����������ĵ���
		real linearDamping;	//�������˶�����������
		real angularDamping;	//�������˶�����ת����
		Vector3 position;		//��������������������ϵ�е�λ��
		Quaternion orientation;		//��������������ϵ����������ϵ�з�λ
		Vector3 velocity;			//��������������������ϵ�е����ٶ�
		Vector3 rotation;			//��������������������ϵ�Ľ��ٶ�
		Matrix4 transformMatrix;	//��������ϵ����������ϵ�ı任����
		Matrix3 inverseInertiaTensor;	//����������������ϵ�еĹ�����������������
		Matrix3 inverseInertiaTensorWorld;	//����������������ϵ�еĹ�����������
		Vector3 forceAccum;		//�������ܵ��ĺ��������ǻ�����������ϵ
		Vector3 torqueAccum;		//�������ܵ��ĺ����أ�����Ҳ�ǻ�����������ϵ������
		Vector3 acceleration;		//������ĺ㶨���ٶȣ��������� �Ǻ㶨���� ��ForceGenerator
		Vector3 lastFrameAcceleration;			//��������һ֡�ļ��ٶ�
		real friction;			//�������Ħ��ϵ��
		real restitution;		//������Ļָ�ϵ��
		CollisionPrimitive *pPrimitive;//������İ�Χ����ָ��
		BoundingSphere Volume;		//������Ĵֲڰ�Χ��
		int PrimitiveType;				//������İ�Χ������ ��SPHERE CIRCLE PLANE BOX
		bool canMove;					//�������Ƿ����ƶ�
		real motion;					//�洢�������˶��������˶�����Сʱ���԰���������˯��
		bool isAwake;				//��������Խ���˯��״̬�����ܵ����º���������ײ��Ӱ��
		bool canSleep;				//��Щ������Զ���ܽ���˯��״̬�������ܿ��Ƶ�����
		long cannotSleeptimes;		//ʣ�µĲ���˯�ߵĴ���
	public:
		//����һЩ������Ҫ������
		RigidBody();
		void SetImportAttributes(real mass,real damping,const Vector3& p,const Vector3 &v,const Vector3 &r,const Matrix3& inertiaTensor,const Vector3 & acc,real friction
			,real restitution,const BoundingSphere& volume,int PrimitiveType,bool canMove,bool canSleep);
		void SetNotMoveBody(const Vector3 &p,real friction,real restitution,const BoundingSphere& volume,int PrimitiveType,bool canSleep);
	public:
		static void CalculateTransformMatrix(Matrix4 &transformMatrix,const Vector3 &position,const Quaternion &orientation)	//����任����
		{
			transformMatrix.SetOrientationAndPos(orientation,position);
		};
		static inline bool CheckInverseInertiaTensor(const Matrix3 &iitWorld)		//��������������
		{return 1; };
		static void TransformInverseInertiaTensorWorld(Matrix3 & iitWorld,const Quaternion &q,const Matrix3 &iitBody,const Matrix4 &rotmat);	//�����ı任
		void CalculateDerivedData();				//����ɱ���֪����������� ����任���� �����������
		
		void AddWorldForceAtWorldCenter(const Vector3 & worldForce);//Ϊ��������һ��������������ϵ�������ڸ������ĵ���
		void AddWorldForceAtWorldPoint(const Vector3 &worldForce, const Vector3 &worldPoint);	//Ϊ��������һ��������������ϵ�������ڸ������
		void AddWorldForceAtLocalPoint(const Vector3 &worldForce,const Vector3 &localPoint);	//Ϊ��������һ�����ڸ�������ϵ�������ڸ������
		void AddLocalForceAtLocalPoint(const Vector3 &localForce,const Vector3 &localPoint);
		void AddLocalForceAtWorldPoint(const Vector3 &localForce,const Vector3 &worldPoint);
		void AddWorldTorque(const Vector3 &worldTorque);	//Ϊ��������һ��������������ϵ������
		
		void ClearAccumulators();					//�����������
		void Integrate(real duration);		//����֡�ʸ��������״̬

		void SetMass(const real mass);		//��������
		real GetMass() const;			//�õ�����
		void SetInverseMass(const real inverseMass);	//���������ĵ���
		real GetInverseMass() const;		//�õ������ĵ���
		bool HasFiniteMass() const;		//�Ƿ������������

		void SetInertiaTensor(const Matrix3 &inertiaTensor);		//������������ϵ�Ĺ�������
		void GetInertiaTensor(Matrix3 *inertiaTensor)const;// �õ���������ϵ�Ĺ������� ����ڲ���ָ��
		Matrix3 GetInertiaTensor() const;//�õ���������ϵ�Ĺ������� ����Ƿ���ֵ
		void GetInertiaTensorWorld(Matrix3 *inertiaTensorWorld) const;//�õ���������ϵ�Ĺ������� ����ڲ���ָ��
		Matrix3 GetInertiaTensorWorld() const;//�õ���������ϵ�Ĺ������� ����Ƿ���ֵ
		void SetInverseInertiaTensor(const Matrix3 &inverseInertiaTensor);//������������ϵ�Ĺ�����������
		void GetInverseInertiaTensor(Matrix3 *inverseInertiaTensor);//�õ���������ϵ�Ĺ����������� ����ڲ���ָ��
		Matrix3 GetInverseInertiaTensor()const;//�õ���������ϵ�Ĺ����������棬����Ƿ���ֵ
		void GetInverseInertiaTensorWorld(Matrix3 *inverseInertiaTensorWorld)const;//�õ���������ϵ�Ĺ����������� ����ڲ���ָ��
		Matrix3 GetInverseInertiaTensorWorld()const;//�õ���������ϵ�Ĺ����������� ����Ƿ���ֵ

		void SetDamping(const real linearDamping,const real angularDamping);//���������������ת����
		void SetLinearDamping(const real linearDamping);//������������
		real GetLinearDamping()const;//�õ���������
		void SetAngularDamping(const real angularDamping);//������ת����
		real GetAngularDamping()const;//�õ���ת����

		void SetPosition(const Vector3 &p);//����������������������ϵ�е�λ��
		void SetPosition(const real x,const real y, const real z);//����������������������ϵ�е�λ��
		void GetPosition(Vector3 *p)const;//�õ�������������������ϵ�е�λ��
		Vector3 GetPosition()const;//�õ�������������������ϵ�е�λ��

		void SetOrientation(const Quaternion &orientation);//���ø�������������ϵ�еķ���
		void SetOrientation(const real r,const real i,const real j,const real k);//���ø�������������ϵ�еķ���
		void GetOrientation(Quaternion *orientation)const;//�õ���������������ϵ�еķ��� ��������ڲ�������Ԫ��ָ��
		Quaternion GetOrientation()const;//�õ���������������ϵ�еķ��� �������һ����Ԫ��
		void GetOrientation(Matrix3 *matrix)const;//�õ���������������ϵ�еķ��� ���������Ԫ����Matrix3��ʽ ����Ԫ�����Եȼ�ת��Ϊ3*3�ľ�����ʽ��
		void GetOrientation(real matrix[9]) const;//�õ���������������ϵ�еķ��� ������س���Ϊ9��������ʽ��ʵ�ʾ���matrix3�ı���

		void GetTransform(Matrix4 *transform)const;//�õ�Matrix4��ʽ�ı任���󣬽�������ڲ���ָ��
		void GetTransform(real matrix[16])const;//�õ�һ������Ϊ16��������ʽ�ı任���󣬽�������ڲ���
		void GetGLTransform(real matrix[16]) const;//�õ�һ��OpenGL�и�ʽ�ı任����ת������float���ͣ�
		Matrix4 GetTransform()const;//����һ��Matrix4��ʽ�ı任����
		real * GetGLTransformReturn(real matrix[16])const;

		Vector3 GetPointInLocalSpace(const Vector3 &worldp)const;//�Ѳ����е���������ת���ɸ�������ϵ������
		Vector3 GetPointInWorldSpace(const Vector3 &localp)const;//�ɲ����еĸ�������ϵ������ת������������
		Vector3 GetDirectionInLocalSpace(const Vector3 &worldD)const;//�Ѳ����е���������ϵ�еķ�������ת���ɸ�������ϵ�е�
		Vector3 GetDirectionInWorldSpace(const Vector3 &localD)const;//�Ѳ����еĸ�������ϵ�еķ�������ת������������ϵ�е�

		void SetVelocity(const Vector3 &v);//���ø�����������������ϵ�е����ٶ�
		void SetVelocity(const real x,const real y,const real z);//���ø������ĵ����ٶ�
		void GetVelocity(Vector3 *v)const;//�õ���������ٶ� ��������ڲ���ָ��
		Vector3 GetVelocity()const;//���ظ�������ٶ�
		void AddVelocity(const Vector3 &deltaVelocity);//Ϊ�����������ٶ�deltav

		void SetRotation(const Vector3 &rotation);//���ø���Ľ��ٶ�
		void SetRotation(const real x, const real y, const real z);//���ø���Ľ��ٶ�
		void GetRotation(Vector3 *rotation) const;//�õ�����Ľ��ٶȣ����������ָ��
		Vector3 GetRotation() const;//���ظ���Ľ��ٶ�
		void AddRotation(const Vector3 &deltaRotation);//Ϊ�������ӽ��ٶ�deltar

		bool GetAwake() const;//���ظ����awake״̬
		void SetAwake(const bool awake=true);//���ø����awake״̬���������˯��������û���ٶȵ�
		bool GetCanSleep() const;//���ظ����Ƿ���sleep
		void SetCanSleep(const bool canSleep=true);//���ø����Ƿ���sleep

		void GetLastFrameAcceleration(Vector3 *linearAcceleration) const;//�õ�������һ֡�ļ��ٶ�
		Vector3 GetLastFrameAcceleration() const;//�õ�������һ֡�ļ��ٶ�
		
		void SetAcceleration(const Vector3 &acceleration);//���ü��ٶ�
		void SetAcceleration(const real x, const real y, const real z);//���ü��ٶ�
		void GetAcceleration(Vector3 *acceleration) const;//�õ����ٶ�
		Vector3 GetAcceleration() const;//�õ����ٶ�

		//�õ�������һ����ٶ� 
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