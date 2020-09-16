#pragma once
//#include"PhysicsCommon.h"
namespace Physics
{
	class DLL_PHYSICS Vector3;
	class DLL_PHYSICS Quaternion;
	class DLL_PHYSICS Matrix3;
	class DLL_PHYSICS Matrix4;
	//Ϊ�������任ʹ�÷������OpenGL���õļ��ݣ����������VectorΪ������
	class DLL_PHYSICS Vector3
	{
	public:
		real data[3];
	public:
		Vector3(){data[0]=data[1]=data[2]=0;}
		Vector3(const real x,const real y,const real z){data[0]=x;data[1]=y;data[2]=z;}
		Vector3(const real p[3]){data[0]=p[0];data[1]=p[1];data[2]=p[2];}
		void SetData(real x,real y,real z);
		void SetData(const real p[3]);
		real Magnitude()const;       //������ģ
		real SquareMagnitude()const;//������ģ��ƽ����
		void Normalize();      //������λ��
		void Clear();				//����������
		void Invert();			//����ֵȡ�෴��
		bool IsNAN();
		bool IsINFIN();
		void AddScaledVector(const Vector3& v,real scale);//��������һ�������ı���

		//ȡֵ����
		real operator[](unsigned i)const;
		real& operator[](unsigned i);

		//����������	Vector1*=realA 
		void operator*=(const real value);					
		//���������� vector1=vector2*realValue 
		Vector3 operator*(const real value)const;				

		//������������ realResult=vector1*vector2  
		//��vector1(x1,y1,z1) vector2(x2,y2,z2) realResult=x1*x2+ y1*y2+ z1*z2
		real operator*(const Vector3 &vector)const;			
		//������������
		real ScalarProduct(const Vector3 &vector)const;

		//�����ļӷ� vector1=vector2+vector3  ���ڰ������ĸ��������
		void operator+=(const Vector3 &vector);			
		//�����ļӷ�
		Vector3 operator+(const Vector3 &vector)const;

		//��������� vector1=vector2-vector3  ���ڰ������ĸ��������
		void operator-=(const Vector3 &vector);				
		//���������
		Vector3 operator-(const Vector3 &vector)const;	

		//�����Ĳ��  ��������ϵ������λ��Ϊi,j,k�� v1=ai+bj+ck��v2=xi+yj+zk
		//��v1%v2=det[ (i,j,k),(a,b,c),(x,y,z)];   detΪ����ʽ
		void operator%=(const Vector3 &vector);			
		//�����Ĳ��
		Vector3 operator %(const Vector3 &vector)const;
		//�����Ĳ��
		Vector3 CrossProduct(const Vector3 &vector)const;

		//�����ķ������ vector1=vector2/vector3;
		//vector1=(x2*x3,y2*y3,z2*z3);
		Vector3 ComponentProduct(const Vector3& vector)const;
		//�����ķ������
		Vector3 operator/(const Vector3 &vector)const;
		//�����ķ������
		void operator/=(const Vector3 &vector);
	};

	//��Ԫ���������������洢��һ������ϵ�任������һ������ϵ�ı任��Ϣ�����任��������
	//���ĺô��� �����ڲ�ֵ�����������ڲ�ֵ��������������С������㣨ŷ��������㣩
	//�����任����Ĳ�����ڣ��任������Դ洢ƽ�� ��ת ���ţ�����Ԫ��ֻ������������ϵ����ת
	class DLL_PHYSICS Quaternion
	{
	public:
		//��Ԫ������һ��ʵ��,һ����������� һ��ɱ�ʾΪ
		//��a��u) a=r(ʵ��)��u=x i + y j + z k; ����i j kΪ��λ����
		//i^2=j^2=k^2=ijk=-1;
		//��Ԫ�����ĸ�ֵ ��һ��q[0]Ϊ��Ԫ����ʵ��������3��
		//����Ԫ����3���鲿
		real data[4];
	public:
		//���캯��
		Quaternion(){for(int i=0;i<4;i++)data[i]=0;}
		Quaternion(const real r,const real i,const real j,const real k)
		{data[0]=r;data[1]=i;data[2]=j;data[3]=k;}
		Quaternion(real q[4])
		{for(int i=0;i<4;i++)this->data[i]=q[i];}

		//��Ԫ����λ�� ���r=i=j=k=0,��r=1;
		void Normalize();

		//��Ԫ���Ĺ��� �����鲿���෴��
		Quaternion GetAdjoint()const;
		void SetAdjoint(Quaternion &q)const;
		void SetAdjoint();


		//��Ԫ���ĸ���˹������һ�㱻��Ϊ��Ԫ���ĳ˷�
		//����q1=(a,u) q2=(t,v) ��q1*q2=at-u.v+av+tu+u%v (u%v)Ϊ�������
		//����u=(b,c,d) v=(x,y,z)
		//��q1*q2=(at-bx-cy-dz) + (bt+ax+cz-dy)i +(ct+ay+dx-bz)j +(dt+za+by-xc)k
		Quaternion operator*(const Quaternion &Q)const;
		//��Ԫ���ĸ���˹����
		void operator*=(const Quaternion &Q);

		//��Ԫ��������ʾ��ת������ϵ�ķ�λ 
		//��λ����vectorΪ��ת����ת180��
		void RotateByVector(const Vector3& vector);

		//����ר�Ŵ�����������ϵ�еĽ��ٶ���scaleʱ���ڶԸ��巽λ�ı仯
		//vector �ǽ��ٶȣ�scale��ʱ�䣨ʱ��Ҫ����ݣ�����׼ȷ
		void AddScaledVector(const Vector3& vector,real scale);

		//ͨ����Ԫ����һ������ת��Ϊ������ϵ�е����� ʹ��ǰӦ�ð���Ԫ����λ��
		//���һ����Ԫ��q(r,i,j,k) ��ʾ������ϵ��ԭ����ϵ�еķ�λ�������������ϵ�е�ĳ��P(x,y,z)
		//ԭ����ϵ�ж�Ӧ��P0(a,b,c)=q*(0,x,y,z)*(q-1)  ����q-1��ʾq����  ����qΪ��λ��Ԫ��������q-1=q*; 
		//q*�ǹ�����Ԫ�� q*=(r, -i, -j, -k)
		Vector3 TransformVectorReturn(const Vector3& vector)const;
		//ͨ����Ԫ����һ������ת��Ϊ������ϵ�е�����
		void TransformVector(Vector3& vector)const;
		//ͨ����Ԫ������������ϵԭ������������ϵ�е�λ�ð� ��������ϵ�е�һ��ת��Ϊ��������ϵ��һ��
		Vector3 TransformPointReturn(const Vector3& point,const Vector3& position)const;	
		void TransformPoint(Vector3& point,const Vector3& position)const;

		//ͨ����Ԫ����һ����������ϵ����ת��Ϊ��������ϵ����
		Vector3 TransformInverseVectorReturn(const Vector3& vector)const;
		//ͨ����Ԫ����һ����������ϵ����ת��Ϊ��������ϵ����
		void TransformInverseVector(Vector3& vector)const;
		//ͨ����Ԫ������������ϵԭ������������ϵ�е�λ�ð� ��������ϵ�е�һ��ת��Ϊ��������ϵ��һ��
		Vector3 TransformInversePointReturn(const Vector3& point,const Vector3& position)const;	
		void TransformInversePoint(Vector3& point,const Vector3& position)const;

		//ͨ��һ��3�׷���������Ԫ��
		void SetOrientation(const Matrix3& m);
		//�õ�������Ԫ������ת����
		static Matrix3 GetMatrix3(const Quaternion& q);
		//�õ�����Ԫ������ת����
		Matrix3 GetMatrix3()const;
	};

	//3�׷��� ��Ҫ���ڴ洢��Ԫ��ת���ı任���� �� ��������������Ҫ�ǹ���������
	class DLL_PHYSICS Matrix3
	{
	public:
		//3�׷����Ԫ�� a11,a12,a13,a21,a22,a23,a31,a32,a33 �ֱ��Ӧ�����0-8
		//ǰ�±�Ϊ�У����±�Ϊ��
		real data[9];
	public:
		//Ĭ�Ϲ��캯��
		Matrix3()	
		{
			data[0]=data[1]=data[2]=
				data[3]=data[4]=data[5]=
				data[6]=data[7]=data[8]=0;
		}
		//ʹ��9��real��������һ��3�׷���
		Matrix3(real a0,real a1,real a2,real a3,real a4,real a5, real a6,real a7,real a8)	
		{
			data[0]=a0;data[1]=a1;data[2]=a2;
			data[3]=a3;data[4]=a4;data[5]=a5;
			data[6]=a6;data[7]=a7;data[8]=a8;
		};
		//3��������һ��3�׷��� ����v��Ϊ������
		Matrix3(const Vector3 &v0,const Vector3&v1,const Vector3&v2)
		{
		}
		//����һ������Ϊ9�����鹹��һ��3�׷���,�������һ������9�����飬
		//���ᱨ���������л��з���
		Matrix3(real data[9])
		{
			for(int i=0;i<9;i++)
				this->data[i]=data[i];
		}

		//3�׷������ ����this����ߣ�other���ұ�
		Matrix3 operator*(const Matrix3 &other)const;
		//��3�׷������ ����this����ߣ�other���ұ�
		void operator*=(const Matrix3 &other);
		//�ҳ�һ�������� �����һ������
		//�ó˷�����Ҫ�������������ڲ�ͬ����ϵ�еı任 3�׷���Ϊ�任����
		Vector3 operator*(const Vector3 &vector)const;
		//�볣�����
		Matrix3 operator*(const real scalar)const;
		//�볣�����
		void operator*=(const real scalar);
		//��3�׷������
		Matrix3 operator+(const Matrix3 &other)const;
		//��3�׷������
		void operator+=(const Matrix3 &other);

		//����this����Ϊ�����ת�þ���
		void SetTranspose();
		//����this����Ϊ���������ת�þ���
		void SetTranspose(const Matrix3 &m);
		//����this�����ת��
		Matrix3 Transpose()const;

		//����this����Ϊ����������
		void SetInverse();
		//����this����Ϊ��������������
		void SetInverse(const Matrix3 &m);
		//����this����������
		Matrix3 GetInverse()const;

		//���öԽ���ֵ ����ط�������Ϊ0
		void SetDiagonal(real a0,real a4,real a8);
		//��3������������һ������
		void SetComponents(const Vector3 &comp0,const Vector3 &comp1,const Vector3 &comp2);
		//����б���Գƾ���
		void SetSkewSymmetric(const Vector3 vector);
		//���ù�������
		void SetInertiaTensorCoeffs(real ix,real iy,real iz,real ixy=0,real ixz=0,real iyz=0);

		//�õ������ĳһ�У�����һ������
		Vector3 GetRowVector(int i)const;
		//�õ������ĳһ�У�����һ������
		Vector3 GetAxisVector(int i)const;
		//��һ���������øþ���任
		Vector3 Transform(const Vector3 &vector)const;
		//��һ���������øþ����ת�ñ任 ���߰�һ���������øþ���任
		Vector3 TransformTranspose(const Vector3 &vector)const;

		//��һ����Ԫ��������ά���� ����һ����Ԫ��ת��Ϊ���Ӧ�ı任����
		void SetOrientation(const Quaternion &q);
		//�Ѳ�������ת��Ϊһ����Ԫ��
		static Quaternion GetQuaternion(const Matrix3 &m);
		//���ظþ������Ԫ����ʽ
		Quaternion GetQuaternion()const;
		//��������Բ�ֵ
		static Matrix3 LinearInterpolate(const Matrix3& a,const Matrix3& b,real prop);
	};

	//4�׷��� �����������任 �����������任ʱ ���һ���ĸ�Ԫ�ض���0,0,0,1 
	//�������������б任ʱҲ����Ӱ��任��������ֵ �����Կ��԰���ȥ��
	class DLL_PHYSICS Matrix4
	{
	public:
		real data[12];
	public:
		Matrix4()
		{
			data[1]=data[2]=data[3]=data[4]=data[6]=data[7]=data[8]=data[9]=data[11]=0;
			data[0]=data[5]=data[10]=1;
		};
		Matrix4(float m[9],float p[3])
		{
			data[0]=m[0];data[1]=m[1];data[2]=m[2];
			data[3]=p[0];
			data[4]=m[3];data[5]=m[4];data[6]=m[5];
			data[7]=p[1];
			data[8]=m[6];data[9]=m[7];data[10]=m[8];
			data[11]=p[2];
		};
		Matrix4(const Matrix3 &m,const Vector3& p)
		{
			data[0]=m.data[0];data[1]=m.data[1];data[2]=m.data[2];
			data[3]=p.data[0];
			data[4]=m.data[3];data[5]=m.data[4];data[6]=m.data[5];
			data[7]=p.data[1];
			data[8]=m.data[6];data[9]=m.data[7];data[10]=m.data[8];
			data[11]=p.data[2];
		}
		Matrix4(float m[12])
		{
			for(int i=0;i<12;i++)
				data[i]=m[i];
		}
	public:
		//���öԽ���
		void SetDiagonal(real a,real b,real c);
		//�ҳ�һ��������
		Vector3 operator*(const Vector3 &vector)const;
		//�ҳ�һ��4�׾���
		Matrix4 operator*(const Matrix4 &other)const;
		//
		void Clear()
		{
			data[1]=data[2]=data[3]=data[4]=data[6]=data[7]=data[8]=data[9]=data[11]=0;
			data[0]=data[5]=data[10]=1;
		}

		//�õ�����ʽ��ֵ
		real GetDeterminant()const;
		//��this��������Ϊ�����������
		void SetInverse(const Matrix4 & m);	
		//�õ�this�������
		Matrix4 GetInverse()const;
		//����this�������
		void SetInverse();
		//���������任��������
		Vector3 TransformDirection(const Vector3 &vector)const;
		//���������������任��������
		Vector3 TransformInverseDirection(const Vector3 &vector)const;
		//��һ�������������任
		Vector3 Transform(const Vector3 &vector)const;	
		//����������������һ��������б任
		Vector3 TransformInverse(const Vector3 &vector)const;
		//���ظþ���ĵ�i�С�i=0,1,2
		Vector3 GetAxisVector(int i)const;	

		//���ݸ�������Ԫ����λ���趨��α任����
		void SetOrientationAndPos(const Quaternion &q,const Vector3 &pos);
		//���ݸ�����3�׷����λ���趨��α任����
		void SetOrientationAndPos(const Matrix3 &m,const Vector3 &pos);

		//�Ѹþ�������ΪOpenGL4*4���󣬽��������һ��16����������
		//OpenGL�ı任����������ı任����Ϊת��
		void GetGLMatrix4(real m[16])const;
		real *GetGLMatrix4Return(real m[16])const;
	};
	//2ά���� 
	class DLL_PHYSICS Vector2
	{
	public:
		real data[2];
	};
}