#pragma once
//#include"PhysicsCommon.h"
namespace Physics
{
	class DLL_PHYSICS Vector3;
	class DLL_PHYSICS Quaternion;
	class DLL_PHYSICS Matrix3;
	class DLL_PHYSICS Matrix4;
	//为了与矩阵变换使用方便和与OpenGL更好的兼容，假设这里的Vector为列向量
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
		real Magnitude()const;       //向量的模
		real SquareMagnitude()const;//向量的模的平方和
		void Normalize();      //向量单位化
		void Clear();				//向量的置零
		void Invert();			//向量值取相反数
		bool IsNAN();
		bool IsINFIN();
		void AddScaledVector(const Vector3& v,real scale);//加上另外一个向量的倍数

		//取值重载
		real operator[](unsigned i)const;
		real& operator[](unsigned i);

		//向量的数乘	Vector1*=realA 
		void operator*=(const real value);					
		//向量的数乘 vector1=vector2*realValue 
		Vector3 operator*(const real value)const;				

		//向量的数量积 realResult=vector1*vector2  
		//设vector1(x1,y1,z1) vector2(x2,y2,z2) realResult=x1*x2+ y1*y2+ z1*z2
		real operator*(const Vector3 &vector)const;			
		//向量的数量积
		real ScalarProduct(const Vector3 &vector)const;

		//向量的加法 vector1=vector2+vector3  等于把向量的各分量相加
		void operator+=(const Vector3 &vector);			
		//向量的加法
		Vector3 operator+(const Vector3 &vector)const;

		//向量的相减 vector1=vector2-vector3  等于把向量的各分量相减
		void operator-=(const Vector3 &vector);				
		//向量的相减
		Vector3 operator-(const Vector3 &vector)const;	

		//向量的叉积  假设坐标系正交单位基为i,j,k。 v1=ai+bj+ck，v2=xi+yj+zk
		//则v1%v2=det[ (i,j,k),(a,b,c),(x,y,z)];   det为行列式
		void operator%=(const Vector3 &vector);			
		//向量的叉积
		Vector3 operator %(const Vector3 &vector)const;
		//向量的叉积
		Vector3 CrossProduct(const Vector3 &vector)const;

		//向量的分量相乘 vector1=vector2/vector3;
		//vector1=(x2*x3,y2*y3,z2*z3);
		Vector3 ComponentProduct(const Vector3& vector)const;
		//向量的分量相乘
		Vector3 operator/(const Vector3 &vector)const;
		//向量的分量相乘
		void operator/=(const Vector3 &vector);
	};

	//四元数的意义在于它存储着一个坐标系变换到另外一个坐标系的变换信息，跟变换矩阵类似
	//它的好处是 适用于插值（矩阵不适用于插值），并且运算量小，无奇点（欧拉角有奇点）
	//他跟变换矩阵的差距在于，变换矩阵可以存储平移 旋转 缩放，而四元数只能适用于坐标系的旋转
	class DLL_PHYSICS Quaternion
	{
	public:
		//四元数是由一个实部,一个虚向量组成 一般可表示为
		//（a，u) a=r(实部)，u=x i + y j + z k; 其中i j k为单位虚数
		//i^2=j^2=k^2=ijk=-1;
		//四元数的四个值 第一个q[0]为四元数的实部，其余3个
		//是四元数的3个虚部
		real data[4];
	public:
		//构造函数
		Quaternion(){for(int i=0;i<4;i++)data[i]=0;}
		Quaternion(const real r,const real i,const real j,const real k)
		{data[0]=r;data[1]=i;data[2]=j;data[3]=k;}
		Quaternion(real q[4])
		{for(int i=0;i<4;i++)this->data[i]=q[i];}

		//四元数单位化 如果r=i=j=k=0,置r=1;
		void Normalize();

		//四元数的共轭 即把虚部置相反数
		Quaternion GetAdjoint()const;
		void SetAdjoint(Quaternion &q)const;
		void SetAdjoint();


		//四元数的格拉斯曼积，一般被称为四元数的乘法
		//假设q1=(a,u) q2=(t,v) 则q1*q2=at-u.v+av+tu+u%v (u%v)为向量叉积
		//假设u=(b,c,d) v=(x,y,z)
		//则q1*q2=(at-bx-cy-dz) + (bt+ax+cz-dy)i +(ct+ay+dx-bz)j +(dt+za+by-xc)k
		Quaternion operator*(const Quaternion &Q)const;
		//四元数的格拉斯曼积
		void operator*=(const Quaternion &Q);

		//四元数用来表示旋转后坐标系的方位 
		//方位按照vector为旋转轴旋转180度
		void RotateByVector(const Vector3& vector);

		//用来专门处理世界坐标系中的角速度在scale时间内对刚体方位的变化
		//vector 是角速度，scale是时间（时间要求短暂）否则不准确
		void AddScaledVector(const Vector3& vector,real scale);

		//通过四元数把一个向量转化为另坐标系中的向量 使用前应该把四元数单位化
		//如果一个四元数q(r,i,j,k) 表示现坐标系在原坐标系中的方位，则对于现坐标系中的某点P(x,y,z)
		//原坐标系中对应的P0(a,b,c)=q*(0,x,y,z)*(q-1)  其中q-1表示q的逆  由于q为单位四元数，所以q-1=q*; 
		//q*是共轭四元数 q*=(r, -i, -j, -k)
		Vector3 TransformVectorReturn(const Vector3& vector)const;
		//通过四元数把一个向量转化为另坐标系中的向量
		void TransformVector(Vector3& vector)const;
		//通过四元数和随体坐标系原点在世界坐标系中的位置把 随体坐标系中的一点转化为世界坐标系的一点
		Vector3 TransformPointReturn(const Vector3& point,const Vector3& position)const;	
		void TransformPoint(Vector3& point,const Vector3& position)const;

		//通过四元数把一个世界坐标系向量转化为随体坐标系向量
		Vector3 TransformInverseVectorReturn(const Vector3& vector)const;
		//通过四元数把一个世界坐标系向量转化为随体坐标系向量
		void TransformInverseVector(Vector3& vector)const;
		//通过四元数和随体坐标系原点在世界坐标系中的位置把 世界坐标系中的一点转化为随体坐标系的一点
		Vector3 TransformInversePointReturn(const Vector3& point,const Vector3& position)const;	
		void TransformInversePoint(Vector3& point,const Vector3& position)const;

		//通过一个3阶方阵设置四元数
		void SetOrientation(const Matrix3& m);
		//得到参数四元数的旋转矩阵
		static Matrix3 GetMatrix3(const Quaternion& q);
		//得到该四元数的旋转矩阵
		Matrix3 GetMatrix3()const;
	};

	//3阶方阵 主要用于存储四元数转化的变换矩阵 和 张量（在这里主要是惯性张量）
	class DLL_PHYSICS Matrix3
	{
	public:
		//3阶方阵的元素 a11,a12,a13,a21,a22,a23,a31,a32,a33 分别对应数组的0-8
		//前下标为行，后下标为列
		real data[9];
	public:
		//默认构造函数
		Matrix3()	
		{
			data[0]=data[1]=data[2]=
				data[3]=data[4]=data[5]=
				data[6]=data[7]=data[8]=0;
		}
		//使用9个real参数构造一个3阶方阵
		Matrix3(real a0,real a1,real a2,real a3,real a4,real a5, real a6,real a7,real a8)	
		{
			data[0]=a0;data[1]=a1;data[2]=a2;
			data[3]=a3;data[4]=a4;data[5]=a5;
			data[6]=a6;data[7]=a7;data[8]=a8;
		};
		//3向量构造一个3阶方阵 其中v均为列向量
		Matrix3(const Vector3 &v0,const Vector3&v1,const Vector3&v2)
		{
		}
		//利用一个长度为9的数组构造一个3阶方阵,如果传入一个少于9的数组，
		//不会报错，但是运行会有风险
		Matrix3(real data[9])
		{
			for(int i=0;i<9;i++)
				this->data[i]=data[i];
		}

		//3阶方阵相乘 其中this在左边，other在右边
		Matrix3 operator*(const Matrix3 &other)const;
		//与3阶方阵相乘 其中this在左边，other在右边
		void operator*=(const Matrix3 &other);
		//右乘一个列向量 结果是一个向量
		//该乘法的主要作用用于向量在不同坐标系中的变换 3阶方阵为变换矩阵
		Vector3 operator*(const Vector3 &vector)const;
		//与常量相乘
		Matrix3 operator*(const real scalar)const;
		//与常量相乘
		void operator*=(const real scalar);
		//与3阶方阵相加
		Matrix3 operator+(const Matrix3 &other)const;
		//与3阶方阵相加
		void operator+=(const Matrix3 &other);

		//设置this矩阵为自身的转置矩阵
		void SetTranspose();
		//设置this矩阵为参数矩阵的转置矩阵
		void SetTranspose(const Matrix3 &m);
		//返回this矩阵的转置
		Matrix3 Transpose()const;

		//设置this矩阵为自身的逆矩阵
		void SetInverse();
		//设置this矩阵为参数矩阵的逆矩阵
		void SetInverse(const Matrix3 &m);
		//返回this矩阵的逆矩阵
		Matrix3 GetInverse()const;

		//设置对角线值 其余地方被设置为0
		void SetDiagonal(real a0,real a4,real a8);
		//用3个列向量设置一个矩阵
		void SetComponents(const Vector3 &comp0,const Vector3 &comp1,const Vector3 &comp2);
		//设置斜交对称矩阵
		void SetSkewSymmetric(const Vector3 vector);
		//设置惯性张量
		void SetInertiaTensorCoeffs(real ix,real iy,real iz,real ixy=0,real ixz=0,real iyz=0);

		//得到矩阵的某一行，返回一个向量
		Vector3 GetRowVector(int i)const;
		//得到局针的某一列，返回一个向量
		Vector3 GetAxisVector(int i)const;
		//把一个列向量用该矩阵变换
		Vector3 Transform(const Vector3 &vector)const;
		//把一个列向量用该矩阵的转置变换 或者把一个行向量用该矩阵变换
		Vector3 TransformTranspose(const Vector3 &vector)const;

		//用一个四元数设置三维矩阵 即把一个四元数转化为其对应的变换矩阵
		void SetOrientation(const Quaternion &q);
		//把参数矩阵转换为一个四元数
		static Quaternion GetQuaternion(const Matrix3 &m);
		//返回该矩阵的四元数形式
		Quaternion GetQuaternion()const;
		//矩阵的线性插值
		static Matrix3 LinearInterpolate(const Matrix3& a,const Matrix3& b,real prop);
	};

	//4阶方阵 用于齐次坐标变换 由于齐次坐标变换时 最后一行四个元素都是0,0,0,1 
	//乘以列向量进行变换时也不会影响变换后向量的值 ，所以可以把其去掉
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
		//设置对角线
		void SetDiagonal(real a,real b,real c);
		//右乘一个列向量
		Vector3 operator*(const Vector3 &vector)const;
		//右乘一个4阶矩阵
		Matrix4 operator*(const Matrix4 &other)const;
		//
		void Clear()
		{
			data[1]=data[2]=data[3]=data[4]=data[6]=data[7]=data[8]=data[9]=data[11]=0;
			data[0]=data[5]=data[10]=1;
		}

		//得到行列式的值
		real GetDeterminant()const;
		//把this矩阵设置为参数矩阵的逆
		void SetInverse(const Matrix4 & m);	
		//得到this矩阵的逆
		Matrix4 GetInverse()const;
		//设置this矩阵的逆
		void SetInverse();
		//用这个矩阵变换方向向量
		Vector3 TransformDirection(const Vector3 &vector)const;
		//用这个矩阵的逆矩阵变换方向向量
		Vector3 TransformInverseDirection(const Vector3 &vector)const;
		//对一个坐标进行坐标变换
		Vector3 Transform(const Vector3 &vector)const;	
		//用这个矩阵的逆矩阵对一个坐标进行变换
		Vector3 TransformInverse(const Vector3 &vector)const;
		//返回该矩阵的第i列。i=0,1,2
		Vector3 GetAxisVector(int i)const;	

		//根据给定的四元数和位置设定齐次变换矩阵
		void SetOrientationAndPos(const Quaternion &q,const Vector3 &pos);
		//根据给定的3阶方阵和位置设定齐次变换矩阵
		void SetOrientationAndPos(const Matrix3 &m,const Vector3 &pos);

		//把该矩阵设置为OpenGL4*4矩阵，结果返回在一个16的数组里面
		//OpenGL的变换矩阵与这里的变换矩阵互为转置
		void GetGLMatrix4(real m[16])const;
		real *GetGLMatrix4Return(real m[16])const;
	};
	//2维向量 
	class DLL_PHYSICS Vector2
	{
	public:
		real data[2];
	};
}