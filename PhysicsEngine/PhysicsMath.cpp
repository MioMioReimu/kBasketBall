#define DLL_PHYSICS_EXIST
#include"Physics.h"

namespace Physics
{
//<Vector3
	//取值重载
	real Vector3::operator[](unsigned i)const
	{
		if(i>2)return data[2];
		return data[i];
	}
	real& Vector3::operator[](unsigned i)
	{
		if(i>2)return data[2];
		return data[i];
	}
	bool Vector3::IsNAN()
	{
		return _isnan(data[0])||_isnan(data[1])||_isnan(data[2]);
	}
	bool Vector3::IsINFIN()
	{
		return (!_finite(data[0]))||(!_finite(data[1]))||(!_finite(data[2]));
	}
	void Vector3::Invert()			//向量值取相反数
	{
		data[0]=-data[0];data[1]=-data[1];data[2]=-data[2];
	}
	void Vector3::AddScaledVector(const Vector3& v,real scale)//加上另外一个向量的倍数
	{
		*this+=v*scale;
	}
	
	void Vector3::SetData(real x,real y,real z)
	{data[0]=x;data[1]=y;data[2]=z;}
	void Vector3::SetData(const real p[3])
	{for(int i=0;i<3;i++)data[i]=p[i];}
	//向量的模
	real Vector3::Magnitude()       const
	{
		return real_sqrt(data[0]*data[0]+data[1]*data[1]+data[2]*data[2]);
	};

		//向量的模的平方和
	real Vector3::SquareMagnitude()const
	{
		return data[0]*data[0]+data[1]*data[1]+data[2]*data[2];
	}

		//向量单位化
	void Vector3::Normalize()      
	{
		real Length=this->Magnitude();
		assert(Length!=0);
		data[0]=data[0]/Length;
		data[1]=data[1]/Length;
		data[2]=data[2]/Length;
	}

		//向量的置零
	void Vector3::Clear()				
	{
		data[0]=data[1]=data[2]=0;
	}

		//Vector1*=realValue 向量的数乘
	void Vector3::operator*=(const real value)							
	{
		for(int i=0;i<3;i++)
			data[i]*=value;
	}

		//vector1=vector2*realValue 向量的数乘
	Vector3 Vector3::operator*(const real value)const				//
	{
		return Vector3(data[0]*value,data[1]*value,data[2]*value);
	}

		//realResult=vector1*vector2  向量的数量积
		//设vector1(x1,y1,z1) vector2(x2,y2,z2) realResult=x1*x2+ y1*y2+ z1*z2
	real Vector3::operator*(const Vector3 &vector)const			
	{
		return data[0]*vector.data[0]+data[1]*vector.data[1]+data[2]*vector.data[2];
	}
	real Vector3::ScalarProduct(const Vector3 &vector)const
	{
		return data[0]*vector.data[0]+data[1]*vector.data[1]+data[2]*vector.data[2];
	}

		//vector1=vector2+vector3 向量的加法 等于把向量的各分量相加
	void Vector3::operator+=(const Vector3 &vector)				
	{
		for(int i=0;i<3;i++)
			data[i]+=vector.data[i];
	}
	Vector3 Vector3::operator+(const Vector3 &vector)const
	{
		return Vector3(data[0]+vector.data[0],data[1]+vector.data[1],data[2]+vector.data[2]);
	}

		//vector1=vector2-vector3 向量的相减 等于把向量的各分量相减
	void Vector3::operator-=(const Vector3 &vector)				
	{
		for(int i=0;i<3;i++)
			data[i]-=vector.data[i];
	}
	Vector3 Vector3::operator-(const Vector3 &vector)const	
	{
		return Vector3(data[0]-vector.data[0],data[1]-vector.data[1],data[2]-vector.data[2]);
	}

		//向量的叉积  假设坐标系正交单位基为i,j,k。 v1=ai+bj+ck，v2=xi+yj+zk
		//则v1%v2=det[ (i,j,k),(a,b,c),(x,y,z)];   det为行列式
		//v1%v2=(bz-cy)i - (az-cx)j + (ay-bx)k
	Vector3 Vector3::CrossProduct(const Vector3& vector)const
	{
		return Vector3(
			data[1]*vector.data[2]-data[2]*vector.data[1],
			data[2]*vector.data[0]-data[0]*vector.data[2],
			data[0]*vector.data[1]-data[1]*vector.data[0]);
	}
		//向量的叉积
	void Vector3::operator%=(const Vector3 &vector)
	{
		*this=CrossProduct(vector);
	}
		//向量的叉积
	Vector3 Vector3::operator %(const Vector3 &vector)const//
	{
		return CrossProduct(vector);
	}

		//向量的分量相乘 vector1=vector2/vector3;
		//vector1=(x2*x3,y2*y3,z2*z3);
	Vector3 Vector3::ComponentProduct(const Vector3& vector)const
	{
		return Vector3(data[0]*vector.data[0],data[1]*vector.data[1],data[2]*vector.data[2]);
	}
		//向量的分量相乘
	Vector3 Vector3::operator/(const Vector3 &vector)const
	{
		return Vector3(data[0]*vector.data[0],data[1]*vector.data[1],data[2]*vector.data[2]);
	}
		//向量的分量相乘
	void Vector3::operator/=(const Vector3 &vector)
	{
		*this=ComponentProduct(vector);
	}
//Vector3>

//<Quaternion
		//四元数单位化 如果r=i=j=k=0,置r=1;
	void Quaternion::Normalize()
	{
		real Length=real_sqrt(data[0]*data[0]+data[1]*data[1]+data[2]*data[2]+data[3]*data[3]);
		if(Length==real(0))
		{
			data[0]=1;
			return;
		}
		for(int i=0;i<4;i++)
			data[i]/=Length;
	}

		//四元数的共轭 即把虚部置相反数
	Quaternion Quaternion::GetAdjoint()const
	{
		return Quaternion(this->data[0],-this->data[1],-this->data[2],-this->data[3]);
	}
		//共轭四元数
	void Quaternion::SetAdjoint(Quaternion &q)const
	{
		q.data[0]=this->data[0];
		for(int i=1;i<4;i++)
			q.data[i]=-this->data[i];
	}
		//共轭四元数
	void Quaternion::SetAdjoint()
	{
		for(int i=1;i<4;i++)
			data[i]=-this->data[i];
	}

		//四元数的格拉斯曼积，一般被称为四元数的乘法
		//假设q1=(a,u) q2=(t,v) 则q1*q2=at-u.v+av+tu+u%v (u%v)为向量叉积
		//假设u=(b,c,d) v=(x,y,z)
		//则q1*q2=(at-bx-cy-dz) + (bt+ax+cz-dy)i +(ct+ay+dx-bz)j +(dt+za+by-xc)k
	Quaternion Quaternion::operator*(const Quaternion &Q)const
	{
		return Quaternion(data[0]*Q.data[0]-data[1]*Q.data[1]-data[2]*Q.data[2]-data[3]*Q.data[3],
			data[0]*Q.data[1]+data[1]*Q.data[0]+data[2]*Q.data[3]-data[3]*Q.data[2],
			data[0]*Q.data[2]+data[2]*Q.data[0]+data[3]*Q.data[1]-data[1]*Q.data[3],
			data[0]*Q.data[3]+data[3]*Q.data[0]+data[1]*Q.data[2]-data[2]*Q.data[1]);
	}
		//四元数的格拉斯曼积
	void Quaternion::operator*=(const Quaternion &Q)
	{
		*this=*this*Q;
	}

		//四元数用来表示旋转后坐标系的方位 
		//方位按照vector为旋转轴旋转180度
	void Quaternion::RotateByVector(const Vector3& vector)
	{
		Quaternion q(0,vector.data[0],vector.data[1],vector.data[2]);
		(*this)*=q;
	}

		//用来专门处理世界坐标系中的角速度在scale时间内对刚体方位的变化
		//vector 是角速度，scale是时间（时间要求短暂）否则不准确 下面是两种算法，本质是一样的，都要求时间短暂
		// p=Tp  T=（cos(deltaS/2),sin(deltaS/2)*v.x,sin(deltaS/2)*v.y,sin(deltaS/2)*v.z)
			//其中p表示该四元数对象, v 是vector的单位向量  , deltaS=vector的模 *scale		精确的算法
		// p=q*p*0.5+p   近似 的算法
			//p表示该四元数对象。q表示三维向量vector和倍数scale变换成的四元数（0，x*scale,y*scale,z*scale)
	void Quaternion::AddScaledVector(const Vector3& vector,real scale)
	{
		real sita=vector.Magnitude()*scale;
		Vector3 v=vector;
		if(v.Magnitude()!=0)
			v.Normalize();
		Quaternion q(cos(real(sita/2)),sin(sita/2)*v.data[0],sin(sita/2)*v.data[1],sin(sita/2)*v.data[2]);
		q*=*this;
		*this=q;
		/*Quaternion q(0,vector.data[0]*scale,vector.data[1]*scale,vector.data[2]*scale);
		q*=(*this);
		data[0]+=q.data[0]*((real)0.5);
		data[1]+=q.data[1]*((real)0.5);
		data[2]+=q.data[2]*((real)0.5);
		data[3]+=q.data[3]*((real)0.5);*/
	}

		//通过四元数把一个向量转化为另坐标系中的向量
	Vector3 Quaternion::TransformVectorReturn(const Vector3& vector)const
	{
		Quaternion v(0,vector.data[0],vector.data[1],vector.data[2]);
		v=*this*v*(this->GetAdjoint());
		return Vector3(v.data[1],v.data[2],v.data[3]);
	}
		//通过四元数把一个向量转化为另坐标系中的向量
	void Quaternion::TransformVector(Vector3& vector)const
	{
		Quaternion v(0,vector.data[0],vector.data[1],vector.data[2]);
		v=*this*v*(this->GetAdjoint());
		vector.data[0]=v.data[1];
		vector.data[1]=v.data[2];
		vector.data[2]=v.data[3];
	}
	Vector3 Quaternion::TransformPointReturn(const Vector3& point,const Vector3& position)const
	{
		Quaternion v(0,point.data[0],point.data[1],point.data[2]);
		v=*this*v*(this->GetAdjoint());
		return Vector3(v.data[1]+position.data[0],v.data[2]+position.data[1],v.data[3]+position.data[2]);
	}
	void Quaternion::TransformPoint(Vector3& point,const Vector3& position)const
	{
		Quaternion v(0,point.data[0],point.data[1],point.data[2]);
		v=*this*v*(this->GetAdjoint());
		point.data[0]=v.data[1]+position.data[0];
		point.data[1]=v.data[2]+position.data[1];
		point.data[2]=v.data[3]+position.data[2];
	}

		//通过四元数把一个世界坐标系向量转化为随体坐标系向量
		Vector3 Quaternion::TransformInverseVectorReturn(const Vector3& vector)const
		{
			Quaternion v(0,vector.data[0],vector.data[1],vector.data[2]);
			v=(this->GetAdjoint())*v*(*this);
			return Vector3(v.data[1],v.data[2],v.data[3]);
		}
			//通过四元数把一个世界坐标系向量转化为随体坐标系向量
		void Quaternion::TransformInverseVector(Vector3& vector)const
		{
			Quaternion v(0,vector.data[0],vector.data[1],vector.data[2]);
			v=(this->GetAdjoint())*v*(*this);
			vector.data[0]=v.data[1];
			vector.data[1]=v.data[2];
			vector.data[2]=v.data[3];
		}
			//通过四元数和随体坐标系原点在世界坐标系中的位置把 世界坐标系中的一点转化为随体坐标系的一点
		Vector3 Quaternion::TransformInversePointReturn(const Vector3& point,const Vector3& position)const
		{
			Quaternion v(0,point.data[0],point.data[1],point.data[2]);
			v=(this->GetAdjoint())*v*(*this);
			return Vector3(v.data[1]-position.data[0],v.data[2]-position.data[1],v.data[3]-position.data[2]);
		}
		void Quaternion::TransformInversePoint(Vector3& point,const Vector3& position)const
		{
			Quaternion v(0,point.data[0],point.data[1],point.data[2]);
			v=(this->GetAdjoint())*v*(*this);
			point.data[0]=v.data[1]-position.data[0];
			point.data[1]=v.data[2]-position.data[1];
			point.data[2]=v.data[3]-position.data[2];
		}

		//通过一个3阶方阵设置四元数
	void Quaternion::SetOrientation(const Matrix3& m)
	{
		*this=m.GetQuaternion();
	}
		//得到参数四元数的旋转矩阵
	Matrix3 Quaternion::GetMatrix3(const Quaternion& q)
	{
		Matrix3 temp;
		temp.data[0]=1-2*(q.data[2]*q.data[2]+q.data[3]*q.data[3]);
		temp.data[1]=2*(q.data[1]*q.data[2]+q.data[0]*q.data[3]);
		temp.data[2]=2*(q.data[1]*q.data[3]-q.data[0]*q.data[2]);
		temp.data[3]=2*(q.data[1]*q.data[2]-q.data[0]*q.data[3]);
		temp.data[4]=1-2*(q.data[1]*q.data[1]+q.data[3]*q.data[3]);
		temp.data[5]=2*(q.data[2]*q.data[3]+q.data[0]*q.data[1]);
		temp.data[6]=2*(q.data[1]*q.data[3]+q.data[0]*q.data[2]);
		temp.data[7]=2*(q.data[2]*q.data[3]-q.data[0]*q.data[1]);
		temp.data[8]=1-2*(q.data[1]*q.data[1]+q.data[2]*q.data[2]);
		return temp;
	}
			//得到该四元数的旋转矩阵
	Matrix3 Quaternion::GetMatrix3()const
	{
		return GetMatrix3(*this);
	}
//Quaternion>

//<Matrix3
		//3阶方阵相乘
	Matrix3 Matrix3::operator*(const Matrix3 &other)const
	{
		return Matrix3(
			data[0]*other.data[0]+data[1]*other.data[3]+data[2]*other.data[6],
			data[0]*other.data[1]+data[1]*other.data[4]+data[2]*other.data[7],
			data[0]*other.data[2]+data[1]*other.data[5]+data[2]*other.data[8],

			data[3]*other.data[0]+data[4]*other.data[3]+data[5]*other.data[6],
			data[3]*other.data[1]+data[4]*other.data[4]+data[5]*other.data[7],
			data[3]*other.data[2]+data[4]*other.data[5]+data[5]*other.data[8],

			data[6]*other.data[0]+data[7]*other.data[3]+data[8]*other.data[6],
			data[6]*other.data[1]+data[7]*other.data[4]+data[8]*other.data[7],
			data[6]*other.data[2]+data[7]*other.data[5]+data[8]*other.data[8]);
	}
		//与3阶方阵相乘
	void Matrix3::operator*=(const Matrix3 &other)
	{
		*this=(*this)*other;
	}
		//与列向量相乘 结果是一个向量
	Vector3 Matrix3::operator*(const Vector3 &vector)const
	{
		return Vector3(
				vector.data[0]*data[0]+vector.data[1]*data[1]+vector.data[2]*data[2],
				vector.data[0]*data[3]+vector.data[1]*data[4]+vector.data[2]*data[5],
				vector.data[0]*data[6]+vector.data[1]*data[7]+vector.data[2]*data[8]);
	}
		//与常量相乘
	Matrix3 Matrix3::operator*(const real scalar)const
	{
		Matrix3 temp;
		for(int i=0;i<9;i++)
			temp.data[i]=this->data[i]*scalar;
		return temp;
	}
		//与常量相乘
	void Matrix3::operator*=(const real scalar)
	{
		for(int i=0;i<9;i++)
			this->data[i]*=scalar;
	}
		//与3阶方阵相加
	Matrix3 Matrix3::operator+(const Matrix3 &other)const
	{
		Matrix3 temp;
		for(int i=0;i<9;i++)
			temp.data[i]=this->data[i]+other.data[i];
		return temp;
	}
		//与3阶方阵相加
	void Matrix3::operator+=(const Matrix3 &other)
	{
		for(int i=0;i<9;i++)
			this->data[i]+=other.data[i];
	}

		//设置this矩阵为自身的转置矩阵
	void Matrix3::SetTranspose()
	{
		SetTranspose(*this);
	}
		//设置this矩阵为参数矩阵的转置矩阵
	void Matrix3::SetTranspose(const Matrix3 &m)
	{
		data[0] = m.data[0];
		data[1] = m.data[3];
		data[2] = m.data[6];
		data[3] = m.data[1];
		data[4] = m.data[4];
		data[5] = m.data[7];
		data[6] = m.data[2];
		data[7] = m.data[5];
		data[8] = m.data[8];
	}
		//返回this矩阵的转置
	Matrix3 Matrix3::Transpose()const
	{
		Matrix3 temp;
		temp.SetTranspose(*this);
		return temp;
	}

		//设置this矩阵为自身的逆矩阵
	void Matrix3::SetInverse()
	{
		SetInverse(*this);
	}
		//设置this矩阵为参数矩阵的逆矩阵
	void Matrix3::SetInverse(const Matrix3 &m)
	{
		real t48=m.data[4]*m.data[8];
		real t57=m.data[5]*m.data[7];
		real t38=m.data[3]*m.data[8];
		real t56=m.data[5]*m.data[6];
		real t37=m.data[3]*m.data[7];
		real t46=m.data[4]*m.data[6];

		real det=m.data[0]*t48-m.data[0]*t57-m.data[1]*t38+m.data[1]*t56+m.data[2]*t37-m.data[2]*t46;

		if(det==(real)0)return;
		det=1/det;
		data[0]=(t48-t57)*det;
		data[1]=(m.data[2]*m.data[7]-m.data[1]*m.data[8])*det;
		data[2]=(m.data[1]*m.data[5]-m.data[2]*m.data[4])*det;
		data[3]=(t56-t38)*det;
		data[4]=(m.data[0]*m.data[8]-m.data[2]*m.data[6])*det;
		data[5]=(m.data[2]*m.data[3]-m.data[0]*m.data[5])*det;
		data[6]=(t37-t46)*det;
		data[7]=(m.data[1]*m.data[6]-m.data[0]*m.data[7])*det;
		data[8]=(m.data[0]*m.data[4]-m.data[1]*m.data[3])*det;

	}
		//返回this矩阵的逆矩阵
	Matrix3 Matrix3::GetInverse()const
	{
		Matrix3 temp;
		temp.SetInverse(*this);
		return temp;
	}

			//设置对角线值
	void Matrix3::SetDiagonal(real a0,real a4,real a8)
	{
		SetInertiaTensorCoeffs(a0,a4,a8);
	}
		//用3个列向量设置一个矩阵
	void Matrix3::SetComponents(const Vector3 &comp0,const Vector3 &comp1,const Vector3 &comp2)
	{
		data[0]=comp0.data[0];
		data[1]=comp1.data[0];
		data[2]=comp2.data[0];
		data[3]=comp0.data[1];
		data[4]=comp1.data[1];
		data[5]=comp2.data[1];
		data[6]=comp0.data[2];
		data[7]=comp1.data[2];
		data[8]=comp2.data[2];
	}
		//设置斜交对称矩阵
	void Matrix3::SetSkewSymmetric(const Vector3 vector)
	{
		data[0]=data[4]=data[8]=0;
		data[1]=-vector.data[2];
		data[2]=vector.data[1];
		data[3]=vector.data[2];
		data[5]=-vector.data[0];
		data[6]=-vector.data[1];
		data[7]=vector.data[0];
	}
		//设置惯性张量
	void Matrix3::SetInertiaTensorCoeffs(real ix,real iy,real iz,real ixy,real ixz,real iyz)
	{
		data[0]=ix;
		data[4]=iy;
		data[8]=iz;
		data[1]=data[3]=-ixy;
		data[2]=data[6]=-ixz;
		data[5]=data[7]=-iyz;
	}

		//得到矩阵的某一行，返回一个向量
	Vector3 Matrix3::GetRowVector(int i)const
	{
		return Vector3(data[i*3],data[i*3+1],data[i*3+2]);
	}
		//得到局针的某一列，返回一个向量
	Vector3 Matrix3::GetAxisVector(int i)const
	{
		return Vector3(data[i],data[i+3],data[i+6]);
	}
		//把一个列向量用该矩阵变换
	Vector3 Matrix3::Transform(const Vector3 &vector)const
	{
		return *this*vector;
	}
		//把一个列向量用该矩阵的转置变换 或者把一个行向量用该矩阵变换
	Vector3 Matrix3::TransformTranspose(const Vector3 &vector)const
	{
		return this->Transpose()*vector;
	}

		//用一个四元数设置三维矩阵 即把一个四元数转化为其对应的变换矩阵
	void Matrix3::SetOrientation(const Quaternion &q)
	{
		/*data[0]=1-2*(q.data[2]*q.data[2]+q.data[3]*q.data[3]);
		data[1]=2*(q.data[1]*q.data[2]+q.data[0]*q.data[3]);
		data[2]=2*(q.data[1]*q.data[3]-q.data[0]*q.data[2]);
		data[3]=2*(q.data[1]*q.data[2]-q.data[0]*q.data[3]);
		data[4]=1-2*(q.data[1]*q.data[1]+q.data[3]*q.data[3]);
		data[5]=2*(q.data[2]*q.data[3]+q.data[0]*q.data[1]);
		data[6]=2*(q.data[1]*q.data[3]+q.data[0]*q.data[2]);
		data[7]=2*(q.data[2]*q.data[3]-q.data[0]*q.data[1]);
		data[8]=1-2*(q.data[1]*q.data[1]+q.data[2]*q.data[2]);*/
		data[0]=1-2*(q.data[2]*q.data[2]+q.data[3]*q.data[3]);
		data[1]=2*(q.data[1]*q.data[2]-q.data[0]*q.data[3]);
		data[2]=2*(q.data[1]*q.data[3]+q.data[0]*q.data[2]);
		data[3]=2*(q.data[1]*q.data[2]+q.data[0]*q.data[3]);
		data[4]=1-2*(q.data[1]*q.data[1]+q.data[3]*q.data[3]);
		data[5]=2*(q.data[2]*q.data[3]-q.data[0]*q.data[1]);
		data[6]=2*(q.data[1]*q.data[3]-q.data[0]*q.data[2]);
		data[7]=2*(q.data[2]*q.data[3]+q.data[0]*q.data[1]);
		data[8]=1-2*(q.data[1]*q.data[1]+q.data[2]*q.data[2]);

	}
		//把参数矩阵转换为一个四元数
	Quaternion Matrix3::GetQuaternion(const Matrix3 &m)
	{
		real Tr = m.data[0] +m.data[4]+m.data[8];
		real temp = 0.0;
		Quaternion q;
		if(Tr > 0.0)
		{
			Tr=Tr+1;
			temp=(real)0.5/(sqrt(Tr));
			q.data[0]=Tr*temp;
			q.data[1]=(m.data[7]-m.data[5])*temp;
			q.data[2]=(m.data[2]-m.data[6])*temp;
			q.data[3]=(m.data[3]-m.data[1])*temp;
		}
		else
		{
			if(m.data[0]>m.data[4]&&m.data[0]>m.data[8])
			{
				Tr=m.data[0]-m.data[4]-m.data[8]+1;
				temp=((real)0.5)/(sqrt(Tr));
				q.data[1]=Tr*temp;
				q.data[0]=(m.data[7]-m.data[5])*temp;
				q.data[2]=(m.data[1]+m.data[3])*temp;
				q.data[3]=(m.data[2]+m.data[6])*temp;
			}
			else
				if(m.data[4]>m.data[8])
				{
					Tr=m.data[4]-m.data[0]-m.data[8]+1;
					temp=((real)0.5)/(sqrt(Tr));
					q.data[2]=Tr*temp;
					q.data[0]=(m.data[2]-m.data[6])*temp;
					q.data[1]=(m.data[1]+m.data[3])*temp;
					q.data[3]=(m.data[7]+m.data[5])*temp;
				}
				else
				{
					Tr=m.data[8]-m.data[4]-m.data[0]+1;
					temp=((real)0.5)/(sqrt(Tr));
					q.data[3]=Tr*temp;
					q.data[0]=(m.data[3]-m.data[1])*temp;
					q.data[1]=(m.data[2]+m.data[6])*temp;
					q.data[2]=(m.data[5]+m.data[7])*temp;
				}
		}
		return q;
	}
		//返回该矩阵的四元数形式
	Quaternion Matrix3::GetQuaternion()const
	{
		return GetQuaternion(*this);
	}
		//矩阵的线性插值
	Matrix3 Matrix3::LinearInterpolate(const Matrix3& a,const Matrix3& b,real prop)
	{
		Matrix3 temp;
		for(int i=0;i<9;i++)
			temp.data[i]=a.data[i]*(1-prop)+b.data[i]*prop;
		return temp;
	}
//Matrix3>

//< Matrix4
	void Matrix4::SetDiagonal(real a,real b,real c)
	{
		data[0]=a;
		data[5]=b;
		data[10]=c;
	}
	Vector3 Matrix4::operator*(const Vector3 &vector)const//该矩阵M*（[vector.x,vector.y,vector.z,1]的转置）
	{
		return Vector3(vector.data[0]*data[0]+vector.data[1]*data[1]+vector.data[2]*data[2]+data[3],
								vector.data[0]*data[4]+vector.data[1]*data[5]+vector.data[2]*data[6]+data[7],
								vector.data[0]*data[8]+vector.data[1]*data[9]+vector.data[2]*data[10]+data[11]);
	}
	Matrix4 Matrix4::operator*(const Matrix4 &other)const
	{
		Matrix4 result;
		result.data[0] = (other.data[0]*data[0]) + (other.data[4]*data[1]) + (other.data[8]*data[2]);
		result.data[4] = (other.data[0]*data[4]) + (other.data[4]*data[5]) + (other.data[8]*data[6]);
		result.data[8] = (other.data[0]*data[8]) + (other.data[4]*data[9]) + (other.data[8]*data[10]);

		result.data[1] = (other.data[1]*data[0]) + (other.data[5]*data[1]) + (other.data[9]*data[2]);
		result.data[5] = (other.data[1]*data[4]) + (other.data[5]*data[5]) + (other.data[9]*data[6]);
		result.data[9] = (other.data[1]*data[8]) + (other.data[5]*data[9]) + (other.data[9]*data[10]);

		result.data[2] = (other.data[2]*data[0]) + (other.data[6]*data[1]) + (other.data[10]*data[2]);
		result.data[6] = (other.data[2]*data[4]) + (other.data[6]*data[5]) + (other.data[10]*data[6]);
		result.data[10] = (other.data[2]*data[8]) + (other.data[6]*data[9]) + (other.data[10]*data[10]);

		result.data[3] = (other.data[3]*data[0]) + (other.data[7]*data[1]) + (other.data[11]*data[2]) + data[3];
		result.data[7] = (other.data[3]*data[4]) + (other.data[7]*data[5]) + (other.data[11]*data[6]) + data[7];
		result.data[11] = (other.data[3]*data[8]) + (other.data[7]*data[9]) + (other.data[11]*data[10]) + data[11];

		return result;
	}
	Vector3 Matrix4::Transform(const Vector3 &vector)const
	{
		return (*this)*vector;
	}
	real Matrix4::GetDeterminant()const
	{
		return data[8]*data[5]*data[2]+
			data[4]*data[9]*data[2]+
			data[8]*data[1]*data[6]-
			data[0]*data[9]*data[6]-
			data[4]*data[1]*data[10]+
			data[0]*data[5]*data[10];
	}
	void Matrix4::SetInverse(const Matrix4 & m)
	{
		real det = GetDeterminant();
		if (det == 0) return;
		det = ((real)1.0)/det;

		data[0] = (-m.data[9]*m.data[6]+m.data[5]*m.data[10])*det;
		data[4] = (m.data[8]*m.data[6]-m.data[4]*m.data[10])*det;
		data[8] = (-m.data[8]*m.data[5]+m.data[4]*m.data[9]* m.data[15])*det;

		data[1] = (m.data[9]*m.data[2]-m.data[1]*m.data[10])*det;
		data[5] = (-m.data[8]*m.data[2]+m.data[0]*m.data[10])*det;
		data[9] = (m.data[8]*m.data[1]-m.data[0]*m.data[9]* m.data[15])*det;

		data[2] = (-m.data[5]*m.data[2]+m.data[1]*m.data[6]* m.data[15])*det;
		data[6] = (+m.data[4]*m.data[2]-m.data[0]*m.data[6]* m.data[15])*det;
		data[10] = (-m.data[4]*m.data[1]+m.data[0]*m.data[5]* m.data[15])*det;

		data[3] = (m.data[9]*m.data[6]*m.data[3]
		-m.data[5]*m.data[10]*m.data[3]
		-m.data[9]*m.data[2]*m.data[7]
		+m.data[1]*m.data[10]*m.data[7]
		+m.data[5]*m.data[2]*m.data[11]
		-m.data[1]*m.data[6]*m.data[11])*det;
		data[7] = (-m.data[8]*m.data[6]*m.data[3]
		+m.data[4]*m.data[10]*m.data[3]
		+m.data[8]*m.data[2]*m.data[7]
		-m.data[0]*m.data[10]*m.data[7]
		-m.data[4]*m.data[2]*m.data[11]
		+m.data[0]*m.data[6]*m.data[11])*det;
		data[11] =(m.data[8]*m.data[5]*m.data[3]
		-m.data[4]*m.data[9]*m.data[3]
		-m.data[8]*m.data[1]*m.data[7]
		+m.data[0]*m.data[9]*m.data[7]
		+m.data[4]*m.data[1]*m.data[11]
		-m.data[0]*m.data[5]*m.data[11])*det;
	}
	Matrix4 Matrix4::GetInverse()const
	{
		Matrix4 result;
		result.SetInverse(*this);
		return result;
	}
	void Matrix4::SetInverse()
	{
		SetInverse(*this);
	}
	Vector3 Matrix4::TransformDirection(const Vector3 &vector)const
	{
		return Vector3(
			vector.data[0] * data[0] +
			vector.data[1] * data[1] +
			vector.data[2] * data[2],

			vector.data[0] * data[4] +
			vector.data[1] * data[5] +
			vector.data[2] * data[6],

			vector.data[0] * data[8] +
			vector.data[1] * data[9] +
			vector.data[2] * data[10]
		);
	}
	Vector3 Matrix4::TransformInverseDirection(const Vector3 &vector)const
	{
		return Vector3(
			vector.data[0] * data[0] +
			vector.data[1] * data[4] +
			vector.data[2] * data[8],

			vector.data[0] * data[1] +
			vector.data[1] * data[5] +
			vector.data[2] * data[9],

			vector.data[0] * data[2] +
			vector.data[1] * data[6] +
			vector.data[2] * data[10]
		);
	}
	Vector3 Matrix4::TransformInverse(const Vector3 &vector)const
	{
		Vector3 tmp = vector;
		tmp.data[0] -= data[3];
		tmp.data[1] -= data[7];
		tmp.data[2] -= data[11];
		return Vector3(
			tmp.data[0] * data[0] +
			tmp.data[1] * data[4] +
			tmp.data[2] * data[8],

			tmp.data[0] * data[1] +
			tmp.data[1] * data[5] +
			tmp.data[2] * data[9],

			tmp.data[0] * data[2] +
			tmp.data[1] * data[6] +
			tmp.data[2] * data[10]
		);
	}
	Vector3 Matrix4::GetAxisVector(int i)const
	{
		return Vector3(data[i], data[i+4], data[i+8]);
	}
	void Matrix4::SetOrientationAndPos(const Quaternion &orientation,const Vector3 &pos)
	{
		data[0] = 1-2*orientation.data[2]*orientation.data[2]-
			2*orientation.data[3]*orientation.data[3];
		data[1] = 2*orientation.data[1]*orientation.data[2] -
			2*orientation.data[0]*orientation.data[3];
		data[2] = 2*orientation.data[1]*orientation.data[3] +
			2*orientation.data[0]*orientation.data[2];
		data[3] = pos.data[0];

		data[4] = 2*orientation.data[1]*orientation.data[2] +
			2*orientation.data[0]*orientation.data[3];
		data[5] = 1-2*orientation.data[1]*orientation.data[1]-
			2*orientation.data[3]*orientation.data[3];
		data[6] = 2*orientation.data[2]*orientation.data[3] -
			2*orientation.data[0]*orientation.data[1];
		data[7] = pos.data[1];

		data[8] = 2*orientation.data[1]*orientation.data[3] -
			2*orientation.data[0]*orientation.data[2];
		data[9] = 2*orientation.data[2]*orientation.data[3] +
			2*orientation.data[0]*orientation.data[1];
		data[10] = 1-2*orientation.data[1]*orientation.data[1]-
			2*orientation.data[2]*orientation.data[2];
		data[11] = pos.data[2];
	}
		//根据给定的3阶方阵和位置设定齐次变换矩阵
	void Matrix4::SetOrientationAndPos(const Matrix3 &m,const Vector3 &pos)
	{
		data[0]=m.data[0];
		data[1]=m.data[1];
		data[2]=m.data[2];
		data[3]=pos.data[0];

		data[4]=m.data[3];
		data[5]=m.data[4];
		data[6]=m.data[5];
		data[7]=pos.data[1];

		data[8]=m.data[6];
		data[9]=m.data[7];
		data[10]=m.data[8];
		data[11]=pos.data[2];
	}
	void Matrix4::GetGLMatrix4(real m[16])const
	{
		m[0] = data[0];
		m[1] = data[4];
		m[2] = data[8];
		m[3] = (real)0;

		m[4] = data[1];
		m[5] = data[5];
		m[6] = data[9];
		m[7] = (real)0;

		m[8] = data[2];
		m[9] = data[6];
		m[10] = data[10];
		m[11] = (real)0;

		m[12] = data[3];
		m[13] = data[7];
		m[14] = data[11];
		m[15] = (real)1;
	}
	real *Matrix4::GetGLMatrix4Return(real m[16])const
	{
		m[0] = data[0];
		m[1] = data[4];
		m[2] = data[8];
		m[3] = (real)0;

		m[4] = data[1];
		m[5] = data[5];
		m[6] = data[9];
		m[7] = (real)0;

		m[8] = data[2];
		m[9] = data[6];
		m[10] = data[10];
		m[11] = (real)0;

		m[12] = data[3];
		m[13] = data[7];
		m[14] = data[11];
		m[15] = (real)1;

		return m;
	}
}