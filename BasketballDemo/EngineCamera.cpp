#include"EngineCamera.h"
void Camera::UpdateViewMatrix()
{
	this->EularAngularToMatrix();
	this->EularAngularToQuaternion();
	this->orientation=this->rotationMatrix.GetQuaternion();
	/*Vector3 deltaPosWorld=this->orientation.TransformVectorReturn(lastLocalMove);*/
	Vector3 deltaPosWorld=this->rotationMatrix.
	Transform(this->lastLocalMove);//把上次的摄像机坐标系中的位置增量转化成默认摄像头坐标系中的位置
	this->cameraPos+=deltaPosWorld;
	viewMatrix.SetOrientationAndPos(orientation,cameraPos);
}
void Camera::EularAngularToQuaternion()
{
	float cosz=cos(delta_x*0.5);float sinz=sin(delta_x*0.5);
	float cosx=cos(delta_y*0.5);float sinx=sin(delta_y*0.5);
	float cosy=cos(delta_z*0.5);float siny=sin(delta_z*0.5);
	orientation.data[0]=cosx*cosy*cosz+sinx*siny*sinz;
	orientation.data[3]=sinx*cosy*cosz-cosx*siny*sinz;
	orientation.data[1]=cosx*siny*cosz+sinx*cosy*sinz;
	orientation.data[2]=cosx*cosy*sinz-sinx*siny*cosz;
	
}
void Camera::EularAngularToMatrix()
{
	float sinx=sin(delta_x);float cosx=cos(delta_x);
	float siny=sin(delta_y);float cosy=cos(delta_y);
	float sinz=sin(delta_z);float cosz=cos(delta_z);
	/*rotationMatrix.data[0]=cosy*cosz;
	rotationMatrix.data[1]=cosy*sinz;
	rotationMatrix.data[2]=-1*siny;
	rotationMatrix.data[3]=sinx*siny*cosz-cosx*sinz;
	rotationMatrix.data[4]=sinx*siny*sinz+cosx*cosz;
	rotationMatrix.data[5]=sinx*cosy;
	rotationMatrix.data[6]=cosx*siny*cosz+sinx*sinz;
	rotationMatrix.data[7]=cosx*siny*sinz-sinx*cosz;
	rotationMatrix.data[8]=cosx*cosy;*/

	Matrix3 Rx(1,0,0,0,cosx,sinx,0,-sinx,cosx);
	Matrix3 Ry(cosy,0,-siny,0,1,0,siny,0,cosy);
	Matrix3 Rz(cosz,sinz,0,-sinz,cosz,0,0,0,1);
	rotationMatrix=Ry*Rx*Rz;//*rotationMatrix;
}
void Camera::UpdateKey()
{
	this->pInput->UpdateKeyboard();
	this->lastLocalMove.SetData(0,0,0);
	if(this->pInput->KeyDown(DIK_LEFT))
	{
		delta_z-=0.04;
		if(delta_z<-REAL_PI)delta_z+=2*REAL_PI;
	}
	if(this->pInput->KeyDown(DIK_RIGHT))
	{
		delta_z+=0.04;
		if(delta_z>REAL_PI)delta_z-=2*REAL_PI;
		
	}
	if(this->pInput->KeyDown(DIK_W))
		this->lastLocalMove.data[2]=1;
	if(this->pInput->KeyDown(DIK_S))
		this->lastLocalMove.data[2]=-1;
	if(this->pInput->KeyDown(DIK_A))
		this->lastLocalMove.data[0]=-1;
	if(this->pInput->KeyDown(DIK_D))
		this->lastLocalMove.data[0]=1;
	if(this->pInput->KeyDown(DIK_Q))
		this->lastLocalMove.data[1]=1;
	if(this->pInput->KeyDown(DIK_E))
		this->lastLocalMove.data[1]=-1;

}
void Camera::UpdateMouse()
{
	this->pInput->UpdateMouse();
	if(this->pInput->ButtonDown(0))
	{
		int dx,dy;
		this->pInput->GetMouseLastMove(dx,dy);
		delta_x-=(float)dy*0.002;
		delta_y-=(float)dx*0.002;
		if(abs(delta_x)>=REAL_PI)delta_x=-(delta_x/abs(delta_x))*REAL_PI;
		if(abs(delta_y)>=REAL_PI)delta_y=-(delta_y/abs(delta_y))*REAL_PI;
	}
}
void Camera::UpdateCamera()
{
	this->UpdateKey();
	this->UpdateMouse();
	this->UpdateViewMatrix();
	Vector3 viewPoint(0,0,1);
	Vector3 upVector(0,1,0);
	viewPoint=cameraPos+this->rotationMatrix.Transform(viewPoint);
	upVector=this->rotationMatrix.Transform(upVector);
	/*viewPoint=cameraPos+this->orientation.TransformVectorReturn(viewPoint);
	upVector=this->orientation.TransformVectorReturn(upVector);*/
	gluLookAt(cameraPos.data[0],cameraPos.data[1],-cameraPos.data[2],
		viewPoint.data[0],viewPoint.data[1],-viewPoint.data[2],
		upVector.data[0],upVector.data[1],-upVector.data[2]);//这里需要把左手坐标系中的坐标转换成右手坐标系中的坐标
}