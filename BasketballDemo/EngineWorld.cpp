#include"EngineWorld.h"
bool Resource::InitResource(char *FileName)
{
	Load3DS load3DStemp;
	return load3DStemp.Import3DS(&resourceModel,FileName);
}
void Resource::LoadTexture()
{
	AUX_RGBImageRec *pImage=NULL;
	FILE *pFile=NULL;
	for(unsigned int i=0;i<this->resourceModel.numMaterials;i++)
	{
		if(!resourceModel.pMaterials[i].textureFileName)
			continue;
		if((pFile=fopen(resourceModel.pMaterials[i].textureFileName,"rb"))==NULL)
		{
			char buf[255];
			sprintf_s(buf,"Unable to load BMP File: %s",resourceModel.pMaterials[i].textureFileName);
			MessageBoxA(NULL, buf, "Error", MB_OK);
			continue;
		};
		pImage=auxDIBImageLoadA(resourceModel.pMaterials[i].textureFileName);
		if(pImage==NULL)
			continue;

		glGenTextures(1,&(resourceModel.pMaterials[i].textureID));
		glBindTexture(GL_TEXTURE_2D,resourceModel.pMaterials[i].textureID);
		glPixelStorei(GL_UNPACK_ALIGNMENT,1);
		gluBuild2DMipmaps(GL_TEXTURE_2D,3,pImage->sizeX,
			pImage->sizeY,GL_RGB,GL_UNSIGNED_BYTE,pImage->data);

		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_NEAREST);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR_MIPMAP_LINEAR);
		glTexEnvf(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);

		if(pImage)
		{
			if(pImage->data)
				free(pImage->data);
			free(pImage);
		}
	};
}
void Resource::LoadMesh()
{
	int liststartIndex=glGenLists(this->resourceModel.numObjects);
	if(liststartIndex==0)
		return;
	drawList=new int[resourceModel.numObjects];
	for(int objectIndex=0;objectIndex<resourceModel.numObjects;objectIndex++)
	{
		this->drawList[objectIndex]=liststartIndex+objectIndex;
		glNewList(drawList[objectIndex],GL_COMPILE);
		glEnable(GL_TEXTURE_2D);
		glColor3ub(255,255,255);
		float temp1[3];
		float temp2[3];
		float temp3[3];
		if(objectIndex>=this->resourceModel.numObjects)return;
		Object &obj=resourceModel.pObjects[objectIndex];
		for(int i=0;i<obj.numFaces;i++)
		{
			FaceIndex &face=obj.pFaces[i];
			//set the material of the face i

			if(obj.hasTexture)
			{
				for(int colorindex=0;colorindex<3;colorindex++)
				{
					temp1[colorindex]=(float)resourceModel.pMaterials[face.materialIndex].ambientColor[colorindex]/(float)255;
					temp2[colorindex]=(float)resourceModel.pMaterials[face.materialIndex].diffuseColor[colorindex]/(float)255;
					temp3[colorindex]=(float)resourceModel.pMaterials[face.materialIndex].specularColor[colorindex]/(float)255;
				}
				//glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,temp1);
				//glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,temp2);
				glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,temp1);
				glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,temp2);
				glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,temp3);
				glMaterialf(GL_FRONT_AND_BACK,GL_SHININESS,resourceModel.pMaterials[face.materialIndex].shinniness);
			}
			//bind texture
			if(obj.hasTexture)
				glBindTexture(GL_TEXTURE_2D,resourceModel.pMaterials[face.materialIndex].textureID);
			//begin triangels
			glBegin(GL_TRIANGLES);
			if(obj.hasTexture)
				glTexCoord2f(obj.pTextureVerts[face.a].data[0],	obj.pTextureVerts[face.a].data[1]);
			glNormal3f(obj.pNormals[face.a].data[0],		obj.pNormals[face.a].data[1],	obj.pNormals[face.a].data[2]);
			glVertex3f(obj.pVerts[face.a].data[0],	obj.pVerts[face.a].data[1],		obj.pVerts[face.a].data[2]);

			if(obj.hasTexture)
				glTexCoord2f(obj.pTextureVerts[face.b].data[0],	obj.pTextureVerts[face.b].data[1]);
			glNormal3f(obj.pNormals[face.b].data[0],		obj.pNormals[face.b].data[1],	obj.pNormals[face.b].data[2]);
			glVertex3f(obj.pVerts[face.b].data[0],	obj.pVerts[face.b].data[1],		obj.pVerts[face.b].data[2]);

			if(obj.hasTexture)
				glTexCoord2f(obj.pTextureVerts[face.c].data[0],	obj.pTextureVerts[face.c].data[1]);
			glNormal3f(obj.pNormals[face.c].data[0],		obj.pNormals[face.c].data[1],	obj.pNormals[face.c].data[2]);
			glVertex3f(obj.pVerts[face.c].data[0],	obj.pVerts[face.c].data[1],		obj.pVerts[face.c].data[2]);

			glEnd();
		}
		glEndList();
	}
}
void Resource::DrawObject(unsigned objectIndex)
{
	glCallList(this->drawList[objectIndex]);
}

void Scene::InitScene(char *FileName)
{
	this->pResource=new Resource;
	pResource->InitResource(FileName);
	pResource->LoadTexture();
	pResource->LoadMesh();

#pragma region 这里写我们的初始化物体代码
	this->AddBall();
	this->AddBall1();
	Actor actor;
	RigidBody *pBody;
	CollisionPlane *plane;
	Vector3 Pos;
	Vector3 Radius;
	Matrix3 m(1,0,0,0,1,0,0,0,1);
	Vector3 Normal;
	//初始化底座1
	actor.objectIndex=1;
	actor.body=new RigidBody;
	actor.objectTobody.SetOrientationAndPos(m,Vector3(0,-0.5,0));

	pBody=actor.body;
	Pos.SetData(0,0.5,145);
	Radius.SetData(10,0.5,5);
	pBody->SetNotMoveBody(Pos,0.1,0.95,BoundingSphere(Pos,(Radius*2).Magnitude()),BOX,true);
	pBody->pPrimitive=new CollisionBox(Radius,pBody);

	this->AddActor(actor);
	this->simulate.PushBackBody(pBody);
	//初始化底座2
	actor.objectIndex=2;
	actor.body=new RigidBody;
	actor.objectTobody.SetOrientationAndPos(m,Vector3(0,-0.5,0));

	pBody=actor.body;
	Pos.SetData(0,0.5,-145);
	//Radius.SetData(10,0.5,5);
	pBody->SetNotMoveBody(Pos,0.1,0.95,BoundingSphere(Pos,(Radius*2).Magnitude()),BOX,true);
	pBody->pPrimitive=new CollisionBox(Radius,pBody);

	this->AddActor(actor);
	this->simulate.PushBackBody(pBody);

	//初始化圆台1
	actor.objectIndex=3;
	actor.body=new RigidBody;
	actor.objectTobody.SetOrientationAndPos(m,Vector3(0,-3,0));

	pBody=actor.body;
	Pos.SetData(0,4,145);
	Radius.SetData(2,3,2);
	pBody->SetNotMoveBody(Pos,0.1,0.95,BoundingSphere(Pos,(Radius*2).Magnitude()),BOX,true);
	pBody->pPrimitive=new CollisionBox(Radius,pBody);

	this->AddActor(actor);
	this->simulate.PushBackBody(pBody);

	//初始化圆台2
	actor.objectIndex=4;
	actor.body=new RigidBody;
	actor.objectTobody.SetOrientationAndPos(m,Vector3(0,-3,0));

	pBody=actor.body;
	Pos.SetData(0,4,-145);
	//Radius.SetData(2,3,2);
	pBody->SetNotMoveBody(Pos,0.1,0.95,BoundingSphere(Pos,(Radius*2).Magnitude()),BOX,true);
	pBody->pPrimitive=new CollisionBox(Radius,pBody);

	this->AddActor(actor);
	this->simulate.PushBackBody(pBody);

	//初始化柱子1
	actor.objectIndex=5;
	actor.body=new RigidBody;
	actor.objectTobody.SetOrientationAndPos(m,Vector3(0,-11.5,0));

	pBody=actor.body;
	Pos.SetData(0,18.5,145);
	Radius.SetData(1,11.5,1);
	pBody->SetNotMoveBody(Pos,0.1,0.95,BoundingSphere(Pos,(Radius*2).Magnitude()),BOX,true);
	pBody->pPrimitive=new CollisionBox(Radius,pBody);

	this->AddActor(actor);
	this->simulate.PushBackBody(pBody);

	//初始化柱子2
	actor.objectIndex=6;
	actor.body=new RigidBody;
	actor.objectTobody.SetOrientationAndPos(m,Vector3(0,-11.5,0));

	pBody=actor.body;
	Pos.SetData(0,18.5,-145);
	//Radius.SetData(1,11.5,1);
	pBody->SetNotMoveBody(Pos,0.1,0.95,BoundingSphere(Pos,(Radius*2).Magnitude()),BOX,true);
	pBody->pPrimitive=new CollisionBox(Radius,pBody);

	this->AddActor(actor);
	this->simulate.PushBackBody(pBody);

	//初始化篮板1
	actor.objectIndex=16;
	actor.body=new RigidBody;
	actor.objectTobody.SetOrientationAndPos(m,Vector3(0,0,-0.5));

	pBody=actor.body;
	Pos.SetData(0,36.5,139.44);
	Radius.SetData(10,7.5,0.5);
	pBody->SetNotMoveBody(Pos,0.1,0.95,BoundingSphere(Pos,(Radius*2).Magnitude()),BOX,true);
	pBody->pPrimitive=new CollisionBox(Radius,pBody);

	this->AddActor(actor);
	this->simulate.PushBackBody(pBody);

	//初始化篮板2
	actor.objectIndex=17;
	actor.body=new RigidBody;
	actor.objectTobody.SetOrientationAndPos(m,Vector3(0,0,0.5));

	pBody=actor.body;
	Pos.SetData(0,36.5,-140.301);
	//Radius.SetData(10,7.5,5);
	pBody->SetNotMoveBody(Pos,0.1,0.95,BoundingSphere(Pos,(Radius*2).Magnitude()),BOX,true);
	pBody->pPrimitive=new CollisionBox(Radius,pBody);

	this->AddActor(actor);
	this->simulate.PushBackBody(pBody);

	//初始化篮筐1
	actor.objectIndex=15;
	actor.body=new RigidBody;
	actor.objectTobody.Clear();
	pBody=actor.body;
	Pos=pResource->resourceModel.pObjects[actor.objectIndex].LtoGMatrix.GetAxisVector(3);
	pBody->SetNotMoveBody(Pos,0.4,0.95,BoundingSphere(Pos,2.3),CIRCLE,true);
	pBody->pPrimitive=new CollisionCircle(2.3,Vector3(0,1,0),pBody);

	this->AddActor(actor);
	this->simulate.PushBackBody(pBody);

	//初始化篮筐2
	actor.objectIndex=18;
	actor.body=new RigidBody;
	actor.objectTobody.Clear();
	pBody=actor.body;
	Pos=pResource->resourceModel.pObjects[actor.objectIndex].LtoGMatrix.GetAxisVector(3);
	pBody->SetNotMoveBody(Pos,0.4,0.95,BoundingSphere(Pos,2.3),CIRCLE,true);
	pBody->pPrimitive=new CollisionCircle(2.3,Vector3(0,1,0),pBody);

	this->AddActor(actor);
	this->simulate.PushBackBody(pBody);

	//初始化地面
	actor.objectIndex=0;
	actor.body=NULL;

	plane=new CollisionPlane(0.1,0.9,0,Vector3(0,1,0));

	this->AddActor(actor);
	this->simulate.PushBackPlane(plane);

	//初始化墙壁1
	actor.objectIndex=11;
	actor.body=NULL;

	plane=new CollisionPlane(0.1,0.9,-85,Vector3(-1,0,0));

	this->AddActor(actor);
	this->simulate.PushBackPlane(plane);

	//初始化墙壁2
	actor.objectIndex=12;
	actor.body=NULL;

	plane=new CollisionPlane(0.1,0.9,-85,Vector3(1,0,0));

	this->AddActor(actor);
	this->simulate.PushBackPlane(plane);

	//初始化墙壁3
	actor.objectIndex=13;
	actor.body=NULL;

	plane=new CollisionPlane(0.1,0.9,-160,Vector3(0,0,1));

	this->AddActor(actor);
	this->simulate.PushBackPlane(plane);

	//初始化墙壁4
	actor.objectIndex=14;
	actor.body=NULL;

	plane=new CollisionPlane(0.1,0.9,-160,Vector3(0,0,-1));

	this->AddActor(actor);
	this->simulate.PushBackPlane(plane);

	//初始化地面2
	actor.objectIndex=10;
	actor.body=NULL;

	this->AddActor(actor);

	//初始化其他不需要物理引擎模拟的物体
	actor.objectIndex=7;
	actor.body=NULL;

	this->AddActor(actor);

	actor.objectIndex=8;
	actor.body=NULL;
	this->AddActor(actor);
#pragma  endregion 这里写我们的初始化物体代码
}

Scene::~Scene()
{
	delete pResource;
}

void Scene::AddActor(RigidBody *body,unsigned int objectIndex,CollisionPrimitive *pPrimitive)
{
	Actor temp;temp.body=body;
	temp.objectIndex=objectIndex;
	temp.body->pPrimitive=pPrimitive;
	this->ActorList.push_back(temp);
}
void Scene::AddActor(Actor& actor)
{
	this->ActorList.push_back(actor);
}
void Scene::Render(float duration)
{
	this->simulate.RunSimulation(duration);
	unsigned int size=this->ActorList.size();
	for(unsigned i=0;i<size;i++)
	{
		glPushMatrix();
		float matrix[16];
		if(ActorList[i].body!=NULL)
		{

			glMultMatrixf(ActorList[i].body->GetGLTransformReturn(matrix));
			glMultMatrixf(ActorList[i].objectTobody.GetGLMatrix4Return(matrix));
		}
		else
		{
			glMultMatrixf(this->pResource->resourceModel.pObjects[ActorList[i].objectIndex].LtoGMatrix.GetGLMatrix4Return(matrix));
		}
		this->pResource->DrawObject(this->ActorList[i].objectIndex);
		glPopMatrix();
	}

}

void Scene::AddBall()
{
	Actor actor;
	RigidBody *pBody;
	Vector3 Pos=Vector3(2,30.5,81.599);
	real  Radius=1.23;
	Matrix3 m(1,0,0,0,1,0,0,0,1);
	//Vector3 Normal;

	actor.objectIndex=9;
	actor.body=new RigidBody;

	pBody=actor.body;
	float randomx=-1.0f*(float)rand()/RAND_MAX;
	pBody->SetImportAttributes(0.6,0.9,Pos,Vector3(randomx,60,50),Vector3(0,0,0),m,Vector3(0,-98,0),0.8,
		0.8,BoundingSphere(Pos,Radius),SPHERE,true,true);
	pBody->pPrimitive=new CollisionSphere(Radius,pBody);
	this->simulate.PushBackBody(pBody);
	this->AddActor(actor);
}
void Scene::AddBall1()
{
	Actor actor;
	RigidBody *pBody;
	float randomx=-1.0f*(float)rand()/RAND_MAX;
	Vector3 Pos=Vector3(randomx/2.0,48,134.498+randomx);
	real  Radius=1.23;
	Matrix3 m(1,0,0,0,1,0,0,0,1);
	Vector3 Normal;

	actor.objectIndex=9;
	actor.body=new RigidBody;

	pBody=actor.body;
	pBody->SetImportAttributes(0.6,0.9,Pos,Vector3(0,0,1),Vector3(0,0,0),m,Vector3(0,-98,0),0.8,
		0.8,BoundingSphere(Pos,Radius),SPHERE,true,true);
	pBody->pPrimitive=new CollisionSphere(Radius,pBody);
	this->simulate.PushBackBody(pBody);
	this->AddActor(actor);
}