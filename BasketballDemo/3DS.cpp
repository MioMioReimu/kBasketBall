#include"3DS.h"
int inline TestFaceIndex(unsigned short a)
{
	a=a<<13;
	a=a>>13;
	return a;//判断是否为顺时针
}
bool Load3DS::Import3DS(Model *pModel,char *FileName)
{
	pFilePointer=NULL;
	char Message[255]={0};
	pFilePointer=fopen(FileName,"rb");//得到文件指针
	if(!pFilePointer)
	{

		::sprintf(Message,"Can not find file: %s ",FileName);
		MessageBoxA(NULL,Message,"Error",MB_OK);
		return false;
	}
	Chunk *pCurrentChunk=new Chunk;//new一个chunk
	ReadChunk(pCurrentChunk);			//得到3ds文件的第一个块
	if(pCurrentChunk->ID!=MAIN)		//如果不是main块，则不是3DS文件
	{
		::sprintf(Message,"%s is not a 3DS file !",FileName);
		MessageBoxA(NULL,Message,"Error",MB_OK);
		return false;
	}
	memset(pModel,0,sizeof(*pModel));
	ProcessMainChunk(pModel,pCurrentChunk);//读取3DS文件
	CalculateLocalPos(pModel);			//计算物体随体坐标系中的顶点
	CalculateNormals(pModel);			//计算物体的顶点法向量
	delete pCurrentChunk;
	fclose(pFilePointer);
	return true;
}
void Load3DS::ProcessMainChunk(Model *pModel,Chunk *pMainChunk)
{
	
	Chunk *pCurrentChunk=new Chunk;			//当前操作的子块
	while(pMainChunk->bytesRead<pMainChunk->length)//如果Main块未读完 则读子块
	{
		ReadChunk(pCurrentChunk);//读子块
		switch(pCurrentChunk->ID)//根据子块ID选择
		{
		case MAIN_VERSION:		//
			{
			unsigned int version=0;
			pCurrentChunk->bytesRead += fread(&version, 1, pCurrentChunk->length - pCurrentChunk->bytesRead, pFilePointer);

			// 如果文件版本号大于3，给出一个警告信息
			if (version > 0x03)
				MessageBox(NULL, L"This 3DS file is over version 3 so it may load incorrectly", L"Warning", MB_OK);
			break;
			}
		case EDITOR://如果子块是编辑器块则进入编辑器块读取
			ProcessEditorChunk(pModel,pCurrentChunk);
			break;
		case KEYFRAME://目前还没有做关键帧块
			fseek(pFilePointer,pCurrentChunk->length-pCurrentChunk->bytesRead,SEEK_CUR);
			pCurrentChunk->bytesRead=pCurrentChunk->length;
			break;
		default:	//不认识的块
			fseek(pFilePointer,pCurrentChunk->length-pCurrentChunk->bytesRead,SEEK_CUR);//直接略过
			pCurrentChunk->bytesRead=pCurrentChunk->length;
			break;
		}
		pMainChunk->bytesRead+=pCurrentChunk->bytesRead;//更新主块已经读取的字节数
	}
	delete pCurrentChunk;
};
void Load3DS::ProcessEditorChunk(Model *pModel,Chunk *pEditorChunk)
{
	unsigned int version=0;		//临时存储网格版本的信息
	Chunk* pCurrentChunk=new Chunk;//当前子块
	Object newObject={0};			//新建一个物体
	Material newTexture={0};//新建一个材质
	while(pEditorChunk->bytesRead<pEditorChunk->length)
	{
		ReadChunk(pCurrentChunk);
		switch(pCurrentChunk->ID)
		{
		case MESH_VERSION://网格版本
			
			pCurrentChunk->bytesRead+=fread(&version,1,4,pFilePointer);
			if (version > 0x03)
				MessageBox(NULL, L"This 3DS Mesh Version is over version 3 so it may load incorrectly", L"Warning", MB_OK);
			break;
		case OBJECT_BLOCK://物体块
			pModel->numObjects++;//物体个数+1
			pModel->pObjects.push_back(newObject);//加入一个物体到可变数组
			::memset(&(pModel->pObjects[pModel->numObjects-1]),0,sizeof(Object));//置0
			pModel->pObjects[pModel->numObjects-1].isVisible=true;
			ProcessObjectBlockChunk(pModel,&(pModel->pObjects[pModel->numObjects-1]),pCurrentChunk);//进入物体块的读取
			break;
		case MATERIAL://材质块
			pModel->numMaterials++;//材质数+1
			pModel->pMaterials.push_back(newTexture);//加入一个材质到可变长度数组
			ProcessMaterialChunk(pModel,
				&(pModel->pMaterials[pModel->numMaterials-1]),
				pCurrentChunk);//进入材质的读取
			break;
		default:
			fseek(pFilePointer,pCurrentChunk->length-pCurrentChunk->bytesRead,SEEK_CUR);
			pCurrentChunk->bytesRead=pCurrentChunk->length;
			break;
		}
		pEditorChunk->bytesRead+=pCurrentChunk->bytesRead;//更新编辑器块的已读字节数
	}
	delete pCurrentChunk;
}

void Load3DS::ProcessObjectBlockChunk(Model *pModel,Object* pCurrentObject,Chunk *pObjBlockChunk)
{
	this->ReadString(pCurrentObject->objectName,pObjBlockChunk);//读取物体块中物体的名字
	Chunk *pCurrentChunk=new Chunk;
	while(pObjBlockChunk->bytesRead<pObjBlockChunk->length)
	{
		ReadChunk(pCurrentChunk);//读取物体块的子块
		switch(pCurrentChunk->ID)
		{
		case TRI_MESH://三角形网格块
			ProcessTriMeshChunk(pModel,pCurrentObject,pCurrentChunk);//
			break;
		default:
			fseek(pFilePointer,pCurrentChunk->length-pCurrentChunk->bytesRead,SEEK_CUR);
			pCurrentChunk->bytesRead=pCurrentChunk->length;
		}
		pObjBlockChunk->bytesRead+=pCurrentChunk->bytesRead;
	}
	delete pCurrentChunk;
}
void Load3DS::ProcessTriMeshChunk(Model *pModel,Object *pCurrentObject,Chunk *pTriMeshChunk)
{
	Chunk *pCurrentChunk=new Chunk;
	Matrix3 coorSys;//临时需要的矩阵
	Vector3 pos;//物体的自身坐标系的位置
	unsigned short tempFB;
	float tempY;
	while(pTriMeshChunk->bytesRead<pTriMeshChunk->length)
	{
		ReadChunk(pCurrentChunk);//读取三角形网格块的子块
		switch(pCurrentChunk->ID)
		{
		case VERTICES_LIST://顶点列表 
			{
			//由于VERTICES_LIST已经是叶子chunk，所以直接在这里读取
			pCurrentChunk->bytesRead+=fread(&(pCurrentObject->numVerts),1,2,pFilePointer);
			pCurrentObject->pVerts=new Vector3[pCurrentObject->numVerts];
			//int temp=sizeof(pCurrentObject->pVerts);
			//::memset(pCurrentObject->pVerts,0,sizeof(pCurrentObject->pVerts));
			pCurrentChunk->bytesRead+=fread(pCurrentObject->pVerts,1,pCurrentChunk->length-pCurrentChunk->bytesRead,pFilePointer);
			//由于3DS坐标系中的坐标系朝向跟OpenGL中不一样
			//这部分代码已经不用了，在得到物体变换矩阵时已经考虑到了这个问题
			for(int i=0;i<pCurrentObject->numVerts;i++)
			{
				tempY=pCurrentObject->pVerts[i].data[1];
				pCurrentObject->pVerts[i].data[1]=pCurrentObject->pVerts[i].data[2];
				pCurrentObject->pVerts[i].data[2]=-tempY;
			}
			break;
			}
		case FACES_DESCRIPTION://面描述块
			//下面读取面描述块的内容
			pCurrentChunk->bytesRead+=fread(&(pCurrentObject->numFaces),1,2,pFilePointer);//读取物体面个数
			pCurrentObject->pFaces=new FaceIndex[pCurrentObject->numFaces];//新建一个物体顶点索引数组
			for(int i=0;i<pCurrentObject->numFaces;i++)
			{
				pCurrentChunk->bytesRead+=fread(&(pCurrentObject->pFaces[i]),1,sizeof(unsigned short)*4,pFilePointer);//读取物体第i面的索引信息
				if(TestFaceIndex((unsigned short)pCurrentObject->pFaces[i].materialIndex))//如果物体的面Flag表明物体面顶点顺序不是逆时针
				{
					//交换顶点索引
					tempFB=pCurrentObject->pFaces[i].b;
					pCurrentObject->pFaces[i].b=pCurrentObject->pFaces[i].c;
					pCurrentObject->pFaces[i].c=tempFB;
					pCurrentObject->pFaces[i].materialIndex=-1;//这个变量本是用于物体的材质ID顺便在这里用一下，因为面材质信息在物体面描述块的子块中，所以这里用不会影响
				}
				else
					pCurrentObject->pFaces[i].materialIndex=-1;
			}
			ProcessFacesDescriptionChunk(pModel,pCurrentObject,pCurrentChunk);//进入面描述块的子块的读取
			break;
		case UV_COOR_LIST://物体UV坐标
			pCurrentChunk->bytesRead+=fread(&(pCurrentObject->numTextureVert),1,2,pFilePointer);
			pCurrentObject->pTextureVerts=new Vector2[pCurrentObject->numTextureVert];
			pCurrentChunk->bytesRead+=fread(pCurrentObject->pTextureVerts,1,pCurrentChunk->length-pCurrentChunk->bytesRead,pFilePointer);
			break;
		case LOCAL_COOR_SYS://物体的坐标系
			{
				fseek(pFilePointer,36,SEEK_CUR);
				pCurrentChunk->bytesRead+=36;
				pCurrentChunk->bytesRead+=fread(&pos,1,sizeof(float)*3,pFilePointer);
				tempY=pos.data[1];
				pos.data[1]=pos.data[2];
				pos.data[2]=-tempY;
				
				float m[9]={1,0,0,0 ,1,0,0,0, 1};		//
				pCurrentObject->LtoGMatrix=Matrix4(m,pos);//local到全局 
				
				pCurrentObject->GtoLMatrix=Matrix4(m,pos*-1);//
				break;
			}
		default:
			fseek(pFilePointer,pCurrentChunk->length-pCurrentChunk->bytesRead,SEEK_CUR);
			pCurrentChunk->bytesRead=pCurrentChunk->length;
		}
		assert(pCurrentChunk->bytesRead==pCurrentChunk->length);
		pTriMeshChunk->bytesRead+=pCurrentChunk->bytesRead;
	}
	delete pCurrentChunk;
};
void Load3DS::ProcessFacesDescriptionChunk(Model *pModel,Object *pCurrentObject,Chunk *pFacesDescripChunk)
{
	char materialName[255];//材质名称缓存
	Chunk* pCurrentChunk=new Chunk;
	unsigned short n,iface;
	while(pFacesDescripChunk->bytesRead<pFacesDescripChunk->length)
	{
		ReadChunk(pCurrentChunk);
		switch(pCurrentChunk->ID)
		{
		case FACES_MATERIAL_LIST://读取面材质块
			ReadString(materialName,pCurrentChunk);//读取面材质块中的材质名称
			pCurrentChunk->bytesRead+=fread(&n,1,sizeof(unsigned short),pFilePointer);//读取该面材质块中用这个材质的面的个数
			for(unsigned int i=0;i<pModel->pMaterials.size();i++)//遍历材质数组中的材质 由于材质块在物体块的前面，所以这里可以遍历而不会出现错误
			{
				if(strcmp(materialName,pModel->pMaterials[i].materialName)==0)//比较材质数组中的材质名称和当前面材质块的名称
				{
					while(n-->0)//遍历每个使用该材质的面的索引
					{
						pCurrentChunk->bytesRead+=fread(&iface,1,2,pFilePointer);//读取使用该材质的面的索引
						pCurrentObject->pFaces[iface].materialIndex=i;//设置该面的材质ID
					}
					if(strlen(pModel->pMaterials[i].textureFileName)>0)
						pCurrentObject->hasTexture=true;//如果存在该材质则设置物体具有材质
				}
			}
			break;
		default:
			fseek(pFilePointer,pCurrentChunk->length-pCurrentChunk->bytesRead,SEEK_CUR);
			pCurrentChunk->bytesRead=pCurrentChunk->length;
		}
		assert(pCurrentChunk->bytesRead==pCurrentChunk->length);
		pFacesDescripChunk->bytesRead+=pCurrentChunk->bytesRead;
	}
	delete pCurrentChunk;
}

void Load3DS::ProcessMaterialChunk(Model *pModel,Material* pCurrentMaterial,Chunk *pMaterialChunk)
{
	Chunk *pCurrentChunk=new Chunk;
	while(pMaterialChunk->bytesRead<pMaterialChunk->length)
	{
		ReadChunk(pCurrentChunk);
		switch(pCurrentChunk->ID)
		{
		case MATERIAL_NAME:
			ReadString(pCurrentMaterial->materialName,pCurrentChunk);
			break;
		case AMBIENT_COLOR:
			ProcessAmbientColorChunk(pModel,pCurrentMaterial,pCurrentChunk);
			break;
		case DIFFUSE_COLOR:
			ProcessDiffuseColorChunk(pModel,pCurrentMaterial,pCurrentChunk);
			break;
		case SPECULAR_COLOR:
			ProcessSpecularColorChunk(pModel,pCurrentMaterial,pCurrentChunk);
			break;
		//case SHININESS_PERCENT:
		case SHININESS_STRENGTH_PERCENT:
			ProcessShininessPercentChunk(pModel,pCurrentMaterial,pCurrentChunk);
			break;
		case TEXTURE_MAP_1:
			ProcessTextureMap1Chunk(pModel,pCurrentMaterial,pCurrentChunk);
		default:
			fseek(pFilePointer,pCurrentChunk->length-pCurrentChunk->bytesRead,SEEK_CUR);
			pCurrentChunk->bytesRead=pCurrentChunk->length;
		}
		pMaterialChunk->bytesRead+=pCurrentChunk->bytesRead;
	}
}
void Load3DS::ProcessAmbientColorChunk(Model *pModel,Material *pCurrentMaterial,Chunk *pAmbientChunk)
{
	Chunk *pCurrentChunk=new Chunk;
	while(pAmbientChunk->bytesRead<=pAmbientChunk->length)
	{
		ReadChunk(pCurrentChunk);
		switch(pCurrentChunk->ID)
		{
		case RGB_BYTE:
			ReadRGBByte(pCurrentMaterial->ambientColor,pCurrentChunk);
			for(int i=0;i<3;i++)
				pCurrentMaterial->diffuseColor[i]=pCurrentMaterial->ambientColor[i];
			break;
		default:
			fseek(pFilePointer,pCurrentChunk->length-pCurrentChunk->bytesRead,SEEK_CUR);
			pCurrentChunk->bytesRead=pCurrentChunk->length;
			break;
		}
		pAmbientChunk->bytesRead+=pCurrentChunk->bytesRead;
	}
	delete pCurrentChunk;
}
void Load3DS::ProcessDiffuseColorChunk(Model *pModel,Material *pCurrentMaterial,Chunk *pDiffuseChunk)
{
	Chunk *pCurrentChunk=new Chunk;
	while(pDiffuseChunk->bytesRead<=pDiffuseChunk->length)
	{
		ReadChunk(pCurrentChunk);
		switch(pCurrentChunk->ID)
		{
		case RGB_BYTE:
			ReadRGBByte(pCurrentMaterial->diffuseColor,pCurrentChunk);
			break;
		default:
			fseek(pFilePointer,pCurrentChunk->length-pCurrentChunk->bytesRead,SEEK_CUR);
			pCurrentChunk->bytesRead=pCurrentChunk->length;
			break;
		}
		pDiffuseChunk->bytesRead+=pCurrentChunk->bytesRead;
	}
	delete pCurrentChunk;
}
void Load3DS::ProcessSpecularColorChunk(Model *pModel,Material *pCurrentMaterial,Chunk *pSpecularChunk)
{
	Chunk *pCurrentChunk=new Chunk;
	while(pSpecularChunk->bytesRead<=pSpecularChunk->length)
	{
		ReadChunk(pCurrentChunk);
		switch(pCurrentChunk->ID)
		{
		case RGB_BYTE:
			ReadRGBByte(pCurrentMaterial->specularColor,pCurrentChunk);
			break;
		default:
			fseek(pFilePointer,pCurrentChunk->length-pCurrentChunk->bytesRead,SEEK_CUR);
			pCurrentChunk->bytesRead=pCurrentChunk->length;
			break;
		}
		pSpecularChunk->bytesRead+=pCurrentChunk->bytesRead;
	}
	delete pCurrentChunk;
}
void Load3DS::ProcessShininessPercentChunk(Model *pModel,Material *pCurrentMaterial,Chunk *pShiniPercChunk)
{
	Chunk *pCurrentChunk=new Chunk;
	while(pShiniPercChunk->bytesRead<pShiniPercChunk->length)
	{
		ReadChunk(pCurrentChunk);
		switch(pCurrentChunk->ID)
		{
		case PERCENT_INT:
			ReadPercentInt(pCurrentMaterial->shinniness,pCurrentChunk);
			pCurrentMaterial->shinniness=100-pCurrentMaterial->shinniness;
			break;
		default:
			fseek(pFilePointer,pCurrentChunk->length-pCurrentChunk->bytesRead,SEEK_CUR);
			pCurrentChunk->bytesRead=pCurrentChunk->length;
		}
		pShiniPercChunk->bytesRead+=pCurrentChunk->bytesRead;
	}
	delete pCurrentChunk;
}
void Load3DS::ProcessTextureMap1Chunk(Model *pModel,Material *pCurrentMaterial,Chunk *pTexMap1Chunk)
{
	Chunk* pCurrentChunk=new Chunk;
	while(pTexMap1Chunk->bytesRead<pTexMap1Chunk->length)
	{
		ReadChunk(pCurrentChunk);
		switch(pCurrentChunk->ID)
		{
		case MAP_FILE_NAME:
			ReadString(pCurrentMaterial->textureFileName,pCurrentChunk);
			//pCurrentMaterial->textureID=pModel->numMaterials-1;//纹理ID由OpenGL分配 
		default:
			fseek(pFilePointer,pCurrentChunk->length-pCurrentChunk->bytesRead,SEEK_CUR);
			pCurrentChunk->bytesRead=pCurrentChunk->length;
		}
		pTexMap1Chunk->bytesRead+=pCurrentChunk->bytesRead;
	}
	delete pCurrentChunk;
}

void Load3DS::ReadRGBByte(byte color[3],Chunk *pCurrentChunk)
{
	pCurrentChunk->bytesRead+=fread(color,1,sizeof(byte)*3,pFilePointer);
	assert(pCurrentChunk->bytesRead==pCurrentChunk->length);
}
void Load3DS::ReadPercentInt(unsigned short& perc,Chunk*pCurrentChunk)
{
	pCurrentChunk->bytesRead+=fread(&perc,1,sizeof(unsigned short),pFilePointer);
	assert(pCurrentChunk->bytesRead==pCurrentChunk->length);
}
void Load3DS::ReadChunk(Chunk *pChunk)
{
	pChunk->bytesRead=fread(&pChunk->ID,1,2,pFilePointer);
	pChunk->bytesRead+=fread(&pChunk->length,1,4,pFilePointer);
}
int Load3DS::ReadString(char *pBuffer,Chunk *pCurrentChunk)
{
	int index=0;
	fread(pBuffer,1,1,pFilePointer);
	while(*(pBuffer+index++)!=0)
	{
		fread(pBuffer+index,1,1,pFilePointer);
	}
	pCurrentChunk->bytesRead+=index;
	return index;
}

void Load3DS::CalculateLocalPos(Model *pModel)//把每个顶点转换为随体坐标系中的坐标
{
	Object *pObject;
	if(pModel->numObjects<=0)
		return;
	for(unsigned int i=0;i<pModel->numObjects;i++)
	{
		pObject=&(pModel->pObjects[i]);
		for(int j=0;j<pObject->numVerts;j++)
		{
			pObject->pVerts[j]=pObject->GtoLMatrix.Transform(pObject->pVerts[j]);
		}
	}
}
void Load3DS::CalculateNormals(Model *pModel)//计算物体的顶点法向量 随体坐标系中的
{
	Vector3 Vab,Va,Vb,Vc;//面向量AB，顶点A，顶点B,顶点C
	Vector3 *Vabc; //面的重心
	float weight; //面顶点权值
	Object *pObject;
	if(pModel->numObjects<=0)return ;
	for(unsigned int i=0;i<pModel->numObjects;i++)
	{
		pObject=&(pModel->pObjects[i]);
		Vabc=new Vector3[pObject->numFaces];
		Vector3 *pFaceNormals=new Vector3[pObject->numFaces];
		pObject->pNormals=new Vector3[pObject->numVerts];
		for(int j=0;j<pObject->numFaces;j++)		//求每个面的单位法向和面重心
		{
			Va=pObject->pVerts[pObject->pFaces[j].a];
			Vb=pObject->pVerts[pObject->pFaces[j].b];
			Vc=pObject->pVerts[pObject->pFaces[j].c];
			Vab=Vb-Va;
			Vabc[j]=(Va+Vb+Vc)*(1.0/3.0);				//第j个面的重心
			pFaceNormals[j]=pObject->pVerts[pObject->pFaces[j].c]-pObject->pVerts[pObject->pFaces[j].b];
			
			pFaceNormals[j]%=Vab;
			pFaceNormals[j].Normalize();					//第j个面单位面法向
		}

		for(int j=0;j<pObject->numVerts;j++)		//遍历每一个顶点，对于每一个顶点遍历每一个面
		{
			float totalweight=0.0;							//第j个顶点的总的权重值
			Vector3 vSum(0.0,0.0,0.0);				//第j个顶点的总法向量
			for(int k=0;k<pObject->numFaces;k++)	
			{
				if((pObject->pFaces[k].a==j)||(pObject->pFaces[k].b==j)||(pObject->pFaces[k].c==j))//如果发现面的某个顶点索引等于j 则计算
				{
					Vab=pObject->pVerts[j]-Vabc[k];				//该顶点到第k个面的向量
					weight=1.0/Vab.SquareMagnitude();		//设置第k个面对第j个顶点的权值
					totalweight+=weight;								//该顶点的总权重值
					vSum+=pFaceNormals[k]*weight;
				}
				
			}
			pObject->pNormals[j]=vSum*(1.0/totalweight);//计算该顶点法向量
			pObject->pNormals [j].Normalize();				//单位化
		}
		delete []pFaceNormals;
		delete []Vabc;
	}
}