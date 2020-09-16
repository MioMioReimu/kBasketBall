#include"3DS.h"
int inline TestFaceIndex(unsigned short a)
{
	a=a<<13;
	a=a>>13;
	return a;//�ж��Ƿ�Ϊ˳ʱ��
}
bool Load3DS::Import3DS(Model *pModel,char *FileName)
{
	pFilePointer=NULL;
	char Message[255]={0};
	pFilePointer=fopen(FileName,"rb");//�õ��ļ�ָ��
	if(!pFilePointer)
	{

		::sprintf(Message,"Can not find file: %s ",FileName);
		MessageBoxA(NULL,Message,"Error",MB_OK);
		return false;
	}
	Chunk *pCurrentChunk=new Chunk;//newһ��chunk
	ReadChunk(pCurrentChunk);			//�õ�3ds�ļ��ĵ�һ����
	if(pCurrentChunk->ID!=MAIN)		//�������main�飬����3DS�ļ�
	{
		::sprintf(Message,"%s is not a 3DS file !",FileName);
		MessageBoxA(NULL,Message,"Error",MB_OK);
		return false;
	}
	memset(pModel,0,sizeof(*pModel));
	ProcessMainChunk(pModel,pCurrentChunk);//��ȡ3DS�ļ�
	CalculateLocalPos(pModel);			//����������������ϵ�еĶ���
	CalculateNormals(pModel);			//��������Ķ��㷨����
	delete pCurrentChunk;
	fclose(pFilePointer);
	return true;
}
void Load3DS::ProcessMainChunk(Model *pModel,Chunk *pMainChunk)
{
	
	Chunk *pCurrentChunk=new Chunk;			//��ǰ�������ӿ�
	while(pMainChunk->bytesRead<pMainChunk->length)//���Main��δ���� ����ӿ�
	{
		ReadChunk(pCurrentChunk);//���ӿ�
		switch(pCurrentChunk->ID)//�����ӿ�IDѡ��
		{
		case MAIN_VERSION:		//
			{
			unsigned int version=0;
			pCurrentChunk->bytesRead += fread(&version, 1, pCurrentChunk->length - pCurrentChunk->bytesRead, pFilePointer);

			// ����ļ��汾�Ŵ���3������һ��������Ϣ
			if (version > 0x03)
				MessageBox(NULL, L"This 3DS file is over version 3 so it may load incorrectly", L"Warning", MB_OK);
			break;
			}
		case EDITOR://����ӿ��Ǳ༭���������༭�����ȡ
			ProcessEditorChunk(pModel,pCurrentChunk);
			break;
		case KEYFRAME://Ŀǰ��û�����ؼ�֡��
			fseek(pFilePointer,pCurrentChunk->length-pCurrentChunk->bytesRead,SEEK_CUR);
			pCurrentChunk->bytesRead=pCurrentChunk->length;
			break;
		default:	//����ʶ�Ŀ�
			fseek(pFilePointer,pCurrentChunk->length-pCurrentChunk->bytesRead,SEEK_CUR);//ֱ���Թ�
			pCurrentChunk->bytesRead=pCurrentChunk->length;
			break;
		}
		pMainChunk->bytesRead+=pCurrentChunk->bytesRead;//���������Ѿ���ȡ���ֽ���
	}
	delete pCurrentChunk;
};
void Load3DS::ProcessEditorChunk(Model *pModel,Chunk *pEditorChunk)
{
	unsigned int version=0;		//��ʱ�洢����汾����Ϣ
	Chunk* pCurrentChunk=new Chunk;//��ǰ�ӿ�
	Object newObject={0};			//�½�һ������
	Material newTexture={0};//�½�һ������
	while(pEditorChunk->bytesRead<pEditorChunk->length)
	{
		ReadChunk(pCurrentChunk);
		switch(pCurrentChunk->ID)
		{
		case MESH_VERSION://����汾
			
			pCurrentChunk->bytesRead+=fread(&version,1,4,pFilePointer);
			if (version > 0x03)
				MessageBox(NULL, L"This 3DS Mesh Version is over version 3 so it may load incorrectly", L"Warning", MB_OK);
			break;
		case OBJECT_BLOCK://�����
			pModel->numObjects++;//�������+1
			pModel->pObjects.push_back(newObject);//����һ�����嵽�ɱ�����
			::memset(&(pModel->pObjects[pModel->numObjects-1]),0,sizeof(Object));//��0
			pModel->pObjects[pModel->numObjects-1].isVisible=true;
			ProcessObjectBlockChunk(pModel,&(pModel->pObjects[pModel->numObjects-1]),pCurrentChunk);//���������Ķ�ȡ
			break;
		case MATERIAL://���ʿ�
			pModel->numMaterials++;//������+1
			pModel->pMaterials.push_back(newTexture);//����һ�����ʵ��ɱ䳤������
			ProcessMaterialChunk(pModel,
				&(pModel->pMaterials[pModel->numMaterials-1]),
				pCurrentChunk);//������ʵĶ�ȡ
			break;
		default:
			fseek(pFilePointer,pCurrentChunk->length-pCurrentChunk->bytesRead,SEEK_CUR);
			pCurrentChunk->bytesRead=pCurrentChunk->length;
			break;
		}
		pEditorChunk->bytesRead+=pCurrentChunk->bytesRead;//���±༭������Ѷ��ֽ���
	}
	delete pCurrentChunk;
}

void Load3DS::ProcessObjectBlockChunk(Model *pModel,Object* pCurrentObject,Chunk *pObjBlockChunk)
{
	this->ReadString(pCurrentObject->objectName,pObjBlockChunk);//��ȡ����������������
	Chunk *pCurrentChunk=new Chunk;
	while(pObjBlockChunk->bytesRead<pObjBlockChunk->length)
	{
		ReadChunk(pCurrentChunk);//��ȡ�������ӿ�
		switch(pCurrentChunk->ID)
		{
		case TRI_MESH://�����������
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
	Matrix3 coorSys;//��ʱ��Ҫ�ľ���
	Vector3 pos;//�������������ϵ��λ��
	unsigned short tempFB;
	float tempY;
	while(pTriMeshChunk->bytesRead<pTriMeshChunk->length)
	{
		ReadChunk(pCurrentChunk);//��ȡ�������������ӿ�
		switch(pCurrentChunk->ID)
		{
		case VERTICES_LIST://�����б� 
			{
			//����VERTICES_LIST�Ѿ���Ҷ��chunk������ֱ���������ȡ
			pCurrentChunk->bytesRead+=fread(&(pCurrentObject->numVerts),1,2,pFilePointer);
			pCurrentObject->pVerts=new Vector3[pCurrentObject->numVerts];
			//int temp=sizeof(pCurrentObject->pVerts);
			//::memset(pCurrentObject->pVerts,0,sizeof(pCurrentObject->pVerts));
			pCurrentChunk->bytesRead+=fread(pCurrentObject->pVerts,1,pCurrentChunk->length-pCurrentChunk->bytesRead,pFilePointer);
			//����3DS����ϵ�е�����ϵ�����OpenGL�в�һ��
			//�ⲿ�ִ����Ѿ������ˣ��ڵõ�����任����ʱ�Ѿ����ǵ����������
			for(int i=0;i<pCurrentObject->numVerts;i++)
			{
				tempY=pCurrentObject->pVerts[i].data[1];
				pCurrentObject->pVerts[i].data[1]=pCurrentObject->pVerts[i].data[2];
				pCurrentObject->pVerts[i].data[2]=-tempY;
			}
			break;
			}
		case FACES_DESCRIPTION://��������
			//�����ȡ�������������
			pCurrentChunk->bytesRead+=fread(&(pCurrentObject->numFaces),1,2,pFilePointer);//��ȡ���������
			pCurrentObject->pFaces=new FaceIndex[pCurrentObject->numFaces];//�½�һ�����嶥����������
			for(int i=0;i<pCurrentObject->numFaces;i++)
			{
				pCurrentChunk->bytesRead+=fread(&(pCurrentObject->pFaces[i]),1,sizeof(unsigned short)*4,pFilePointer);//��ȡ�����i���������Ϣ
				if(TestFaceIndex((unsigned short)pCurrentObject->pFaces[i].materialIndex))//����������Flag���������涥��˳������ʱ��
				{
					//������������
					tempFB=pCurrentObject->pFaces[i].b;
					pCurrentObject->pFaces[i].b=pCurrentObject->pFaces[i].c;
					pCurrentObject->pFaces[i].c=tempFB;
					pCurrentObject->pFaces[i].materialIndex=-1;//�������������������Ĳ���ID˳����������һ�£���Ϊ�������Ϣ����������������ӿ��У����������ò���Ӱ��
				}
				else
					pCurrentObject->pFaces[i].materialIndex=-1;
			}
			ProcessFacesDescriptionChunk(pModel,pCurrentObject,pCurrentChunk);//��������������ӿ�Ķ�ȡ
			break;
		case UV_COOR_LIST://����UV����
			pCurrentChunk->bytesRead+=fread(&(pCurrentObject->numTextureVert),1,2,pFilePointer);
			pCurrentObject->pTextureVerts=new Vector2[pCurrentObject->numTextureVert];
			pCurrentChunk->bytesRead+=fread(pCurrentObject->pTextureVerts,1,pCurrentChunk->length-pCurrentChunk->bytesRead,pFilePointer);
			break;
		case LOCAL_COOR_SYS://���������ϵ
			{
				fseek(pFilePointer,36,SEEK_CUR);
				pCurrentChunk->bytesRead+=36;
				pCurrentChunk->bytesRead+=fread(&pos,1,sizeof(float)*3,pFilePointer);
				tempY=pos.data[1];
				pos.data[1]=pos.data[2];
				pos.data[2]=-tempY;
				
				float m[9]={1,0,0,0 ,1,0,0,0, 1};		//
				pCurrentObject->LtoGMatrix=Matrix4(m,pos);//local��ȫ�� 
				
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
	char materialName[255];//�������ƻ���
	Chunk* pCurrentChunk=new Chunk;
	unsigned short n,iface;
	while(pFacesDescripChunk->bytesRead<pFacesDescripChunk->length)
	{
		ReadChunk(pCurrentChunk);
		switch(pCurrentChunk->ID)
		{
		case FACES_MATERIAL_LIST://��ȡ����ʿ�
			ReadString(materialName,pCurrentChunk);//��ȡ����ʿ��еĲ�������
			pCurrentChunk->bytesRead+=fread(&n,1,sizeof(unsigned short),pFilePointer);//��ȡ������ʿ�����������ʵ���ĸ���
			for(unsigned int i=0;i<pModel->pMaterials.size();i++)//�������������еĲ��� ���ڲ��ʿ���������ǰ�棬����������Ա�����������ִ���
			{
				if(strcmp(materialName,pModel->pMaterials[i].materialName)==0)//�Ƚϲ��������еĲ������ƺ͵�ǰ����ʿ������
				{
					while(n-->0)//����ÿ��ʹ�øò��ʵ��������
					{
						pCurrentChunk->bytesRead+=fread(&iface,1,2,pFilePointer);//��ȡʹ�øò��ʵ��������
						pCurrentObject->pFaces[iface].materialIndex=i;//���ø���Ĳ���ID
					}
					if(strlen(pModel->pMaterials[i].textureFileName)>0)
						pCurrentObject->hasTexture=true;//������ڸò���������������в���
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
			//pCurrentMaterial->textureID=pModel->numMaterials-1;//����ID��OpenGL���� 
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

void Load3DS::CalculateLocalPos(Model *pModel)//��ÿ������ת��Ϊ��������ϵ�е�����
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
void Load3DS::CalculateNormals(Model *pModel)//��������Ķ��㷨���� ��������ϵ�е�
{
	Vector3 Vab,Va,Vb,Vc;//������AB������A������B,����C
	Vector3 *Vabc; //�������
	float weight; //�涥��Ȩֵ
	Object *pObject;
	if(pModel->numObjects<=0)return ;
	for(unsigned int i=0;i<pModel->numObjects;i++)
	{
		pObject=&(pModel->pObjects[i]);
		Vabc=new Vector3[pObject->numFaces];
		Vector3 *pFaceNormals=new Vector3[pObject->numFaces];
		pObject->pNormals=new Vector3[pObject->numVerts];
		for(int j=0;j<pObject->numFaces;j++)		//��ÿ����ĵ�λ�����������
		{
			Va=pObject->pVerts[pObject->pFaces[j].a];
			Vb=pObject->pVerts[pObject->pFaces[j].b];
			Vc=pObject->pVerts[pObject->pFaces[j].c];
			Vab=Vb-Va;
			Vabc[j]=(Va+Vb+Vc)*(1.0/3.0);				//��j���������
			pFaceNormals[j]=pObject->pVerts[pObject->pFaces[j].c]-pObject->pVerts[pObject->pFaces[j].b];
			
			pFaceNormals[j]%=Vab;
			pFaceNormals[j].Normalize();					//��j���浥λ�淨��
		}

		for(int j=0;j<pObject->numVerts;j++)		//����ÿһ�����㣬����ÿһ���������ÿһ����
		{
			float totalweight=0.0;							//��j��������ܵ�Ȩ��ֵ
			Vector3 vSum(0.0,0.0,0.0);				//��j��������ܷ�����
			for(int k=0;k<pObject->numFaces;k++)	
			{
				if((pObject->pFaces[k].a==j)||(pObject->pFaces[k].b==j)||(pObject->pFaces[k].c==j))//����������ĳ��������������j �����
				{
					Vab=pObject->pVerts[j]-Vabc[k];				//�ö��㵽��k���������
					weight=1.0/Vab.SquareMagnitude();		//���õ�k����Ե�j�������Ȩֵ
					totalweight+=weight;								//�ö������Ȩ��ֵ
					vSum+=pFaceNormals[k]*weight;
				}
				
			}
			pObject->pNormals[j]=vSum*(1.0/totalweight);//����ö��㷨����
			pObject->pNormals [j].Normalize();				//��λ��
		}
		delete []pFaceNormals;
		delete []Vabc;
	}
}