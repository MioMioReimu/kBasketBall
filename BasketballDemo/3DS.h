
#include"main.h"
#include"3DSChunk.h"
using namespace std;

struct FaceIndex	//������
{
	unsigned short a,b,c;//����ABC������
	short materialIndex;//����ID
};
struct Chunk		//3DS�ļ��Ŀ�
{
	unsigned short ID;//��ID
	unsigned int length;//��ĳ���
	unsigned int bytesRead;//Ŀǰ����鱻�����ֽ���
};

struct Object
{
	int numVerts;//�������
	int numFaces;//��ĸ���
	int numTextureVert;//����UV����
	//int numMaterials;//�����
	//int materialID;//�����
	bool isVisible;
	bool hasTexture;//�Ƿ��������
	char objectName[255];//��������
	Vector3 *pVerts;//��������ָ�� �洢�����������������ϵ������
	Vector3 *pNormals;//��������ָ��
	Vector2 *pTextureVerts;//UV��������ָ��
	FaceIndex *pFaces;//����������ָ��
	Matrix4 LtoGMatrix;//�����Local��Global�ı任����
	Matrix4 GtoLMatrix;//�����Global��Local�ı任����
};
struct Material
{
	char materialName[255];		//��������
	char textureFileName[255];	//�����ļ�����
	byte ambientColor[3];			//������ϵ��
	//byte ambientColorGamma[3];//���ڻ�����Ҫ��
	byte diffuseColor[3];				//������ϵ��
	//byte diffuseColorGamma[3];//���ڻ�����Ҫ��
	byte specularColor[3];			//���淴���ϵ��
	//byte specularColorGamma[3];//���ڻ�����Ҫ��
	unsigned short shinniness;	//�߹�ϵ��
	unsigned int textureID;						//����ID
	//float uTitle;//���ڻ�����Ҫ��
	//float vTitle;//���ڻ�����Ҫ��
	//float uOffset;//���ڻ�����Ҫ��
	//float vOffset;//���ڻ�����Ҫ��
};
struct Model
{
	unsigned int numObjects;
	unsigned int numMaterials;
	unsigned int numCameras;
	unsigned int numLights;
	vector<Material> pMaterials;
	vector<Object> pObjects;
	//vector<Camera> pCameras;
	//vector<Light> pLights;
};

class Load3DS
{
private:
	FILE* pFilePointer;
public:
	bool Import3DS(Model *pModel,char *FileName);
private:
	void ReadChunk(Chunk *pChunk);
	int ReadString(char *pBuffer,Chunk *pCurrentChunk);
	void ReadRGBByte(byte color[3],Chunk *pCurrentChunk);
	void ReadPercentInt(unsigned short& perc,Chunk*pCurrentChunk);

	void ProcessMainChunk(Model *pModel,Chunk *pMainChunk);
	void ProcessEditorChunk(Model *pModel,Chunk *pEditorChunk);
	
	void ProcessObjectBlockChunk(Model *pModel,Object *pCurrentObject,Chunk *pObjectBlockChunk);
	void ProcessTriMeshChunk(Model *pModel,Object *pCurrentObject,Chunk *pTriMeshChunk);
	void ProcessFacesDescriptionChunk(Model *pModel,Object *pCurrentObject,Chunk *pFacesDescripChunk);

	void ProcessMaterialChunk(Model *pModel,Material *pCurrentMaterial,Chunk *pMaterialChunk);
	void ProcessAmbientColorChunk(Model *pModel,Material *pCurrentMaterial,Chunk *pAmbientChunk);
	void ProcessDiffuseColorChunk(Model *pModel,Material *pCurrentMaterial,Chunk *pDiffuseChunk);
	void ProcessSpecularColorChunk(Model *pModel,Material *pCurrentMaterial,Chunk *pSpecularChunk);
	void ProcessShininessPercentChunk(Model *pModel,Material *pCurrentMaterial,Chunk *pShiniPercChunk);
	void ProcessTextureMap1Chunk(Model *pModel,Material *pCurrentMaterial,Chunk *pTexMap1Chunk);

private:
	void CalculateLocalPos(Model *pModel);//��ÿ������ת��Ϊ��������ϵ�е�����
	void CalculateNormals(Model *pModel);//��������Ķ��㷨���� ��������ϵ�е�
};