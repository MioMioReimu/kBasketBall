
#include"main.h"
#include"3DSChunk.h"
using namespace std;

struct FaceIndex	//面索引
{
	unsigned short a,b,c;//顶点ABC的索引
	short materialIndex;//材质ID
};
struct Chunk		//3DS文件的块
{
	unsigned short ID;//块ID
	unsigned int length;//块的长度
	unsigned int bytesRead;//目前这个块被读的字节数
};

struct Object
{
	int numVerts;//顶点个数
	int numFaces;//面的个数
	int numTextureVert;//纹理UV坐标
	//int numMaterials;//冗余的
	//int materialID;//冗余的
	bool isVisible;
	bool hasTexture;//是否具有纹理
	char objectName[255];//物体名称
	Vector3 *pVerts;//顶点数组指针 存储的是物体的随体坐标系的坐标
	Vector3 *pNormals;//法线数组指针
	Vector2 *pTextureVerts;//UV坐标数组指针
	FaceIndex *pFaces;//面索引数组指针
	Matrix4 LtoGMatrix;//物体的Local到Global的变换矩阵
	Matrix4 GtoLMatrix;//物体的Global到Local的变换矩阵
};
struct Material
{
	char materialName[255];		//材质名称
	char textureFileName[255];	//纹理文件名称
	byte ambientColor[3];			//环境光系数
	//byte ambientColorGamma[3];//现在还不需要的
	byte diffuseColor[3];				//漫反射系数
	//byte diffuseColorGamma[3];//现在还不需要的
	byte specularColor[3];			//镜面反射光系数
	//byte specularColorGamma[3];//现在还不需要的
	unsigned short shinniness;	//高光系数
	unsigned int textureID;						//纹理ID
	//float uTitle;//现在还不需要的
	//float vTitle;//现在还不需要的
	//float uOffset;//现在还不需要的
	//float vOffset;//现在还不需要的
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
	void CalculateLocalPos(Model *pModel);//把每个顶点转换为随体坐标系中的坐标
	void CalculateNormals(Model *pModel);//计算物体的顶点法向量 随体坐标系中的
};