#include"CFunc.h"
int TestFaceIndex(unsigned short a)
{
	a=a<<13;
	a=a>>13;
	return a;//�ж��Ƿ�Ϊ˳ʱ��
}