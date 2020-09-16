#define DLL_PHYSICS_EXIST
#include"Physics.h"
namespace  Physics{
	Simulation::~Simulation()
	{
		this->ClearBodyList();
		this->contactList.clear();
		this->PlaneList.clear();
		this->potentialContactList.clear();
	}
	void Simulation::PushBackBody(RigidBody *body)
	{
		this->BodyList.push_back(body);
	}
	bool Simulation::PopBackBody()
	{
		if(this->BodyList.size()<=0)return false;
		this->BodyList.pop_back();
		return true;
	}
	void Simulation::ClearBodyList()
	{
		this->BodyList.clear();
	}
	void Simulation::PushBackPlane(CollisionPlane*plane)
	{
		this->PlaneList.push_back(plane);
	}
	bool Simulation::PopBackPlane()
	{
		if(this->PlaneList.size()<=0)return false;
		this->PlaneList.pop_back();
		return true;
	}
	void Simulation::ClearPlaneList()
	{
		this->PlaneList.clear();
	}
	void Simulation::GetPotentialContactList()
	{
		int size=this->BodyList.size();//得到物体数组个数-1
		if(size<=0)return;//如果物体个数小于等于0则返回
		BVHNode root(NULL,BodyList[0]->Volume,BodyList[0]);//创建BV树的根节点
		for(int i=1;i<size;i++)
		{
			root.Insert(BodyList[i],BodyList[i]->Volume);
		}
		root.GetPotentialContacts(&potentialContactList);
	}
	void Simulation::GetContactList()
	{
		int size=potentialContactList.size();
		Contact temp;
		for(unsigned int i=0;i<this->BodyList.size();i++)
		{
			this->BodyList[i]->CalculateDerivedData();
			this->BodyList[i]->pPrimitive->CalculateInternals();
		}
		size=this->BodyList.size();
		for(int i=0;i<size;i++)
		{
			for(int j=i+1;j<size;j++)
			{
				switch(BodyList[i]->PrimitiveType+BodyList[j]->PrimitiveType)
				{
				case SPHERE_SPHERE:
					{
						CollisionDetector::SphereAndSphere(*((CollisionSphere*)BodyList[i]->pPrimitive),*((CollisionSphere*)BodyList[j]->pPrimitive),&contactList);
						break;
					}
				case SPHERE_BOX:
					{
						if(BodyList[i]->PrimitiveType==SPHERE)
							CollisionDetector::BoxAndSphere(*((CollisionBox*)BodyList[j]->pPrimitive),*((CollisionSphere*)BodyList[i]->pPrimitive),&contactList);
						else
							CollisionDetector::BoxAndSphere(*((CollisionBox*)BodyList[i]->pPrimitive),*((CollisionSphere*)BodyList[j]->pPrimitive),&contactList);
						break;
					}
				case SPHERE_CIRCLE:
					{
						if(BodyList[i]->PrimitiveType==SPHERE)
							CollisionDetector::SphereAndCircle(*((CollisionSphere*)BodyList[i]->pPrimitive),*((CollisionCircle*)BodyList[j]->pPrimitive),&contactList);
						else
							CollisionDetector::SphereAndCircle(*((CollisionSphere*)BodyList[j]->pPrimitive),*((CollisionCircle*)BodyList[i]->pPrimitive),&contactList);
						break;
					}
				case BOX_BOX:
					{
						CollisionDetector::BoxAndBox(*((CollisionBox*)BodyList[i]->pPrimitive),*((CollisionBox*)BodyList[j]->pPrimitive),&contactList);
						break;
					}
				default:
					break;
				}
			}
		}

		/*
		///<处理物体间的碰撞
		for(int i=0;i<size;i++)
		{
			switch(potentialContactList[i].ContactType)
			{
			case SPHERE_SPHERE:
				{
					CollisionDetector::SphereAndSphere(*((CollisionSphere*)potentialContactList[i].body[0]->pPrimitive),
						*((CollisionSphere*)potentialContactList[i].body[1]->pPrimitive),&(this->contactList));
					break;
				}
			case SPHERE_BOX:
				{
					if(potentialContactList[i].body[0]->PrimitiveType==SPHERE)
						CollisionDetector::BoxAndSphere(*((CollisionBox*)potentialContactList[i].body[1]->pPrimitive),
						*((CollisionSphere*)potentialContactList[i].body[0]->pPrimitive),&(this->contactList));
					else
						CollisionDetector::BoxAndSphere(*((CollisionBox*)potentialContactList[i].body[0]->pPrimitive),
						*((CollisionSphere*)potentialContactList[i].body[1]->pPrimitive),&(this->contactList));
					break;
				}
			case SPHERE_CIRCLE:
				{
					if(potentialContactList[i].body[0]->PrimitiveType==SPHERE)
						CollisionDetector::SphereAndCircle(*((CollisionSphere*)potentialContactList[i].body[0]->pPrimitive),
						*((CollisionCircle*)potentialContactList[i].body[1]->pPrimitive),&(this->contactList));
					else
						CollisionDetector::SphereAndCircle(*((CollisionSphere*)potentialContactList[i].body[1]->pPrimitive),
						*((CollisionCircle*)potentialContactList[i].body[0]->pPrimitive),&(this->contactList));
					break;
				}
			case BOX_BOX:
				{
					CollisionDetector::BoxAndBox(*((CollisionBox*)potentialContactList[i].body[0]->pPrimitive),
						*((CollisionBox*)potentialContactList[i].body[1]->pPrimitive),&(this->contactList));
					break;
				}
			case BOX_CIRCLE:
				{
					break;
				}
			case CIRCLE_CIRCLE:
				{
					break;
				}
			default:
				break;
			};
		}*/
		///处理物体间的碰撞>

		///<处理物体与平面的碰撞
		size=this->PlaneList.size();
		int bodySize=this->BodyList.size();
		for(int i=0;i<size;i++)
		{
			for(int j=0;j<bodySize;j++)
			{
				switch(BodyList[j]->PrimitiveType)
				{
				case SPHERE:
					CollisionDetector::SphereAndHalfSpace(*((CollisionSphere*)BodyList[j]->pPrimitive),*PlaneList[i],&contactList);
					break;
				case BOX:
					CollisionDetector::BoxAndHalfSpace(*((CollisionBox*)BodyList[j]->pPrimitive),*PlaneList[i],&contactList);
					break;
				case CIRCLE:
					break;
				default:
					break;
				}
			}
		}
		///处理物体与平面的碰撞>
	};
	void Simulation::IntegrateContact(real duration)
	{
		this->contactSolution.SetIterations(8*this->contactList.size());
		this->contactSolution.ResolveContacts(contactList,contactList.size(),duration);
	}
	void Simulation::RunSimulation(real duration)
	{
		int bodyListSize=this->BodyList.size();
		this->GetPotentialContactList();
		this->GetContactList();
		
		
		this->IntegrateContact(duration);
		this->contactList.clear();
		this->potentialContactList.clear();
		for(int i=0;i<bodyListSize;i++)
		{
			BodyList[i]->Integrate(duration);
		}
	}
}