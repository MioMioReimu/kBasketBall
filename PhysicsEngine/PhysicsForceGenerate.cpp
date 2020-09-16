#define DLL_PHYSICS_EXIST
#include "Physics.h"
namespace Physics
{
	void ForceRegistry::updateForces(real duration)
	{
		for(Registry::iterator i=this->registrations.begin();
			i!=registrations.end();i++)
			i->forceGen->updateForce(i->body,duration);
	}
	void ForceRegistry::add(RigidBody *body,ForceGenerator *forceGen)
	{
		ForceRegistration registration;
		registration.body=body;
		registration.forceGen=forceGen;
		this->registrations.push_back(registration);
	}
	void ForceRegistry::remove(RigidBody *body,ForceGenerator *forceGen)
	{
		for(Registry::iterator i=this->registrations.begin();
			i!=registrations.end();i++)
				if((i->body)==body&&i->forceGen==forceGen)
					this->registrations.erase(i);
	}
	void ForceRegistry::clear()
	{
		this->registrations.clear();
	}

	Gravity::Gravity(const Vector3 &gravity):gravity(gravity)
	{	};
	void Gravity::updateForce(RigidBody *body,real duration)
	{
		if(!body->HasFiniteMass())return;
		body->AddWorldForceAtWorldCenter(gravity*body->GetMass());
	}
}