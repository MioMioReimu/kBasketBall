#define DLL_PHYSICS_EXIST
#include"Physics.h"
using namespace Physics;
void Contact::SetBodyData(RigidBody* one, RigidBody *two,
	real friction, real restitution)
{
	Contact::body[0] = one;
	Contact::body[1] = two;
	Contact::friction = friction;
	Contact::restitution = restitution;
}

void Contact::MatchAwakeState()
{
	// Collisions with the world never cause a body to wake up.
	if (!body[1]) return;

	bool body0awake = body[0]->GetAwake();
	bool body1awake = body[1]->GetAwake();

	// Wake up only the sleeping one
	if (body0awake ^ body1awake) {
		if (body0awake) body[1]->SetAwake();
		else body[0]->SetAwake();
	}
}

void Contact::SwapBodies()
{
	contactNormal *= -1;

	RigidBody *temp = body[0];
	body[0] = body[1];
	body[1] = temp;
}

	void Contact::CalculateContactBasis()
{
	Vector3 contactTangent[2];

	
	if(real_abs(contactNormal.data[0]) > real_abs(contactNormal.data[1]))
	{
		
		const real s = (real)1.0f/real_sqrt(contactNormal.data[2]*contactNormal.data[2] +
			contactNormal.data[0]*contactNormal.data[0]);


		contactTangent[0].data[0] = contactNormal.data[2]*s;
		contactTangent[0].data[1] = 0;
		contactTangent[0].data[2] = -contactNormal.data[0]*s;

		contactTangent[1].data[0] = contactNormal.data[1]*contactTangent[0].data[0];
		contactTangent[1].data[1] = contactNormal.data[2]*contactTangent[0].data[0] -
			contactNormal.data[0]*contactTangent[0].data[2];
		contactTangent[1].data[2] = -contactNormal.data[1]*contactTangent[0].data[0];
	}
	else
	{
		
		const real s = (real)1.0/real_sqrt(contactNormal.data[2]*contactNormal.data[2] +
			contactNormal.data[1]*contactNormal.data[1]);

		
		contactTangent[0].data[0] = 0;
		contactTangent[0].data[1] = -contactNormal.data[2]*s;
		contactTangent[0].data[2] = contactNormal.data[1]*s;

		
		contactTangent[1].data[0] = contactNormal.data[1]*contactTangent[0].data[2] -
			contactNormal.data[2]*contactTangent[0].data[1];
		contactTangent[1].data[1] = -contactNormal.data[0]*contactTangent[0].data[2];
		contactTangent[1].data[2] = contactNormal.data[0]*contactTangent[0].data[1];
	}

	
	contactToWorld.SetComponents(
		contactNormal,
		contactTangent[0],
		contactTangent[1]);
}
Vector3 Contact::CalculateLocalVelocity(unsigned bodyIndex, real duration)
{
	RigidBody *thisBody = body[bodyIndex];

	
	Vector3 velocity =
		thisBody->GetRotation() % relativeContactPosition[bodyIndex];
	velocity += thisBody->GetVelocity();

	
	Vector3 contactVelocity = contactToWorld.TransformTranspose(velocity);

	
	Vector3 accVelocity = thisBody->GetLastFrameAcceleration() * duration;

	accVelocity = contactToWorld.TransformTranspose(accVelocity);


	accVelocity.data[0] = 0;

	
	contactVelocity += accVelocity;
	
	return contactVelocity;
}

void Contact::CalculateDesiredDeltaVelocity(real duration)
{
	const static real velocityLimit = (real)0.25f;

	
	real velocityFromAcc = body[0]->GetLastFrameAcceleration() * duration * contactNormal;

	if (body[1])
	{
		velocityFromAcc -= body[1]->GetLastFrameAcceleration() * duration * contactNormal;
	}

	real thisRestitution = restitution;
	if (real_abs(contactVelocity.data[0]) < velocityLimit)
	{
		thisRestitution = (real)0.0f;
	}

	desiredDeltaVelocity =
		-contactVelocity.data[0]
		-thisRestitution * (contactVelocity.data[0] - velocityFromAcc);
	
}


void Contact::CalculateInternals(real duration)
{
	
	if (!body[0]) SwapBodies();
	assert(body[0]);

	
	CalculateContactBasis();

	
	relativeContactPosition[0] = contactPoint - body[0]->GetPosition();
	if (body[1]) {
		relativeContactPosition[1] = contactPoint - body[1]->GetPosition();
	}

	contactVelocity = CalculateLocalVelocity(0, duration);
	if (body[1]) {
		contactVelocity -= CalculateLocalVelocity(1, duration);
	}

	CalculateDesiredDeltaVelocity(duration);
}


void Contact::ApplyVelocityChange(Vector3 velocityChange[2],
	Vector3 rotationChange[2])
{
	Matrix3 inverseInertiaTensor[2];
	body[0]->GetInverseInertiaTensorWorld(&inverseInertiaTensor[0]);

	
	Vector3 impulseContact;

	
	if (friction == (real)0.0)
	{
		
		Vector3 deltaVelWorld = relativeContactPosition[0] % contactNormal;
		deltaVelWorld = inverseInertiaTensor[0].Transform(deltaVelWorld);
		deltaVelWorld = deltaVelWorld % relativeContactPosition[0];

		
		real deltaVelocity = deltaVelWorld * contactNormal;

		
		deltaVelocity += body[0]->GetInverseMass();

		
		if (body[1])
		{
			
			body[1]->GetInverseInertiaTensorWorld(&inverseInertiaTensor[1]);

			
			Vector3 deltaVelWorld = relativeContactPosition[1] % contactNormal;
			deltaVelWorld = inverseInertiaTensor[1].Transform(deltaVelWorld);
			deltaVelWorld = deltaVelWorld % relativeContactPosition[1];

			
			deltaVelocity += deltaVelWorld * contactNormal;

			
			deltaVelocity += body[1]->GetInverseMass();
		}
		
		impulseContact.data[0] = desiredDeltaVelocity / deltaVelocity;
		impulseContact.data[1] = 0;
		impulseContact.data[2] = 0;
		
	}

	
	else
	{
		
		real inverseMass = body[0]->GetInverseMass();

		
		Matrix3 impulseToTorque;
		impulseToTorque.SetSkewSymmetric(relativeContactPosition[0]);

		
		Matrix3 deltaVelWorld = impulseToTorque;
		deltaVelWorld *= inverseInertiaTensor[0];
		deltaVelWorld *= impulseToTorque;
		deltaVelWorld *= -1;

		
		if (body[1])
		{
			
			body[1]->GetInverseInertiaTensorWorld(&inverseInertiaTensor[1]);

			
			impulseToTorque.SetSkewSymmetric(relativeContactPosition[1]);

			
			Matrix3 deltaVelWorld2 = impulseToTorque;
			deltaVelWorld2 *= inverseInertiaTensor[1];
			deltaVelWorld2 *= impulseToTorque;
			deltaVelWorld2 *= -1;

			
			deltaVelWorld += deltaVelWorld2;
			
			inverseMass += body[1]->GetInverseMass();
			
		}

		Matrix3 deltaVelocity = contactToWorld.Transpose();
		deltaVelocity *= deltaVelWorld;
		deltaVelocity *= contactToWorld;
		
		deltaVelocity.data[0] += inverseMass;
		deltaVelocity.data[4] += inverseMass;
		deltaVelocity.data[8] += inverseMass;

		
		Matrix3 impulseMatrix = deltaVelocity.GetInverse();

		
		Vector3 velKill(desiredDeltaVelocity,
			-contactVelocity.data[1],
			-contactVelocity.data[2]);

		
		impulseContact = impulseMatrix.Transform(velKill);

		
		real planarImpulse = real_sqrt(impulseContact.data[1]*impulseContact.data[1] +
			impulseContact.data[2]*impulseContact.data[2]);
		if (planarImpulse > impulseContact.data[0] * friction)
		{
			
			impulseContact.data[1] /= planarImpulse;
			impulseContact.data[2] /= planarImpulse;

			impulseContact.data[0] = deltaVelocity.data[0] +
				deltaVelocity.data[1]*friction*impulseContact.data[1] +
				deltaVelocity.data[2]*friction*impulseContact.data[2];
			impulseContact.data[0] = desiredDeltaVelocity / impulseContact.data[0];
			impulseContact.data[1] *= friction * impulseContact.data[0];
			impulseContact.data[2] *= friction * impulseContact.data[0];
		}
		
	}

	
	Vector3 impulse = contactToWorld.Transform(impulseContact);
	
	Vector3 impulsiveTorque = relativeContactPosition[0] % impulse;
	rotationChange[0] = inverseInertiaTensor[0].Transform(impulsiveTorque);
	velocityChange[0].Clear();
	velocityChange[0].AddScaledVector(impulse, body[0]->GetInverseMass());
	if(velocityChange[0].IsINFIN()||velocityChange[0].IsNAN())
		velocityChange[0].SetData(0,0,0);
	if(rotationChange[0].IsINFIN()||rotationChange[0].IsNAN())
		rotationChange[0].SetData(0,0,0);
	
	body[0]->AddVelocity(velocityChange[0]);
	body[0]->AddRotation(rotationChange[0]);

	if (body[1])
	{
		
		Vector3 impulsiveTorque = impulse % relativeContactPosition[1];
		rotationChange[1] = inverseInertiaTensor[1].Transform(impulsiveTorque);
		velocityChange[1].Clear();
		velocityChange[1].AddScaledVector(impulse, -body[1]->GetInverseMass());
		if(velocityChange[1].IsINFIN()||velocityChange[1].IsNAN())
			velocityChange[1].SetData(0,0,0);
		if(rotationChange[1].IsINFIN()||rotationChange[1].IsNAN())
			rotationChange[1].SetData(0,0,0);
		
		body[1]->AddVelocity(velocityChange[1]);
		body[1]->AddRotation(rotationChange[1]);
	}
}

void Contact::ApplyPositionChange(Vector3 velocityChange[2],
	Vector3 rotationDirection[2],
	real rotationAmount[2],
	real penetration)
{
	real angularLimit = (real)1000;
	real angularMove[2],linearMove[2];
	int b;

	real totalInertia = 0;
	real linearInertia[2];
	real angularInertia[2];

	
	for (unsigned i = 0; i < 2; i++) {
		if (body[i]) {
			Matrix3 inverseInertiaTensor;
			body[i]->GetInverseInertiaTensorWorld(&inverseInertiaTensor);

			
			Vector3 angularInertiaWorld = relativeContactPosition[i] % contactNormal;
			angularInertiaWorld = inverseInertiaTensor.Transform(angularInertiaWorld);
			angularInertiaWorld = angularInertiaWorld % relativeContactPosition[i];
			angularInertia[i] = angularInertiaWorld * contactNormal;

			
			linearInertia[i] = body[i]->GetInverseMass();

			
			totalInertia += linearInertia[i] + angularInertia[i];
		}
	}
	

	real inverseMass[2];

	totalInertia = angularInertia[0] + body[0]->GetInverseMass();

	if(body[1])
	{
		inverseMass[1] = angularInertia[1] + body[1]->GetInverseMass();
		totalInertia+=inverseMass[1];

		angularMove[1] = -penetration*angularInertia[1]/totalInertia;
		linearMove[1] = -penetration*body[1]->GetInverseMass()/totalInertia;

		
		Vector3 projection = relativeContactPosition[1];
		projection.AddScaledVector(contactNormal,
			-relativeContactPosition[1].ScalarProduct(contactNormal));
		real max = angularLimit*relativeContactPosition[1].Magnitude();

		if(real_abs(angularMove[1]) > max)
		{
			real pp=angularMove[1]+linearMove[1];
			angularMove[1]=angularMove[1]>0?max:-max;
			linearMove[1]=pp-angularMove[1];
		}
	}

	angularMove[0] = penetration*angularInertia[0]/totalInertia;
	linearMove[0] = penetration*body[0]->GetInverseMass()/totalInertia;

	
	Vector3 projection = relativeContactPosition[0];
	projection.AddScaledVector(contactNormal,
		-relativeContactPosition[0].ScalarProduct(contactNormal));
	real max = angularLimit*relativeContactPosition[0].Magnitude();

	if(real_abs(angularMove[0]) > max)
	{
		real pp=angularMove[0]+linearMove[0];
		angularMove[0]=angularMove[0]>0?max:-max;
		linearMove[0]=pp-angularMove[0];
	}

	for(b=0;b<2;b++) if(body[b])
	{
		Vector3 t;
		if(angularMove[b]!=((real)0.0))
		{
			t = relativeContactPosition[b].CrossProduct(contactNormal);

			Matrix3 inverseInertiaTensor;
			body[b]->GetInverseInertiaTensorWorld(&inverseInertiaTensor);
			rotationDirection[b] = inverseInertiaTensor.Transform(t);

			rotationAmount[b] = angularMove[b] / angularInertia[b];

			assert(rotationAmount[b]!=((real)0.0));
		}
		else
		{
			rotationDirection[b].Clear();
			rotationAmount[b]=1;
		}

		velocityChange[b] = contactNormal;
		velocityChange[b] *= linearMove[b]/rotationAmount[b];

		Vector3 pos;
		body[b]->GetPosition(&pos);
		pos.AddScaledVector(contactNormal,linearMove[b]);
		body[b]->SetPosition(pos);

		Quaternion q;
		body[b]->GetOrientation(&q);
		q.AddScaledVector(rotationDirection[b], rotationAmount[b] * ((real)0.5));
		body[b]->SetOrientation(q);
	}
}







ContactResolver::ContactResolver(unsigned iterations,
	real velocityEpsilon,
	real positionEpsilon)
{
	SetIterations(iterations, iterations);
	SetEpsilon(velocityEpsilon, positionEpsilon);
}

ContactResolver::ContactResolver(unsigned velocityIterations,
	unsigned positionIterations,
	real velocityEpsilon,
	real positionEpsilon)
{
	SetIterations(velocityIterations);
	SetEpsilon(velocityEpsilon, positionEpsilon);
}

void ContactResolver::SetIterations(unsigned iterations)
{
	SetIterations(iterations, iterations);
}

void ContactResolver::SetIterations(unsigned velocityIterations,
	unsigned positionIterations)
{
	ContactResolver::velocityIterations = velocityIterations;
	ContactResolver::positionIterations = positionIterations;
}

void ContactResolver::SetEpsilon(real velocityEpsilon,
	real positionEpsilon)
{
	ContactResolver::velocityEpsilon = velocityEpsilon;
	ContactResolver::positionEpsilon = positionEpsilon;
}


void ContactResolver::ResolveContacts(ContactListType &contacts,
	unsigned numContacts,
	real duration)
{
	
	if (numContacts == 0) return;
	
	if (!IsValid()) return;
	
	PrepareContacts(contacts, numContacts, duration);

	
	AdjustPositions(contacts, numContacts, duration);

	
	AdjustVelocities(contacts, numContacts, duration);
}

void ContactResolver::PrepareContacts(ContactListType &contacts,
	unsigned numContacts,
	real duration)
{
	
	
	for(unsigned int i=0; i<numContacts; i++)
	{
		
		contacts[i].CalculateInternals(duration);
	}
}


void ContactResolver::AdjustVelocities(ContactListType &c,
	unsigned numContacts,
	real duration)
{
	Vector3 velocityChange[2], rotationChange[2];
	Vector3 cp;

	
	velocityIterationsUsed = 0;
	while(velocityIterationsUsed < velocityIterations)
	{
		
		real max = velocityEpsilon;
		unsigned index = numContacts;
		for(unsigned i = 0; i < numContacts; i++)
		{
			if (c[i].desiredDeltaVelocity > max)
			{
				max = c[i].desiredDeltaVelocity;
				index = i;
			}
		}
		if (index == numContacts) break;

		
		c[index].MatchAwakeState();

		
		c[index].ApplyVelocityChange(velocityChange, rotationChange);

		
		for (unsigned i = 0; i < numContacts; i++)
		{
			if (c[i].body[0])
			{
				if (c[i].body[0] == c[index].body[0])
				{
					cp = rotationChange[0].CrossProduct(c[i].
						relativeContactPosition[0]);

					cp += velocityChange[0];

					c[i].contactVelocity +=
						c[i].contactToWorld.TransformTranspose(cp);
					c[i].CalculateDesiredDeltaVelocity(duration);

				}
				else if (c[i].body[0]==c[index].body[1])
				{
					cp = rotationChange[1].CrossProduct(c[i].
						relativeContactPosition[0]);

					cp += velocityChange[1];

					c[i].contactVelocity +=
						c[i].contactToWorld.TransformTranspose(cp);
					c[i].CalculateDesiredDeltaVelocity(duration);
				}
			}

			if (c[i].body[1])
			{
				if (c[i].body[1]==c[index].body[0])
				{
					cp = rotationChange[0].CrossProduct(c[i].
						relativeContactPosition[1]);

					cp += velocityChange[0];

					c[i].contactVelocity -=
						c[i].contactToWorld.TransformTranspose(cp);
					c[i].CalculateDesiredDeltaVelocity(duration);
				}
				else if (c[i].body[1]==c[index].body[1])
				{
					cp = rotationChange[1].CrossProduct(c[i].
						relativeContactPosition[1]);

					cp += velocityChange[1];

					c[i].contactVelocity -=
						c[i].contactToWorld.TransformTranspose(cp);
					c[i].CalculateDesiredDeltaVelocity(duration);
				}
			}
		}
		velocityIterationsUsed++;
	}
}


void ContactResolver::AdjustPositions(ContactListType &c,
	unsigned numContacts,
	real duration)
{
	unsigned i,index;
	Vector3 velocityChange[2], rotationChange[2];
	real rotationAmount[2];
	real max;
	Vector3 cp;

	
	positionIterationsUsed = 0;
	while(positionIterationsUsed < positionIterations)
	{
		
		max = positionEpsilon;
		index = numContacts;
		for(i=0;i<numContacts;i++) {
			if(c[i].penetration > max)
			{
				max=c[i].penetration;
				index=i;
			}
		}
		if (index == numContacts) break;

		
		c[index].MatchAwakeState();

		
		c[index].ApplyPositionChange(velocityChange,
			rotationChange,
			rotationAmount,
			max);
		
		for(i = 0; i < numContacts; i++)
		{
			if(c[i].body[0] == c[index].body[0])
			{
				cp = rotationChange[0].CrossProduct(c[i].
					relativeContactPosition[0]);

				cp += velocityChange[0];

				c[i].penetration -=
					rotationAmount[0]*cp.ScalarProduct(c[i].contactNormal);
			}
			else if(c[i].body[0]==c[index].body[1])
			{
				cp = rotationChange[1].CrossProduct(c[i].
					relativeContactPosition[0]);

				cp += velocityChange[1];

				c[i].penetration -=
					rotationAmount[1]*cp.ScalarProduct(c[i].contactNormal);
			}

			if(c[i].body[1])
			{
				if(c[i].body[1]==c[index].body[0])
				{
					cp = rotationChange[0].CrossProduct(c[i].
						relativeContactPosition[1]);

					cp += velocityChange[0];

					c[i].penetration +=
						rotationAmount[0]*cp.ScalarProduct(c[i].contactNormal);
				}
				else if(c[i].body[1]==c[index].body[1])
				{
					cp = rotationChange[1].CrossProduct(c[i].
						relativeContactPosition[1]);

					cp += velocityChange[1];

					c[i].penetration +=
						rotationAmount[1]*cp.ScalarProduct(c[i].contactNormal);
				}
			}
		}
		positionIterationsUsed++;
	}
}