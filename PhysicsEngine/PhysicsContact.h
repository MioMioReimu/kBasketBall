#pragma once
//#include"PhysicsForceGenerate.h"
namespace Physics
{

    class DLL_PHYSICS ContactResolver;

    class DLL_PHYSICS Contact
    {

        friend ContactResolver;
    public:

        RigidBody* body[2];
        real friction;
        real restitution;
        Vector3 contactPoint;
        Vector3 contactNormal;
        real penetration;
        void SetBodyData(RigidBody* one, RigidBody *two,
                         real friction, real restitution);
        Matrix3 contactToWorld;
        Vector3 contactVelocity;
        real desiredDeltaVelocity;
        Vector3 relativeContactPosition[2];
    protected:
        void CalculateInternals(real duration);
        void SwapBodies();
        void MatchAwakeState();
        void CalculateDesiredDeltaVelocity(real duration);
        Vector3 CalculateLocalVelocity(unsigned bodyIndex, real duration);
        void CalculateContactBasis();
        void ApplyImpulse(const Vector3 &impulse, RigidBody *body, 
                          Vector3 *velocityChange, Vector3 *rotationChange);
        void ApplyVelocityChange(Vector3 velocityChange[2], 
                                 Vector3 rotationChange[2]);
        void ApplyPositionChange(Vector3 velocityChange[2], 
            Vector3 rotationDirection[2],
            real rotationAmount[2],
            real penetration);
    };
	//Åö×²Êý×é

	template class DLL_PHYSICS allocator<Contact>;
	template class DLL_PHYSICS vector<Contact,allocator<Contact>>;
	typedef vector<Contact> ContactListType;
    class DLL_PHYSICS ContactResolver
    {
    protected:
        unsigned velocityIterations;
        unsigned positionIterations;
        real velocityEpsilon;
        real positionEpsilon;

    public:
        unsigned velocityIterationsUsed;
        unsigned positionIterationsUsed;

    private:
        bool validSettings;
    public:
		ContactResolver()
		{
			velocityIterations=positionIterations=1024;
			velocityEpsilon=(real)0.01;
			positionEpsilon=(real)0.001;
		};
        ContactResolver(unsigned iterations, 
            real velocityEpsilon=(real)0.01,
            real positionEpsilon=(real)0.01);
        ContactResolver(unsigned velocityIterations, 
            unsigned positionIterations,
            real velocityEpsilon=(real)0.01,
            real positionEpsilon=(real)0.01);
        bool IsValid()
        {
            return (velocityIterations > 0) && 
                   (positionIterations > 0) &&
                   (positionEpsilon >= 0.0f) && 
                   (positionEpsilon >= 0.0f);
        }
        void SetIterations(unsigned velocityIterations, 
                           unsigned positionIterations);
        void SetIterations(unsigned iterations);
        void SetEpsilon(real velocityEpsilon, 
                        real positionEpsilon);
        void ResolveContacts(ContactListType &contactList,
            unsigned numContacts,
            real duration);
    protected:
        void PrepareContacts(ContactListType &contactList, unsigned numContacts,
            real duration);
        void AdjustVelocities(ContactListType &contactList,
            unsigned numContacts,
            real duration);
        void AdjustPositions(ContactListType &contactList,
            unsigned numContacts,
            real duration);

    };

    class ContactGenerator
    {
    public:
        virtual unsigned AddContact(Contact *contact, unsigned limit) const = 0;
    };
}