#ifndef PHYSICS_H
#define PHYSICS_H
#ifdef DLL_PHYSICS_EXIST
#define DLL_PHYSICS __declspec(dllexport)
#else
#define DLL_PHYSICS __declspec(dllimport)
#endif
#include"PhysicsCommon.h"

#include"PhysicsMath.h"
#include"PhysicsCollisionCoarse.h"
#include"PhysicsRigidBody.h"
#include"PhysicsForceGenerate.h"
#include"PhysicsContact.h"

#include"PhysicsCollisionFine.h"
#include"PhysicsWorld.h"
#endif