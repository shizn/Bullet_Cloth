#ifndef CLOTH_DEMO_H
#define CLOTH_DEMO_H

#ifdef _WINDOWS
#include "Win32DemoApplication.h"
#define PlatformDemoApplication Win32DemoApplication
#else
#include "GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication
#endif

#include "LinearMath/btAlignedObjectArray.h"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

class ClothDemo : public PlatformDemoApplication
{

    //keep the collision shapes, for deletion/cleanup
    btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

    btBroadphaseInterface*	m_broadphase;

    btCollisionDispatcher*	m_dispatcher;

    btConstraintSolver*	m_solver;

    btDefaultCollisionConfiguration* m_collisionConfiguration;

public:

    ClothDemo()
    {
    }
    virtual ~ClothDemo()
    {
        exitPhysics();
    }
    void	initPhysics();

    void	exitPhysics();

    virtual void clientMoveAndDisplay();

    virtual void displayCallback();
    virtual void	clientResetScene();

    static DemoApplication* Create()
    {
        ClothDemo* demo = new ClothDemo;
        demo->myinit();
        demo->initPhysics();
        return demo;
    }


};

#endif //Cloth_DEMO_H