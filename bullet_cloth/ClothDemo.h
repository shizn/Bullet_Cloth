#ifndef CLOTH_DEMO_H
#define CLOTH_DEMO_H


//Depending on platform, may '#include <GL/glew.h>', which must be included before <GL/gl.h>
#include "ScreenSpaceFluidRendererGL.h"	

#ifdef _WINDOWS
#include "Win32DemoApplication.h"
#define PlatformDemoApplication Win32DemoApplication
#else
#include "GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication
#endif


#include "MarchingCubes.h"

#include "LinearMath/btAlignedObjectArray.h"

//
#include "BulletFluids/Sph/btFluidSphSolver.h"

#include "btFluidSoftRigidCollisionConfiguration.h"
#include "btFluidSoftRigidDynamicsWorld.h"
#include "GLDebugDrawer.h"

//Assimp
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
//Assimp

class btCollisionShape;
class btBroadphaseInterface;
class btCollisionDispatcher;
class btConstraintSolver;
class btDefaultCollisionConfiguration;

class btFluidSoftRigidDynamicsWorld;

enum FluidRenderMode
{
    FRM_Points = 0,
    FRM_MediumSpheres,
    FRM_LargeSpheres,
    FRM_ScreenSpace,
    FRM_MarchingCubes
};

class ClothDemo : public PlatformDemoApplication
{
    //Bullet
    btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;	//Keep the collision shapes, for deletion/cleanup
    btBroadphaseInterface* m_broadphase;
    btCollisionDispatcher* m_dispatcher;
    btConstraintSolver*	m_solver;
    btDefaultCollisionConfiguration* m_collisionConfiguration;

    //Fluid system
    btFluidSoftRigidDynamicsWorld* m_fluidSoftRigidWorld;
    btFluidSph* m_fluidSph;

    btFluidSphSolver* m_fluidSphSolver;

    //Rendering
    FluidRenderMode m_fluidRenderMode;
    ScreenSpaceFluidRendererGL* m_screenSpaceRenderer;

    GLDebugDrawer m_debugDrawer;

    //Assimp
    // Create an instance of the Importer class
    Assimp::Importer importer;
    const aiScene* scene;
    float* vertexArray;
    float* normalArray;
    float* uvArray;
    int numVerts;
    //Assimp

public:

    ClothDemo();
    virtual ~ClothDemo();

    void initPhysics();		//Initialize Bullet/fluid system here
    void exitPhysics();		//Deactivate Bullet/fluid system here

    //
    virtual void clientMoveAndDisplay();	//Simulation is updated/stepped here
    virtual void displayCallback();			//Rendering occurs here

    void renderFluids();
    void processClothDiffusion();
    //
    virtual void keyboardCallback(unsigned char key, int x, int y);
    virtual void specialKeyboard(int key, int x, int y);
    virtual void setShootBoxShape();
    virtual void myinit();
    virtual void reshape(int w, int h);
    virtual void clientResetScene();

    //Assimp
    bool ImportObjMesh(const std::string& pFile);
    //Assimp

    static DemoApplication* Create()
    {
        ClothDemo* demo = new ClothDemo;
        demo->myinit();
        return demo;
    }
};

#endif //Cloth_DEMO_H