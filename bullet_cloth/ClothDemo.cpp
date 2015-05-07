/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#include "ClothDemo.h"

#include <stdio.h> 	//printf debugging

#include "GlutStuff.h"
#include "GLDebugFont.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btRandom.h"		//GEN_rand(), GEN_RAND_MAX

#include "BulletSoftBody/btSoftBodyHelpers.h"

#include "BulletFluids/Sph/btFluidSph.h"

//Current SPH-softbody interaction method has issues with jitter and penetration, 
//especially if 2-way interaction is enabled
//#define USE_TUBE_MESH_FOR_SOFT_BODY
#ifdef USE_TUBE_MESH_FOR_SOFT_BODY
#include "tubeTriangleMesh.h"
#endif

ClothDemo::ClothDemo()
{
    setTexturing(true);
    setShadows(true);
    setCameraDistance(50.0);
    m_fluidRenderMode = FRM_ScreenSpace;

    m_screenSpaceRenderer = 0;

    initPhysics();
}
ClothDemo::~ClothDemo()
{
    if (m_screenSpaceRenderer) delete m_screenSpaceRenderer;
    exitPhysics();
}


void ClothDemo::initPhysics()
{
    m_collisionConfiguration = new btFluidSoftRigidCollisionConfiguration();

    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
    m_broadphase = new btDbvtBroadphase();
    m_solver = new btSequentialImpulseConstraintSolver();

    m_fluidSphSolver = new btFluidSphSolverDefault();

    m_dynamicsWorld = new btFluidSoftRigidDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration, m_fluidSphSolver);
    m_fluidSoftRigidWorld = static_cast<btFluidSoftRigidDynamicsWorld*>(m_dynamicsWorld);

    m_fluidSoftRigidWorld->setGravity(btVector3(0.0, -9.8, 0.0));
    m_fluidSoftRigidWorld->setDebugDrawer(&m_debugDrawer);

    //Create a very large static box as the ground
    //We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
    {
        btCollisionShape* groundShape = new btBoxShape(btVector3(50.0, 50.0, 50.0));
        m_collisionShapes.push_back(groundShape);

        btScalar mass(0.f);

        //Rigid bodies are dynamic if and only if mass is non zero, otherwise static
        btVector3 localInertia(0, 0, 0);
        if (mass != 0.f) groundShape->calculateLocalInertia(mass, localInertia);

        btTransform groundTransform;
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3(0, -50, 0));

        //Using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
        btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);

        m_fluidSoftRigidWorld->addRigidBody(body);
    }

    //Create btFluidSph(s), which contain groups of particles
    {
        const int MAX_PARTICLES = 8192;
        btFluidSph* fluidSph = new btFluidSph(m_fluidSoftRigidWorld->getGlobalParameters(), MAX_PARTICLES);

        {
            btFluidSphParametersLocal FL = fluidSph->getLocalParameters();

            const btScalar AABB_EXTENT(25.0);
            FL.m_aabbBoundaryMin = btVector3(-AABB_EXTENT, -AABB_EXTENT, -AABB_EXTENT);
            //FL.m_aabbBoundaryMax = btVector3(AABB_EXTENT, AABB_EXTENT*btScalar(100.0), AABB_EXTENT);
            FL.m_aabbBoundaryMax = btVector3(AABB_EXTENT, AABB_EXTENT, AABB_EXTENT);
            FL.m_enableAabbBoundary = 1;

            //FL.m_particleMass = btScalar(0.0001);	//Mass of particles when colliding with rigid/soft bodies; def 0.00020543
            //FL.m_boundaryErp = btScalar(0.375);	//Increase m_boundaryErp to reduce penetration at the cost of increased jitter; def 0.0375

            fluidSph->setLocalParameters(FL);


        }

        const bool ENABLE_CCD = true;
        if (ENABLE_CCD) fluidSph->setCcdMotionThreshold(fluidSph->getLocalParameters().m_particleRadius);

        m_fluidSoftRigidWorld->addFluidSph(fluidSph);
        m_fluidSph = fluidSph;

    }
    //Create a soft-body cloth patch
    {
        const btScalar POSITION_Y(10.0);
        const btScalar EXTENT(16.0);

        const int RESOLUTION = 20;

#ifndef USE_TUBE_MESH_FOR_SOFT_BODY		
        btSoftBody* softBody = btSoftBodyHelpers::CreatePatch(m_fluidSoftRigidWorld->getWorldInfo(), btVector3(-EXTENT, POSITION_Y, -EXTENT),
            btVector3(EXTENT, POSITION_Y, -EXTENT),
            btVector3(-EXTENT, POSITION_Y, EXTENT),
            btVector3(EXTENT, POSITION_Y, EXTENT),
            RESOLUTION, RESOLUTION, 1 + 2 + 4 + 8, true);
#else
        btSoftBody* softBody = btSoftBodyHelpers::CreateFromTriMesh(m_fluidSoftRigidWorld->getWorldInfo(),
            TUBE_TRIANGLE_VERTICES, TUBE_TRIANGLE_INDICIES, TUBE_NUM_TRIANGLES, true);

        softBody->transform(btTransform(btQuaternion::getIdentity(), btVector3(0.0, 50.0, 0.0)));	//Set softbody position

        for (int i = 0; i < TUBE_NUM_FIXED_VERTICIES; ++i) softBody->setMass(TUBE_FIXED_VERTICIES[i], btScalar(0.0));
#endif

        btSoftBody::Material* material = softBody->appendMaterial();
        material->m_kLST = 0.5;
        material->m_kAST = 0.5;
        material->m_flags -= btSoftBody::fMaterial::DebugDraw;

        softBody->m_cfg.kDF = 1.0;
        softBody->m_cfg.kSRHR_CL = 1.0;
        softBody->m_cfg.kSR_SPLT_CL = 0.5;
        softBody->m_cfg.collisions = btSoftBody::fCollision::CL_SS + btSoftBody::fCollision::CL_RS;

        softBody->setTotalMass(2.0);
        softBody->generateBendingConstraints(2, material);
        softBody->generateClusters(0); 	//Pass zero in generateClusters to create a cluster for each tetrahedron or triangle
        //ssxx draw flag
        m_fluidSoftRigidWorld->setDrawFlags(fDrawFlags::SsxxCustom);
        softBody->m_faces[30].m_absorb = 10.0;
        //ssxx
        m_fluidSoftRigidWorld->addSoftBody(softBody);
    }
}
void ClothDemo::exitPhysics()
{
    //Cleanup in the reverse order of creation/initialization
    for (int i = m_fluidSoftRigidWorld->getNumCollisionObjects() - 1; i >= 0; i--)
    {
        btCollisionObject* obj = m_fluidSoftRigidWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState()) delete body->getMotionState();

        m_fluidSoftRigidWorld->removeCollisionObject(obj);
        delete obj;
    }

    //Delete collision shapes
    for (int j = 0; j < m_collisionShapes.size(); j++)
    {
        btCollisionShape* shape = m_collisionShapes[j];
        delete shape;
    }
    m_collisionShapes.clear();

    //
    delete m_fluidSoftRigidWorld;
    delete m_solver;
    delete m_broadphase;
    delete m_dispatcher;
    delete m_collisionConfiguration;
    delete m_fluidSphSolver;
}

inline int emitParticle(btFluidSph* fluid, const btVector3& position, const btVector3& velocity)
{
    int index = fluid->addParticle(position);
    if (index != fluid->numParticles()) fluid->setVelocity(index, velocity);
    else
    {
        index = (fluid->numParticles() - 1) * GEN_rand() / GEN_RAND_MAX;		//Random index

        fluid->setPosition(index, position);
        fluid->setVelocity(index, velocity);
    }

    return index;
}

void ClothDemo::clientMoveAndDisplay()
{
    btScalar secondsElapsed = getDeltaTimeMicroseconds() * btScalar(0.000001);

    const btFluidSphParametersGlobal& FG = m_fluidSoftRigidWorld->getGlobalParameters();
    if (m_fluidSoftRigidWorld)
    {
        const bool USE_SYNCRONIZED_TIME_STEP = false;	//Default: Rigid bodies == 16ms, Sph particles == 3ms time step
        if (USE_SYNCRONIZED_TIME_STEP)
        {
            btScalar timeStep = m_fluidSoftRigidWorld->getGlobalParameters().m_timeStep;
            m_fluidSoftRigidWorld->stepSimulation(timeStep, 0, timeStep);
        }
        else m_fluidSoftRigidWorld->stepSimulation(btScalar(1.0 / 60.0), 0);
    }

    {
        static int counter = 0;
        static int fluidcounter = 0;
        if (++counter > 10)
        {
            counter = 0;
            //ssxx
            //add fluids here
            if (m_fluidSph && (++fluidcounter))
            {
                emitParticle(m_fluidSph, btVector3(0.0, 30.0, 0.0), btVector3(0.0, -1.0, 0.0));
                emitParticle(m_fluidSph, btVector3(0.0, 30.0, 1.0), btVector3(0.0, -1.0, 0.0));
                emitParticle(m_fluidSph, btVector3(1.0, 30.0, 0.0), btVector3(0.0, -1.0, 0.0));
                emitParticle(m_fluidSph, btVector3(-1.0, 30.0, -1.0), btVector3(0.0, -1.0, 0.0));
                emitParticle(m_fluidSph, btVector3(1.0, 30.0, 1.0), btVector3(0.0, -1.0, 0.0));
                emitParticle(m_fluidSph, btVector3(-1.0, 30.0, 0.0), btVector3(0.0, -1.0, 0.0));
                emitParticle(m_fluidSph, btVector3(0.0, 30.0, -1.0), btVector3(0.0, -1.0, 0.0));
                emitParticle(m_fluidSph, btVector3(1.0, 30.0, -1.0), btVector3(0.0, -1.0, 0.0));
                emitParticle(m_fluidSph, btVector3(-1.0, 30.0, 1.0), btVector3(0.0, -1.0, 0.0));
            }


            for (int i = 0; i < m_fluidSoftRigidWorld->getNumFluidSph(); ++i)
            {
                //if (m_fluidSoftRigidWorld->getFluidSph(i)->numParticles()<2)
                //{
                //    emitParticle(m_fluidSph, btVector3(0.0, 30.0, 0.0), btVector3(0.0, -1.0, 0.0));
                //    emitParticle(m_fluidSph, btVector3(0.0, 30.0, 1.0), btVector3(0.0, -1.0, 0.0));
                //}
                printf("m_fluidSoftRigidWorld->getFluidSph(%d)->numParticles(): %d \n", i, m_fluidSoftRigidWorld->getFluidSph(i)->numParticles());
            }
            //ssxx
        }
        {
            /*
            static int i = 0;
            if (i>10)
            {
                processClothDiffusion();
                i = 0;
            }
            i++;
            */
        }
    }

    displayCallback();
}


GLuint generateSphereList(float radius)
{
    //Sphere generation code from FLUIDS v.2
    GLuint glSphereList = glGenLists(1);
    glNewList(glSphereList, GL_COMPILE);
    glBegin(GL_TRIANGLE_STRIP);
    for (float tilt = -90.0f; tilt <= 90.0f; tilt += 10.0f)
    {
        for (float ang = 0.f; ang <= 360.0f; ang += 30.0f)
        {
            const float DEGREES_TO_RADIANS = 3.141592f / 180.0f;

            float ang_radians = ang * DEGREES_TO_RADIANS;
            float tilt_radians = tilt * DEGREES_TO_RADIANS;
            float tilt1_radians = (tilt + 10.0f) * DEGREES_TO_RADIANS;

            float x = sin(ang_radians) * cos(tilt_radians);
            float y = cos(ang_radians) * cos(tilt_radians);
            float z = sin(tilt_radians);
            float x1 = sin(ang_radians) * cos(tilt1_radians);
            float y1 = cos(ang_radians) * cos(tilt1_radians);
            float z1 = sin(tilt1_radians);

            glNormal3f(x, y, z);	glVertex3f(x*radius, y*radius, z*radius);
            glNormal3f(x1, y1, z1);	glVertex3f(x1*radius, y1*radius, z1*radius);
        }
    }
    glEnd();
    glEndList();

    return glSphereList;
}
inline void drawSphere(GLuint glSphereList, const btVector3& position, float r, float g, float b)
{
    glPushMatrix();
    glColor3f(r, g, b);
    glTranslatef(position.x(), position.y(), position.z());
    glCallList(glSphereList);
    glPopMatrix();
}

void getFluidColors(bool drawFluidsWithMultipleColors, int fluidIndex, btFluidSph* fluid, int particleIndex, float* out_r, float* out_g, float* out_b)
{
    const float COLOR_R = 0.3f;
    const float COLOR_G = 0.7f;
    const float COLOR_B = 1.0f;

    if (!drawFluidsWithMultipleColors)
    {
        float brightness = fluid->getVelocity(particleIndex).length() * 2.0f;
        if (brightness < 0.f)brightness = 0.f;
        if (brightness > 1.f)brightness = 1.f;

        const float MIN_BRIGHTNESS(0.15f);
        brightness = brightness * (1.0f - MIN_BRIGHTNESS) + MIN_BRIGHTNESS;

        *out_r = COLOR_R * brightness;
        *out_g = COLOR_G * brightness;
        *out_b = COLOR_B * brightness;
    }
    else
    {
        *out_r = COLOR_R;
        *out_g = COLOR_G;
        *out_b = COLOR_B;

        if (fluidIndex % 2)
        {
            *out_r = 1.0f - COLOR_R;
            *out_g = 1.0f - COLOR_G;
            *out_b = 1.0f - COLOR_B;
        }
    }
}
void ClothDemo::processClothDiffusion()
{
    const int ENABLE_DIFFUSION = true;
    if (ENABLE_DIFFUSION)
    {
        btAlignedObjectArray<btSoftBody*> softBodies = m_fluidSoftRigidWorld->getSoftBodyArray();
        for (int i = 0; i < softBodies.size(); ++i)
        {
            //For each soft body, Process diffusion.
            btSoftBody* softbody = softBodies[i];
            //printf("softbody 349 %x\n", softbody);
            for (int j = 0; j < softbody->m_faces.size(); ++j)
            {
                btSoftBody::Face& f = softbody->m_faces[j];
                f.m_diffuse = 0;
                f.m_deltaabsorb = 0;
            }
            for (int j = 0; j < softbody->m_faces.size(); ++j)
            {
                btSoftBody::Face& f = softbody->m_faces[j];
                //if (f.m_absorb > 0)
                {
                    // If it has absorbed mass, process diffusion
                    // Get neighboring triangle
                    btSoftBody::Node* node[3] = { f.m_n[0], f.m_n[1], f.m_n[2] };
                    btAlignedObjectArray<btSoftBody::Face*> neighFaces;
                    btScalar kdiffusion = 0.1;
                    for (int n = 0; n < softbody->m_faces.size(); ++n)
                    {
                        btSoftBody::Face& neighF = softbody->m_faces[n];
                        int c = 0;
                        for (int k = 0; k < 3; ++k)
                        {
                            if (neighF.m_n[k] == node[0]||
                                neighF.m_n[k] == node[1]||
                                neighF.m_n[k] == node[2])
                            {
                                //c |= 1 << k;
                                c++;
                            }
                        }
                        if (2==c)
                        {
                            //Here we get a neighboring triangle, sharing a edge
                            //Diffusion start
                            neighFaces.push_back(&neighF);
                            neighF.m_deltadiffuse = min(0, kdiffusion*(f.m_absorb - neighF.m_absorb));
                            f.m_diffuse += neighF.m_deltadiffuse;
                        }
                    }
                    btScalar normfactor = 0.0;
                    if (f.m_diffuse > f.m_absorb)
                    {
                        normfactor = f.m_absorb / f.m_diffuse;
                    }
                    else
                    {
                        normfactor = 1.0;
                    }
                    for (int p = 0; p < neighFaces.size(); ++p)
                    {
                        //Compute change in saturation 
                        f.m_deltaabsorb = f.m_deltaabsorb - normfactor* neighFaces[p]->m_deltadiffuse;
                        neighFaces[p]->m_deltaabsorb = neighFaces[p]->m_deltaabsorb + normfactor*neighFaces[p]->m_deltadiffuse *(f.m_ra / neighFaces[p]->m_ra);
                        //printf("neighFaces[%d] = %f\n", p, neighFaces[p]->m_diffuse);
                    }                  
                    for (int x = 0; x < softbody->m_faces.size(); ++x)
                    {
                        btSoftBody::Face& fx = softbody->m_faces[x];
                        if (0 != fx.m_deltaabsorb)
                        {
                            //printf("fx.m_deltaabsorb[%d] = %f\n", x, fx.m_deltaabsorb);
                        }
                    }       
                    
                }
            }
            //printf("softbody 415 %x\n", softbody);
           // printf("m_diffuse[31] = %f\n", softbody->m_faces[31].m_diffuse);
           // printf("m_diffuse[32] = %f\n", softbody->m_faces[32].m_diffuse);
            for (int j = 0; j < softbody->m_faces.size(); ++j)
            {
                btSoftBody::Face& f = softbody->m_faces[j];
                /*
                if (0!= f.m_diffuse)
                {
                    printf("f.m_diffuse[%d] = %f\n", j, f.m_diffuse);
                }
                */
                f.m_absorb += f.m_deltaabsorb;
                
                if (0 != f.m_absorb)
                {
                    //printf("f.m_absorb[%d] = %f\n", j, f.m_absorb);
                }
                
            }
        }

    }
}
void ClothDemo::renderFluids()
{
    static bool areSpheresGenerated = false;
    static GLuint glSmallSphereList;
    static GLuint glMediumSphereList;
    static GLuint glLargeSphereList;
    if (!areSpheresGenerated)
    {
        const float PARTICLE_RADIUS = 1.0f;

        areSpheresGenerated = true;
        glSmallSphereList = generateSphereList(0.1f);
        glMediumSphereList = generateSphereList(PARTICLE_RADIUS * 0.3f);
        glLargeSphereList = generateSphereList(PARTICLE_RADIUS);
    }

    bool drawFluidsWithMultipleColors = false;

    if (m_fluidRenderMode != FRM_MarchingCubes && m_fluidRenderMode != FRM_ScreenSpace)
    {
        //BT_PROFILE("Draw fluids - spheres");

        GLuint glSphereList;
        switch (m_fluidRenderMode)
        {
        case FRM_LargeSpheres:
            glSphereList = glLargeSphereList;
            break;
        case FRM_MediumSpheres:
            glSphereList = glMediumSphereList;
            break;
        case FRM_Points:
        default:
            glSphereList = glSmallSphereList;
            break;
        }

        //for (int i = 0; i < m_fluids.size(); ++i)
        for (int n = 0; n < m_fluidSph->numParticles(); ++n)
        {
            float r, g, b;
            getFluidColors(drawFluidsWithMultipleColors, 0, m_fluidSph, n, &r, &g, &b);

            drawSphere(glSphereList, m_fluidSph->getPosition(n), r, g, b);
        }
    }
    else if (m_fluidRenderMode == FRM_ScreenSpace)
    {
        //BT_PROFILE("Draw fluids - screen space");

        if (m_screenSpaceRenderer)
        {
            if (m_ortho)
            {
                printf("Orthogonal rendering not implemented for ScreenSpaceFluidRendererGL.\n");
                return;
            }

           // for (int i = 0; i < m_fluids.size(); ++i)
           // {
            const btFluidSphParametersLocal& FL = m_fluidSph->getLocalParameters();
                btScalar particleRadius = FL.m_particleRadius;

                float r = 0.5f;
                float g = 0.8f;
                float b = 1.0f;

                //Beer's law constants
                //Controls the darkening of the fluid's color based on its thickness
                //For a constant k, (k > 1) == darkens faster; (k < 1) == darkens slower; (k == 0) == disable
                float absorptionR = 2.5;
                float absorptionG = 1.0;
                float absorptionB = 0.5;

                if (drawFluidsWithMultipleColors)
                {
                    r = 0.3f;
                    g = 0.7f;
                    b = 1.0f;
                    if (1 % 2)
                    {
                        r = 1.0f - r;
                        g = 1.0f - g;
                        b = 1.0f - b;
                    }

                    absorptionR = 1.0;
                    absorptionG = 1.0;
                    absorptionB = 1.0;
                }

                m_screenSpaceRenderer->render(m_fluidSph->internalGetParticles().m_pos, particleRadius * 1.5f,
                    r, g, b, absorptionR, absorptionG, absorptionB);
           // }
        }
    }
    else 	//(m_fluidRenderMode == FRM_MarchingCubes)
    {
        //BT_PROFILE("Draw fluids - marching cubes");

        const int CELLS_PER_EDGE = 32;
        static MarchingCubes* marchingCubes = 0;
        if (!marchingCubes)
        {
            marchingCubes = new MarchingCubes;
            marchingCubes->initialize(CELLS_PER_EDGE);
        }

        //for (int i = 0; i < m_fluids.size(); ++i)
        //{
        marchingCubes->generateMesh(*m_fluidSph);
        const btAlignedObjectArray<float>& vertices = marchingCubes->getTriangleVertices();
        if (vertices.size())
        {
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

            float r = 0.4f;
            float g = 0.4f;
            float b = 0.9f;
            if (drawFluidsWithMultipleColors)
            {
                r = 0.3f;
                g = 0.7f;
                b = 1.0f;
                if (1 % 2)
                {
                    r = 1.0f - r;
                    g = 1.0f - g;
                    b = 1.0f - b;
                }
            }

            glEnableClientState(GL_VERTEX_ARRAY);
            glColor4f(r, g, b, 0.6f);
            glVertexPointer(3, GL_FLOAT, 0, &vertices[0]);
            glDrawArrays(GL_TRIANGLES, 0, vertices.size() / 3);
            glDisableClientState(GL_VERTEX_ARRAY);
        }
        //}
    }
}

void ClothDemo::displayCallback(void)
{
    //BT_PROFILE() does not work correctly in this function;
    //timings are captured only when the camera is moving.
    //BT_PROFILE("ClothDemo::displayCallback()");

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    renderme();

    renderFluids();

    /*
    static bool areSpheresGenerated = false;
    static GLuint glLargeSphereList;
    static GLuint glSmallSphereList;
    if (!areSpheresGenerated)
    {
        areSpheresGenerated = true;

        const float RADIUS(1.0);	//Particle collision radius in btFluidSph::getLocalParameters() should also be changed if this is modified
        glSmallSphereList = generateSphereList(RADIUS*0.333f);
        glLargeSphereList = generateSphereList(RADIUS);
    }



    GLuint sphereList = glSmallSphereList;
    if (debugDrawer && !(debugDrawer->getDebugMode() & btIDebugDraw::DBG_DrawWireframe)) sphereList = glLargeSphereList;

    //Draw SPH particles
    
    {
        for (int i = 0; i < m_fluidSoftRigidWorld->getNumFluidSph(); ++i)
        {
            btFluidSph* fluidSph = m_fluidSoftRigidWorld->getFluidSph(i);
            for (int n = 0; n < fluidSph->numParticles(); ++n)
            {
                const btVector3& position = fluidSph->getPosition(n);
                drawSphere(sphereList, position, 0.6f, 0.9f, 1.0f);
            }
        }
    }
    */
    btIDebugDraw* debugDrawer = m_fluidSoftRigidWorld->getDebugDrawer();
    //Draw soft bodies
    if (debugDrawer && !(debugDrawer->getDebugMode() & btIDebugDraw::DBG_DrawWireframe))
    {
        btAlignedObjectArray<btSoftBody*> softBodies = m_fluidSoftRigidWorld->getSoftBodyArray();
        for (int i = 0; i < softBodies.size(); ++i)
        {
            btSoftBody*	softBody = static_cast<btSoftBody*>(softBodies[i]);
            btSoftBodyHelpers::DrawFrame(softBody, debugDrawer);
            btSoftBodyHelpers::Draw(softBody, debugDrawer, m_fluidSoftRigidWorld->getDrawFlags());
        }
    }

    if (m_fluidSoftRigidWorld) m_fluidSoftRigidWorld->debugDrawWorld();		//Optional but useful: debug drawing to detect problems

    if ((getDebugMode() & btIDebugDraw::DBG_NoHelpText) == 0)
    {
        setOrthographicProjection();
        glDisable(GL_LIGHTING);
        glColor4f(0.0f, 0.0f, 0.0f, 1.0f);

        const int LINE_WIDTH = 360;
        int position_x = m_glutScreenWidth - LINE_WIDTH;
        int position_y = 20;

        glRasterPos3f(position_x, position_y, 0);
        GLDebugDrawString(position_x, position_y, "Press / to spray SPH particles");

        resetPerspectiveProjection();
        glEnable(GL_LIGHTING);
    }

    glFlush();
    swapBuffers();
}



void ClothDemo::keyboardCallback(unsigned char key, int x, int y)
{
    switch (key)
    {
    case 'q':
    {
        int currentRenderMode = static_cast<int>(m_fluidRenderMode);
        m_fluidRenderMode = static_cast<FluidRenderMode>(currentRenderMode + 1);

        if (m_fluidRenderMode > FRM_MarchingCubes)m_fluidRenderMode = FRM_Points;
        return;
    }
    case ' ':
        clientResetScene();
        break;

    case '/':
        if (m_fluidSph)
        {
            const btScalar SPEED(1.0);
            btVector3 position = getCameraPosition();
            position.setY(position.y() - btScalar(5.0));

            btVector3 direction = (getRayTo(x, y) - position).normalized();
            btVector3 velocity = direction * SPEED;

            const btScalar SPACING(2.5);
            const btVector3 X_AXIS(1, 0, 0);
            const btVector3 Y_AXIS(0, 1, 0);
            const btVector3 Z_AXIS(0, 0, 1);
            btQuaternion rotation = shortestArcQuat(Z_AXIS, direction);
            btVector3 up = quatRotate(rotation, Y_AXIS) * SPACING;
            btVector3 left = quatRotate(rotation, X_AXIS) * SPACING;

            emitParticle(m_fluidSph, position, velocity);

            emitParticle(m_fluidSph, position + up, velocity);
            emitParticle(m_fluidSph, position - up, velocity);
            emitParticle(m_fluidSph, position + left, velocity);
            emitParticle(m_fluidSph, position - left, velocity);

            emitParticle(m_fluidSph, position + up + left, velocity);
            emitParticle(m_fluidSph, position + up - left, velocity);
            emitParticle(m_fluidSph, position - up + left, velocity);
            emitParticle(m_fluidSph, position - up - left, velocity);
        }
        break;

    default:
        PlatformDemoApplication::keyboardCallback(key, x, y);
        break;
    }
}

void ClothDemo::specialKeyboard(int key, int x, int y)
{
    switch (key)
    {
    case GLUT_KEY_END:
    {
        int numObj = getDynamicsWorld()->getNumCollisionObjects();
        if (numObj)
        {
            btCollisionObject* obj = getDynamicsWorld()->getCollisionObjectArray()[numObj - 1];
            if (btFluidSph::upcast(obj)) return;	//Deleting btFluidSph will cause crashes in ClothDemo

            getDynamicsWorld()->removeCollisionObject(obj);

            btRigidBody* body = btRigidBody::upcast(obj);
            if (body && body->getMotionState()) delete body->getMotionState();
            delete obj;

        }
        return;
    }
    }

    PlatformDemoApplication::specialKeyboard(key, x, y);
}

void ClothDemo::setShootBoxShape()
{
    if (!m_shootBoxShape)
    {
        const btScalar BOX_DIMENSIONS(3.0);

        btBoxShape* box = new btBoxShape(btVector3(BOX_DIMENSIONS, BOX_DIMENSIONS, BOX_DIMENSIONS));
        box->initializePolyhedralFeatures();
        m_shootBoxShape = box;
    }
}

void ClothDemo::myinit()
{
    DemoApplication::myinit();

    //ScreenSpaceFluidRendererGL may initialize GLEW, which requires an existing OpenGL context
    if (!m_screenSpaceRenderer) m_screenSpaceRenderer = new ScreenSpaceFluidRendererGL(m_glutScreenWidth, m_glutScreenHeight);
}

void ClothDemo::reshape(int w, int h)
{
    DemoApplication::reshape(w, h);

    if (m_screenSpaceRenderer)
    {
        m_screenSpaceRenderer->setWindowResolution(w, h);
        m_screenSpaceRenderer->setRenderingResolution(w, h);
    }
}

void ClothDemo::clientResetScene()
{
    exitPhysics();
    initPhysics();
}

