#ifndef FLUID_SOFT_INTERACTION_H
#define FLUID_SOFT_INTERACTION_H

#include "LinearMath/btQuickprof.h"
#include "LinearMath/btAabbUtil2.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletSoftBody/btSoftBodyInternals.h"
#include "BulletFluids/Sph/btFluidSph.h"
static bool resolveFuildSoftContactNode(btVector3& aa, btVector3& bb, btVector3& cc, btVector3& pp)
{
    btVector3 v0 = cc - aa;     //AC
    btVector3 v1 = bb - aa;     //AB
    btVector3 v2 = pp - aa;     //AP

    if (v0.isZero() || v1.isZero() || v2.isZero())
    {
        return false;
    }
    btVector3 cro01 = btCross(v0, v1);
    btScalar coplanr = btDot(cro01, v2);
    //if the four point coplanar
    if (coplanr != 0.0f)
    {
        return false;
    }
    btScalar dot00 = btDot(v0, v0); 
    btScalar dot01 = btDot(v0, v1); 
    btScalar dot02 = btDot(v0, v2); 
    btScalar dot11 = btDot(v1, v1);
    btScalar dot12 = btDot(v1, v2); 

    btScalar inverDeno = 1.0 / (dot00 * dot11 - dot01 *dot01);
    btScalar u = (dot11 * dot02 - dot01 * dot12) * inverDeno;
    if (u < 0 || u > 1) // if u out of range, return directly
    {
        return false;
    }

    float v = (dot00 * dot12 - dot01 * dot02) * inverDeno;
    if (v < 0 || v > 1) // if v out of range, return directly
    {
        return false;
    }

    return u + v <= 1;
}
static void resolveFluidSoftContactImpulse(const btFluidSphParametersGlobal& FG, btSoftBody* softbody, btFluidSph* fluid, int sphParticleIndex,
    btSoftBody::Cluster* softCluster, const btVector3& normalOnSoftBody,
    const btVector3&pointOnSoftBody, btScalar distance)
{
    const btFluidSphParametersLocal& FL = fluid->getLocalParameters();

    if (distance < btScalar(0.0))
    {
        int i = sphParticleIndex;
        btFluidParticles& particles = fluid->internalGetParticles();

        btSoftBody::Body clusterBody(softCluster);
        btVector3 softLocalHitPoint = pointOnSoftBody - clusterBody.xform().getOrigin();

        btVector3 fluidVelocity = fluid->getVelocity(i);
        btVector3 softVelocity = clusterBody.velocity(softLocalHitPoint);
        softVelocity *= FG.m_simulationScale;

        btVector3 relativeVelocity = fluidVelocity - softVelocity;
        btScalar penetratingMagnitude = relativeVelocity.dot(-normalOnSoftBody);
        if (penetratingMagnitude < btScalar(0.0)) penetratingMagnitude = btScalar(0.0);

        btVector3 penetratingVelocity = -normalOnSoftBody * penetratingMagnitude;
        btVector3 tangentialVelocity = relativeVelocity - penetratingVelocity;

        penetratingVelocity *= btScalar(1.0) + FL.m_boundaryRestitution;

        btScalar positionError = (-distance) * (FG.m_simulationScale / FG.m_timeStep) * FL.m_boundaryErp;
        btVector3 particleImpulse = -(penetratingVelocity + (-normalOnSoftBody*positionError) + tangentialVelocity*FL.m_boundaryFriction);

        const bool APPLY_IMPULSE_TO_SOFT_BODY = true;
        if (APPLY_IMPULSE_TO_SOFT_BODY)
        {
            btScalar inertiaParticle = btScalar(1.0) / FL.m_particleMass;

            btVector3 relPosCrossNormal = softLocalHitPoint.cross(normalOnSoftBody);
            btScalar inertiaSoft = clusterBody.invMass() + (relPosCrossNormal * clusterBody.invWorldInertia()).dot(relPosCrossNormal);

            particleImpulse *= btScalar(1.0) / (inertiaParticle + inertiaSoft);

            btVector3 worldScaleImpulse = -particleImpulse / FG.m_simulationScale;

            //	Apply the impulse to all nodes(vertices) in the soft body cluster
            //	this is incorrect, but sufficient for demonstration purposes

            btVector3 perNodeImpulse = worldScaleImpulse / static_cast<btScalar>(softCluster->m_nodes.size());
            perNodeImpulse /= FG.m_timeStep;		//Impulse is accumulated as force

            for (int j = 0; j < softCluster->m_nodes.size(); ++j)
            {
                btSoftBody::Node* node = softCluster->m_nodes[j];
                node->m_f += perNodeImpulse;
            }


            // ssxx
            // Try Apply the impulse to the correct vertices in the soft body cluster
            for (int j = 0; j < softCluster->m_nodes.size(); j = j + 3)
            {
                int size = softCluster->m_nodes.size();
                btVector3 aa = softCluster->m_nodes[j]->m_x;
                btVector3 bb = softCluster->m_nodes[j + 1]->m_x;
                btVector3 cc = softCluster->m_nodes[j + 2]->m_x;
                btVector3 pp = pointOnSoftBody;

                if (resolveFuildSoftContactNode(aa, bb, cc, pp))
                {

                    /*
                    btVector3 perNodeImpulse = worldScaleImpulse / 3.0;
                    perNodeImpulse /= FG.m_timeStep;
                    softCluster->m_nodes[j]->m_f += perNodeImpulse;
                    softCluster->m_nodes[j + 1]->m_f += perNodeImpulse;
                    softCluster->m_nodes[j + 2]->m_f += perNodeImpulse;
                    */
                    //absorb
                    /*
                    if (softCluster->m_nodes[j]->m_absorb < 9)
                    {
                    softCluster->m_nodes[j]->m_absorb += 1;
                    //fluid->markParticleForRemoval(i);
                    }
                    else if (softCluster->m_nodes[j + 1]->m_absorb < 9)
                    {
                    softCluster->m_nodes[j + 1]->m_absorb += 1;
                    //fluid->markParticleForRemoval(i);
                    }
                    else if (softCluster->m_nodes[j + 2]->m_absorb < 9)
                    {
                    softCluster->m_nodes[j + 2]->m_absorb += 1;
                    //fluid->markParticleForRemoval(i);
                    }
                    else
                    {
                    }
                    */
                    //find the face that shares the same nodes with this cluster
                    /**/
                    for (int x = 0; x < softbody->m_faces.size(); ++x)
                    {
                        btSoftBody::Face& f = softbody->m_faces[x];
                        btSoftBody::Node* node[3] = { f.m_n[0], f.m_n[1], f.m_n[2] };
                        int c = 0;
                        for (int y = 0; y < 3; ++y)
                        {
                            if (softCluster->m_nodes[y] == node[0] ||
                                softCluster->m_nodes[y] == node[1] ||
                                softCluster->m_nodes[y] == node[2])
                            {
                                c++;
                            }
                        }
                        if (3 == c)
                        {
                            // found the face
                            // do the absorb
                            if (f.m_absorb < 3.f)
                            {
                                f.m_absorb++;
                                fluid->markParticleForRemoval(i);
                            }

                        }

                    }
                }
            }
            // ssxx
            particleImpulse *= inertiaParticle;

            // Particles vel of fluid need to be modified
            btVector3& vel = particles.m_vel[i];
            btVector3& vel_eval = particles.m_vel_eval[i];

            //Leapfrog integration
            btVector3 velNext = vel + particleImpulse;
            vel_eval = (vel + velNext) * btScalar(0.5);
            vel = velNext;
            vel.setY(vel.getY()*0.1);
        }
        else
        {
            btVector3& vel = particles.m_vel[i];
            btVector3& vel_eval = particles.m_vel_eval[i];

            //Leapfrog integration
            btVector3 velNext = vel + particleImpulse;
            vel_eval = (vel + velNext) * btScalar(0.5);
            vel = velNext;
        }
    }
}

///Preliminary soft body - SPH fluid interaction demo; this class is not supported.
struct ParticleSoftBodyCollisionCallback : public btDbvt::ICollide
{
	//All members must be set
	const btFluidSphParametersGlobal* m_globalParameters;
	
	btFluidSph* m_fluidSph;
	btSoftBody* m_softBody;
	
	int m_sphParticleIndex;
	btCollisionObject* m_particleObject;
	
	
	virtual void Process(const btDbvtNode* leaf)
	{
		BT_PROFILE("FluidSoft Process softBodyClusterLeaf");
		
		const btFluidSphParametersLocal& FL = m_fluidSph->getLocalParameters();
		
		btSoftBody::Cluster* softCluster = static_cast<btSoftBody::Cluster*>(leaf->data);
		btSoftClusterCollisionShape clusterShape(softCluster);
		
		btTransform& particleTransform = m_particleObject->getWorldTransform();
		btConvexShape* particleShape = static_cast<btConvexShape*>( m_particleObject->getCollisionShape() );
		
		btGjkEpaSolver2::sResults contactResult;            
		if( btGjkEpaSolver2::SignedDistance(&clusterShape, btTransform::getIdentity(), 
											particleShape, particleTransform, btVector3(1,0,0), contactResult) )
		{
			btVector3 normalOnCluster = -contactResult.normal;
			btVector3 pointOnCluster = contactResult.witnesses[0];
			btScalar distance = contactResult.distance - FL.m_particleRadius;
			
            resolveFluidSoftContactImpulse(*m_globalParameters, m_softBody, m_fluidSph, m_sphParticleIndex,
                softCluster, normalOnCluster, pointOnCluster, distance);
		}
		
	}
};

///Preliminary soft body - SPH fluid interaction demo; this class is not supported.
struct FluidSphSoftBodyCollisionCallback : public btFluidSortingGrid::AabbCallback
{
	//All members must be set
	const btFluidSphParametersGlobal* m_globalParameters;
	
	btFluidSph* m_fluidSph;
	btSoftBody* m_softBody;
	
	btCollisionObject* m_particleObject;
	
	FluidSphSoftBodyCollisionCallback() {}
	
	
	virtual bool processParticles(const btFluidGridIterator FI, const btVector3& aabbMin, const btVector3& aabbMax)
	{
		BT_PROFILE("FluidSoft processParticles()");
		btTransform& particleTransform = m_particleObject->getWorldTransform();
		
		//Collide each SPH particle as a rigid body sphere against the soft body's clusters 
		for(int n = FI.m_firstIndex; n <= FI.m_lastIndex; ++n)
		{
			particleTransform.setOrigin( m_fluidSph->getPosition(n) );
		
			btVector3 particleMin, particleMax;
			m_particleObject->getCollisionShape()->getAabb(particleTransform, particleMin, particleMax);
			
			ParticleSoftBodyCollisionCallback dbvtCallback;
			dbvtCallback.m_globalParameters = m_globalParameters;
			dbvtCallback.m_fluidSph = m_fluidSph;
			dbvtCallback.m_softBody = m_softBody;
			dbvtCallback.m_sphParticleIndex = n;
			dbvtCallback.m_particleObject = m_particleObject;
		
			const btDbvt& softBodyClusterDbvt = m_softBody->m_cdbvt;
			btDbvtVolume particleAabb = btDbvtVolume::FromMM(particleMin, particleMax);
			
			softBodyClusterDbvt.collideTV(softBodyClusterDbvt.m_root, particleAabb, dbvtCallback);
		}
		
		return true;
	}

};

///Preliminary soft body - SPH fluid interaction demo; this class is not supported.
struct FluidSoftInteractor
{
	static void collideFluidSphWithSoftBody(const btFluidSphParametersGlobal& FG, 
											btFluidSph* fluidSph, btSoftBody* softBody, const btVector3& softAabbMin, const btVector3& softAabbMax)
	{
		BT_PROFILE("collideFluidSphWithSoftBody()");
	
		const btFluidSphParametersLocal& FL = fluidSph->getLocalParameters();
		const btFluidSortingGrid& grid = fluidSph->getGrid();
		
		btSphereShape particleShape(FL.m_particleRadius);
		
		btCollisionObject particleObject;
		particleObject.setCollisionShape(&particleShape);
		particleObject.getWorldTransform().setIdentity();
		
		//btFluidSortingGrid uses only the particle centers(without radius)
		//add the radius to the soft body AABB to avoid missing collisions
		const btVector3 fluidRadius(FL.m_particleRadius, FL.m_particleRadius, FL.m_particleRadius);
		btVector3 expandedSoftMin = softAabbMin - fluidRadius;
		btVector3 expandedSoftMax = softAabbMax + fluidRadius;
		
		FluidSphSoftBodyCollisionCallback fluidSoftCollider;
		fluidSoftCollider.m_globalParameters = &FG;
		fluidSoftCollider.m_fluidSph = fluidSph;
		fluidSoftCollider.m_softBody = softBody;
		fluidSoftCollider.m_particleObject = &particleObject;
        //ssxx debug
        /*
        int sum0 = 0;
        int sum1 = 0;
        int sum2 = 0;
        int sum3 = 0;
        int sum4 = 0;
        int sum5 = 0;
        int sum6 = 0;
        int sum7 = 0;
        int sum8 = 0;
        int sum9 = 0;
        for (int i = 0; i < softBody->m_nodes.size(); ++i)
        {
            int curr = softBody->m_nodes[i].m_absorb;
            switch (curr)
            {
            case 0:
                ++sum0;
                break;
            case 1:
                ++sum1;
                break;
            case 2:
                ++sum2;
                break;
            case 3:
                ++sum3;
                break;
            case 4:
                ++sum4;
                break;
            case 5:
                ++sum5;
                break;
            case 6:
                ++sum6;
                break;
            case 7:
                ++sum7;
                break;
            case 8:
                ++sum8;
                break;
            case 9:
                ++sum9;
                break;
            default:
                break;
            }

        }
        printf("Sum0 %d\n", sum0);
        printf("Sum1 %d\n", sum1);
        printf("Sum2 %d\n", sum2);
        printf("Sum3 %d\n", sum3);
        printf("Sum4 %d\n", sum4);
        printf("Sum5 %d\n", sum5);
        printf("Sum6 %d\n", sum6);
        printf("Sum7 %d\n", sum7);
        printf("Sum8 %d\n", sum8);
        printf("Sum9 %d\n", sum9);
        //ssxx debug
        */
		//Call FluidSphSoftBodyCollisionCallback::processParticles() for
		//each SPH fluid grid cell intersecting with the soft body's AABB
		grid.forEachGridCell(expandedSoftMin, expandedSoftMax, fluidSoftCollider);
	}
	
	static void performCollisionDetectionAndResponse(const btFluidSphParametersGlobal& FG, btAlignedObjectArray<btFluidSph*>& fluids,
													btAlignedObjectArray<btSoftBody*>& softBodies, btScalar timeStep)
	{
		BT_PROFILE("btFluidSph - btSoftBody interaction");
	
		for(int i = 0; i < fluids.size(); ++i)
		{
			btFluidSph* fluidSph = fluids[i];
			if( !fluidSph->numParticles() ) continue;
			btVector3 fluidAabbMin, fluidAabbMax;
			fluidSph->getAabb(fluidAabbMin, fluidAabbMax);
			
			for(int n = 0; n < softBodies.size(); ++n)
			{
				btSoftBody* softBody = softBodies[n];
				btVector3 softAabbMin, softAabbMax;
				softBody->getAabb(softAabbMin, softAabbMax);
				if( TestAabbAgainstAabb2(fluidAabbMin, fluidAabbMax, softAabbMin, softAabbMax) )
					collideFluidSphWithSoftBody(FG, fluidSph, softBody, softAabbMin, softAabbMax);
			}
		}
	}
};


#endif