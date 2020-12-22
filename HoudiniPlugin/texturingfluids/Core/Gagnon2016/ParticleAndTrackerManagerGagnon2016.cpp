#include "ParticleAndTrackerManagerGagnon2016.h"

#include <vector>
#include <algorithm>
#include <SYS/SYS_Math.h>
#include <UT/UT_Interrupt.h>
#include <UT/UT_Matrix3.h>
#include <UT/UT_Matrix4.h>
#include <GU/GU_Detail.h>
#include <GU/GU_PrimPoly.h>
#include <PRM/PRM_Include.h>
#include <PRM/PRM_SpareData.h>
#include <SOP/SOP_Guide.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <GA/GA_ElementWrangler.h>
#include <algorithm>
#include <ctime>
#include <GEO/GEO_PointTree.h>
#include <GEO/GEO_PrimClassifier.h>
#include <GEO/GEO_PointClassifier.h>
#include <GEO/GEO_PrimConnector.h>
#include <GEO/GEO_PrimVDB.h>
#include <GU/GU_NeighbourList.h>
#include <GU/GU_RayIntersect.h>
#include <GU/GU_Flatten.h>



//#include <Strategies/StrategySurfaceTextureSynthesis.h>
#include <Core/HoudiniUtils.h>


ParticleAndTrackerManagerGagnon2016::ParticleAndTrackerManagerGagnon2016(GU_Detail *surfaceGdp, GU_Detail *trackersGdp, ParametersDeformablePatches params)
{
    this->numberOfPatches = 0;
    this->maxId = 0;
    this->markerAdvectionTime = 0;
    this->numberOfConcealedPatches = 0;
    this->numberOfNewPatches = 0;
    this->numberOfDetachedPatches = 0;
    this->numberOfLonelyTracker = 0;

    this->attN =  GA_RWHandleV3(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));
    this->attCenterUV =  GA_RWHandleV3(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"centerUV", 3));
    //GA_RWHandleV3   attV(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"v", 3));
    this->attV = GA_RWHandleV3(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"v", 3));

    this->attId  = GA_RWHandleI(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"id",1));
    this->attLife  = GA_RWHandleF(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"life",1));
    this->attSpawn  = GA_RWHandleI(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"spawn",1));
    this->attActive  = GA_RWHandleI(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"active", 1));
    this->attIsMature  = GA_RWHandleI(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"isMature", 1));
    this->attDensity =  GA_RWHandleI(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"density", 1));
    this->attBlend  = GA_RWHandleF(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"temporalComponetKt", 1));
    this->attRandT  = GA_RWHandleF(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,randomThresholdDistortion,1));
    this->attMaxDeltaOnD  = GA_RWHandleF(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"maxDeltaOnD",1));
    this->attDeleteFaster =  GA_RWHandleI(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"deleteFaster", 1));
    this->refAttV  = GA_RWHandleV3(surfaceGdp->addFloatTuple(GA_ATTRIB_POINT,"v", 3));
    this->refAttN  = GA_RWHandleV3(surfaceGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));
    this->temporalComponentKt = GA_RWHandleF(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"temporalComponetKt", 1));
    this->attFadeIn  = GA_RWHandleI(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"fadeIn",1));
    this->isTangeantTracker = GA_RWHandleI(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"isTrangeantTracker",1));
    this->AttCd = GA_RWHandleV3(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"Cd", 3));

    this->attNumberOfPrimitives= GA_RWHandleI(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"numberOfPrimitives",1));

    this->attVSurface = GA_RWHandleV3(surfaceGdp->addFloatTuple(GA_ATTRIB_POINT,"v", 3));
    this->attDivergence = GA_RWHandleF(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"divergence",1));

    this->trackersGdp = trackersGdp;
    this->surface = surfaceGdp;
    this->params = params;

}

vector<GA_Offset> ParticleAndTrackerManagerGagnon2016::InitializeTrackersAndTangeants(GU_Detail* surface,GU_Detail *trackers, GA_PointGroup *surfaceGroup, ParametersDeformablePatches params)
{

}


vector<GA_Offset> ParticleAndTrackerManagerGagnon2016::PoissonDiskSamplingDistribution(GU_Detail *levelSet, float diskRadius, float angleNormalThreshold)
{
    cout << "[Bridson2012PoissonDiskDistributionGagnon2020] on level set using a threshold of "<<angleNormalThreshold<<endl;

    std::clock_t addPoissonDisk;
    addPoissonDisk = std::clock();

    GA_RWHandleV3   attN(trackersGdp->findFloatTuple(GA_ATTRIB_POINT,"N", 3));
    GA_RWHandleI    attActive(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"active", 1));
    GA_RWHandleI    attId(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"id",1));
    GA_RWHandleI    attDensity(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"density", 1));


    vector<GA_Offset> newPoissonDisk;

    this->numberOfNewPoints = 0;

    // Find first vdb primitive of input 0
    GEO_Primitive* prim;
    GEO_PrimVDB* phi = 0x0;
    GA_FOR_ALL_PRIMITIVES(levelSet, prim)
    {
        if (phi = dynamic_cast<GEO_PrimVDB*>(prim))
            break;
    }

    if (!phi || !phi->hasGrid())
    {
        cout << "[Bridson2012PoissonDiskDistributionGagnon2026] Input geometry 0 has no VDB grid!"<<endl;
        return newPoissonDisk;
    }

//    cout << "Grid name: " << phi->getGridName() << std::endl;
//    cout << "Storage type: " << phi->getStorageType() << ", " << phi->getTupleSize() << std::endl;
//    cout << "JSON: " << phi->getJSON() << std::endl;

    float a = 0.25; //promote this variable to the user interface
    this->poissonDiskRadius = diskRadius;
    float killDistance = (1-a)*diskRadius/2;

    cout << "[Bridson2012PoissonDiskDistributionGagnon2026] We have a valid vdb"<<endl;

    GA_RWHandleI    attDeleteFaster(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"deleteFaster", 1));
    GA_Offset ppt;
    // Only if we want to delete too close patches, which is not the case when we don't use fading in

    GA_FOR_ALL_PTOFF(trackersGdp,ppt)
    {
        int numberOfClosePoint;

        UT_Vector3 pointPosition = trackersGdp->getPos3(ppt);
        UT_Vector3 pointNormal   = attN.get(ppt);
        if (attId.get(ppt) > this->maxId)
            this->maxId = attId.get(ppt);


        bool meetPoissonDiskCriterion = this->RespectCriterion(pointPosition, pointNormal, killDistance,  numberOfClosePoint, ppt);
        attDensity.set(ppt,numberOfClosePoint);

        if (attActive.get(ppt) == 0)
            continue;
        //If we have fading in, we are using 2019's approach
        //if (params.fadingIn == 1)
        {
            //attActive.set(ppt,meetPoissonDiskCriterion);
        }
        if (!meetPoissonDiskCriterion)
        {
            //cout << "We should delete point "<<attId.get(ppt)<<", is in kill distance" <<killDistance<<endl;
        }
    }


    t = 30;

//    cout << "Grid name: " << phi->getGridName() << std::endl;
//    cout << "Storage type: " << phi->getStorageType() << ", " << phi->getTupleSize() << std::endl;
//    cout << "JSON: " << phi->getJSON() << std::endl;

    openvdb::GridBase::Ptr ptr = phi->getGridPtr();
    openvdb::FloatGrid::Ptr gridSurface = openvdb::gridPtrCast<openvdb::FloatGrid>(ptr);
    if(!gridSurface)
    {
        cout << "[Bridson2012PoissonDiskDistributionGagnon2026] Surface grid can't be converted in FloatGrid"<<endl;
        return newPoissonDisk;
    }
    if ((gridSurface->getGridClass() != openvdb::GRID_LEVEL_SET))
    {
        cout<< "[Bridson2012PoissonDiskDistributionGagnon2026] Surface grid is not a Level-set FloatGrid!"<<endl;
        return newPoissonDisk;
    }

    //=================================================================
    //                         OPEN VDB ACCESSORS
    //=================================================================

    // Create the gradient field
    openvdb::tools::Gradient<openvdb::FloatGrid> gradientOperator(*gridSurface);
    openvdb::VectorGrid::ConstPtr gridGradient = gradientOperator.process();
    //gridGradient->setName(ssGrad.str());
    openvdb::VectorGrid::ConstAccessor accessorGradient = gridGradient->getConstAccessor();
    openvdb::tools::GridSampler<openvdb::VectorGrid::ConstAccessor, openvdb::tools::BoxSampler>
            samplerGradient(accessorGradient, gridGradient->transform());
     // Get the sampler for the boundary grid (tri-linear filtering, in surface grid index space)
    openvdb::FloatGrid::Accessor accessorSurface = gridSurface->getAccessor();
    openvdb::tools::GridSampler<openvdb::FloatGrid::Accessor, openvdb::tools::BoxSampler>
            samplerSurface(accessorSurface, gridSurface->transform());

    //=================================================================
    // 1: for all grid cells C where φ changes sign do
    //=================================================================
    int nbOfCell = 0;

    //cout << "[Bridson2012PoissonDiskDistributionGagnon2026] Step 1: for all grid cells C where φ changes sign do"<<endl;
    for (openvdb::FloatGrid::ValueOnCIter gridCellIt = gridSurface->cbeginValueOn(); gridCellIt; ++gridCellIt)
    {


        float x = gridCellIt.getCoord().x();
        float y = gridCellIt.getCoord().y();
        float z = gridCellIt.getCoord().z();

        float offset = 0.5;

        openvdb::Vec3f cellPosition(x,y,z);
        openvdb::Vec3f worldCellPos = gridSurface->transform().indexToWorld(cellPosition);
        float boundaryDist = samplerSurface.wsSample(worldCellPos);
        //openvdb::Vec3f p    = it.getCoord();
        //if (boundaryDist <= 0.0)// && grad.length() > 0.0)
        {
            //if it is not close to the surface, continue
            if (abs(boundaryDist) > params.CellSize/2.0f) // We should use a threshold defined by the user
                continue;
            //=================================================================
            //2:  for t attempts do
            //=================================================================
            bool ableToInsertPoint = false;
            for(int i =0; i < t; i++)
            {
                //=================================================================
                //3:      Generate random point p in C
                //=================================================================
                int seed = i;
                //we want it to oscillate between -0.5 and 0.5

                srand(seed);
                float rx = (((double) rand()/(RAND_MAX)-offset));
                srand(seed+1);
                float ry = (((double) rand()/(RAND_MAX)-offset));
                srand(seed+2);
                float rz = (((double) rand()/(RAND_MAX)-offset));

                openvdb::Vec3f randomPosition(rx,ry,rz);
                randomPosition *= params.CellSize;

                openvdb::Vec3f p = worldCellPos+randomPosition;

                float newPointDistance = samplerSurface.wsSample(p);
                if (abs(newPointDistance) > params.poissondiskradius)
                {
                    //cout << "random point is outside of range"<<endl;
                    continue;
                }
                //cout << "abs(newPointDistance) > poissonDiskRadius"<<endl;
                //=================================================================
                //4:      Project p to surface of φ
                //=================================================================
                openvdb::Vec3f grad = samplerGradient.wsSample(p);
                if (grad.length() < 0.0001)
                    continue;
                openvdb::Vec3f poissonDisk = projectPointOnLevelSet(p,newPointDistance,grad);
                UT_Vector3 newPointPosition = UT_Vector3(poissonDisk.x(),poissonDisk.y(),poissonDisk.z());
                grad.normalize();
                UT_Vector3 newPointNormal = UT_Vector3(grad.x(),grad.y(),grad.z());

                //=================================================================
                //5:      if p meets the Poisson Disk criterion in S then
                //=================================================================
                int numberOfClosePoint;
                //cout << "Trying to fit "<<newPointPosition<<endl;
                bool meetPoissonDiskCriterion = this->RespectCriterion(newPointPosition, newPointNormal, poissonDiskRadius, numberOfClosePoint, -1);
                if (meetPoissonDiskCriterion)
                {
                    //=================================================================
                    //6:          S ← S ∪ {p}
                    //=================================================================

                    GA_Offset newPoint = this->CreateAParticle(newPointPosition, newPointNormal);
                    newPoissonDisk.push_back(newPoint);
                    break;
                }
            }
            if (!ableToInsertPoint)
            {
                //cout << "after "<<t<<" attemps, there is no possible insertion."<<endl;
            }
        }
        nbOfCell++;
    }
    this->numberOfNewPatches = this->numberOfNewPoints;
    cout << this->approachName<<"[Bridson2012PoissonDiskDistributionGagnon2026] poisson disk sample result: "<< this->numberOfNewPatches<< " new point(s)"<<endl;
    this->poissondisk += (std::clock() - addPoissonDisk) / (double) CLOCKS_PER_SEC;
    cout << this->approachName<<"[Bridson2012PoissonDiskDistributionGagnon2026] Total :"<<trackersGdp->getNumPoints()<<endl;
    cout << "[Bridson2012PoissonDiskDistributionGagnon2026] "<< nbOfCell <<  "cells have been treated."<<endl;
    return newPoissonDisk;
}


//================================================================================================

//                                 PROJECT POINT ON LEVEL SET

//================================================================================================


openvdb::Vec3f ParticleAndTrackerManagerGagnon2016::projectPointOnLevelSet(openvdb::Vec3f point, float distance, openvdb::Vec3f grad)
{
    //get the norm of the gradient
    openvdb::Vec3f gradNorm = grad;
    gradNorm.normalize();

    //=================================================================
    //4:      Project p to surface of φ
    //=================================================================
    //projection
    //cout << "old p "<<p<<endl;
    //p = p - dist * (grad/gradNorm);
    point = point - distance * gradNorm;

    return point;
}

bool ParticleAndTrackerManagerGagnon2016::RespectCriterion(UT_Vector3 newPointPosition, UT_Vector3 newPointNormal, float killDistance, int &numberOfClosePoint,   GA_Offset exclude )
{
    numberOfClosePoint = 0;

    GA_RWHandleV3   attN(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));
    GA_RWHandleI    attActive(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"active", 1));

    GEO_PointTreeGAOffset::IdxArrayType close_particles_indices;

    this->trackerTree.findAllCloseIdx(newPointPosition,
                         params.poissondiskradius*2,
                         close_particles_indices);

    int l = (int)close_particles_indices.entries();
    GA_Offset neighbor;
    bool tooClose = false;

    float cs    = params.CellSize;
    float r     = params.poissondiskradius;

    newPointNormal.normalize();
    float kd = killDistance;

    UT_Vector3 defaultDirection(1.012f,0.123f,0.002f);
    UT_Vector3 S,T;

    for(int j=0; j<l;j++)
    {
        neighbor = close_particles_indices.array()[j];
        if (attActive.get(neighbor) == 0)
            continue;
        if (neighbor == exclude)
            continue;

        UT_Vector3 pos          = trackersGdp->getPos3(neighbor);

        UT_Vector3 N            = attN.get(neighbor);
        N.normalize();
        S = cross(N,defaultDirection);
        S.normalize();
        T = cross(S,N);
        T.normalize();


        // Transform into local patch space (where STN is aligned with XYZ at the origin)
        const UT_Vector3 relativePosistion = pos - newPointPosition;
        UT_Vector3 poissonDiskSpace;
        poissonDiskSpace.x() = relativePosistion.dot(S);
        poissonDiskSpace.y() = relativePosistion.dot(T);
        poissonDiskSpace.z() = relativePosistion.dot(N);

        float dotN              = dot(N,newPointNormal);
        bool samePlane          = dotN > params.poissonAngleNormalThreshold;

        //(x/a)2 + (y/b)2 + (z/c)2 = 1
        float x = poissonDiskSpace.x();
        float y = poissonDiskSpace.y();
        float z = poissonDiskSpace.z();
        float a = r;
        float b = r;
        float c = cs*2;

        float a2 = kd;
        float b2 = kd;
        float c2 = cs;

        float smallEllipse = (x/a2)*(x/a2) + (y/b2)*(y/b2) + (z/c2)*(z/c2);
        float bigEllipse = (x/a)*(x/a) + (y/b)*(y/b) + (z/c)*(z/c);

        bool outsideOfSmallEllipse = false;
        bool insideBigEllipse = false;

        if (bigEllipse <= 1)
            insideBigEllipse = true;
        if (smallEllipse > 1)
            outsideOfSmallEllipse = true;

        //It is too close to the current point ?
        if(samePlane && !outsideOfSmallEllipse)
        {
            tooClose = true;
        }

        if(insideBigEllipse && samePlane)
            numberOfClosePoint++;
    }
    return !tooClose;
}


GA_Offset ParticleAndTrackerManagerGagnon2016::CreateAParticle(UT_Vector3 p, UT_Vector3 N)
{

    cout << "[ParticleAndTrackerManagerGagnon2016] Create a particle."<<endl;
    GA_RWHandleF    attExistingLife(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"life", 1));

    int divider = 1;
    if (params.useTangeantTracker == 1)
        divider = 2;
    if (trackersGdp->getNumPoints()/divider > this->maxId) //existing points
    {
        //cout << "New Max Id = "<<trackersGdp->getNumPoints()/divider<<endl;
        this->maxId = trackersGdp->getNumPoints()/divider;
    }
    int id = this->maxId+1;
    this->maxId = id;
    cout << "New Max Id = "<<this->maxId<<endl;
    GA_Offset newPoint = trackersGdp->appendPoint();
    trackersGdp->setPos3(newPoint, p);
    attN.set(newPoint,N);
    attActive.set(newPoint,true);
    attId.set(newPoint,id);
    attSpawn.set(newPoint,0);
    attLife.set(newPoint,0.001f);
    attNumberOfPrimitives.set(newPoint,0);
    attIsMature.set(newPoint,0);
    attMaxDeltaOnD.set(newPoint,0);
    cout << "Set random color"<<endl;
    UT_Vector3 patchColor = this->SetRandomColor(id);
    AttCd.set(newPoint,patchColor);

    //---------- Tangeant Tracker -----------
    UT_Vector3 defaultDirection(1,0,0);
    UT_Vector3 S,T;
    float Tlenght = params.poissondiskradius/2.0f;
    GA_Offset tracker_offset;


    cout << "Add tangeant tracker"<<endl;
    GA_RWHandleI    isTangeantTracker(trackersGdp->addIntTuple(GA_ATTRIB_POINT, "isTrangeantTracker",1));
    //---------- ADD TANGEANT TRACKER ----------
    //cout << "Add Tangeant Tracker"<<endl;
    //put this in a function, and/or move this where we already add point
    S = cross(N,defaultDirection);
    S.normalize();
    T = cross(S,N);
    T.normalize();
    UT_Vector3 translation = T*Tlenght;
    //cout << "Translation: "<<translation<<endl;
    UT_Vector3 tangeantPosition = p + translation;
    //cout << "adding tangeant tracker"<<endl;
    tracker_offset = trackersGdp->appendPoint();
    isTangeantTracker.set(tracker_offset,1);
    attId.set(tracker_offset,id);
    trackersGdp->setPos3(tracker_offset,tangeantPosition);


    //cout << "Ending adding particule."<<endl;
    if(params.startFrame == params.frame)
    {
        attExistingLife.set(newPoint,params.fadingTau);
    }

    if(params.startFrame == params.frame)
    {
        attLife.set(newPoint,params.fadingTau);
    }
    this->trackerTree.build(trackersGdp);

    this->numberOfNewPoints++;

    return newPoint;
}

UT_Vector3 ParticleAndTrackerManagerGagnon2016::SetRandomColor(int patchNumber)
{
    //initialize random seed
    srand(patchNumber);
    float r = ((double) rand()/(RAND_MAX));
    srand(patchNumber+1);
    float g = ((double) rand()/(RAND_MAX));
    srand(patchNumber+2);
    float b = ((double) rand()/(RAND_MAX));
    UT_Vector3 patchColor(r,g,b);

    return patchColor;
}


//================================================================================================

//                                      CREATE TRACKERS BASED ON POISSON DISK

//================================================================================================


void ParticleAndTrackerManagerGagnon2016::CreateAndUpdateTrackersBasedOnPoissonDisk(GU_Detail *surface, GU_Detail *trackersGdp, GA_PointGroup *surfaceGroup,  ParametersDeformablePatches params)
{

    cout << "[ParticleAndTrackerManagerGagnon2016] CreateTrackersBasedOnPoissonDisk"<<endl;

    if (surfaceGroup == 0x0)
        return;

    GA_PointGroup *markerGrp = (GA_PointGroup *)trackersGdp->pointGroups().find(this->markerGroupName.c_str());
    if (markerGrp == 0x0)
        markerGrp = trackersGdp->newPointGroup(markerGroupName.c_str());

    GA_PrimitiveGroup *markerGrpPrims;
    if (tackerPolygon)
    {
        markerGrpPrims = (GA_PrimitiveGroup *)trackersGdp->primitiveGroups().find(this->markerGroupName.c_str());
        if (markerGrpPrims == 0x0)
            markerGrpPrims = trackersGdp->newPrimitiveGroup(markerGroupName.c_str());
    }

    UT_Vector3 position;
    UT_Vector3 N;

    float thresholdDistance = params.maximumProjectionDistance;

    GU_MinInfo mininfo;
    GU_RayIntersect ray(surface);
    ray.init();

    int id = 0;
    GA_Offset ppt;

    int nbOfPoint = 1;
    GA_FOR_ALL_PTOFF(trackersGdp,ppt)
    {
        id = attId.get(ppt);
        //int active = attActive.get(ppt);
        float currentLife = attLife.get(ppt);
        int currentSpawn = attSpawn.get(ppt);

        UT_Vector3 velocity;
        UT_Vector3 centerUV = attCenterUV.get(ppt);

        //============================ PROJECTION ON MESH =======================
        UT_Vector3 p1 = trackersGdp->getPos3(ppt);
        mininfo.init(thresholdDistance,0.0001);
        ray.minimumPoint(p1,mininfo);

        if (!mininfo.prim)
        {
            //cout << "No primitive to project on"<<endl;
            continue;
        }

        const GEO_Primitive *geoPrim = mininfo.prim;
        int vertexCount = geoPrim->getVertexCount();
        if (vertexCount != 3)
        {
            //cout << "vertex count "<<vertexCount<<" for primitive "<<geoPrim->getMapOffset()<<endl;
            continue;
        }
        //get pos of hit
        UT_Vector4 hitPos;
        mininfo.prim->evaluateInteriorPoint(hitPos,mininfo.u1,mininfo.v1);
        if (distance3d(p1,hitPos) < thresholdDistance)
        {
            p1 = hitPos;
            //------------------------------PARAMETRIC COORDINATE -----------------------------------
            GA_Offset primOffset = mininfo.prim->getMapOffset();
            float u = mininfo.u1;
            float v = mininfo.v1;
            GEO_Primitive *prim = surface->getGEOPrimitive(primOffset);

            GA_Offset vertexOffset0 = prim->getVertexOffset(0);

            GA_Offset pointOffset0  = surface->vertexPoint(vertexOffset0);
            UT_Vector3 n0 = refAttN.get(pointOffset0);
            UT_Vector3 v0 = refAttV.get(pointOffset0);

            GA_Offset vertexOffset1 = prim->getVertexOffset(1);
            GA_Offset pointOffset1  = surface->vertexPoint(vertexOffset1);
            UT_Vector3 n1 = refAttN.get(pointOffset1);
            UT_Vector3 v1 = refAttV.get(pointOffset1);

            GA_Offset vertexOffset2 = prim->getVertexOffset(2);
            GA_Offset pointOffset2  = surface->vertexPoint(vertexOffset2);
            UT_Vector3 n2 = refAttN.get(pointOffset2);
            UT_Vector3 v2 = refAttV.get(pointOffset2);

            N        = n0+u*(n1-n0)+v*(n2-n0);
            velocity = v0+u*(v1-v0)+v*(v2-v0);
        }
        else
        {
            //can't project, we delete
            attLife.set(ppt,0);
            attActive.set(ppt,0);
        }


        if (currentLife < 0)
            currentLife = 0;
        //==============================================

        position = p1;
        trackersGdp->setPos3(ppt,position);
        N.normalize();
        attV.set(ppt,velocity);
        attN.set(ppt,N);
        attCenterUV.set(ppt,centerUV);
        trackersGdp->setPos3(ppt,position);

        float life = currentLife;
        attLife.set(ppt,life);

        attSpawn.set(ppt,currentSpawn);

        float randt = (((double) rand() / (RAND_MAX)));
        attRandT.set(ppt,randt);

        //numberOfPatches++;
        if (isTangeantTracker.get(ppt) == 0)
            nbOfPoint++;
    }
    cout <<" DONE"<<endl;
}

//================================================================================================

//                                      CREATE TRACKERS BASED ON POISSON DISK

//================================================================================================


void ParticleAndTrackerManagerGagnon2016::UpdateTrackersAndTangeant(GU_Detail *surface, GU_Detail *trackersGdp, GA_PointGroup *surfaceGroup,  ParametersDeformablePatches params)
{

    bool useDynamicTau = params.useDynamicTau;
    cout << "[ParticleTracker] CreateAndUpdateTrackersAndTangeantsBasedOnPoissonDisk";

    if (surfaceGroup == 0x0)
        return;

    GA_PointGroup *markerGrp = (GA_PointGroup *)trackersGdp->pointGroups().find(this->markerGroupName.c_str());
    if (markerGrp == 0x0)
        markerGrp = trackersGdp->newPointGroup(markerGroupName.c_str());


    GA_PrimitiveGroup *markerGrpPrims;
    if (tackerPolygon)
    {
        markerGrpPrims = (GA_PrimitiveGroup *)trackersGdp->primitiveGroups().find(this->markerGroupName.c_str());
        if (markerGrpPrims == 0x0)
            markerGrpPrims = trackersGdp->newPrimitiveGroup(markerGroupName.c_str());
    }

    UT_Vector3 position;
    UT_Vector3 N;

    float thresholdDistance = params.maximumProjectionDistance;

    GU_MinInfo mininfo;
    GU_RayIntersect ray(surface);
    ray.init();

    int id = 0;
    GA_Offset ppt;
    GA_FOR_ALL_PTOFF(trackersGdp,ppt)
    {
        id = attId.get(ppt);
        int active = attActive.get(ppt);
        float currentLife = attLife.get(ppt);
        int currentSpawn = attSpawn.get(ppt);


        UT_Vector3 velocity;

        float dynamicTau = attMaxDeltaOnD.get(ppt);
        UT_Vector3 centerUV = attCenterUV.get(ppt);

        //Dead patches are not updated
        if (currentLife <= 0 && active == 0)
        {
            continue;
        }

        //============================ PROJECTION ON MESH =======================
        UT_Vector3 p1 = trackersGdp->getPos3(ppt);
        mininfo.init(thresholdDistance,0.0001);
        ray.minimumPoint(p1,mininfo);

        if (!mininfo.prim)
        {
            //cout << "No primitive to project on"<<endl;
            continue;
        }

        const GEO_Primitive *geoPrim = mininfo.prim;
        int vertexCount = geoPrim->getVertexCount();
        if (vertexCount != 3)
        {
            //cout << "vertex count "<<vertexCount<<" for primitive "<<geoPrim->getMapOffset()<<endl;
            continue;
        }
        //get pos of hit
        UT_Vector4 hitPos;
        mininfo.prim->evaluateInteriorPoint(hitPos,mininfo.u1,mininfo.v1);
        if (distance3d(p1,hitPos) < thresholdDistance)
        {
            p1 = hitPos;

            //------------------------------PARAMETRIC COORDINATE -----------------------------------
            GA_Offset primOffset = mininfo.prim->getMapOffset();
            float u = mininfo.u1;
            float v = mininfo.v1;
            GEO_Primitive *prim = surface->getGEOPrimitive(primOffset);

            GA_Offset vertexOffset0 = prim->getVertexOffset(0);

            GA_Offset pointOffset0  = surface->vertexPoint(vertexOffset0);
            UT_Vector3 n0 = refAttN.get(pointOffset0);
            UT_Vector3 v0 = refAttV.get(pointOffset0);

            GA_Offset vertexOffset1 = prim->getVertexOffset(1);
            GA_Offset pointOffset1  = surface->vertexPoint(vertexOffset1);
            UT_Vector3 n1 = refAttN.get(pointOffset1);
            UT_Vector3 v1 = refAttV.get(pointOffset1);

            GA_Offset vertexOffset2 = prim->getVertexOffset(2);
            GA_Offset pointOffset2  = surface->vertexPoint(vertexOffset2);
            UT_Vector3 n2 = refAttN.get(pointOffset2);
            UT_Vector3 v2 = refAttV.get(pointOffset2);

            N                   = n0+u*(n1-n0)+v*(n2-n0);
            velocity = v0+u*(v1-v0)+v*(v2-v0);
        }
        else
        {
            //can't project, we delete
            attLife.set(ppt,0);
            attActive.set(ppt,0);
        }


        //========================================================================

        //========================= UPDATE ===============================
        //we want to fade out poisson disk that are flagged a inactive and that are mature (life spawn greater than the fading in time)
        //or that are too close to each other

        int maxNumberOfNeighbour = 5; // TODO promotve that variable
        int density = attDensity.get(ppt);
        //-------------- deleting faster logic ------------------
        //Can we move this to the ParticleTracker update ?
        int deleteFaster = attDeleteFaster.get(ppt);
        int numberOfNeighbourThreshold = 1; // TODO: promote this variable
        if (density > numberOfNeighbourThreshold && deleteFaster == 0)
        {
            attDeleteFaster.set(ppt, 1);
        }
        else if(deleteFaster == 1 && density <= numberOfNeighbourThreshold)
        {
            attDeleteFaster.set(ppt, 0);
        }
        //-------------------------------------------------------




        currentLife++;
        currentSpawn++;

        float deletionLife = params.fadingTau;
        float blending = (float)currentLife/(float(deletionLife));
        attBlend.set(ppt,blending);

        //==============================================

        position = p1;
        trackersGdp->setPos3(ppt,position);
        N.normalize();

        attV.set(ppt,velocity);
        attN.set(ppt,N);
        attCenterUV.set(ppt,centerUV);
        trackersGdp->setPos3(ppt,position);

        float life = currentLife;
        attLife.set(ppt,life);

        float temporalComponetKt = ((float)life)/params.fadingTau;

        attBlend.set(ppt,temporalComponetKt);
        attSpawn.set(ppt,currentSpawn);
        attMaxDeltaOnD.set(ppt,dynamicTau);
        float randt = (((double) rand() / (RAND_MAX)));
        attRandT.set(ppt,randt);

        //numberOfPatches++;
    }
    cout <<" DONE"<<endl;
}

//================================================================================================

//                                      ADVECT SINGLE MARKERS

//================================================================================================


void ParticleAndTrackerManagerGagnon2016::AdvectSingleTrackers(GU_Detail *surfaceGdp,GU_Detail *trackersGdp, ParametersDeformablePatches params)
{
    cout <<this->approachName<< " Advect Markers"<<endl;

    std::clock_t startAdvection;
    startAdvection = std::clock();

    GA_RWHandleV3 refAttN(surfaceGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));

    numberOfPatches = 0;
    maxId = 0;

    if (attV.isInvalid())
    {
        cout << "Markers have no velocity";
        return;
    }

    if (attDensity.isInvalid())
    {
        cout << "The is no density defined"<<endl;
        return;
    }

    UT_Vector3 v;
    UT_Vector3 p;
    UT_Vector3 p1;
    float dt = 1.0f/24.0f;
    float thresholdDistance = params.maximumProjectionDistance;
    //--------------------------------------------------
    {
        GU_MinInfo mininfo;

        GU_RayIntersect ray(surfaceGdp);
        ray.init();

        UT_Vector3 refDir(1,0,0);
        UT_Vector3 N;
        GA_Offset ppt;
        int id;
        int density;
        float currentLife = 0;
        GA_FOR_ALL_PTOFF(trackersGdp,ppt)
        {

            v = attV.get(ppt);
            N = attN.get(ppt);
            density = attDensity.get(ppt);
            //cout << "advecting point "<<ppt<<endl;
            if (N.length() < epsilon)
            {
                //cout << "N lenght is too small"<<endl;
                continue;
            }
            p = trackersGdp->getPos3(ppt);

            id = attId.get(ppt);
            if(id > maxId)
                maxId = id;

            currentLife = attLife.get(ppt);

            //-----------------------------------------
            //advection
            UT_Vector3 d = v*dt;
            p1 = p+d;
            trackersGdp->setPos3(ppt,p1);
            //-----------------------------------------

            p1 = trackersGdp->getPos3(ppt);
            mininfo.init(thresholdDistance,0.0001);
            ray.minimumPoint(p1,mininfo);

            if (!mininfo.prim)
            {
                cout << "No primitive to project on"<<endl;
                continue;
            }

            const GEO_Primitive *geoPrim = mininfo.prim;
            int vertexCount = geoPrim->getVertexCount();
            if (vertexCount != 3)
            {
                //cout << "vertex count "<<vertexCount<<" for primitive "<<geoPrim->getMapOffset()<<endl;
                continue;
            }
            //get pos of hit
            UT_Vector4 hitPos;
            mininfo.prim->evaluateInteriorPoint(hitPos,mininfo.u1,mininfo.v1);
            if (distance3d(p1,hitPos) < thresholdDistance)
            {
                p1 = hitPos;
                trackersGdp->setPos3(ppt,p1);
                AttCd.set(ppt,UT_Vector3(0,1,0));

                //------------------------------PARAMETRIC COORDINATE -----------------------------------
                GA_Offset primOffset = mininfo.prim->getMapOffset();
                float u = mininfo.u1;
                float v = mininfo.v1;
                GEO_Primitive *prim = surfaceGdp->getGEOPrimitive(primOffset);

                GA_Offset vertexOffset0 = prim->getVertexOffset(0);

                GA_Offset pointOffset0  = surfaceGdp->vertexPoint(vertexOffset0);
                UT_Vector3 n0 = refAttN.get(pointOffset0);
                UT_Vector3 v0 = refAttV.get(pointOffset0);

                GA_Offset vertexOffset1 = prim->getVertexOffset(1);
                GA_Offset pointOffset1  = surfaceGdp->vertexPoint(vertexOffset1);
                UT_Vector3 n1 = refAttN.get(pointOffset1);
                UT_Vector3 v1 = refAttV.get(pointOffset1);

                GA_Offset vertexOffset2 = prim->getVertexOffset(2);
                GA_Offset pointOffset2  = surfaceGdp->vertexPoint(vertexOffset2);
                UT_Vector3 n2 = refAttN.get(pointOffset2);
                UT_Vector3 v2 = refAttV.get(pointOffset2);

                N                   = n0+u*(n1-n0)+v*(n2-n0);
                UT_Vector3 velocity = v0+u*(v1-v0)+v*(v2-v0);
                attV.set(ppt,velocity);

                attN.set(ppt,N);
                //------------------------------------------------------------------------------------
                numberOfPatches++;
            }
            else
            {
                //delete this point because we can't project it, probably because of a sudden topological change.
                //cout << "delete "<<id<<" because we can't project it, probably because of a sudden topological change."<<endl;
                AttCd.set(ppt,UT_Vector3(1,0,0));

                //detached poisson disks have to be deleted directly, not fading out.
                attLife.set(ppt,0);
                attActive.set(ppt,0);

                numberOfPatches--;
                numberOfDetachedPatches++;

                trackersGdp->setPos3(ppt,p1);
            }
        }
    }

    //----------------------------------
    cout << this->approachName<< " There are "<<trackersGdp->getNumPoints() << " trackers after advection"<<endl;
    cout << this->approachName<< " There are "<<numberOfDetachedPatches<< " detached trackers"<<endl;
    cout << this->approachName<< " There are "<<numberOfPatches << " number of patches"<<endl;

    this->markerAdvectionTime += (std::clock() - startAdvection) / (double) CLOCKS_PER_SEC;

}

//================================================================================================

//                                      ADVECT MARKERS

//================================================================================================


void ParticleAndTrackerManagerGagnon2016::AdvectTrackersAndTangeants(GU_Detail *surfaceGdp, GU_Detail *trackersGdp, ParametersDeformablePatches params)
{
    cout <<this->approachName<< " Advect Trackers And Tangeants"<<endl;

    std::clock_t startAdvection;
    startAdvection = std::clock();

    GA_RWHandleV3 refAttN(surfaceGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));

    numberOfPatches = 0;
    maxId = 0;

    if (attV.isInvalid())
    {
        cout << "Markers have no velocity";
        return;
    }

    if (attDensity.isInvalid())
    {
        cout << "The is no density defined"<<endl;
        return;
    }

    UT_Vector3 S,T;

    UT_Vector3 v;
    UT_Vector3 p;
    UT_Vector3 p1;
    float dt = 1.0f/24.0f;
    float thresholdDistance = params.maximumProjectionDistance;
    //--------------------------------------------------
    {
        GU_MinInfo mininfo;

        GU_RayIntersect ray(surfaceGdp);
        ray.init();

        UT_Vector3 refDir(1,0,0);
        UT_Vector3 N;
        GA_Offset ppt;
        int id;
        int density;
        float currentLife = 0;
        int isTangeant = 0;
        GA_Offset numPoint = trackersGdp->getNumPointOffsets();
        GA_FOR_ALL_PTOFF(trackersGdp,ppt)
        {
            v = attV.get(ppt);
            N = attN.get(ppt);
            density = attDensity.get(ppt);
            //cout << "advecting point "<<ppt<<endl;
            if (N.length() < epsilon)
            {
                //cout << "N lenght is too small"<<endl;
                continue;
            }
            p = trackersGdp->getPos3(ppt);

            id = attId.get(ppt);
            if(id > maxId)
                maxId = id;

            currentLife = attLife.get(ppt);

            //-----------------------------------------
            //advection
            UT_Vector3 d = v*dt;
            p1 = p+d;
            trackersGdp->setPos3(ppt,p1);
            //-----------------------------------------

            p1 = trackersGdp->getPos3(ppt);
            mininfo.init(thresholdDistance,0.0001);
            ray.minimumPoint(p1,mininfo);

            if (!mininfo.prim)
            {
                cout << "No primitive to project on"<<endl;
                continue;
            }

            const GEO_Primitive *geoPrim = mininfo.prim;
            int vertexCount = geoPrim->getVertexCount();
            if (vertexCount != 3)
            {
                //cout << "vertex count "<<vertexCount<<" for primitive "<<geoPrim->getMapOffset()<<endl;
                continue;
            }
            //get pos of hit
            UT_Vector4 hitPos;
            mininfo.prim->evaluateInteriorPoint(hitPos,mininfo.u1,mininfo.v1);
            if (distance3d(p1,hitPos) < thresholdDistance)
            {
                p1 = hitPos;

                AttCd.set(ppt,UT_Vector3(0,1,0));

                //------------------------------PARAMETRIC COORDINATE -----------------------------------
                GA_Offset primOffset = mininfo.prim->getMapOffset();
                float u = mininfo.u1;
                float v = mininfo.v1;
                GEO_Primitive *prim = surfaceGdp->getGEOPrimitive(primOffset);

                GA_Offset vertexOffset0 = prim->getVertexOffset(0);

                GA_Offset pointOffset0  = surfaceGdp->vertexPoint(vertexOffset0);
                UT_Vector3 n0 = refAttN.get(pointOffset0);
                UT_Vector3 v0 = refAttV.get(pointOffset0);

                GA_Offset vertexOffset1 = prim->getVertexOffset(1);
                GA_Offset pointOffset1  = surfaceGdp->vertexPoint(vertexOffset1);
                UT_Vector3 n1 = refAttN.get(pointOffset1);
                UT_Vector3 v1 = refAttV.get(pointOffset1);

                GA_Offset vertexOffset2 = prim->getVertexOffset(2);
                GA_Offset pointOffset2  = surfaceGdp->vertexPoint(vertexOffset2);
                UT_Vector3 n2 = refAttN.get(pointOffset2);
                UT_Vector3 v2 = refAttV.get(pointOffset2);

                N                   = n0+u*(n1-n0)+v*(n2-n0);
                UT_Vector3 velocity = v0+u*(v1-v0)+v*(v2-v0);
                attV.set(ppt,velocity);

                attN.set(ppt,N);


                //replace the tangeant tracker
                isTangeant = isTangeantTracker.get(ppt);
                if (isTangeant == 1)
                    continue;
                trackersGdp->setPos3(ppt,p1);
                GA_Offset tracker_offset = ppt+1;
                UT_Vector3 currentDirection = trackersGdp->getPos3(tracker_offset)-p1;

                S = cross(N,currentDirection);
                S.normalize();
                T = cross(S,N);
                T.normalize();
                UT_Vector3 translation = T*params.poissondiskradius/2.0f;
                //cout << "Translation: "<<translation<<endl;
                UT_Vector3 tangeantPosition = p1 + translation;
                trackersGdp->setPos3(tracker_offset,tangeantPosition);
                //------------------------------------------------------------------------------------
                numberOfPatches++;
            }
            else
            {
                //delete this point because we can't project it, probably because of a sudden topological change.
                //cout << "delete "<<id<<" because we can't project it, probably because of a sudden topological change."<<endl;
                AttCd.set(ppt,UT_Vector3(1,0,0));

                //detached poisson disks have to be deleted directly, not fading out.
                attLife.set(ppt,0);
                attActive.set(ppt,0);

                numberOfPatches--;
                numberOfDetachedPatches++;

                trackersGdp->setPos3(ppt,p1);
                //cout << "new new position "<<p1<<endl;
            }
        }
    }

    //----------------------------------
    cout << this->approachName<< " There are "<<trackersGdp->getNumPoints() << " trackers after advection"<<endl;
    cout << this->approachName<< " There are "<<numberOfDetachedPatches<< " detached trackers"<<endl;
    cout << this->approachName<< " There are "<<numberOfPatches << " number of patches"<<endl;

    this->markerAdvectionTime += (std::clock() - startAdvection) / (double) CLOCKS_PER_SEC;

}


//================================================================================================

//                                      COMPUTE DENSITY

//================================================================================================

void ParticleAndTrackerManagerGagnon2016::ComputeDensity(GU_Detail *surfaceGdp, GU_Detail *trackers, ParametersDeformablePatches params, GEO_PointTreeGAOffset &tree)
{

    cout <<this->approachName<< " Compute Density"<<endl;

    UT_Vector3 v,vn,p;

    float epsilon = 0.001;

    GA_RWHandleI attDensity(trackers->addIntTuple(GA_ATTRIB_POINT,"density",1));

    //GA_RWHandleV3 attV(trackers->addFloatTuple(GA_ATTRIB_POINT,"v", 3));
    float patchRadius = params.poissondiskradius;
    GA_Offset ppt;

    GA_FOR_ALL_PTOFF(trackers,ppt)
    {

        p = trackers->getPos3(ppt);

        GEO_PointTreeGAOffset::IdxArrayType close_particles_indices;
        tree.findAllCloseIdx(p,
                             patchRadius,
                             close_particles_indices);

        int l = close_particles_indices.entries();

        attDensity.set(ppt,l);

    }
}



//================================================================================================

//                                      COMPUTE DIVERGENCE

//================================================================================================

void ParticleAndTrackerManagerGagnon2016::ComputeDivergence(GU_Detail *surfaceGdp, GU_Detail *trackers, ParametersDeformablePatches params, GEO_PointTreeGAOffset &tree)
{

    cout <<this->approachName<< " Compute Divergence"<<endl;

    UT_Vector3 v,vn,p;
    UT_Vector3 N,Nn;
    float epsilon = 0.001;


    //GA_RWHandleV3 attV(trackers->addFloatTuple(GA_ATTRIB_POINT,"v", 3));
    float patchRadius = params.poissondiskradius*3;
    GA_Offset ppt;

    GA_FOR_ALL_PTOFF(trackers,ppt)
    {
        v = attV.get(ppt);
        v.normalize();

        float sumDotWeighted = 0;
        float w_k = 0;

        p = trackers->getPos3(ppt);

        GEO_PointTreeGAOffset::IdxArrayType close_particles_indices;
        tree.findAllCloseIdx(p,
                             patchRadius,
                             close_particles_indices);

        unsigned l = close_particles_indices.entries();
        if (l > 0)
        {
            GA_Offset neighbor;
            for(int j=0; j<l;j++ )
            {
                neighbor = close_particles_indices.array()[j];
                vn = attVSurface.get(neighbor);
                float ln = vn.length();
                if (ln < epsilon)
                    continue;
                vn.normalize();
                //float d = dot(vn,v);
                float dw = dot(vn,v) * ln;
                w_k += ln;
                sumDotWeighted += dw;
            }
        }

        float divergence = (1+sumDotWeighted/w_k)/2;
        attDivergence.set(ppt,divergence);

    }
}

