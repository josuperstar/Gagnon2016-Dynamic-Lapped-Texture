#include "LappedSurfaceGagnon2016.h"
#include <vector>
#include <algorithm>
#include <array>        // std::array
#include <random>       // std::default_random_engine
#include <chrono>       // std::chrono::system_clock
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
#include <GU/GU_NeighbourList.h>
#include <GU/GU_RayIntersect.h>
#include <GU/GU_Flatten.h>

//#include <Strategies/StrategySurfaceTextureSynthesis.h>
#include <Core/Gagnon2016/Bridson2012PoissonDiskDistribution.h>
#include <Core/HoudiniUtils.h>


LappedSurfaceGagnon2016::LappedSurfaceGagnon2016(GU_Detail *surface, GU_Detail *trackersGdp) : ParticleAndTrackerManagerGagnon2016(surface, trackersGdp)
{
    this->numberOfPatches = 0;
    this->maxId = 0;



//    uvsArray->clear(this->uvsAtt);
//    patchIdsAtt->clear(patchIdsArrayAttrib);
//    alphaAtt->clear(alphaArrayAtt);

    //=========================== PATCH ID ARRAY ATTRIB ==========================
    UT_String patchName(this->patchIdsName);
    patchIdsArrayAttrib = surface->findIntArray(GA_ATTRIB_POINT,
                                            patchName,
                                            // Allow any tuple size to match
                                            -1, -1);
    if (!patchIdsArrayAttrib)
    {
        patchIdsArrayAttrib = surface->addIntArray(GA_ATTRIB_POINT,
                                    patchName,
                    // Tuple size one means we group the array into
                    // logical groups of 1.  It does *NOT* affect
                    // the length of the arrays, which are always
                    // measured in ints.
                                    1);
    }
    // Make sure we are an array.  Note tuples do not match to this,
    // nor do GA_*Handle* match!
    // We will match both int and float here, however.
    // (For string, getAIFSharedStringArray)
    patchIdsAtt = patchIdsArrayAttrib->getAIFNumericArray();
    if (!patchIdsAtt)
    {
        UT_WorkBuffer           buf;
        buf.sprintf("Attribute \"%s\" not a numeric array!",
                    (const char *) patchName);
        //addError(SOP_MESSAGE, buf.buffer());
        //return error();
    }
    patchIdsArrayAttrib->clearDataId();

    //=========================== ALPHA ARRAY ATTRIB ==========================
    UT_String uvname(this->uvArrayName);
    uvsAtt = surface->findFloatArray(GA_ATTRIB_POINT,
                                            uvname,
                                            // Allow any tuple size to match
                                            -1, -1);

    if (!uvsAtt)
    {
        uvsAtt = surface->addFloatArray(GA_ATTRIB_POINT,
                                    uvname,
                    // Tuple size one means we group the array into
                    // logical groups of 1.  It does *NOT* affect
                    // the length of the arrays, which are always
                    // measured in ints.
                                    1);
    }

    uvsArray = uvsAtt->getAIFNumericArray();
    if (!uvsArray)
    {
        UT_WorkBuffer           buf;
        buf.sprintf("Attribute \"%s\" not a numeric array!",
                    (const char *) uvname);
        //addError(SOP_MESSAGE, buf.buffer());
        //return error();
    }
    uvsAtt->clearDataId();
    //=========================== ALPHA ARRAY ATTRIB ==========================

    UT_String aname(this->alphaArrayName);
    alphaArrayAtt = surface->findFloatArray(GA_ATTRIB_POINT,
                                            aname,
                                            // Allow any tuple size to match
                                            -1, -1);
    if (!alphaArrayAtt)
    {
        alphaArrayAtt = surface->addFloatArray(GA_ATTRIB_POINT,
                                    aname,
                    // Tuple size one means we group the array into
                    // logical groups of 1.  It does *NOT* affect
                    // the length of the arrays, which are always
                    // measured in ints.
                                    1);
    }
    // Make sure we are an array.  Note tuples do not match to this,
    // nor do GA_*Handle* match!
    // We will match both int and float here, however.
    // (For string, getAIFSharedStringArray)
    alphaAtt = alphaArrayAtt->getAIFNumericArray();
    if (!alphaAtt)
    {
        UT_WorkBuffer           buf;
        buf.sprintf("Attribute \"%s\" not a numeric array!",
                    (const char *) aname);
        //addError(SOP_MESSAGE, buf.buffer());
        //return error();
    }
    alphaArrayAtt->clearDataId();

    this->markerAdvectionTime = 0;
    this->patchCreationTime = 0;
    this->updatePatchesTime = 0;
    this->numberOfConcealedPatches = 0;
    this->numberOfNewPatches = 0;
    this->numberOfDetachedPatches = 0;
}

LappedSurfaceGagnon2016::~LappedSurfaceGagnon2016()
{
    cout << "[LappedSurfaceGagnon2016] Destroying"<<endl;
    this->rays.clear();
}


void LappedSurfaceGagnon2016::ShufflePoints(GU_Detail *trackersGdp)
{
    //Create a list of trackers combined to shuffle
    cout << "LappedSurfaceGagnon2016::ShufflePoints"<<endl;

    struct tracker
    {
        GA_Offset p;
        GA_Offset t;
        int id;
    };

    vector<tracker> trackers;
    GA_Offset ppt;
    GA_FOR_ALL_PTOFF(trackersGdp, ppt)
    {
        int isTangeant = isTangeantTracker.get(ppt);
        if (isTangeant == 1)
            continue;

        int patchNumber = attId.get(ppt);

        tracker t;
        t.id = patchNumber;
        t.p = ppt;
        t.t = ppt+1;
        trackers.push_back(t);
    }

    // obtain a time-based seed:
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

    shuffle (trackers.begin(), trackers.end(), std::default_random_engine(seed));

    vector<tracker>::iterator itTrack;
    int trackerNumber = 1;
    for (itTrack = trackers.begin(); itTrack != trackers.end(); itTrack++)
    {
        attId.set((*itTrack).p, trackerNumber);
        attId.set((*itTrack).t, trackerNumber);
        trackerNumber++;
    }
}


//================================================================================================

//                                     POISSON DISK SAMPLING

//================================================================================================


void LappedSurfaceGagnon2016::PoissonDiskSampling(GU_Detail *levelSet, GU_Detail *trackersGdp, ParametersDeformablePatches params)
{

    //This is a function that does a Poisson Disk Sampling using the approach of Bridson 2012 paper
    //This function is a wrapper to the Bridson2012PoissonDiskDistribution class.
    //It basically take the points from Houdini and fill it to the approach.

    std::clock_t addPoissonDisk;
    addPoissonDisk = std::clock();

    cout << "[LappedSurfaceGagnon2016:PoissonDiskSampling]"<<endl;

    GEO_PointTreeGAOffset trackerTree;
    trackerTree.build(trackersGdp, NULL);

    //cout << "[Yu2011] we have "<<numberOfPoints << " existing point(s) in trackersGdp"<<endl;
    Bridson2012PoissonDiskDistribution poissonDiskDistribution;
    poissonDiskDistribution.PoissonDiskSampling(trackersGdp, trackerTree, levelSet, params.poissonAngleNormalThreshold, params);

    cout << "[LappedSurfaceGagnon2016] poisson disk sample "<<trackersGdp->getNumPoints()<< " point(s)"<<endl;
    this->poissondisk += (std::clock() - addPoissonDisk) / (double) CLOCKS_PER_SEC;
}

void LappedSurfaceGagnon2016::CreateAPatch(GU_Detail *trackersGdp,  ParametersDeformablePatches params)
{
    //This is a function that does a Poisson Disk Sampling using the approach of Bridson 2012 paper
    //This function is a wrapper to the Bridson2012PoissonDiskDistribution class.
    //It basically take the points from Houdini and fill it to the approach.

    std::clock_t addPoissonDisk;
    addPoissonDisk = std::clock();

    GEO_PointTreeGAOffset trackerTree;
    trackerTree.build(trackersGdp, NULL);

    //cout << "[Yu2011] we have "<<numberOfPoints << " existing point(s) in trackersGdp"<<endl;
    Bridson2012PoissonDiskDistribution poissonDiskDistribution;
    int numberOfClosePoint = 0;
    poissonDiskDistribution.CreateAParticle(trackersGdp, trackerTree, UT_Vector3(0,0,0),UT_Vector3(0,1,0),1,numberOfClosePoint,params);

    cout << "[Yu2011] poisson disk sample "<<trackersGdp->getNumPoints()<< " point(s)"<<endl;

    this->poissondisk += (std::clock() - addPoissonDisk) / (double) CLOCKS_PER_SEC;
}

//================================================================================================

//                                       ADD PATCHES USING BARYCENTRIC COORDONATE

//================================================================================================


void LappedSurfaceGagnon2016::AddSolidPatchesUsingBarycentricCoordinates(GU_Detail *surfaceGdp, GU_Detail *trackersGdp, ParametersDeformablePatches params, GEO_PointTreeGAOffset &surfaceTree)
{

    //This function is used to transfer the uv list from the deformable patches to the surface where the texture will be synthesis.
    cout << "[AddSolidPatchesUsingBarycentricCoordinates]" << endl;

    std::clock_t addPatchesStart;
    addPatchesStart = std::clock();
    this->patchCreationTime = 0;

    map<int, set<int> > surfacePatchIds;

    fpreal patchRadius = 2*params.poissondiskradius*2;

    //================================ CREATE PATCH GROUPS ==============================
    GA_RWHandleV3 attNSurface(surfaceGdp->addFloatTuple(GA_ATTRIB_POINT,"N", 3));
    GA_RWHandleI attNumberOfPatch(surfaceGdp->addIntTuple(GA_ATTRIB_POINT,"numberOfPatch",1));

    if (attLife.isInvalid())
    {
        cout << "particles have no life"<<endl;
        return;
    }

    set<int> patchTreated;
    float r = params.poissondiskradius;

    UT_Vector3 N;
    UT_Vector3 NN;
    UT_Vector3 position;

    int isTangeant = 0;
    GA_Offset surfacePointOffset;
    int patchNumber = 0;
    {
        //================== CREATE PATCHES =================
        // create a group of point based on trackers position
        GA_Offset ppt;
        GA_FOR_ALL_PTOFF(trackersGdp, ppt)
        {
            isTangeant = isTangeantTracker.get(ppt);
            if (isTangeant == 1)
                continue;

            patchNumber = attId.get(ppt);
            //cout << "id for point "<<ppt<<" is "<<patchNumber<<endl;
            int active = attActive.get(ppt);
            float life = attLife.get(ppt);
            if (active == 0 && life <= 0 )
                continue;
            if (params.testPatch == 1 && params.patchNumber != patchNumber)
                continue;
            N = attN.get(ppt);
            position = trackersGdp->getPos3(ppt);

            //getting neigborhood
            // Close particles indices
            GEO_PointTreeGAOffset::IdxArrayType surfaceNeighborhoodVertices;
            surfaceTree.findAllCloseIdx(position,
                                 patchRadius,
                                 surfaceNeighborhoodVertices);

            unsigned close_particles_count = surfaceNeighborhoodVertices.entries();

            string str = std::to_string(patchNumber);
            UT_String patchGroupName("patch"+str);
            //cout << "Create patch "<<patchGroupName<<endl;
            GA_PointGroup* patchGroup = surfaceGdp->newPointGroup(patchGroupName.c_str(), 0);

            set<GA_Offset> neighborhood;
            for(int j=0; j<close_particles_count;j++ )
            {
                surfacePointOffset = surfaceNeighborhoodVertices.array()[j];
                neighborhood.insert(surfacePointOffset);
            }

            set<GA_Offset>::iterator itG;
            for(itG = neighborhood.begin(); itG != neighborhood.end(); ++itG)
            {
                surfacePointOffset = *itG;
                NN = attNSurface.get(surfacePointOffset);
                float dotP = dot(N,NN); //exlude points that are not in the same plane.
                if (dotP < params.angleNormalThreshold)
                    continue;

                //add current patch id in the set:
                surfacePatchIds[surfacePointOffset].insert(patchNumber);
                //the next line creates a weird bug where the list of point in trackergdp is modified:
                patchGroup->addOffset(surfacePointOffset);

                numberOfPatcheCreated++;
            }
            neighborhood.clear();
        }
    }
    {
        GA_Offset ppt;
        cout << "We have now "<<trackersGdp->getNumPoints()<<" points"<<endl;
        GA_FOR_ALL_PTOFF(trackersGdp, ppt)
        {
            isTangeant = isTangeantTracker.get(ppt);
            if (isTangeant == 1)
                continue;

            patchNumber = attId.get(ppt);
            //cout << "id for point "<<ppt<<" is "<<patchNumber<<endl;

            if (patchTreated.count(patchNumber) > 0)
            {
                cout << "We already treated patch "<< patchNumber << endl;
                continue;
            }
            patchTreated.insert(patchNumber);

            int active = attActive.get(ppt);
            float life = attLife.get(ppt);

            //cout << active << " life "<<life<<endl;
            if (active == 0 && life <= 0 )
                continue;

            //for test purposes
            if (params.testPatch == 1 && params.patchNumber != patchNumber)
                continue;

            string str = std::to_string(patchNumber);
            UT_String pointGroupName("patch"+str);
            GA_PrimitiveGroup* primGrp = surfaceGdp->newPrimitiveGroup(pointGroupName);
            GA_GroupType groupType = GA_GROUP_POINT;
            const GA_GroupTable *gtable = surfaceGdp->getGroupTable(groupType);
            GA_OffsetArray primitives;
            GA_PointGroup* pointGrp = (GA_PointGroup*)gtable->find(primGrp->getName());
            if (pointGrp == 0x0)
                continue;
            GA_Offset surfacePpt;
            GA_FOR_ALL_GROUP_PTOFF(surfaceGdp,pointGrp,surfacePpt)
            {
                surfaceGdp->getPrimitivesReferencingPoint(primitives,surfacePpt);
                for(GA_OffsetArray::const_iterator prims_it = primitives.begin(); !prims_it.atEnd(); ++prims_it)
                {
                    primGrp->addOffset(*prims_it);
                }
            }
        }
     }

    //for each point of the surface, register the set of patch ids:
    {
        cout << "register patch ids to surface points"<<endl;
        UT_IntArray         patchArrayData;
        GA_Offset ppt;
        set<int>::iterator it;

        GA_FOR_ALL_PTOFF(surfaceGdp, ppt)
        {
            set<int> ids = surfacePatchIds[ppt];
            patchIdsAtt->get(patchIdsArrayAttrib,ppt, patchArrayData);

            for(it = ids.begin(); it != ids.end(); it++)
            {
                int patchNumber = *it;
                patchArrayData.append(patchNumber);
                int numberOfPatch = attNumberOfPatch.get(ppt);
                numberOfPatch++;
                attNumberOfPatch.set(ppt,numberOfPatch);
            }
            patchIdsAtt->set(patchIdsArrayAttrib,ppt, patchArrayData);
        }
    }
    this->patchCreationTime += (std::clock() - addPatchesStart) / (double) CLOCKS_PER_SEC;
}

void LappedSurfaceGagnon2016::OrthogonalUVProjection(GU_Detail* surface, GU_Detail *trackersGdp, ParametersDeformablePatches params)
{
    cout << "[LappedSurfaceGagnon2016] OrthogonalUVProjection" << endl;
    this->orthogonalUVProjectionTime = 0;
    std::clock_t projectionStart;
    projectionStart = std::clock();
    GA_GroupType groupType = GA_GROUP_POINT;
    const GA_GroupTable *gtable = surface->getGroupTable(groupType);
    int patchNumber=0;
    GA_Offset ppt;
    UT_Vector3 N;
    UT_Vector3 S,T;
    int isTangeant = 0;
    UT_Vector3 trackerPosition;

    GA_RWHandleV3 attUVTracker(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"uv", 3));

    GA_FOR_ALL_PTOFF(trackersGdp, ppt)
    {
        patchNumber = attId.get(ppt);
        if (params.testPatch == 1 && params.patchNumber != patchNumber)
            continue;

        //replace the tangeant tracker
        isTangeant = isTangeantTracker.get(ppt);
        if (isTangeant == 1)
            continue;
        //cout << "Project UV for patch "<<patchNumber<<endl;
        N = attN.get(ppt);
        trackerPosition = trackersGdp->getPos3(ppt);
        GA_Offset tracker_offset = ppt+1;
        UT_Vector3 tangeantPosition = trackersGdp->getPos3(tracker_offset);
        UT_Vector3 currentDirection = tangeantPosition-trackerPosition;
        currentDirection.normalize();
        N.normalize();
        S = cross(N,currentDirection);
        S.normalize();
        T = cross(S,N);
        T.normalize();

        GA_RWHandleV3 surfaceAttUV(surface->addFloatTuple(GA_ATTRIB_POINT,"uv"+std::to_string(patchNumber), 3));

        // Transform into local patch space (where STN is aligned with XYZ at the origin)
        UT_Vector3 trackerTrianglePos;
        trackerTrianglePos.x() = trackerPosition.dot(S);
        trackerTrianglePos.y() = trackerPosition.dot(T);
        trackerTrianglePos.z() = trackerPosition.dot(N);

        //this is not working.
        UT_Vector3 centerUv;
        centerUv.x() = trackerTrianglePos.x();
        centerUv.y() = trackerTrianglePos.y();
        centerUv.z() = 0;//trackerTrianglePos.z();

        float mid = 0.5;
        centerUv /= params.UVScaling;
        centerUv += mid;

        attUVTracker.set(ppt,centerUv);

        string patchGroupName= "patch"+std::to_string(patchNumber);

        GA_PointGroup*pointGrp = (GA_PointGroup*)gtable->find(patchGroupName.c_str());

        if (pointGrp)
        {
            GA_Offset pointOffset;
            int nbTreated = 0;
            UT_Vector3 centerUV = UT_Vector3(0,0,0);
            GA_FOR_ALL_GROUP_PTOFF(surface, pointGrp, pointOffset)
            {
                //cout << "Projection for patch "<<pointGrp->getName()<<endl;
                //----------------------- UV PROJECTION --------------
                UT_Vector3 currentPosition = surface->getPos3(pointOffset);

                // Transform into local patch space (where STN is aligned with XYZ at the origin)
                const UT_Vector3 relativePosistion = currentPosition-trackerPosition;
                UT_Vector3 triangleSpacePos;
                triangleSpacePos.x() = relativePosistion.dot(S);
                triangleSpacePos.y() = relativePosistion.dot(T);
                triangleSpacePos.z() = relativePosistion.dot(N);

                UT_Vector3 uv;
                uv.x() = triangleSpacePos.x();
                uv.y() = triangleSpacePos.y();
                uv.z() = triangleSpacePos.z();

                float mid = 0.5;
                uv /= params.UVScaling;
                uv += mid;
                //uv.z should be zero
                //uv -= centerUv;

                UT_FloatArray         fdata;
                UT_IntArray patchArrayData;
                // Fetch array value
                patchIdsAtt->get(patchIdsArrayAttrib, pointOffset, patchArrayData);
                int nb = patchArrayData.size();
                int index = -1;
                for (int i = 0; i< nb; i++)
                {
                    if (patchArrayData.array()[i] == patchNumber)
                        index = i;
                }

                // Fetch array value
                uvsArray->get(uvsAtt, pointOffset, fdata);

                if (index == -1)
                    continue;

                fdata.insertAt(uv.x(), (index*3)+0);
                fdata.insertAt(uv.y(), (index*3)+1);
                fdata.insertAt(uv.z(), (index*3)+2);

                // Write back
                uvsArray->set(uvsAtt, pointOffset, fdata);

                surfaceAttUV.set(pointOffset,uv);
                centerUV += uv;
                nbTreated++;
            }

            if (nbTreated > 0)
            {
                centerUV /= nbTreated;
                attCenterUV.set(ppt,centerUV);
            }
        }
        attCenterUV.set(ppt,UT_Vector3(0.5,0.5,0.0));
    }
    this->orthogonalUVProjectionTime += (std::clock() - projectionStart) / (double) CLOCKS_PER_SEC;
}

//======================================================================================================================================
//                                                  UpdateUsingBridson2012PoissonDisk
//======================================================================================================================================

void LappedSurfaceGagnon2016::DeleteUnusedPatches(GU_Detail *gdp, GU_Detail *trackersGdp, ParametersDeformablePatches params)
{
    cout << this->approachName<<" Update Using Bridson 2012 Poisson Disk with "<<numberOfPatches<<" existing trackers"<<endl;
    std::clock_t startUpdatePatches;
    startUpdatePatches = std::clock();

    //--------------------------------------------------------------------------

    /*
    GA_RWHandleI attId(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"id",1));
    GA_RWHandleF attLife(trackersGdp->findFloatTuple(GA_ATTRIB_POINT,"life",1));
    GA_RWHandleI attSpawn(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"spawn",1));
    GA_RWHandleI attActive(trackersGdp->addIntTuple(GA_ATTRIB_POINT,"active", 1));

    GA_RWHandleI attIsMature(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"isMature", 1));
    */
    GU_Detail::GA_DestroyPointMode mode = GU_Detail::GA_DESTROY_DEGENERATE;

    GA_Offset ppt;
    int beforeAddingNumber = numberOfPatches;

    GA_PointGroup *grpToDestroy = (GA_PointGroup *)trackersGdp->newPointGroup("ToDelete");
    GA_GroupType groupType = GA_GROUP_POINT;
    const GA_GroupTable *gtable = gdp->getGroupTable(groupType);
    GA_GroupType primGroupType = GA_GROUP_PRIMITIVE;
    const GA_GroupTable *gPrimTable = gdp->getGroupTable(primGroupType);
    set<int> toDelete;
    //--------------------------- DELETE DEAD PATCH --------------------------------------------
    {
        GA_FOR_ALL_PTOFF(trackersGdp,ppt)
        {
            int id = attId.get(ppt);
            float life = attLife.get(ppt);
            int active = attActive.get(ppt);
            if (active == 0 && life <= 0.0f && params.frame != params.startFrame)
            {
                //cout << "Deleting deformable grid "<<id<<" mature "<<attIsMature.get(ppt)<<endl;
                toDelete.insert(id);
                numberOfPatches--;
                numberOfConcealedPatches++;

                string str = std::to_string(id);
                string groupName = "grid"+str;
                GA_PrimitiveGroup *primGroup = (GA_PrimitiveGroup*)gPrimTable->find(groupName.c_str());
                GA_PointGroup* pointGrp = (GA_PointGroup*)gtable->find(groupName.c_str());

                if (pointGrp != 0x0)
                {
                    //cout << "delete points "<<endl;
                    gdp->deletePoints(*pointGrp,mode);
                    //cout << "delete point group "<<groupName<<endl;
                    gdp->destroyPointGroup(pointGrp);
                }
                if (primGroup != 0x0)
                {
                    //cout << "delete point group "<<groupName<<endl;
                    gdp->destroyPrimitiveGroup(primGroup);
                }
            }
        }
    }
    {
        GA_FOR_ALL_PTOFF(trackersGdp,ppt)
        {
            int id = attId.get(ppt);
            if (toDelete.count(id) > 0)
            {
                //cout << "Delete poisson disk point "<<id<<endl;
                grpToDestroy->addOffset(ppt);
            }
        }
    }
    //destroying trackers
    trackersGdp->deletePoints(*grpToDestroy,mode);
    trackersGdp->destroyPointGroup(grpToDestroy);

    cout <<this->approachName<< " Added "<<(numberOfPatches-beforeAddingNumber) <<" new patches"<<endl;
    cout <<this->approachName<< " Removed "<<(numberOfConcealedPatches)<<" patches "<<endl;
    cout <<this->approachName<< "uv projection time "<<orthogonalUVProjectionTime<< endl;
    this->updatePatchesTime += (std::clock() - startUpdatePatches) / (double) CLOCKS_PER_SEC;
    cout << this->approachName<<" TOTAL "<<numberOfPatches<< " patches"<<endl;
}

