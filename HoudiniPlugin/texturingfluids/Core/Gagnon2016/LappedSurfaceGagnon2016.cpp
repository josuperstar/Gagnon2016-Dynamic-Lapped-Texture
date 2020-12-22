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

#include <Core/HoudiniUtils.h>


LappedSurfaceGagnon2016::LappedSurfaceGagnon2016(GU_Detail *surface, GA_PointGroup *surfaceGroup, GU_Detail *trackersGdp, ParametersDeformablePatches params) : ParticleAndTrackerManagerGagnon2016(surface, surfaceGroup, trackersGdp, params)
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

    cout << "[LappedSurfaceGagnon2016] Building surface tree"<<endl;
    surfaceTree.build(surfaceGdp, NULL);
}

LappedSurfaceGagnon2016::~LappedSurfaceGagnon2016()
{
    cout << "[LappedSurfaceGagnon2016] Destroying"<<endl;
    this->rays.clear();
}



//================================================================================================

//                                       ADD PATCHES USING BARYCENTRIC COORDONATE

//================================================================================================


void LappedSurfaceGagnon2016::AddSolidPatchesUsingBarycentricCoordinates()
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

void LappedSurfaceGagnon2016::OrthogonalUVProjection()
{
    cout << "[LappedSurfaceGagnon2016] OrthogonalUVProjection" << endl;
    this->orthogonalUVProjectionTime = 0;
    std::clock_t projectionStart;
    projectionStart = std::clock();
    GA_GroupType groupType = GA_GROUP_POINT;
    const GA_GroupTable *gtable = surfaceGdp->getGroupTable(groupType);
    int patchNumber=0;
    GA_Offset ppt;
    UT_Vector3 N;
    UT_Vector3 S,T;
    int isTangeant = 0;
    UT_Vector3 trackerPosition;

    GA_RWHandleV3 attUVTracker(trackersGdp->addFloatTuple(GA_ATTRIB_POINT,"uv", 3));

    // We need to sorted trackers according to their id
    set<int> sortedTrackers;
    set<int>::iterator sortedIt;
    map<int, GA_Offset> trackersMap;
    {
        GA_FOR_ALL_PTOFF(trackersGdp, ppt)
        {
            patchNumber = attId.get(ppt);
            isTangeant = isTangeantTracker.get(ppt);
            if (isTangeant == 1)
            {
                continue;
            }
            sortedTrackers.insert(patchNumber);
            trackersMap[patchNumber] = ppt;
        }
    }
//    cout << "Resulting map "<<endl;
//    for (sortedIt = sortedTrackers.begin(); sortedIt != sortedTrackers.end(); sortedIt++)
//    {
//        patchNumber = *sortedIt;
//        ppt = trackersMap[patchNumber];
//        cout << patchNumber << " "<<ppt<<endl;
//    }

    for (sortedIt = sortedTrackers.begin(); sortedIt != sortedTrackers.end(); sortedIt++)
    {

        //patchNumber = attId.get(ppt);
        patchNumber = *sortedIt;
        GA_Offset trackerPpt = trackersMap[patchNumber];

        cout << "Dealing with "<<patchNumber;

//        if (params.testPatch == 1 && params.patchNumber != patchNumber)
//            continue;

        //replace the tangeant tracker
        isTangeant = isTangeantTracker.get(trackerPpt);
        if (isTangeant == 1)
        {
            cout << "Is a tangeant tracker"<<endl;
            continue;
        }
        cout << " working on it"<<endl;
        //cout << "Project UV for patch "<<patchNumber<<endl;
        N = attN.get(trackerPpt);
        trackerPosition = trackersGdp->getPos3(trackerPpt);
        GA_Offset tracker_offset = trackerPpt+1;
        UT_Vector3 tangeantPosition = trackersGdp->getPos3(tracker_offset);
        UT_Vector3 currentDirection = tangeantPosition-trackerPosition;
        currentDirection.normalize();
        N.normalize();
        S = cross(N,currentDirection);
        S.normalize();
        T = cross(S,N);
        T.normalize();

        GA_RWHandleV3 surfaceAttUV(surfaceGdp->addFloatTuple(GA_ATTRIB_POINT,"uv"+std::to_string(patchNumber), 3));

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

        attUVTracker.set(trackerPpt,centerUv);

        string patchGroupName= "patch"+std::to_string(patchNumber);

        GA_PointGroup*pointGrp = (GA_PointGroup*)gtable->find(patchGroupName.c_str());

        if (pointGrp)
        {
            GA_Offset pointOffset;
            int nbTreated = 0;
            UT_Vector3 centerUV = UT_Vector3(0,0,0);
            GA_FOR_ALL_GROUP_PTOFF(surfaceGdp, pointGrp, pointOffset)
            {
                //cout << "Projection for patch "<<pointGrp->getName()<<endl;
                //----------------------- UV PROJECTION --------------
                UT_Vector3 currentPosition = surfaceGdp->getPos3(pointOffset);

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
                attCenterUV.set(trackerPpt,centerUV);
            }
        }
        attCenterUV.set(trackerPpt,UT_Vector3(0.5,0.5,0.0));
    }
    this->orthogonalUVProjectionTime += (std::clock() - projectionStart) / (double) CLOCKS_PER_SEC;
}

//======================================================================================================================================
//                                                  UpdateUsingBridson2012PoissonDisk
//======================================================================================================================================

void LappedSurfaceGagnon2016::DeleteUnusedPatches()
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
    const GA_GroupTable *gtable = surfaceGdp->getGroupTable(groupType);
    GA_GroupType primGroupType = GA_GROUP_PRIMITIVE;
    const GA_GroupTable *gPrimTable = surfaceGdp->getGroupTable(primGroupType);
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
                    surfaceGdp->deletePoints(*pointGrp,mode);
                    //cout << "delete point group "<<groupName<<endl;
                    surfaceGdp->destroyPointGroup(pointGrp);
                }
                if (primGroup != 0x0)
                {
                    //cout << "delete point group "<<groupName<<endl;
                    surfaceGdp->destroyPrimitiveGroup(primGroup);
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

