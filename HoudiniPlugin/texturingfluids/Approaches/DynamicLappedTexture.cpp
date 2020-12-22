#include "DynamicLappedTexture.h"

#include <fstream>
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
#include <GU/GU_NeighbourList.h>

#include <GU/GU_Flatten.h>
#include <GU/GU_RayIntersect.h>

#include <Core/Gagnon2016/LappedSurfaceGagnon2016.h>


// random generator function:
int myrandom (GA_Offset i) { return std::rand()%i;}

DynamicLappedTexture::DynamicLappedTexture()
{
}

DynamicLappedTexture::~DynamicLappedTexture()
{
}

void DynamicLappedTexture::Synthesis(GU_Detail *surfaceGdp, GU_Detail *trackersGdp, GU_Detail *levelSet, ParametersDeformablePatches params)
{

    const string surfaceGroupName = "surface";
    GA_PointGroup *surfaceGroup = (GA_PointGroup *)surfaceGdp->pointGroups().find(surfaceGroupName.c_str());
    if (surfaceGroup == 0x0)
    {
        cout << "There is no surface group to synthesis"<<endl;
        return;
    }

    LappedSurfaceGagnon2016 surface(surfaceGdp, surfaceGroup, trackersGdp, params);
    cout << "[DynamicLappedTexture::Synthesis] "<<params.frame<<endl;
    //params.useDynamicTau = false;

    std::clock_t start;
    start = std::clock();
    cout << "reference gdp created"<<endl;
    const GA_SaveOptions *options;
    UT_StringArray *errors;


    //=======================================================

    GEO_PointTreeGAOffset surfaceTree;
    surfaceTree.build(surfaceGdp, NULL);
    vector<GA_Offset> newPatchesPoints;
    GA_RWHandleI    attId(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"id",1));

    //=========================== CORE ALGORITHM ============================
    bool shufflePatchIds = false; //When this is true, we have a weird texture synthesis artifacté

    if(params.startFrame == params.frame)
    {

        newPatchesPoints = surface.PoissonDiskSamplingDistribution(levelSet,params.poissondiskradius, params.poissonAngleNormalThreshold);
        if (shufflePatchIds)
        {
            std::srand ( unsigned ( std::time(0) ) );
            vector<GA_Offset>::iterator itPoint;
            cout << "random shuffle offset values"<<endl;
            std::random_shuffle ( newPatchesPoints.begin(), newPatchesPoints.end(), myrandom);
            int index = 0;
            for (itPoint = newPatchesPoints.begin(); itPoint != newPatchesPoints.end(); itPoint++)
            {
                index++;
                GA_Offset newPoint = *itPoint;
                attId.set(newPoint,index);
                // taking the next point offset which is supposed to the tangeant tracker.
                GA_Offset tangeantNewPoint = newPoint + 1;
                attId.set(tangeantNewPoint, index);
            }
        }
    }
    else
    {
        surface.AdvectTrackersAndTangeants();
        surface.UpdateTrackersAndTangeant();
        surface.PoissonDiskSamplingDistribution(levelSet,params.poissondiskradius, params.poissonAngleNormalThreshold);
    }
    cout << "Using less arguments"<<endl;
    surface.CreateAndUpdateTrackersBasedOnPoissonDisk();
    //For the blending computation, we create uv array per vertex that we called patch
    surface.AddSolidPatchesUsingBarycentricCoordinates();
    surface.OrthogonalUVProjection();

    //=======================================================================

    cout << surface.approachName<<" Done"<<endl;
    cout << "Clear surface tree"<<endl;
    surfaceTree.clear();

    const char* filenameTrackers = params.trackersFilename.c_str();//"dlttest.bgeo";
    cout << surface.approachName<< " saving trackers data in "<<filenameTrackers<<endl;

    trackersGdp->save(filenameTrackers,options,errors);

    //================================================================
    std::clock_t cleaningStart;
    cleaningStart = std::clock();
    cout<< "Clear, Destroy and merge"<<endl;

    int nbPatches = surface.GetNumberOfPatches();

    float cleaningSurface = (std::clock() - cleaningStart) / (double) CLOCKS_PER_SEC;
    cout << "--------------------------------------------------------------------------------"<<endl;
    cout << surface.approachName<<" Poisson Disk Sampling "<<surface.poissondisk<<endl;
    cout << surface.approachName<<" Tracker advection time "<<surface.markerAdvectionTime<<endl;
    cout << surface.approachName<<" Patch creation time "<<surface.patchCreationTime<< " for "<<surface.numberOfPatcheCreated<<" patches"<<endl;
    cout << surface.approachName<<" UV Orthogonal Projection "<<surface.orthogonalUVProjectionTime<<endl;
    cout << surface.approachName<<" Clear and Destroy "<<cleaningSurface<<endl;
    cout << surface.approachName<<" Update distribution "<<surface.updatePatchesTime<<endl;

    float total = (std::clock() - start) / (double) CLOCKS_PER_SEC;
    cout << surface.approachName<< " TOTAL: "<<total<<endl;

    std::ofstream outfile;
    outfile.open("core.csv", std::ios_base::app);
    outfile <<surface.poissondisk<< ","<<surface.markerAdvectionTime
            <<","<<surface.patchCreationTime << ","<<surface.updatePatchesTime<<","<<nbPatches<<endl;

    cout << "--------------------------------------------------------------------------------"<<endl;
}

