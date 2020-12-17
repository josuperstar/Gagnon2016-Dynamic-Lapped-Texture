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
#include <Core/Gagnon2016/Bridson2012PoissonDiskDistribution.h>

DynamicLappedTexture::DynamicLappedTexture()
{
}

DynamicLappedTexture::~DynamicLappedTexture()
{
}

void DynamicLappedTexture::Synthesis(GU_Detail *surfaceGdp, GU_Detail *trackersGdp, GU_Detail *levelSet, ParametersDeformablePatches params)
{
    LappedSurfaceGagnon2016 surface(surfaceGdp, trackersGdp);
    cout << "[DynamicLappedTexture::Synthesis] "<<params.frame<<endl;
    //params.useDynamicTau = false;

    std::clock_t start;
    start = std::clock();
    cout << "reference gdp created"<<endl;
    const GA_SaveOptions *options;
    UT_StringArray *errors;

    GA_PointGroup *surfaceGroup = (GA_PointGroup *)surfaceGdp->pointGroups().find(surface.surfaceGroupName.c_str());
    if (surfaceGroup == 0x0)
    {
        cout << "There is no surface group to synthesis"<<endl;
        return;
    }
    //=======================================================

    GEO_PointTreeGAOffset surfaceTree;
    surfaceTree.build(surfaceGdp, NULL);

    //=========================== CORE ALGORITHM ============================

    if(params.startFrame == params.frame)
    {
        surface.PoissonDiskSampling(levelSet,trackersGdp,params);
        //surface.ShufflePoints(trackersGdp);


    }
    else
    {
        surface.AdvectTrackersAndTangeants(surfaceGdp, trackersGdp, params);
        surface.UpdateTrackersAndTangeant(surfaceGdp,trackersGdp, surfaceGroup,params);
        surface.PoissonDiskSampling(levelSet,trackersGdp,params);
    }

    surface.CreateAndUpdateTrackersBasedOnPoissonDisk(surfaceGdp,trackersGdp,surfaceGroup,params);
    //For the blending computation, we create uv array per vertex that we called patch
    surface.AddSolidPatchesUsingBarycentricCoordinates(surfaceGdp,trackersGdp, params,surfaceTree);
    surface.OrthogonalUVProjection(surfaceGdp,trackersGdp,params);

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

