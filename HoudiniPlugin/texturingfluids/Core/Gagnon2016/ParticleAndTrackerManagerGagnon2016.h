#ifndef __ParticleAndTrackerManager_h__
#define __ParticleAndTrackerManager_h__

#include <Math/Vec3.h>
#include <Strategies/StrategyPatchSurfaceSynthesis.h>
#include <GEO/GEO_PointTree.h>
#include <GU/GU_RayIntersect.h>

#include <openvdb/openvdb.h>
#include <openvdb/tools/Interpolation.h>
#include <openvdb/tools/GridOperators.h>

namespace TexturingFluids {

#define VERBOSE 0


class ParticleAndTrackerManagerGagnon2016
{
public:

    ParticleAndTrackerManagerGagnon2016(GU_Detail *surfaceGdp, GA_PointGroup *surfaceGroup, GU_Detail *trackersGdp, ParametersDeformablePatches params);
    vector<GA_Offset>  InitializeTrackersAndTangeants();
    void  CreateAndUpdateTrackersBasedOnPoissonDisk();
    void  UpdateTrackersAndTangeant();
    void AdvectSingleTrackers();
    void AdvectTrackersAndTangeants();
    void ComputeDivergence();
    void ComputeDensity();
    void DeleteTracker(GU_Detail* trackers,int trackerId);
    vector<GA_Offset> PoissonDiskSamplingDistribution(GU_Detail *levelSet, float diskRadius, float angleNormalThreshold);
    bool RespectCriterion(UT_Vector3 newPointPosition, UT_Vector3 newPointNormal, float killDistance, int &numberOfClosePoint,   GA_Offset exclude );
    GA_Offset CreateAParticle(UT_Vector3 p, UT_Vector3 N);
    int GetNumberOfPatches(){return numberOfPatches;}
    UT_Vector3 SetRandomColor(int patchNumber);
    openvdb::Vec3f projectPointOnLevelSet(openvdb::Vec3f point, float distance, openvdb::Vec3f grad);

    const string markerGroupName = "markers";
    const string surfaceGroupName = "surface";
    const string approachName   = "[Particle Tracker]";

    double  markerAdvectionTime;

    double poissondisk;


protected :

    bool tackerPolygon = false;
    int numberOfPatches;
    int numberOfConcealedPatches;
    int numberOfNewPatches;
    int numberOfDetachedPatches;
    int numberOfLonelyTracker;
    int numberOfNewPoints;
    int maxId = 0;
    const char*    randomThresholdDistortion = "t_rand";
    float epsilon = 0.0001;

    float poissonDiskRadius;    //radius
    int k;      //the limit of samples to choose before rejection in the algorithm, typically k = 30
    int n = 3; // n-dimensional
    int t; // number of attemps

    GA_RWHandleV3   attN;
    GA_RWHandleV3   attV;
    GA_RWHandleV3   attCenterUV;
    GA_RWHandleI    attId;
    GA_RWHandleF    attLife;
    GA_RWHandleI    attSpawn;
    GA_RWHandleI    attActive;
    GA_RWHandleI    attIsMature;
    GA_RWHandleI    attDensity;
    GA_RWHandleF    attBlend;
    GA_RWHandleF    attRandT;
    GA_RWHandleF    attMaxDeltaOnD;
    GA_RWHandleI    attDeleteFaster;
    GA_RWHandleV3   refAttV;
    GA_RWHandleV3   refAttN;
    GA_RWHandleV3   tangeant;
    GA_RWHandleI    attFadeIn;
    GA_RWHandleI    isTangeantTracker;
    GA_RWHandleF    temporalComponentKt;
    GA_RWHandleV3 AttCd;

    GA_RWHandleV3 attVSurface;
    //GA_RWHandleV3 attNSurface;
    GA_RWHandleF attDivergence;

    GA_RWHandleI    attNumberOfPrimitives;

    GU_Detail *trackersGdp;
    GU_Detail *surfaceGdp;
    GEO_PointTreeGAOffset trackerTree;
    GEO_PointTreeGAOffset surfaceTree;

    GA_PointGroup *surfaceGroup;

    ParametersDeformablePatches params;

};

}

#endif
