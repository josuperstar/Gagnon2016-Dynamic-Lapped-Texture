#ifndef __LAPPED_SURFACE_H__
#define __LAPPED_SURFACE_H__

#include <Math/Vec3.h>
#include <Core/Gagnon2016/ParticleAndTrackerManagerGagnon2016.h>
#include <GEO/GEO_PointTree.h>
#include <GU/GU_RayIntersect.h>

namespace TexturingFluids {

#define VERBOSE 0


class LappedSurfaceGagnon2016 : public ParticleAndTrackerManagerGagnon2016
{
public:

    LappedSurfaceGagnon2016(GU_Detail *surface, GU_Detail *trackersGdp);
    ~LappedSurfaceGagnon2016();

    void PoissonDiskSampling(GU_Detail *surfaceGdp, GU_Detail *trackers, ParametersDeformablePatches params);
    void AddSolidPatchesUsingBarycentricCoordinates(GU_Detail* surface, GU_Detail *trackersGdp, ParametersDeformablePatches params,  GEO_PointTreeGAOffset &surfaceTree);
    void ShufflePoints(GU_Detail *trackers);
    void OrthogonalUVProjection(GU_Detail* surface, GU_Detail *trackersGdp, ParametersDeformablePatches params);
    void DeleteUnusedPatches(GU_Detail *gdp, GU_Detail *trackersGdp, ParametersDeformablePatches params);
    void FillSurfaceHoles(GU_Detail *surfaceGdp, GU_Detail *trackers, ParametersDeformablePatches params);
    //for test purpose
    void CreateAPatch(GU_Detail *trackers, ParametersDeformablePatches params);

    double poissondisk;
    double  patchCreationTime;
    double  updatePatchesTime;
    double orthogonalUVProjectionTime;
    const string approachName   = "[DynamicLappedTexture]";
    const string uvArrayName = "uvs";
    const string alphaArrayName = "alphas";
    const string patchIdsName = "patchIds";
    int numberOfPatcheCreated=0;
private :

    //-------------- VERTEX ARRAY -----------------
    GA_Attribute        *uvsAtt;
    const GA_AIFNumericArray *uvsArray;

    GA_Attribute        *patchIdsArrayAttrib;
    const GA_AIFNumericArray *patchIdsAtt;

    GA_Attribute        *alphaArrayAtt;
    const GA_AIFNumericArray *alphaAtt;
    //---------------------------------------------

    map<string,GU_RayIntersect*> rays;


};

}

#endif
