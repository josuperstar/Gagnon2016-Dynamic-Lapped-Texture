#ifndef __PatchedSurfaceGagnon2016__
#define __PatchedSurfaceGagnon2016__

#include <Math/Vec3.h>
#include <Core/Gagnon2016/ParticleAndTrackerManagerGagnon2016.h>
#include <GEO/GEO_PointTree.h>
#include <GU/GU_RayIntersect.h>

namespace TexturingFluids {

#define VERBOSE 0


class PatchedSurface : public ParticleAndTrackerManagerGagnon2016
{
public:

    PatchedSurface(GU_Detail *surface, GA_PointGroup *surfaceGroup, GU_Detail *trackersGdp, ParametersDeformablePatches params);
    ~PatchedSurface();


    void AddDeformablePatcheUsingBarycentricCoordinates(GU_Detail *deformableGridsGdp,GU_Detail *surfaceGdp, GU_Detail *trackersGdp, GA_Offset ppt, ParametersDeformablePatches params, GEO_PointTreeGAOffset &surfaceTree,  GU_RayIntersect &ray);
    void AddDeformablePatchesUsingBarycentricCoordinates(GU_Detail *gdp, GU_Detail* surface, GU_Detail *trackersGdp, ParametersDeformablePatches params,  GEO_PointTreeGAOffset &surfaceTree, GU_RayIntersect &ray);
    void DeleteUnusedPatches(GU_Detail *gdp, GU_Detail *trackersGdp, ParametersDeformablePatches params);

    double poissondisk;
    double  patchCreationTime;
    double  updatePatchesTime;
    const string approachName   = "[Patched Surface]";

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

    const string uvArrayName = "uvs";
    const string alphaArrayName = "alphas";
    const string patchIdsName = "patchIds";


};

}

#endif
