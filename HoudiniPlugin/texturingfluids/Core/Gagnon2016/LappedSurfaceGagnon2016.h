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

    LappedSurfaceGagnon2016(GU_Detail *surface, GA_PointGroup *surfaceGroup, GU_Detail *trackersGdp, ParametersDeformablePatches params);
    ~LappedSurfaceGagnon2016();

    void AddSolidPatchesUsingBarycentricCoordinates();
    void OrthogonalUVProjection();
    void DeleteUnusedPatches();
    void FillSurfaceHoles();

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
