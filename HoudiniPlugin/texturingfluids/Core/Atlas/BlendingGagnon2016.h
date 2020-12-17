#ifndef __BlendingGagnon2016_h__
#define __BlendingGagnon2016_h__

#include <string>
#include <vector>
#include "Blending.h"


namespace TexturingFluids {

class BlendingGagnon2016 : public Blending
{

public:

    static Pixel Blend(GU_Detail* trackersGdp,GU_Detail* deformableGrids, int i, int j, float w, float h,
                int pixelPositionX, int pixelPositionY,
                vector<int> &sortedPatches,
                vector<UT_Vector3> &surfaceUv,
                vector<UT_Vector3> &surfacePosition,
                map<int,UT_Vector3> &trackersPosition,
                map<int,UT_Vector3> &trackersUVPosition,
                bool useDeformableGrids,
                map<string,GU_RayIntersect*> &rays,
                map<int,Pixel> &patchColors,
                Pixel alphaColor,
                Pixel RM,           //Mean Value
                GA_ROHandleF &attAlpha,
                GA_RWHandleV3 &attPointUV,
                int life,
                map<int,float> &patchBlend,
                map<int, vector<UT_Vector3> > &patchUvs,
                map<int, vector<float> > &alphasMap,
                ImageCV *textureExemplar1Image,
                ImageCV *textureExemplar1ImageMask,
                ImageCV *displacementMapImage,
                bool computeDisplacement,
                bool renderColoredPatches,
                Pixel &displacementSum,
                ParametersDeformablePatches params);


};
}

#endif
