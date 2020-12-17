#ifndef __AtlasGagnon2016_h__
#define __AtlasGagnon2016_h__

#include "Atlas.h"
#include "../../Images/ImageCV.h"
#include <SOP/SOP_Node.h>
#include <GEO/GEO_PrimPart.h>
#include <GEO/GEO_PointTree.h>
#include <GU/GU_RayIntersect.h>
#include "Core/Deformations/ParametersDeformablePatches.h"


namespace TexturingFluids {


class AtlasGagnon2016 : public Atlas
{

public:

    ~AtlasGagnon2016();

   bool BuildAtlas(int w, int h, int life);

   void SetSurface(GU_Detail *data) {surface = data;}
   void SetDeformableGrids(GU_Detail *data) {deformableGrids = data;}
   void SetTrackers(GU_Detail *data) {trackers = data;}
   void SetTrackersPosition(map<int,UT_Vector3> data){ trackerPosition = data;}

   void SetTextureExemplar1(string data){textureExemplar1Name = data;}
   void SetTextureExemplar1Mask(string data){textureExemplar1MaskName = data;}
   void SetDisplacementMap1(string data){displacementMapImageName = data;}
   void RenderColoredPatches(bool data) {renderColoredPatches = data;}

   void RasterizePrimitive(GA_Offset primOffset, int w,int h, ParametersDeformablePatches params);
   void RasterizePrimitiveYu2011BlendingFunction(GA_Offset primOffset, int w,int h,ParametersDeformablePatches params);

   void SaveAtlas();

   void RenderColoredPatches() {renderColoredPatches = true;}
   void UseDeformableGrids() {useDeformableGrids = true;}


private:

   void BoundingBox2D(UT_Vector3 a, UT_Vector3 b, UT_Vector3 c, UT_Vector3 &min,UT_Vector3 &max);
   bool IsPointInTriangle(UT_Vector3  p, UT_Vector3 a,UT_Vector3 b,UT_Vector3 c);

   void CreateListGUDetails();
   Pixel SetRandomColor(int patchNumber);
   map<int,Pixel> patchColors;
   void initPatchColors(GU_Detail *trackersGdp);

   GU_Detail* surface;
   GU_Detail* deformableGrids;
   GU_Detail* trackers;

   ImageCV *diffuseImageBlendingGagnon;
   ImageCV *diffuseImageBlendingYu2011Equation3;
   ImageCV *diffuseImageBlendingYu2011Equation4;
   ImageCV *displacementMap;

   ImageCV *textureExemplar1Image;
   Pixel RM;
   string textureExemplar1Name;


   ImageCV *textureExemplar1ImageMask;
   string textureExemplar1MaskName;

   ImageCV *displacementMapImage;
   string displacementMapImageName;

   GEO_PointTreeGAOffset surfaceTree;
   GEO_PointTreeGAOffset gridTree;


   const GA_AIFNumericArray *patchArray;
   GA_Attribute        *patchIds;

   GA_Attribute        *alphas;
   const GA_AIFNumericArray *alphaArray;

   GA_Attribute        *uvsAtt;
   const GA_AIFNumericArray *uvsArray;

   GA_RWHandleV3 attUV;
   GA_RWHandleV3 attGridUV;
   GA_ROHandleF attAlpha;
   GA_RWHandleI attInitOffset;
   GA_RWHandleV3 attPointUV;
   float attLife;
   GA_ROHandleI attFadeIn;
   GA_RWHandleF attBlend;
   GA_RWHandleI    isTangeantTracker;

   bool computeDisplacement = false;
   GA_GroupType pointGroupType = GA_GROUP_POINT;
   GA_GroupType primGroupType = GA_GROUP_PRIMITIVE;
   const GA_GroupTable *pointGroupTable;
   const GA_GroupTable *primGroupTable;

   map<string,GU_RayIntersect*> rays;
   map<string,GU_Detail*> patchesGeo;
   map<GA_Offset,GA_Offset> pointsList;
   //map<string,map<GA_Offset,GA_Offset> > initialOffsetList;

   vector< vector<bool> > pixelUsed;
   map<int,float> patchBlend;
   map<int,UT_Vector3> trackerPosition;
   map<int,UT_Vector3> trackerUVPosition;


   bool useDeformableGrids = false;
   bool renderColoredPatches = false;
   bool debug = false;

   bool debugPatch = false;
   int patchNumber = 15;

   //GU_RayIntersect ray;

};
}

#endif
