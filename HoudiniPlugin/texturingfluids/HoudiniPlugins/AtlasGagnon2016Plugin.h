#ifndef __AtlasGagnon2016Plugin_h__
#define __AtlasGagnon2016Plugin_h__
#include <vector>
#include <SOP/SOP_Node.h>
#include <GEO/GEO_PrimPart.h>
//#include <GA_ElementGroup.h>
#include <Math/Vec3.h>
#include "Images/Image.h"
#include "Set/SpatialGrid.h"
#include "Approaches/AtlasGagnon2016Synthesis.h"




#define STR_PARM(name, vi, t) \
                { evalString(str, name, vi, t); }

using namespace std;

namespace TexturingFluids {


class AtlasGagnon2016Plugin : public SOP_Node
{
public:
    AtlasGagnon2016Plugin(OP_Network *net, const char *name, OP_Operator *op);
    virtual ~AtlasGagnon2016Plugin();

	/// This method is created so that it can be called by handles.  It only
	/// cooks the input group of this SOP.  The geometry in this group is
	/// the only geometry manipulated by this SOP.
	virtual OP_ERROR		 cookInputGroups(OP_Context &context, 
						int alone = 0);
    static OP_Node		*myConstructor(OP_Network*, const char *,
                                OP_Operator *);
    static PRM_Template		 myTemplateList[];
    void  DeformableGridsFilename(UT_String &str, fpreal t)
    { STR_PARM("DeformableGridsFilename",  0, t) }
    void  TrackersFilename(UT_String &str, fpreal t)
    { STR_PARM("TrackersFilename",  0, t) }
    void  TextureExemplar1(UT_String &str, fpreal t)
    { STR_PARM("TextureExemplar1",  0, t) }
    void  TextureExemplarMask1(UT_String &str, fpreal t)
    { STR_PARM("TextureExemplarMask1",  0, t) }
    void  DisplacementMap1(UT_String &str, fpreal t)
    { STR_PARM("DisplacementMap1",  0, t) }
    void  OutputName(UT_String &str, fpreal t)
    { STR_PARM("OutputName",  0, t) }

    fpreal  TextureAtlasHeight() { return evalFloat("TextureAtlasHeight", 0, 0); }
    fpreal  TextureAtlasWidth() { return evalFloat("TextureAtlasWidth", 0, 0); }
    fpreal  UseDeformableGrids() { return evalFloat("UseDeformableGrids", 0, 0); }
    fpreal  RenderColoredPatches() { return evalFloat("RenderColoredPatches", 0, 0); }
    fpreal  PoissonDiskRadius() { return evalFloat("PoissonDiskRadius", 0, 0); }
    fpreal  UVScaling(fpreal t) { return evalFloat("UVScaling", 0, t); }
    fpreal  PatchScaling(fpreal t) { return evalFloat("PatchScaling", 0, t); }
    fpreal  TestPatch() { return evalFloat("TestPatch", 0, 0); }
    fpreal  PatchNumber() { return evalFloat("PatchNumber", 0, 0); }

protected:
    //virtual unsigned		 disableParms();
	virtual const char          *inputLabel(unsigned idx) const;


	/// Method to cook geometry for the SOP
	virtual OP_ERROR		 cookMySop(OP_Context &context);


private:
	

	void	getGroups(UT_String &str){ evalString(str, "group", 0, 0); }

	/// This variable is used together with the call to the "checkInputChanged"
	/// routine to notify the handles (if any) if the input has changed.
	GU_DetailGroupPair	 myDetailGroupPair;
	//const GA_PointGroup	*myGroup;
	const GA_EdgeGroup	*myGroup;
	const GA_PrimitiveGroup *primGroup;

    AtlasGagnon2016Synthesis interface;

    UT_String trackersFilename;
    UT_String deformableGridsFilename;
    UT_String textureExemplar1Name;
    UT_String textureExemplar1MaskName;
    UT_String displacementMap1Name;
    UT_String outputName;

	//Util util;
	
};
} // End HDK_Sample namespace

#endif
