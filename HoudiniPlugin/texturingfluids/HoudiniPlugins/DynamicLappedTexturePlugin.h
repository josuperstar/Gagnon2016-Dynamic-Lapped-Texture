

#ifndef __DynamicLappedTexturePlugin__
#define __DynamicLappedTexturePlugin__
#include <vector>
#include <SOP/SOP_Node.h>
#include <GEO/GEO_PrimPart.h>
//#include <GA_ElementGroup.h>
#include <Math/Vec3.h>
#include "Images/Image.h"
#include "Set/SpatialGrid.h"
#include "Approaches/DynamicLappedTexture.h"




#define STR_PARM(name, vi, t) \
                { evalString(str, name, vi, t); }

using namespace std;

namespace TexturingFluids {


class DynamicLappedTexturePlugin : public SOP_Node
{
public:
    DynamicLappedTexturePlugin(OP_Network *net, const char *name, OP_Operator *op);
    virtual ~DynamicLappedTexturePlugin();

	/// This method is created so that it can be called by handles.  It only
	/// cooks the input group of this SOP.  The geometry in this group is
	/// the only geometry manipulated by this SOP.
	virtual OP_ERROR		 cookInputGroups(OP_Context &context, 
						int alone = 0);
    static OP_Node		*myConstructor(OP_Network*, const char *,
                                OP_Operator *);
    static PRM_Template		 myTemplateList[];
    fpreal  PoissonDiskRadius() { return evalFloat("PoissonDiskRadius", 0, 0); }
    fpreal  StartFrame() { return evalFloat("StartFrame", 0, 0); }
    fpreal  UpdateDistribution() { return evalFloat("UpdateDistribution", 0, 0); }
    fpreal  MinimumDistanceProjection() { return evalFloat("MinimumDistanceProjection", 0, 0); }
    fpreal  PatchAngleNormalThreshold() { return evalFloat("PatchAngleNormalThreshold", 0, 0); }
    fpreal  PoissonAngleNormalThreshold() { return evalFloat("PoissonAngleNormalThreshold", 0, 0); }
    fpreal  UVScaling() { return evalFloat("UVScaling", 0, 0); }
    fpreal  TestPatch() { return evalFloat("TestPatch", 0, 0); }
    fpreal  PatchNumber() { return evalFloat("PatchNumber", 0, 0); }
    void  TrackersFilename(UT_String &str, fpreal t)
    { STR_PARM("TrackersFilename",  0, t) }
    fpreal  CellSize() { return evalFloat("CellSize", 0, 0); }



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

    DynamicLappedTexture interface;

    UT_String trackersFilename;
    UT_String textureExemplar1Name;
    UT_String textureExemplar1MaskName;
    UT_String displacementMap1Name;


	//Util util;
	
};
} // End HDK_Sample namespace

#endif
