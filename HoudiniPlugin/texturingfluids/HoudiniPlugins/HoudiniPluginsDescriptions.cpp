
#include <OP/OP_Director.h>


#include "DynamicLappedTexturePlugin.h"
#include "AtlasGagnon2016Plugin.h"


// -----------------------------------------------------------------------------
// Add our plugins to Houdini's plugins list
// -----------------------------------------------------------------------------
void newSopOperator(OP_OperatorTable *table)
{




     table->addOperator(new OP_Operator("hdk_Gagnon2016",
                                        "DynamicLappedTexture",
                                        DynamicLappedTexturePlugin::myConstructor,
                                        DynamicLappedTexturePlugin::myTemplateList,
                                        3,
                                        3,
                                        0));



     table->addOperator(new OP_Operator("hdk_AtlasGagnon2016",
                                       "AtlasGagnon2016",
                                       AtlasGagnon2016Plugin::myConstructor,
                                       AtlasGagnon2016Plugin::myTemplateList,
                                       2,
                                       2,
                                       0));


}
