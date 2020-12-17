#include "AtlasGagnon2016.h"
#include <GU/GU_PrimPoly.h>
//#include "TBBAtlas.h"
#include "BlendingGagnon2016.h"
#include "../HoudiniUtils.h"


AtlasGagnon2016::~AtlasGagnon2016()
{
    cout <<"Destorying AtlasGagnon2016";
    if (this->diffuseImageBlendingGagnon && this->diffuseImageBlendingGagnon->IsValid())
        delete this->diffuseImageBlendingGagnon;

    if (this->diffuseImageBlendingYu2011Equation3 && this->diffuseImageBlendingYu2011Equation3->IsValid())
        delete this->diffuseImageBlendingYu2011Equation3;

    if (this->diffuseImageBlendingYu2011Equation4 && this->diffuseImageBlendingYu2011Equation4->IsValid())
        delete this->diffuseImageBlendingYu2011Equation4;
    if (this->textureExemplar1Image && this->textureExemplar1Image->IsValid())
        delete this->textureExemplar1Image;
    if (this->textureExemplar1ImageMask && this->textureExemplar1ImageMask->IsValid())
        delete this->textureExemplar1ImageMask;
    if (computeDisplacement)
    {
        delete this->displacementMapImage;
        delete this->displacementMap;
    }

    GA_PrimitiveGroup *primGroup;
    GA_FOR_ALL_PRIMGROUPS(surface,primGroup)
    {
         string name = primGroup->getName().toStdString();
         delete rays[name];
    }
    //patchesGeo.clear();

    trackerPosition.clear();
    cout <<" Done"<<endl;
    this->rays.clear();
    this->pixelUsed.clear();
    this->patchColors.clear();

}


bool AtlasGagnon2016::BuildAtlas(int w, int h, int life)
{
    if (surface == 0x0 || (deformableGrids == 0x0 && useDeformableGrids) || trackers == 0x0)
        return false;

    cout << "[HoudiniAtlas::BuildAtlas]("<<w<<","<< h <<")"<<endl;

    cout << "[HoudiniAtlas::BuildAtlas] setting varialbes"<<endl;
    //-------------------------------------------------------
    UT_String patchname("patchIds");
    patchIds = surface->findIntArray(GA_ATTRIB_POINT,patchname,-1, -1);
    if (!patchIds)
    {
        cout << "[HoudiniAtlas::BuildAtlas] There is no patch id attribute"<<endl;
        return false;
    }
    patchArray = patchIds->getAIFNumericArray();
    //-------------------------------------------------------

    //-------------------------------------------------------
    UT_String aname("alphas");
    alphas = surface->findFloatArray(GA_ATTRIB_POINT,aname,-1, -1);
    if (!alphas)
    {
        cout << "[HoudiniAtlas::BuildAtlas] There is no alpha array attribute"<<endl;
        return false;
    }
    alphaArray = alphas->getAIFNumericArray();

    //-------------------------------------------------------
    UT_String uvname("uvs");
    uvsAtt = surface->findFloatArray(GA_ATTRIB_POINT, uvname,-1, -1);
    if (!uvsAtt)
    {
        cout << "[HoudiniAtlas::BuildAtlas] There is no uv array attribute"<<endl;
        return false;
    }
    uvsArray = uvsAtt->getAIFNumericArray();
    //--------------------------------------------------------------------------

    //---------------------------------------------------------------

    surfaceTree.build(surface, NULL);

    //attLife = GA_ROHandleI(trackers->findIntTuple(GA_ATTRIB_POINT,"life", 1));
    attLife = life;
    attFadeIn = GA_ROHandleI(trackers->findIntTuple(GA_ATTRIB_POINT,"fadeIn", 1));
    isTangeantTracker = GA_RWHandleI(trackers->findIntTuple(GA_ATTRIB_POINT,"isTrangeantTracker",1));
    attBlend = GA_RWHandleF(trackers->findFloatTuple(GA_ATTRIB_POINT,"blend", 1));
    attPointUV = GA_RWHandleV3(surface->findFloatTuple(GA_ATTRIB_POINT,"uvw", 3));
    attAlpha = GA_ROHandleF(surface->findFloatTuple(GA_ATTRIB_POINT,"Alpha", 1));

    GA_RWHandleV3   attCenterUV(trackers->addFloatTuple(GA_ATTRIB_POINT,"centerUV", 3));

    pointGroupTable = surface->getGroupTable(pointGroupType);
    primGroupTable = surface->getGroupTable(primGroupType);

    attUV = GA_RWHandleV3(surface->findFloatTuple(GA_ATTRIB_VERTEX,"uv", 3));
    if (attUV.isInvalid())
    {
        cout << "[HoudiniAtlas::BuildAtlas] There is no uv on the surface"<<endl;
        return false;
    }

    if (textureExemplar1Name.size() == 0)
    {
        cout << "[HoudiniAtlas::BuildAtlas] There is no texture exemplar name assigned"<<endl;
        return false;
    }
    if (textureExemplar1MaskName.size() == 0)
    {
        cout << "[HoudiniAtlas::BuildAtlas] There is no texture exemplar mask name assigned"<<endl;
        return false;
    }

    diffuseImageBlendingGagnon = new ImageCV();
    diffuseImageBlendingGagnon->CreateImage(w,h,-1);

    diffuseImageBlendingYu2011Equation3 = new ImageCV();
    diffuseImageBlendingYu2011Equation3->CreateImage(w,h,-1);

    diffuseImageBlendingYu2011Equation4 = new ImageCV();
    diffuseImageBlendingYu2011Equation4->CreateImage(w,h,-1);


    textureExemplar1Image = new ImageCV();
    cout << "[HoudiniAtlas::BuildAtlas] Opening "<<textureExemplar1Name<<endl;
    bool opened = textureExemplar1Image->OpenImage(textureExemplar1Name,-1);
    if (!opened)
    {
        cout << "[HoudiniAtlas::BuildAtlas] Can't open "<< textureExemplar1Name<<endl;
        return false;
    }

    RM = textureExemplar1Image->MeanValue();
    cout << "RM = ";
    RM.Print();
    cout<<endl;

    textureExemplar1ImageMask = new ImageCV();
    cout << "[HoudiniAtlas::BuildAtlas] Opening "<<textureExemplar1MaskName<<endl;
    opened = textureExemplar1ImageMask->OpenImage(textureExemplar1MaskName,-1);
    if (!opened)
    {
        cout << "[HoudiniAtlas::BuildAtlas] Can't open "<< textureExemplar1MaskName<<endl;
        return false;
    }

    if (displacementMapImageName.size() != 0)
    {
        displacementMapImage = new ImageCV();
        cout << "[HoudiniAtlas::BuildAtlas] Opening "<<displacementMapImageName<<endl;
        computeDisplacement = displacementMapImage->OpenImage(displacementMapImageName,-1);
        cout << "[HoudiniAtlas::BuildAtlas] Done"<<endl;
        if (computeDisplacement)
        {
            displacementMap = new ImageCV();
            displacementMap->CreateImage(w,h,-1);
        }
        else
        {
            cout << "[HoudiniAtlas::BuildAtlas] Can't open displacement map file "<<displacementMapImageName<<endl;
        }
    }
    else
    {
        cout << "[HoudiniAtlas::BuildAtlas] Displacement map name is not defined"<<endl;
    }

    GA_PrimitiveGroup *primGroup;
    GA_FOR_ALL_PRIMGROUPS(surface,primGroup)
    {
        //GA_PrimitiveGroup *primGroup = (GA_PrimitiveGroup*)gPrimTable->find(groupName.c_str());
        string name = primGroup->getName().toStdString();
        GU_RayIntersect *ray = new GU_RayIntersect(surface,primGroup);
        ray->init();
        rays[name] = ray;
    }

    cout << "[AtlasGagnon2016::BuildAtlas] There is " << trackers->getNumPoints() << " trackers" << endl;


    GA_RWHandleI    attId(trackers->findIntTuple(GA_ATTRIB_POINT,"id",1));
    GA_Offset ppt;
    GA_FOR_ALL_PTOFF(trackers,ppt)
    {
        float blend = attBlend.get(ppt);
        int patchId =   attId.get(ppt);
        int isTangeant = isTangeantTracker.get(ppt);
        if (isTangeant == 1)
            continue;

        if (isinf(blend))
            blend = 1.0f;
        //temporalComponetKt[patchId] = blend;
        //cout << "Buidling tracker uv position "<<attCenterUV.get(ppt)<<endl;
        trackerUVPosition[patchId] = attCenterUV.get(ppt);
    }


    if(renderColoredPatches)
        initPatchColors(trackers);

    for(int i =0; i < w; i++)
    {
        vector<bool> line;
        for(int j =0; j < h; j++)
        {
            line.push_back(false);
            //this->colors[i][j] = false;
        }
        this->pixelUsed.push_back(line);
    }
    cout << "[AtlasGagnon2016::BuildAtlas] Done"<<endl;
    return true;
}

//================================= RASTERIZE PRIMITIVE =================================

void AtlasGagnon2016::RasterizePrimitive(GA_Offset primOffset, int w, int h, ParametersDeformablePatches params)
{
    //rasterize primitive

    bool debug = false;
    //if (primOffset == 880)
    //    debug = true;

    GA_Primitive *prim = surface->getPrimitive(primOffset);
    if(prim == 0x0)
        return;

    //get triangle vertex position in UV space
    GA_Size vertexCount = prim->getVertexCount();
    if (vertexCount != 3)
    {
        cout << "Primitive "<<prim->getMapOffset()<< " has "<<vertexCount<< " vertices"<<endl;
        return;
    }
    if (debug)
        cout << "Rasterizing primitive "<<primOffset<<endl;

    //--------------------- SORTED DATA PER PATCH ------------------------
    vector<UT_Vector3> surfaceTexturePosition;
    vector<UT_Vector3> surfaceUv;
    vector<UT_Vector3> surfacePosition;
    vector<int> sortedPatches;

    map<int,int> numberOfLinkedPatch;
    map<int, vector<UT_Vector3> > patchUvs;
    map<int, vector<float> > alphasMap;


    //-------------initializinb maps------------------
    for(int i = 0; i < vertexCount; i++)
    {
        GA_Offset vertexPoint = prim->getVertexOffset(i);
        GA_Offset pointOffset = surface->vertexPoint(vertexPoint);

        UT_IntArray         patchesData;
        patchArray->get(patchIds, pointOffset, patchesData);


        vector<UT_Vector3> uvTemp;
        uvTemp.resize(3);
        vector<float> alphaTemp;
        //alphaTemp.resize(3);
        alphaTemp.push_back(1.0f);
        alphaTemp.push_back(1.0f);
        alphaTemp.push_back(1.0f);
        int nb = patchesData.size();
        for (int k = 0; k< nb; k++)
        {
            int index = patchesData.array()[k];
            numberOfLinkedPatch[index] = 0;
            patchUvs[index]            = uvTemp;
            alphasMap[index]           = alphaTemp;
        }
    }

    //------------filling maps---------------
    for(int vertexIt = 0; vertexIt < vertexCount; vertexIt++)
    {
        GA_Offset vertexPoint = prim->getVertexOffset(vertexIt);
        GA_Offset pointOffset = surface->vertexPoint(vertexPoint);

        UT_Vector3 uv = attUV.get(vertexPoint);
        surfaceUv.push_back(uv);
        uv.x() *= w;
        uv.y() *= h;
        surfaceTexturePosition.push_back(uv);

        surfacePosition.push_back(surface->getPos3(pointOffset));

        //put this in another function

        UT_IntArray         patchesData;
        patchArray->get(patchIds, pointOffset, patchesData);

        UT_FloatArray         uvsData;
        uvsArray->get(uvsAtt, pointOffset, uvsData);

        UT_FloatArray         alphasData;
        alphaArray->get(alphas, pointOffset, alphasData);


        if (debug)
        {
            cout << "adding point "<<pointOffset<<endl;
        }

        //for this vertex, we go through all patches
        //we are trying to keep only patches that are on the three vertices
        //vector<UT_Vector3> sortedUVs;
        int nb = patchesData.size();
        for (int patchIndex = 0; patchIndex< nb; patchIndex++)
        {

            int patchId = patchesData.array()[patchIndex];
            int data = numberOfLinkedPatch[patchId];
            data++;
            numberOfLinkedPatch[patchId] = data;
            if (data == 3)
            {
                //cout << "w00t !"<<endl;
                if (params.testPatch == 1 && params.patchNumber == patchId)
                {
                    if (debug)
                        cout << "using patch number only"<<endl;
                    sortedPatches.push_back(patchId);
                }
                else if (params.testPatch == 0)
                {
                    //cout << "not using patch number test"<<endl;
                    if (debug)
                    {
                        cout << "adding patch "<<patchId<<endl;
                    }
                    sortedPatches.push_back(patchId);
                }
            }

            //debug = false;
            int uvIndex = patchIndex*3;
            UT_Vector3 uvPatch = UT_Vector3(uvsData.array()[uvIndex],uvsData.array()[uvIndex+1],uvsData.array()[uvIndex+2]);
            patchUvs[patchId][vertexIt] = uvPatch;
            //alphasMap[patchId][vertexIt] = alphasData.array()[patchIndex];
        }

        //sortedPatches.push_back(patchList);
    }
    if (debug)
    {
        cout << "sorted patches "<<endl;
        vector<int>::iterator itP;
        for(itP = sortedPatches.begin(); itP != sortedPatches.end(); itP++)
            cout << *itP<<endl;
        //cout << "patch uv "<<patchUvs<<endl;
        map<int,int>::iterator itLink;

        cout << "numberOfLinkedPatch "<<endl;
        for(itLink = numberOfLinkedPatch.begin(); itLink != numberOfLinkedPatch.end(); itLink++)
            cout << *itLink<<endl;
    }
    //-------------------------------------------------------------------

    UT_Vector3 min, max;
    BoundingBox2D(surfaceTexturePosition[0],surfaceTexturePosition[1],surfaceTexturePosition[2],min,max);

    int pixelCellSize = 5;
    Pixel color;
    Pixel displacement;
    Pixel alphaColor;
    UT_Vector3 point;

    debug = false;

    //-----------------------------------------------------------------
    for(int i =min.x()-pixelCellSize; i < max.x()+pixelCellSize; i++)
    {
        for(int j =min.y()-pixelCellSize; j < max.y()+pixelCellSize; j++)
        {
            if (i < 0 || j < 0)
                continue;

            Pixel Cf = Pixel(0,0,0);
            Cf.A = 1;

            Pixel R_eq3;

            Pixel colorSum0 = Pixel(0,0,0);
            colorSum0.A = 1;

            Pixel colorSum1 = Pixel(0,0,0);
            colorSum1.A = 1;

            Pixel colorSum2 = Pixel(0,0,0);
            colorSum2.A = 1;

            float sumW2 = 0;
            float sumW = 0;

            color = Pixel(0,0,0);
            color.A = 1;

            displacement = Pixel(0,0,0);
            Pixel displacementSum = Pixel(0,0,0);

            point.x() = i;
            point.y() = j;
            point.z() = 0;

            int pixelPositionX = i;
            int pixelPositionY = j;

            while (pixelPositionX >= w)
                pixelPositionX -= w;
            while (pixelPositionY >= h)
                pixelPositionY -= h;
            while (pixelPositionX < 0)
                pixelPositionX += w;
            while (pixelPositionY < 0)
                pixelPositionY += h;

            /*
            if((pixelPositionX == 500) && pixelPositionY == 500)
                debug = true;
            else
                debug = false;

            */
            //cout << "Is "<<point<<" inside "<<uvs[0]<< " "<<uvs[1]<< " "<<uvs[2]<<endl;
            //remove this line to be able to add pixel around the triangle to remove border artifact?
            if (IsPointInTriangle(point,surfaceTexturePosition[0],surfaceTexturePosition[1],surfaceTexturePosition[2]) || !this->pixelUsed[pixelPositionX][pixelPositionY]  )
            {
                //test color
                color.R = 1;
                color.G = 1;
                color.B = 1;

                /*
    static Pixel Blend(GU_Detail* trackersGdp,GU_Detail* deformableGrids, int i, int j, float w, float h,
                int pixelPositionX, int pixelPositionY,
                vector<int> &sortedPatches,
                vector<UT_Vector3> &surfaceUv,
                vector<UT_Vector3> &surfacePosition,
                map<int,UT_Vector3> &trackersPosition,
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
                Pixel &R1,
                Pixel &displacementSum,
                ParametersDeformablePatches params);
                 * */

                //======================== Test encapsulated function =====================
                Pixel Cf = BlendingGagnon2016::Blend(trackers,deformableGrids,i,j,w,h,
                                          pixelPositionX,pixelPositionY,
                                          sortedPatches,
                                          surfaceUv,
                                          surfacePosition,
                                          trackerPosition,
                                          trackerUVPosition,
                                          false,
                                          rays,
                                          patchColors,
                                          alphaColor,
                                          RM,
                                          attAlpha,
                                          attPointUV,
                                          primOffset,
                                          patchBlend,
                                          patchUvs,
                                          alphasMap,
                                          textureExemplar1Image,
                                          textureExemplar1ImageMask,
                                          displacementMapImage,
                                          computeDisplacement,
                                          renderColoredPatches,
                                          displacementSum,
                                          params);

                //cout << "Color : "<< Cf.R << " " << Cf.G << " " << Cf.B << " " << Cf.A << endl;
                diffuseImageBlendingGagnon->SetColor(pixelPositionX,h-pixelPositionY,0,Cf);

                if (computeDisplacement)
                    displacementMap->SetColor(pixelPositionX,h-pixelPositionY,0,displacementSum);


                if (IsPointInTriangle(point,surfaceTexturePosition[0],surfaceTexturePosition[1],surfaceTexturePosition[2]))
                    this->pixelUsed[pixelPositionX][pixelPositionY] = true;

            }
        }
    }//------------------------ FIN RASTERISATION ---------------------
}



void AtlasGagnon2016::SaveAtlas()
{
    //write the image to the disk
    diffuseImageBlendingGagnon->SaveImageAs(outputFilename);
    cout << "Save texture atlas"<<outputFilename<<endl;
    if (computeDisplacement)
    {
        //write the image to the disk
        displacementMap->SaveImageAs(outputFilename+"displacement.png"); //HARDCODED NAME !!!
        cout << "Save texture atlas"<<outputFilename+"displacement.png"<<endl;
    }
}

void AtlasGagnon2016::BoundingBox2D(UT_Vector3 a, UT_Vector3 b, UT_Vector3 c,UT_Vector3 &min,UT_Vector3 &max)
{
    min.x() = a.x();
    min.y() = a.y();
    //min.z = a.z;
    max = min;

    // check min

    if (min.x() > b.x())
    {
        min.x() = b.x();
    }
    if (min.y() > b.y())
    {
        min.y() = b.y();
    }

    if (min.x() > c.x())
    {
        min.x() = c.x();
    }
    if (min.y() > c.y())
    {
        min.y() = c.y();
    }

    //check max

    if (max.x() < b.x())
    {
        max.x() = b.x();
    }
    if (max.y() < b.y())
    {
        max.y() = b.y();
    }

    if (max.x() < c.x())
    {
        max.x() = c.x();
    }
    if (max.y() < c.y())
    {
        max.y() = c.y();
    }

}

bool AtlasGagnon2016::IsPointInTriangle(UT_Vector3  p, UT_Vector3 a,UT_Vector3 b,UT_Vector3 c)
{

    UT_Vector3 v0 = c - a;
    UT_Vector3 v1 = b - a;
    UT_Vector3 v2 = p - a;


    // Compute dot products
    float dot00 = dot(v0,v0);
    float dot01 = dot(v0,v1);
    float dot02 = dot(v0,v2);
    float dot11 = dot(v1,v1);
    float dot12 = dot(v1,v2);

    // Compute barycentric coordinates
    float invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
    float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

    // Check if point is in triangle
    return (u >= 0) && (v >= 0) && (u + v <= 1);
}


Pixel AtlasGagnon2016::SetRandomColor(int patchNumber)
{
    //initialize random seed
    srand(patchNumber);
    float r = ((double) rand()/(RAND_MAX));
    srand(patchNumber+1);
    float g = ((double) rand()/(RAND_MAX));
    srand(patchNumber+2);
    float b = ((double) rand()/(RAND_MAX));
    Pixel patchColor;
    patchColor.A = 1;
    patchColor.R = r;
    patchColor.G = g;
    patchColor.B = b;

    return patchColor;


}

void AtlasGagnon2016::initPatchColors(GU_Detail *trackersGdp)
{

    GA_RWHandleI    attM(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"isTrangeantTracker",0));
    GA_ROHandleI    attId(trackersGdp->findIntTuple(GA_ATTRIB_POINT,"id",1));
    GA_Offset ppt;
    GA_FOR_ALL_PTOFF(trackersGdp,ppt)
    {
        if (attM.get(ppt) == 1)
        {
            int patchId = attId.get(ppt);
            patchColors[patchId] = SetRandomColor(patchId);
        }
    }

}

