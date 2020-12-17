#include "BlendingGagnon2016.h"
#include "../HoudiniUtils.h"



/* how it should be called:
 *
                Pixel Cf = BlendingGagnon2016::Blend(trackersGdp,deformableGrids,i,j,w,h,
                                          pixelPositionX,pixelPositionY,
                                          sortedPatches,
                                          surfaceUv,
                                          surfacePosition,
                                          useDeformableGrids,
                                          rays,
                                          patchColors,
                                          alphaColor,
                                          RM,
                                          attAlpha,
                                          attPointUV,
                                          attLife,
                                          patchBlend,
                                          patchUvs,
                                          alphasMap,
                                          textureExemplar1Image,
                                          textureExemplar1ImageMask,
                                          displacementMapImage,
                                          computeDisplacement,
                                          renderColoredPatches,
                                          displacementSum);

 */

//================================= RASTERIZE PRIMITIVE =================================

Pixel BlendingGagnon2016::Blend(GU_Detail* trackersGdp,GU_Detail* deformableGrids, int i, int j, float w, float h,
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
                                ParametersDeformablePatches params)
{

    bool debug = false;

    //if (life == 880)
    //    debug = true;

    float thresholdDistance = 0.5;
    Pixel Cf = Pixel(0,0,0);
    Cf.A = 1;

    Pixel colorSum1 = Pixel(0,0,0);
    colorSum1.A = 1;

    Pixel colorSum2 = Pixel(0,0,0);
    colorSum2.A = 1;

    Pixel color = Pixel(0,0,0);
    color.A = 1;

    Pixel displacement = Pixel(0,0,0);
    //Pixel displacementSum = Pixel(0,0,0);

    UT_Vector3 point;
    point.x() = i;
    point.y() = j;
    point.z() = 0;

    //test color
    color.R = 1;
    color.G = 1;
    color.B = 1;

    UT_Vector3 pixelPositionOnSurface;
    pixelPositionOnSurface.x() = ((float)pixelPositionX/(w-1));
    pixelPositionOnSurface.y() = ((float)pixelPositionY/(h-1));
    pixelPositionOnSurface.z() = 0;

    UT_Vector3 positionOnSurface = HoudiniUtils::GetBarycentricPosition(surfaceUv[0],surfaceUv[1],surfaceUv[2],surfacePosition[0],surfacePosition[1],surfacePosition[2],pixelPositionOnSurface);

    if (debug)
    {
        cout << "============================"<<endl;
        cout << "There are "<<sortedPatches.size() << " elements in sortedPatches"<<endl;
        //debug = false;
    }

    // compute color according to the list of patch
    vector<int>::iterator itPatch;
    int k = 0;
    //for(itPatch = sortedPatches.begin(); itPatch != sortedPatches.end(); itPatch++)
    for(itPatch = --sortedPatches.end(); itPatch != --sortedPatches.begin(); itPatch--)
    {
        //int patchId = patches[k];
        int patchId = *itPatch;

        //if (params.testPatch == 1 && params.patchNumber != patchId)
        //    continue;

        //cout << "blending patch "<<patchId<<endl;


        //------------------------------PARAMETRIC COORDINATE -----------------------------------


        UT_Vector3 positionInPolygon;
        UT_Vector3 centerUV = trackersUVPosition[patchId];//UT_Vector3(0.5,0.5,0.0);

        UT_Vector3 uvPatch1 = patchUvs[patchId][0];
        UT_Vector3 uvPatch2 = patchUvs[patchId][1];
        UT_Vector3 uvPatch3 = patchUvs[patchId][2];

        uvPatch1 -= centerUV;
        uvPatch2 -= centerUV;
        uvPatch3 -= centerUV;

        uvPatch1 *= params.UVScaling;
        uvPatch2 *= params.UVScaling;
        uvPatch3 *= params.UVScaling;

        uvPatch1 += centerUV;
        uvPatch2 += centerUV;
        uvPatch3 += centerUV;

        positionInPolygon = HoudiniUtils::GetBarycentricPosition(surfaceUv[0],surfaceUv[1],surfaceUv[2],uvPatch1,uvPatch2,uvPatch3,pixelPositionOnSurface);



        int w = textureExemplar1Image->GetWidth();
        int h = textureExemplar1Image->GetHeight();
        int wm = textureExemplar1ImageMask->GetWidth();
        int hm = textureExemplar1ImageMask->GetHeight();

        int i2 = static_cast<int>(floor(positionInPolygon.x()*w));
        int j2 = (h-1)-static_cast<int>(floor((positionInPolygon.y())*h));

        int i3 = static_cast<int>(floor(positionInPolygon.x()*wm));
        int j3 = (hm-1)-static_cast<int>(floor((positionInPolygon.y())*hm));

        //textureExemplar1Image->GetColor(pixelPositionX,pixelPositionY,0,color);
        if (renderColoredPatches)
        {
            //set random colors per patch
            color.R = 1;
            color.G = 1;
            color.B = 1;
            color = patchColors[patchId];
        }
        else
        {
            textureExemplar1Image->GetColor(i2,j2,0,color);
        }
        textureExemplar1ImageMask->GetColor(i3,j3,0,alphaColor);
        if (computeDisplacement)
            displacementMapImage->GetColor(i2,j2,0,displacement);

        float alpha = ((alphaColor.R+alphaColor.G+alphaColor.B)/3);

        if (debug)
            cout << "Adding color "<<color.R<<" "<<color.G<<" "<<color.B<<" "<<alpha<<endl;


        centerUV.z() = 0;
        positionInPolygon.z() = 0;
        float d_P = distance3d(positionInPolygon,centerUV);
        //cout <<centerUV<<" "<<positionInPolygon<< " "<< d_P<<endl;

        //HAACCKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKK
        alpha = 1;

        if (d_P > 0.125*params.PatchScaling)
        {
            if (debug)
                cout << "d_P "<<d_P << " > "<<0.125*params.PatchScaling<<endl;
            alpha = 0;
        }
        color.A = alpha;

        //---------------- Transparency Equation -----------------------

        Cf.R =  (alpha)*(color.R) + (1.0f-alpha)*(Cf.R);
        Cf.G =  (alpha)*(color.G) + (1.0f-alpha)*(Cf.G);
        Cf.B =  (alpha)*(color.B) + (1.0f-alpha)*(Cf.B);

        if (debug)
            cout << "patch "<<patchId<<" alpha "<<alpha<<endl;

        if (computeDisplacement)
        {
            displacementSum.R = alpha*displacement.R + (1.0f-alpha)*displacementSum.R;
            displacementSum.G = alpha*displacement.G + (1.0f-alpha)*displacementSum.G;
            displacementSum.B = alpha*displacement.B + (1.0f-alpha)*displacementSum.B;
        }
        k++;
    }

    if (debug)
    {
        cout << "Cd = "<<Cf.R<<" "<<Cf.G<<" "<<Cf.B<<endl;
    }

    if (Cf.R > 1)
        Cf.R = 1;
    if (Cf.G > 1)
        Cf.G = 1;
    if (Cf.B > 1)
        Cf.B = 1;

    if (Cf.R < 0)
        Cf.R = 0;
    if (Cf.G < 0)
        Cf.G = 0;
    if (Cf.B < 0)
        Cf.B = 0;

    return Cf;
}

