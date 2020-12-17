#ifndef __GaussianPyramid_h__
#define __GaussianPyramid_h__

#include "Color.h"
#include <opencv2/opencv.hpp>
#include <Images/ImageCV.h>

using namespace std;

class GaussianPyramid
{
	public:

        GaussianPyramid();
        ~GaussianPyramid();
        void BuildGaussianPyramid(ImageCV source, int numberOfLevel, long &idCounter);
        void BuildGaussianPyramidRandomColor(ImageCV source, int numberOfLevel, long &idCounter);
        //~GaussianPyramid();

        const ImageCV* GetImageAtLevel(int level) const;
        ImageCV* GetImageAtLevel(int level);
        void UpsampleToLevel(int targetLevel);
        int numberOfLevel;
        void Clear();


protected :

    vector<ImageCV> pyramid;

		
};

#endif // __GaussianPyramid_h__
