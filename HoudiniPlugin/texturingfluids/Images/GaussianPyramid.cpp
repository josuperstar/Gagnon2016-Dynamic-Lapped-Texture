#include "GaussianPyramid.h"
#include <cassert>



GaussianPyramid::GaussianPyramid()
{
    this->pyramid.clear();

    /*
    Mat features(numData,numDimensions,CV_32F), query(numQueries,numDimensions,CV_32F);
    // KdTree with 5 random trees
    cv::flann::KDTreeIndexParams indexParams(5);

    // You can also use LinearIndex
    //cv::flann::LinearIndexParams indexParams;

    // Create the Index
    cv::flann::Index kdtree(features, indexParams);
    */
}

GaussianPyramid::~GaussianPyramid()
{
    Clear();

}

void GaussianPyramid::Clear()
{
    //cout << "Clearing Gaussian Pyramid"<<endl;
    this->pyramid.clear();
}

const ImageCV* GaussianPyramid::GetImageAtLevel(int level) const
{
    if(level < pyramid.size())
        return &pyramid[level];

    //cout << "Cannot find image at level "<<level<<endl;
    return NULL;
}

ImageCV* GaussianPyramid::GetImageAtLevel(int level)
{
    if(level < pyramid.size())
        return &pyramid[level];

    //cout << "Cannot find image at level "<<level<<endl;
    return NULL;
}

/**
 * @brief Update the image at the given level by up-sampling from the lower level
 * @param targetLevel Level to up-sample to (from targetLevel - 1)
 */
void GaussianPyramid::UpsampleToLevel(int targetLevel)
{
    assert(targetLevel > 0 && targetLevel < pyramid.size());

    cv::pyrUp(pyramid[targetLevel - 1].image, pyramid[targetLevel].image);
}

void GaussianPyramid::BuildGaussianPyramid(ImageCV source, int numberOfLevel,long &idCounter)
{
    //cout << "------------------------------"<<endl;
    cout << "Build image gaussian pyramid of "<<numberOfLevel<<" level(s)"<<endl;

    this->pyramid = source.GaussianPyramid(numberOfLevel);

    this->numberOfLevel = numberOfLevel;
    //cout << "------------------------------"<<endl;
}

void GaussianPyramid::BuildGaussianPyramidRandomColor(ImageCV source, int numberOfLevel, long &idCounter)
{

    ImageCV random = source;
    for(int y = 0; y < source.GetHeight(); y++)
    {
        for(int x = 0; x < source.GetWidth(); x++)
        {
            Pixel color;
            int randomx = rand() % source.GetWidth();
            int randomy = rand() % source.GetHeight();
            source.GetColor(randomx,randomy,0,color);
            random.SetColor(x,y,0,color);
        }
    }

    this->pyramid = random.GaussianPyramid(numberOfLevel);

    this->numberOfLevel = numberOfLevel;
}
