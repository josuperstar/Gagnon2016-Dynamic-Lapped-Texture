#ifndef __IMAGEVOLUMETRIC_h__
#define __IMAGEVOLUMETRIC_h__

#include "Color.h"
#include "vector"
#include "Image.h"


using namespace std;
//using namespace cv;
using namespace TexturingFluids;

struct VolumeHeader
{
    char magic[4];
    int version;
    char texName[256];
    bool wrap;
    int volSize;
    int numChannels;
    int bytesPerChannel;
};


class ImageVolumetric : public Image
{
	public:
    ~ImageVolumetric();
    bool OpenImage(const string& file_name, long id);
    void CreateImage(int Width, int Height, int Depth, long id);
    void GetColor(int x, int y, int z, Pixel& color);
    void SetColor(int x, int y, int z, Pixel& color);
    //void GetColor(int x, int y, Pixel& color);
    //int GetWidth() {return m_width;}
    //int GetHeight() {return m_height;}
    void SaveImageAs(const std::string& file_name);

    //string GetFilename(){return filename;}

private :
    void AdjustCoordinates(int &x, int &y, int &z);

protected :

    //int m_width;
    //int m_height;

    //string filename;
    vector<TexturingFluids::Vec3d> colors;
    //unsigned char * data;
    VolumeHeader  header;




		
};

#endif // __IMAGEVOLUMETRIC_h__
