#include "ImageVolumetric.h"
#include "cstring"

#define VERBOSE false



ImageVolumetric::~ImageVolumetric()
{
    //this->header = NULL;
    this->colors.clear();
}


//========================================================================================
//                                  OPEN IMAGE
//========================================================================================

    bool ImageVolumetric::OpenImage(const string& file_name, long id)
    {
        FILE * fin = fopen(file_name.c_str(), "rb");
        if (fin == 0x0)
        {
#if VERBOSE
            cout << "file is not open"<<endl;
#endif
            return false;
        }
        char buf[4096];
        fread(buf, 4096, 1, fin);
#if VERBOSE
        cout << "file opened"<<endl;
#endif

        header = (VolumeHeader&)buf;
#if VERBOSE
        cout <<"texName" << header.texName<<endl;
        cout <<"version" << header.version<<endl;
        cout <<"bytesPerChannel" << header.bytesPerChannel<<endl;
        cout <<"magic" << header.magic<<endl;
        cout <<"wrap" << header.wrap<<endl;
        cout <<"volSize" << header.volSize<<endl;
        cout << "number of chanel "<<header.numChannels<<endl;
#endif
        bool works = true;

        /*if ((header.magic[0] != 'V') || (header.magic[1] != 'O') || (header.magic[2] != 'L') || (header.magic[3] != 'U'))
        {
            cout << "bad header: invalid magic"<<endl;
            works = false;
        }
        */
        if (header.version != 4)
        {
            //cout << "bad header: version != 4"<<endl;
            works = false;
        }

        if (header.bytesPerChannel != 1)
        {
            //cout << "bad header: only byte textures supported"<<endl;
            works = false;
        }
        if (!works)
        {
            fclose(fin);
            return false;
        }
        int volBytes = header.volSize*header.volSize*header.volSize*header.numChannels*header.bytesPerChannel;

        unsigned char * data;
        data = new unsigned char[volBytes];
        fread(data, volBytes, 1, fin);
        fclose(fin);
        //m_width = image.size()[1];

        this->m_width = header.volSize;
        this->m_height = header.volSize;
        this->m_depth = header.volSize;

        /*
        this->numOfChannels = header->numChannels;
        this->bytesPerChannel = header->bytesPerChannel;
        this->version = header->version;
        this->magic = header->magic;
        this->wrap = header->wrap;
        this->texName = header->texName;
        */

        this->filename = file_name;


        long size = header.volSize*header.volSize*header.volSize;
        int iteratorSize =header.numChannels*header.bytesPerChannel;


        //vector<Vec3d>colors;

        for(long i=0; i<size;i++)
        {


            float r = (float)data[i*iteratorSize];
            float g = (float)data[i*iteratorSize+header.bytesPerChannel];
            float b = (float)data[i*iteratorSize+header.bytesPerChannel*2];

            Vec3d color(r,g,b);
            colors.push_back(color);

        }

#if VERBOSE
        cout << "there is "<<colors.size() << " voxels"<<endl;
        cout <<" the size of one axe is "<<header.volSize<<endl;

        cout << " Ok"<<endl;
#endif
        return true;
    }



    //========================================================================================
    //                                  CREATE IMAGE
    //========================================================================================
    void ImageVolumetric::CreateImage(int Width, int Height, int Depth, long id)
    {
        header.bytesPerChannel = 1;
        header.version = 4;
        header.numChannels = 3;
        strcpy(header.magic,"VOL");
        header.wrap = 1;
        header.volSize = Width;

        this->m_depth = Depth;
        this->m_height = Height;
        this->m_width = Width;

        //colors.reserve(Width*Height*Depth);
        colors.resize(Width*Height*Depth,Vec3d(0,0,0));
    }

    //========================================================================================
    //                                  SAVE IMAGE AS
    //========================================================================================


    void ImageVolumetric::SaveImageAs(const std::string& file_name)
    {
        cout <<"save volumetric image as "<<file_name<<endl;

        //open file
        FILE * fin = fopen(file_name.c_str(), "w");

        //save header
        //char buf[4096];
        //memcpy(buf,header,sizeof(header));

        int buffersize = 4096;
        //int buffersize = sizeof(header);
        fwrite(&header, buffersize, 1, fin);


        int iteratorSize =header.numChannels*header.bytesPerChannel;
        int volBytes = header.volSize*header.volSize*header.volSize*header.numChannels*header.bytesPerChannel;
        Vec3d color;

        unsigned char * data;
        data = new unsigned char[volBytes];

        vector<Vec3d>::iterator it;
        int i = 0;
        //long size = header.volSize*header.volSize*header.volSize;
        //for(long i=0; i<size;i++)
        bool oneColor = false;

        for(it = this->colors.begin(); it != this->colors.end(); ++it)
        {
            color = this->colors[i];
            if (color.x != 0)
                oneColor = true;
                //    cout << "color "<<color<<endl;
            data[i*iteratorSize]                            = (float)color.x;
            data[i*iteratorSize+header.bytesPerChannel]     = (float)color.y;
            data[i*iteratorSize+header.bytesPerChannel*2]   = (float)color.z;
            i++;
        }

        if (oneColor)
            cout << "save volume with at least one color"<<endl;

        //save data
        //int volBytes = header.volSize*header.volSize*header.volSize*header.numChannels*header.bytesPerChannel;
        fwrite(data,volBytes,1,fin);

        //close file
        fclose(fin);

    }


    //========================================================================================
    //                                  GET COLOR
    //========================================================================================
    void ImageVolumetric::GetColor(int x, int y,int z, Pixel& color)
    {

        if (this->colors.size() == 0)
        {
            //cout << "no image "<<this->filename<<endl;
            return;
        }

        AdjustCoordinates(x,y,z);

        if (x < 0 || y < 0 || x >= this->header.volSize || y >=this->header.volSize)
        {
            cout << "This is wrong !!!"<<endl;
        }

        long index = x + (y*this->header.volSize)+(z*(this->header.volSize)*(this->header.volSize));

        if (index > colors.size())
            return;


        Vec3d colorData = colors[index];

        int nbColor = 255;

        colorData.x /= nbColor;
        colorData.y /= nbColor;
        colorData.z /= nbColor;

        if (colorData.x < 0)
            colorData.x = 0;
        if (colorData.x > 1)
            colorData.x = 1;
        if (colorData.y < 0)
            colorData.y = 0;
        if (colorData.y > 1)
            colorData.y = 1;
        if (colorData.z < 0)
            colorData.z = 0;
        if (colorData.z > 1)
            colorData.z = 1;


        color.R = colorData.x;
        color.G = colorData.y;
        color.B = colorData.z;
        color.A = 1;
        //cout << "Get color " <<x<<"x"<<y<<" "<<color->R<<" "<<color->G<<" "<<color->B<<" "<<color->A<<" "<<endl;
    }


    //========================================================================================
    //                                  SET COLOR
    //========================================================================================
    void ImageVolumetric::SetColor(int x, int y,int z, Pixel& color)
    {

        if (this->colors.size() == 0)
        {
            //cout << "no image "<<this->filename<<endl;
            return;
        }

        AdjustCoordinates(x,y,z);

        if (x < 0 || y < 0 || x >= this->header.volSize || y >=this->header.volSize)
        {
            cout << "This is wrong !!!"<<endl;
        }

        long index = x + (y*this->header.volSize)+(z*(this->header.volSize)*(this->header.volSize));

        if (index > colors.size())
            return;

        Vec3d c;
        c.x = color.R;
        c.y = color.G;
        c.z = color.B;

        int nbColor = 255;

        c.x *= nbColor;
        c.y *= nbColor;
        c.z *= nbColor;

        this->colors[index] = c;
        //cout << "Get color " <<x<<"x"<<y<<" "<<color->R<<" "<<color->G<<" "<<color->B<<" "<<color->A<<" "<<endl;
    }

    //========================================================================================
    //                                  ADJUST COORDINATES
    //========================================================================================

    void ImageVolumetric::AdjustCoordinates(int &x, int &y, int &z)
    {
        while (x < 0)
            x = this->m_width + x;
        //if (x < 0)
        //    x = 0;

        while (y < 0)
            y = this->m_height + y;
        while (z < 0)
            z = this->m_depth + z;
        //if (y < 0)
        //    y = 0;

        while (x >= this->m_width )
            x -= this->m_width;
        //if (x >= this->m_width)
        //    x = this->m_width-1;

        while (y >= this->m_height )
            y -= this->m_height;
        while (z >= this->m_depth )
            z -= this->m_depth;
        //if (y >= this->m_height )
        //    y = this->m_height-1;
    }




