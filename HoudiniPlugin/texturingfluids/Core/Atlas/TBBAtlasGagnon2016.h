#ifndef __TBB_Atlas_Gagnon2016_h__
#define __TBB_Atlas_Gagnon2016_h__

#include "AtlasGagnon2016.h"

using namespace TexturingFluids;

struct Gagnon2016_executor
{
  Gagnon2016_executor(AtlasGagnon2016 &rasterizer, int w, int h,
           ParametersDeformablePatches params) : _rasterizer(rasterizer),
            _w(w), _h(h), _params(params)
  {
  }

  void operator()(const tbb::blocked_range<size_t>& r) const
  {

    for (size_t i=r.begin();i!=r.end();++i)
    {
      _rasterizer.RasterizePrimitive(GA_Offset(i), _w, _h, _params);
    }
  }

  AtlasGagnon2016& _rasterizer;
  int _w;
  int _h;
  ParametersDeformablePatches _params;

};

#endif
