#ifndef _ALTAS_H_
#define _ALTAS_H_

#include "common.h"

using std::string;

class Atlas {
public:
    MapStatus atlas[MapSize][MapSize];
    int color[MapSize][MapSize];
    int ColorCount;

    Atlas() = default;
    ~Atlas() = default;
    
    void ColorAtlas();

    /************Below variables and functions are for debug***************/
    void AtlasInitByMapTxt(string path);
    void AtlasPrintColor(int top, int left, int bottom, int right);
    void AtlasPrintMap();
    
};

#endif