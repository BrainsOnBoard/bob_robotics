#include "image/camparams.h"
#include <iostream>

int
main(int argc, char **argv)
{
    Image::CamParams params;
    params.m_FilePath = "yaml_write_test.yaml";
    params.m_SizeSource.width = 1280;
    params.m_SizeSource.height = 400;
    params.m_Center.x = 582;
    params.m_Center.y = 82;
    params.m_RadiusInner = 35;
    params.m_RadiusOuter = 76;
    params.m_Flipped = false;
    params.m_DegreeOffset = 0;
    params.write();
}
