#include <cmath>
#include <iostream>

#include "ConeXYZColorScore.h"
#include "CDT.h"
#include <Triangulation.h>
#include <CDTUtils.h>

double distance(ConeXYZColorScore a, ConeXYZColorScore b){
    return sqrt(pow(a.x-b.x, 2) + pow(a.y-b.y,2) + pow(a.z-b.z,2));
}