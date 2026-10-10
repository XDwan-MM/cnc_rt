#include "datatype.h"
#include "cnc_math.h"
#include <math.h>

double POS_Distance(POS* p1, POS* p2){
    return sqrt(pow((p1->x - p2->x), 2)+pow((p1->y - p2->y), 2)+pow((p1->z - p2->z), 2));
}

POS POS_VECTOR(POS* p1, POS* p2){
    POS p;
    p.x = p2->x - p1->x;
    p.y = p2->y - p1->y;
    p.z = p2->z - p1->z;
    return p;
}

double POS_Norm(POS p){
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

double POS_Dot(POS p1, POS p2){
    return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
}

void POS_Divide(POS* p1, double div){
    if(IsEqualTo(div, 0.0)) return;
    p1->x /= div;
    p1->y /= div;
    p1->z /= div;
}

VECTOR VECTOR_Unit(POS* p1,  POS* p2){
    VECTOR r = POS_VECTOR(p1, p2);
    // 计算单位矢量
    POS_Divide(&r, POS_Norm(r));
    return r;
}

void POS_COPY(POS* dest, POS* src){
    dest->x = src->x;
    dest->y = src->y;
    dest->z = src->z;
}

//向量叉乘求模
double CrossVector(POS* AB,POS* BC ){
    double cross_val = sqrt((AB->y*BC->z - AB->z*BC->y)*(AB->y*BC->z - AB->z*BC->y) + (AB->x*BC->z - AB->z*BC->x)*(AB->x*BC->z - AB->z*BC->x) + (AB->x*BC->y - AB->y*BC->x)*(AB->x*BC->y - AB->y*BC->x));
    return cross_val;
}