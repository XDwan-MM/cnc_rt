#include "common.h"

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

double POS_Dot(POS p1,  POS p2){
    return p1.x*p2.x + p1.y*p2.y +p1.z*p2.z;
}
