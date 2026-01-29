#ifndef COORDINATE_H
#define COORDINATE_H

#include <QObject>

struct CoordPoint
{
    double x;
    double y;
    double z;
    double a;
    double b;
    double c;
    CoordPoint operator+(const CoordPoint &coord) const
    {
        CoordPoint sum = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        sum = {
            x + coord.x,
            y + coord.y,
            z + coord.z,
            a + coord.a,
            b + coord.b,
            c + coord.c
        };
        return sum;
    }

    CoordPoint operator-(const CoordPoint &coord) const
    {
        CoordPoint difference = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        difference = {
            x - coord.x,
            y - coord.y,
            z - coord.z,
            a - coord.a,
            b - coord.b,
            c - coord.c
        };
        return difference;
    }
};

//参考点坐标
extern CoordPoint g_refPoint[4];

class NCITPVariable;
class Coordinate
{
public:
    Coordinate(NCITPVariable &variable);
    // 选择当前绝对坐标系，共有6个绝对坐标系
    void setCurrentAbsSys(int currentAbsSys);
    int currentAbsSys() const;

    // 设定绝对坐标系原点
//    void setAbsOrigin(CoordPoint origin, int i);
    CoordPoint AbsCoordOrigin(int i) const;

    // 设定局部坐标系，不使用时设定为0
    void setChildOrigin(CoordPoint origin);
    void clearChildOrigin();
    CoordPoint ChildOrigin() const;

    // 设定G92外部偏移坐标系
    void setOffsetCoordOriginG92(CoordPoint origin);
    void clearOffsetCoordOriginG92();
    CoordPoint OffsetCoordOriginG92() const;

    // 设定G92.1偏移坐标系
    void setOffsetCoordOriginG92_1(CoordPoint origin);
    void clearOffsetCoordOriginG92_1();
    CoordPoint OffsetCoordOriginG92_1() const;

    // 局部坐标返回机械坐标
    CoordPoint child_To_Mach(CoordPoint childCoord) const;
    double getAxesX(double posX) const;
    double getAxesY(double posY) const;
    double getAxesZ(double posZ) const;
    double getAxesA(double posA) const;
    double getAxesB(double posB) const;
    double getAxesC(double posC) const;

private:
    int m_currentAbsSys = 1;
    CoordPoint childCoordOrigin;
    CoordPoint offsetCoordOriginG92_1;
    CoordPoint offsetCoordOriginG92;
//    CoordPoint coordOrigin[7];
//    int m_currentCoordSys;
    NCITPVariable *m_variable;
    CoordPoint doublePrecision(CoordPoint coordPoint) const;
    bool judgeCoordSysSum(int i) const;

    CoordPoint mach_To_Child(CoordPoint machCoord, CoordPoint currentAbsPoint) const;

public:
    CoordPoint AbsCoord(CoordPoint machCoord,int i) const;
    CoordPoint sumoOfOffsetCoord() const;
};

#endif // COORDINATE_H
