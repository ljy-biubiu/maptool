#ifndef LINEINTERSECT_H
#define LINEINTERSECT_H

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <iostream>

namespace cti {
namespace test {

class LineCommon
{
public:
    struct LinePoint
    {
        LinePoint(){
            x = 0;
            y = 0;
        }
        LinePoint(double p_x, double p_y){
            x = p_x;
            y = p_y;
        }
        double x,y;
    };
    LineCommon();
    LineCommon(float x1, float y1, float x2, float y2);
    LineCommon(LinePoint p1, LinePoint p2);
    ~LineCommon();
    void setLinePoint(LinePoint &p1, LinePoint &p2);
    bool isIntersect(LineCommon &another_line, LinePoint &intersect_point);
    bool isNearPoint(const LinePoint &point, LinePoint &nearPoint);
    bool isInRange(LinePoint p);
    inline LinePoint getLineStart(){
        return LP1;
    }
    inline LinePoint getLineEnd(){
        return LP2;
    }
    inline bool isLine(){
        return !(LP1.x == LP2.x && LP1.y == LP2.y);
    }
    bool   isIntersect(LineCommon &line);
    double getLineLenght(void);
    double getLineYaw(void);
    double computatDistance(const LinePoint &p1, const LinePoint &p2);
protected:
    inline bool isAscendingX() const {
        return this->LP1.x < this->LP2.x;
    }
    inline bool isAscendingY() const {
        return this->LP1.y < this->LP2.y;
    }
    inline bool isInRangeX(LinePoint p){
      return this->isAscendingX()
             ? p.x >= this->LP1.x && p.x <= this->LP2.x
             : p.x >= this->LP2.x && p.x <= this->LP1.x;
    }
    inline bool isInRangeY(LinePoint p){
      return this->isAscendingY()
             ? p.y >= this->LP1.y && p.y <= this->LP2.y
             : p.y >= this->LP2.y && p.y <= this->LP1.y;
    }
    double multi(LinePoint &a, LinePoint &b, LinePoint &c);
    // let the linear equation be "ax + by + c = 0"
    // if there are two points (x1,y1) , (x2,y2), a = "y2-y1, b = "(-1) * x2 - x1" ,c = "(-1) * (y2-y1)x1 + (x2-x1)y1"
    inline void getLinearEquation(LinePoint start, LinePoint end, double *a, double *b, double *c){
        *a = end.y - start.y;
        *b = (-1) * (end.x - start.x);
        *c = (-1) * (end.y - start.y) * start.x + (end.x - start.x) * start.y;
    }
private:
    LinePoint LP1;
    LinePoint LP2;
};

}
}
#endif

