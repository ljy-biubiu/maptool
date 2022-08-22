#include "test/linecommon.h"

namespace cti {
namespace test {

LineCommon::LineCommon()
{
    this->LP1.x = 0;
    this->LP1.y = 0;
    this->LP2.x = 0;
    this->LP2.y = 0;
}
LineCommon::LineCommon(float x1, float y1, float x2, float y2)
{
    this->LP1.x = x1;
    this->LP1.y = y1;
    this->LP2.x = x2;
    this->LP2.y = y2;
}
LineCommon::LineCommon(LinePoint p1, LinePoint p2)
   :LP1(p1), LP2(p2)
{
}

LineCommon::~LineCommon()
{
}

void LineCommon::setLinePoint(LinePoint &p1, LinePoint &p2)
{
    this->LP1 = p1;
    this->LP2 = p2;
}

double LineCommon::multi(LinePoint &a, LinePoint &b, LinePoint &c)
{
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool LineCommon::isIntersect(LineCommon &line, LinePoint &intersect_point)
{
    bool ret = false;
    double a,b,c,a_oth,b_oth,c_oth;
    getLinearEquation(this->LP1, this->LP2, &a, &b, &c);
    //printf("a=%f,b=%f,c=%f\n",a,b,c);
    getLinearEquation(line.LP1, line.LP2, &a_oth, &b_oth, &c_oth);
    //printf("a_oth=%f,b_oth=%f,c_oth=%f\n",a_oth,b_oth,c_oth);
    double D = a*b_oth - a_oth*b;
    if(D != 0) {
        intersect_point.x = (b*c_oth - b_oth*c)/D;
        intersect_point.y = (a_oth*c - a*c_oth)/D;
        if(isInRange(intersect_point) && line.isInRange(intersect_point)){
            ret = true;
        }
    }
    return ret;
}

bool LineCommon::isIntersect(LineCommon &line)
{
    LinePoint line_start = line.getLineStart();
    LinePoint line_end   = line.getLineEnd();
    double u = multi(this->LP1,this->LP2,line_start);
    double v = multi(this->LP1,this->LP2,line_end);
    double w = multi(line_start,line_end,this->LP1);
    double z = multi(line_start,line_end,this->LP2);
    return (u*v <= 1e-6 && w*z <= 1e-6);
}

bool LineCommon::isNearPoint(const LinePoint &point, LinePoint &nearPoint)
{
    double error = pow(10, -5);
    bool ret = false;
    double dx = this->LP1.x - this->LP2.x;
    double dy = this->LP1.y - this->LP2.y;
    if(fabs(dx) > error || fabs(dy) > error)
    {
        double u = (point.x-this->LP1.x)*(this->LP1.x-this->LP2.x) + \
                   (point.y-this->LP1.y)*(this->LP1.y-this->LP2.y);
        u = u/(pow(dx,2.0)+pow(dy,2.0));
        nearPoint.x = this->LP1.x+u*dx;
        nearPoint.y = this->LP1.y+u*dy;
        ret = true;
    }
    return ret;
}

double LineCommon::getLineYaw(void)
{
    return atan2(this->LP2.y - this->LP1.y, this->LP2.x - this->LP1.x);
}

double LineCommon::getLineLenght(void)
{
    return computatDistance(this->LP1,this->LP2);
}

bool LineCommon::isInRange(LinePoint p)
{
    return isInRangeX(p) && isInRangeY(p);
}

double LineCommon::computatDistance(const LinePoint &p1, const LinePoint &p2)
{
    return sqrt(pow((p2.x - p1.x), 2) + pow((p2.y - p1.y), 2));
}

}
}
