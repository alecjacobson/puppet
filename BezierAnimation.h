#ifndef BEZIERANIMATION_H
#define BEZIERANIMATION_H
#include "BezierKeyframe.h"
#include "Animation.h"
template <typename T>
class BezierAnimation : public Animation<BezierKeyframe<T> >
{
};
#endif
