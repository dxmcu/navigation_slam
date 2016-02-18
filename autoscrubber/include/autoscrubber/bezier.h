/* Copyright(C) Gaussian Automation. All rights reserved.
 */

/**
 * @file bezier.h
 * @brief cubic bezier curve generator
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2016-02-11
 */

#ifndef AUTOSCRUBBER_INCLUDE_AUTOSCRUBBER_BEZIER_H_
#define AUTOSCRUBBER_INCLUDE_AUTOSCRUBBER_BEZIER_H_

namespace autoscrubber {

typedef struct _BezierPoint {
  double x;
  double y;
} BezierPoint;

void cubic_bezier(unsigned int n, BezierPoint* pts,
                  double x1, double y1, double x2, double y2,
                  double x3, double y3, double x4, double y4);

};  // namespace autoscrubber

#endif  // AUTOSCRUBBER_INCLUDE_AUTOSCRUBBER_BEZIER_H_
