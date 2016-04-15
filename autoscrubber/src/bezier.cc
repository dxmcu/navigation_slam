/* Copyright(C) Gaussian Automation. All rights reserved.
 */

/**
 * @file bezier.cc
 * @brief cubic bezier generator implementation
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2016-02-11
 */

#include "autoscrubber/bezier.h"

#include <math.h>
#include <assert.h>

namespace autoscrubber {

void cubic_bezier(unsigned int n, BezierPoint* pts,
                  double x1, double y1, double x2, double y2,
                  double x3, double y3, double x4, double y4) {
  // make sure n >= 2
  assert(n > 1);

  for (unsigned int i = 0; i < n; ++i) {
    double t = static_cast<double>(i) / static_cast<double>(n - 1);

    double a = pow((1.0 - t), 3.0);
    double b = 3.0 * t * pow((1.0 - t), 2.0);
    double c = 3.0 * pow(t, 2.0) * (1.0 - t);
    double d = pow(t, 3.0);

    double x = a * x1 + b * x2 + c * x3 + d * x4;
    double y = a * y1 + b * y2 + c * y3 + d * y4;
    pts[i].x = x;
    pts[i].y = y;
  }
}

};  // namespace autoscrubber
