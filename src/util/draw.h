#ifndef __UMF_DRAW_H
#define __UMF_DRAW_H

#include "image.h"
#include "renderer.h"
#include <Eigen/Core>

namespace umf {


void drawLine(Renderer *r, Eigen::Vector2i startP, Eigen::Vector2i endP, Eigen::Vector3i lineColor, int width);

void drawLineEq(Renderer *r, Eigen::Vector3f line, Eigen::Vector3i drawColor, int width);

void drawCircle(Renderer *r, Eigen::Vector2i center, int radius, Eigen::Vector3i lineColor, int width);

/**
 * @brief draw and arrow starting from point startP to endP
 * @param img
 * @param startP
 * @param endP
 * @param lineColor
 * @param width
 */
void drawArrow(Renderer *r, Eigen::Vector2i startP, Eigen::Vector2i endP, Eigen::Vector3i lineColor, int width);

/**
 * @brief draw two parallel lines between startP and endP
 * @param img
 * @param startP
 * @param endP
 * @param lineColor
 * @param width
 */
void drawEquals(Renderer *r, Eigen::Vector2i startP, Eigen::Vector2i endP, Eigen::Vector3i lineColor, int width);

Eigen::Vector3f hsv2rgb(Eigen::Vector3f in);

Eigen::Vector3f rgb2hsv(Eigen::Vector3f in);

}

#endif // DRAW_H
