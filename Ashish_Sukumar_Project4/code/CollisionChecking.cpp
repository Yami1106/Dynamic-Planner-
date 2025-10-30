///////////////////////////////////////
// RBE 550
// Project 4
// Authors: Ashish Sukumar
//////////////////////////////////////

#include "CollisionChecking.h"

#include <algorithm> // std::min, std::max
#include <cmath>     // std::cos, std::sin
#include <vector>    // std::vector

// TODO: Copy your implementation from previous projects

bool isValidPoint(double x, double y, const std::vector<Rectangle> &obstacles)
{
    for (const auto &obs : obstacles)
    {
        double x_lb = obs.x;
        double x_ub = obs.x + obs.width;
        double y_lb = obs.y;
        double y_ub = obs.y + obs.height;

        // If the point is inside the bounds of this rectangle, it's in collision
        if (x >= x_lb && x <= x_ub &&
            y >= y_lb && y <= y_ub)
        {
            return false;
        }
    }
    return true;
}

bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle> &obstacles)
{
    double r2 = radius * radius;

    for (const auto &obs : obstacles)
    {
        double x_lb = obs.x;
        double x_ub = obs.x + obs.width;
        double y_lb = obs.y;
        double y_ub = obs.y + obs.height;

        // Find the closest point on the rectangle to the circle center
        double closestX = std::max(x_lb, std::min(x, x_ub));
        double closestY = std::max(y_lb, std::min(y, y_ub));

        double dx = x - closestX;
        double dy = y - closestY;
        double dist2 = dx * dx + dy * dy;

        // If the closest point is within the radius, we collide
        if (dist2 <= r2)
        {
            return false;
        }
    }

    return true;
}

bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle> &obstacles)
{
    double half_side = sideLength / 2.0;

    // Square corners in local (robot-centered) frame before rotation
    std::vector<std::pair<double,double>> localCorners = {
        { half_side,  half_side},
        {-half_side,  half_side},
        {-half_side, -half_side},
        { half_side, -half_side}
    };

    // Rotate + translate to world frame
    std::vector<std::pair<double,double>> worldCorners;
    worldCorners.reserve(4);

    for (auto &c : localCorners)
    {
        double xr =  c.first * std::cos(theta) - c.second * std::sin(theta);
        double yr =  c.first * std::sin(theta) + c.second * std::cos(theta);

        worldCorners.push_back({ x + xr, y + yr });
    }

    // Compute axis-aligned bounding box (AABB) of rotated square
    double x_min = worldCorners[0].first;
    double x_max = worldCorners[0].first;
    double y_min = worldCorners[0].second;
    double y_max = worldCorners[0].second;

    for (auto &c : worldCorners)
    {
        x_min = std::min(x_min, c.first);
        x_max = std::max(x_max, c.first);
        y_min = std::min(y_min, c.second);
        y_max = std::max(y_max, c.second);
    }

    // Now check that AABB against each obstacle AABB
    for (const auto &obs : obstacles)
    {
        double x_lb = obs.x;
        double x_ub = obs.x + obs.width;
        double y_lb = obs.y;
        double y_ub = obs.y + obs.height;

        bool overlap =
            (x_max >= x_lb) && (x_min <= x_ub) &&
            (y_max >= y_lb) && (y_min <= y_ub);

        if (overlap)
            return false;
    }

    return true;
}
