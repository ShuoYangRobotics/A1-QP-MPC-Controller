#pragma once
#include "bezier.h"
#include <vector>

namespace TestBeziers
{
    namespace Creators
    {
        // Visualization: https://www.desmos.com/calculator/fivneeogmh
        Bezier::Bezier<3> defaultCubicBezier()
        {
            return Bezier::Bezier<3>({
                {120, 160},
                {35,  200},
                {220, 260},
                {220,  40}
            });
        }
        
        Bezier::Bezier<2> defaultQuadraticBezier()
        {
            return Bezier::Bezier<2>({
                {70, 155},
                {20, 110},
                {100, 75}
            });
        }
    }

    namespace Default
    {
        static const Bezier::Bezier<2> QuadraticBezier = Creators::defaultQuadraticBezier();
        static const Bezier::Bezier<3> CubicBezier = Creators::defaultCubicBezier();
    }
}
