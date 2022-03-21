/*
 * Copyright © 2017 Oystein Myrmo
 *
 * Permission to use, copy, modify, distribute, and sell this software and its
 * documentation for any purpose is hereby granted without fee, provided that
 * the above copyright notice appear in all copies and that both that copyright
 * notice and this permission notice appear in supporting documentation, and
 * that the name of the copyright holders not be used in advertising or
 * publicity pertaining to distribution of the software without specific,
 * written prior permission.  The copyright holders make no representations
 * about the suitability of this software for any purpose.  It is provided "as
 * is" without express or implied warranty.
 *
 * THE COPYRIGHT HOLDERS DISCLAIM ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
 * INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO
 * EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THIS SOFTWARE.
 */
#include "bezier.h"
#include "assert.h"
#include "test_beziers.h"

void derivatives_tests()
{
    Bezier::Bezier<3> bezier3 = TestBeziers::Default::CubicBezier;
    Bezier::Bezier<2> bezier2 = bezier3.derivative();
    Bezier::Bezier<1> bezier1 = bezier2.derivative();
    Bezier::Bezier<0> bezier0 = bezier1.derivative();

    BEZIER_ASSERT(bezier3.size() == 4);
    BEZIER_ASSERT(bezier2.size() == 3);
    BEZIER_ASSERT(bezier1.size() == 2);
    BEZIER_ASSERT(bezier0.size() == 1);

    BEZIER_FUZZY_ASSERT_POINT(bezier2[0], -255.0, 120.0);
    BEZIER_FUZZY_ASSERT_POINT(bezier2[1], 555.0, 180.0);
    BEZIER_FUZZY_ASSERT_POINT(bezier2[2], 0.0, -660.0);

    BEZIER_FUZZY_ASSERT_POINT(bezier1[0], 1620.0, 120.0);
    BEZIER_FUZZY_ASSERT_POINT(bezier1[1], -1110.0, -1680.0);

    BEZIER_FUZZY_ASSERT_POINT(bezier0[0], -2730.0, -1800.0);

    bezier2 = TestBeziers::Default::QuadraticBezier;
    bezier1 = bezier2.derivative();
    bezier0 = bezier1.derivative();

    BEZIER_ASSERT(bezier2.size() == 3);
    BEZIER_ASSERT(bezier1.size() == 2);
    BEZIER_ASSERT(bezier0.size() == 1);

    BEZIER_FUZZY_ASSERT_POINT(bezier1[0], -100.0, -90.0);
    BEZIER_FUZZY_ASSERT_POINT(bezier1[1], 160.0, -70.0);

    BEZIER_FUZZY_ASSERT_POINT(bezier0[0], 260.0, 20.0);
}

void extreme_values_tests()
{
    Bezier::Bezier<3> bezier3 = TestBeziers::Default::CubicBezier;

    Bezier::ExtremeValues xVals = bezier3.derivativeZero();
    BEZIER_ASSERT(xVals.size() == 3);
    BEZIER_FUZZY_ASSERT(xVals[0].t, 0.186811984f);
    BEZIER_FUZZY_ASSERT(xVals[1].t, 1.0f);
    BEZIER_FUZZY_ASSERT(xVals[2].t, 0.437850952f);

    Bezier::ExtremePoints xPoints = bezier3.extremePoints();
    BEZIER_ASSERT(xPoints.size() == 4);
    BEZIER_FUZZY_ASSERT_POINT(xPoints[0], 97.6645355f, 182.555649f);
    BEZIER_FUZZY_ASSERT_POINT(xPoints[1], 220.0f, 40.0f);
    BEZIER_FUZZY_ASSERT_POINT(xPoints[2], 125.442337f, 198.86235f);
    BEZIER_FUZZY_ASSERT_POINT(xPoints[3], 120.0f, 160.0f);

    Bezier::AABB aabb = bezier3.aabb();
    BEZIER_ASSERT(aabb.size() == 4);
    BEZIER_FUZZY_ASSERT(aabb.minX(), 97.6645355f);
    BEZIER_FUZZY_ASSERT(aabb.maxX(), 220.0f);
    BEZIER_FUZZY_ASSERT(aabb.minY(), 40.0f);
    BEZIER_FUZZY_ASSERT(aabb.maxY(), 198.86235f);
    BEZIER_FUZZY_ASSERT(aabb.width(), 122.335464f);
    BEZIER_FUZZY_ASSERT(aabb.height(), 158.86235f);
    BEZIER_FUZZY_ASSERT(aabb.area(), 19434.5f);

    Bezier::TBB tbb = bezier3.tbb();
    BEZIER_ASSERT(tbb.size() == 4);
    BEZIER_FUZZY_ASSERT(tbb.minX(), 92.568962f);
    BEZIER_FUZZY_ASSERT(tbb.maxX(), 261.989441f);
    BEZIER_FUZZY_ASSERT(tbb.minY(), 36.2565613f);
    BEZIER_FUZZY_ASSERT(tbb.maxY(), 222.517883f);
    BEZIER_FUZZY_ASSERT(tbb.width(), 60.5054359f);
    BEZIER_FUZZY_ASSERT(tbb.height(), 192.036713f);
    BEZIER_FUZZY_ASSERT(tbb.area(), 11619.2646f);

    Bezier::Bezier<2> bezier2 = TestBeziers::Default::QuadraticBezier;

    xVals = bezier2.derivativeZero();
    BEZIER_ASSERT(xVals.size() == 1);
    BEZIER_FUZZY_ASSERT(xVals[0].t, 0.384615391f);

    xPoints = bezier2.extremePoints();
    BEZIER_ASSERT(xPoints.size() == 3);
    BEZIER_FUZZY_ASSERT_POINT(xPoints[0], 50.7692261f, 121.863899f);
    BEZIER_FUZZY_ASSERT_POINT(xPoints[1], 70.0f, 155.0f);
    BEZIER_FUZZY_ASSERT_POINT(xPoints[2], 100.0f, 75.0f);

    aabb = bezier2.aabb(xPoints);
    BEZIER_ASSERT(aabb.size() == 4);
    BEZIER_FUZZY_ASSERT(aabb.minX(), 50.7692261f);
    BEZIER_FUZZY_ASSERT(aabb.maxX(), 100.0f);
    BEZIER_FUZZY_ASSERT(aabb.minY(), 75.0f);
    BEZIER_FUZZY_ASSERT(aabb.maxY(), 155.0f);
    BEZIER_FUZZY_ASSERT(aabb.width(), 49.2307739f);
    BEZIER_FUZZY_ASSERT(aabb.height(), 80.0f);
    BEZIER_FUZZY_ASSERT(aabb.area(), 3938.46191f);

    tbb = bezier2.tbb();
    BEZIER_ASSERT(tbb.size() == 4);
    BEZIER_FUZZY_ASSERT(tbb.minX(), 40.6849289f);
    BEZIER_FUZZY_ASSERT(tbb.maxX(), 100.0f);
    BEZIER_FUZZY_ASSERT(tbb.minY(), 64.0068511f);
    BEZIER_FUZZY_ASSERT(tbb.maxY(), 155.0f);
    BEZIER_FUZZY_ASSERT(tbb.width(), 31.3085079f);
    BEZIER_FUZZY_ASSERT(tbb.height(), 85.4400406f);
    BEZIER_FUZZY_ASSERT(tbb.area(), 2674.99951f);
}

void tangents_tests()
{
    Bezier::Bezier<3> bezier3 = TestBeziers::Default::CubicBezier;
    Bezier::ExtremeValues xVals = bezier3.derivativeZero();
    Bezier::Tangent tangent;
    BEZIER_ASSERT(xVals.size() == 3);

    tangent = bezier3.tangentAt(0.0);
    BEZIER_FUZZY_ASSERT(tangent.x, -0.904818713f);
    BEZIER_FUZZY_ASSERT(tangent.y, 0.425797045f);
    BEZIER_FUZZY_ASSERT(tangent.angle(), 2.70175004f);
    BEZIER_FUZZY_ASSERT(tangent.angleDeg(), 154.798874f);

    tangent = bezier3.tangentAt(0.25);
    BEZIER_FUZZY_ASSERT(tangent.x, 0.567925274f);
    BEZIER_FUZZY_ASSERT(tangent.y, 0.823080122f);
    BEZIER_FUZZY_ASSERT(tangent.angle(), 0.966813385f);
    BEZIER_FUZZY_ASSERT(tangent.angleDeg(), 55.3943253f);

    tangent = bezier3.tangentAt(0.50);
    BEZIER_FUZZY_ASSERT(tangent.x, 0.978549778f);
    BEZIER_FUZZY_ASSERT(tangent.y, -0.206010476f);
    BEZIER_FUZZY_ASSERT(tangent.angle(), -0.207496226f);
    BEZIER_FUZZY_ASSERT(tangent.angleDeg(), -11.8886576f);

    tangent = bezier3.tangentAt(0.75);
    BEZIER_FUZZY_ASSERT(tangent.x, 0.544241607f);
    BEZIER_FUZZY_ASSERT(tangent.y, -0.83892852f);
    BEZIER_FUZZY_ASSERT(tangent.angle(), -0.995311498f);
    BEZIER_FUZZY_ASSERT(tangent.angleDeg(), -57.0271492f);

    tangent = bezier3.tangentAt(0.666f);
    BEZIER_FUZZY_ASSERT(tangent.x, 0.738803029f);
    BEZIER_FUZZY_ASSERT(tangent.y, -0.673921466f);
    BEZIER_FUZZY_ASSERT(tangent.angle(), -0.73950386f);
    BEZIER_FUZZY_ASSERT(tangent.angleDeg(), -42.3704491f);

    tangent = bezier3.tangentAt(0.333f);
    BEZIER_FUZZY_ASSERT(tangent.x, 0.911237537f);
    BEZIER_FUZZY_ASSERT(tangent.y, 0.411881238f);
    BEZIER_FUZZY_ASSERT(tangent.angle(), 0.424517602f);
    BEZIER_FUZZY_ASSERT(tangent.angleDeg(), 24.3230667f);

    tangent = bezier3.tangentAt(xVals[0].t);
    BEZIER_FUZZY_ASSERT(tangent.x, 0.0f);
    BEZIER_FUZZY_ASSERT(tangent.y, 1.0f);
    BEZIER_FUZZY_ASSERT(tangent.angle(), Bezier::Math::PI / 2.0f);
    BEZIER_FUZZY_ASSERT(tangent.angleDeg(), 90.0f);

    tangent = bezier3.tangentAt(xVals[1].t);
    BEZIER_FUZZY_ASSERT(tangent.x, 0.0f);
    BEZIER_FUZZY_ASSERT(tangent.y, -1.0f);
    BEZIER_FUZZY_ASSERT(tangent.angle(), -Bezier::Math::PI / 2.0f);
    BEZIER_FUZZY_ASSERT(tangent.angleDeg(), -90.0f);

    tangent = bezier3.tangentAt(xVals[2].t);
    BEZIER_FUZZY_ASSERT(tangent.x, 1.0f);
    BEZIER_FUZZY_ASSERT(tangent.y, 0.0f);
    BEZIER_FUZZY_ASSERT(tangent.angle(), 0.0f);
    BEZIER_FUZZY_ASSERT(tangent.angleDeg(), 0.0f);
}

void normals_tests()
{
    Bezier::Bezier<3> bezier3 = TestBeziers::Default::CubicBezier;
    Bezier::ExtremeValues xVals = bezier3.derivativeZero();
    Bezier::Normal normal;
    BEZIER_ASSERT(xVals.size() == 3);

    normal = bezier3.normalAt(0.0);
    BEZIER_FUZZY_ASSERT(normal.x, -0.425797045f);
    BEZIER_FUZZY_ASSERT(normal.y, -0.904818713f);
    BEZIER_FUZZY_ASSERT(normal.angle(), -2.01063895f);
    BEZIER_FUZZY_ASSERT(normal.angleDeg(), -115.201126f);

    normal = bezier3.normalAt(0.25);
    BEZIER_FUZZY_ASSERT(normal.x, -0.823080122f);
    BEZIER_FUZZY_ASSERT(normal.y, 0.567925274f);
    BEZIER_FUZZY_ASSERT(normal.angle(), 2.53760958f);
    BEZIER_FUZZY_ASSERT(normal.angleDeg(), 145.394318f);

    normal = bezier3.normalAt(0.50);
    BEZIER_FUZZY_ASSERT(normal.x, 0.206010476f);
    BEZIER_FUZZY_ASSERT(normal.y, 0.978549778f);
    BEZIER_FUZZY_ASSERT(normal.angle(), 1.36330009f);
    BEZIER_FUZZY_ASSERT(normal.angleDeg(), 78.1113433f);

    normal = bezier3.normalAt(0.75);
    BEZIER_FUZZY_ASSERT(normal.x, 0.83892852f);
    BEZIER_FUZZY_ASSERT(normal.y, 0.544241607f);
    BEZIER_FUZZY_ASSERT(normal.angle(), 0.575484872f);
    BEZIER_FUZZY_ASSERT(normal.angleDeg(), 32.9728546f);

    normal = bezier3.normalAt(0.666f);
    BEZIER_FUZZY_ASSERT(normal.x, 0.673921466f);
    BEZIER_FUZZY_ASSERT(normal.y, 0.738803029f);
    BEZIER_FUZZY_ASSERT(normal.angle(), 0.83129245f);
    BEZIER_FUZZY_ASSERT(normal.angleDeg(), 47.6295471f);

    normal = bezier3.normalAt(0.333f);
    BEZIER_FUZZY_ASSERT(normal.x, -0.411881238f);
    BEZIER_FUZZY_ASSERT(normal.y, 0.911237537f);
    BEZIER_FUZZY_ASSERT(normal.angle(), 1.99531388f);
    BEZIER_FUZZY_ASSERT(normal.angleDeg(), 114.323067f);

    normal = bezier3.normalAt(xVals[0].t);
    BEZIER_FUZZY_ASSERT(normal.x, -1.0f);
    BEZIER_FUZZY_ASSERT(normal.y, 0.0f);
    BEZIER_FUZZY_ASSERT(normal.angle(), -Bezier::Math::PI);
    BEZIER_FUZZY_ASSERT(normal.angleDeg(), -180.0f);

    normal = bezier3.normalAt(xVals[1].t);
    BEZIER_FUZZY_ASSERT(normal.x, 1.0f);
    BEZIER_FUZZY_ASSERT(normal.y, 0.0f);
    BEZIER_FUZZY_ASSERT(normal.angle(), 0.0f);
    BEZIER_FUZZY_ASSERT(normal.angleDeg(), 0.0f);

    normal = bezier3.normalAt(xVals[2].t);
    BEZIER_FUZZY_ASSERT(normal.x, 0.0f);
    BEZIER_FUZZY_ASSERT(normal.y, 1.0f);
    BEZIER_FUZZY_ASSERT(normal.angle(), Bezier::Math::PI / 2.0f);
    BEZIER_FUZZY_ASSERT(normal.angleDeg(), 90.0f);
}

int main()
{
    printf("Starting tests.\n");

    printf("Running derivative_tests().\n");        derivatives_tests();
    printf("Running extreme_values_tests().\n");    extreme_values_tests();
    printf("Running tangents_tests().\n");          tangents_tests();
    printf("Running normals_tests().\n");           normals_tests();

    printf("All tests completed.\n");
    return 0;
}
