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
#include <cstdio>

void binomial_tests()
{
    BEZIER_ASSERT(Bezier::Bezier<0>::binomialCoefficients.size() == 1);
    BEZIER_ASSERT(Bezier::Bezier<1>::binomialCoefficients.size() == 2);
    BEZIER_ASSERT(Bezier::Bezier<2>::binomialCoefficients.size() == 3);
    BEZIER_ASSERT(Bezier::Bezier<3>::binomialCoefficients.size() == 4);
    BEZIER_ASSERT(Bezier::Bezier<4>::binomialCoefficients.size() == 5);
    BEZIER_ASSERT(Bezier::Bezier<5>::binomialCoefficients.size() == 6);
    BEZIER_ASSERT(Bezier::Bezier<10>::binomialCoefficients.size() == 11);
    BEZIER_ASSERT(Bezier::Bezier<15>::binomialCoefficients.size() == 16);
    BEZIER_ASSERT(Bezier::Bezier<20>::binomialCoefficients.size() == 21);
    BEZIER_ASSERT(Bezier::Bezier<50>::binomialCoefficients.size() == 51);

    // N = 0 --> [1]
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<0>(0, 1);

    // N = 1 --> [1 1]
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<1>(0, 1);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<1>(1, 1);

    // N = 2 --> [1 2 1]
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<2>(0, 1);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<2>(1, 2);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<2>(2, 1);

    // N = 3 --> [1 3 3 1]
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<3>(0, 1);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<3>(1, 3);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<3>(2, 3);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<3>(3, 1);

    // N = 4 --> [1 4 6 4 1]
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<4>(0, 1);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<4>(1, 4);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<4>(2, 6);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<4>(3, 4);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<4>(4, 1);

    // N = 5 --> [1 5 10 10 5 1]
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<5>(0, 1);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<5>(1, 5);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<5>(2, 10);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<5>(3, 10);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<5>(4, 5);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<5>(5, 1);

    // N = 10 --> [1 10 45 120 210 252 210 120 45 10 1]
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<10>(0, 1);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<10>(1, 10);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<10>(2, 45);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<10>(3, 120);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<10>(4, 210);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<10>(5, 252);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<10>(6, 210);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<10>(7, 120);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<10>(8, 45);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<10>(9, 10);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<10>(10, 1);

    // N = 15 --> [1 15 105 455 1365 3003 5005 6435 6435 5005 3003 1365 455 105 15 1]
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<15>(0, 1);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<15>(1, 15);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<15>(2, 105);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<15>(3, 455);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<15>(4, 1365);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<15>(5, 3003);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<15>(6, 5005);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<15>(7, 6435);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<15>(8, 6435);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<15>(9, 5005);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<15>(10, 3003);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<15>(11, 1365);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<15>(12, 455);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<15>(13, 105);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<15>(14, 15);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<15>(15, 1);

    // N = 20 --> [1 20 190 1140 4845 15504 38760 77520 125970 167960 184756 167960 125970 77520 38760 15504 4845 1140 190 20 1]
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<20>(0, 1);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<20>(1, 20);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<20>(2, 190);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<20>(3, 1140);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<20>(4, 4845);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<20>(5, 15504);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<20>(6, 38760);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<20>(7, 77520);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<20>(8, 125970);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<20>(9, 167960);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<20>(10, 184756);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<20>(11, 167960);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<20>(12, 125970);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<20>(13, 77520);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<20>(14, 38760);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<20>(15, 15504);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<20>(16, 4845);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<20>(17, 1140);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<20>(18, 190);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<20>(19, 20);
    BEZIER_ASSERT_BINOMIAL_COEFFICIENT<20>(20, 1);
}

void polynomial_tests()
{
    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<0>(0, 0, 0);

    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<1>(0, 1, 0);
    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<1>(1, 0, 1);

    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<2>(0, 2, 0);
    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<2>(1, 1, 1);
    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<2>(2, 0, 2);

    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<3>(0, 3, 0);
    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<3>(1, 2, 1);
    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<3>(2, 1, 2);
    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<3>(3, 0, 3);

    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<4>(0, 4, 0);
    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<4>(1, 3, 1);
    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<4>(2, 2, 2);
    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<4>(3, 1, 3);
    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<4>(4, 0, 4);

    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<5>(0, 5, 0);
    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<5>(1, 4, 1);
    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<5>(2, 3, 2);
    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<5>(3, 2, 3);
    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<5>(4, 1, 4);
    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<5>(5, 0, 5);

    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<10>(0, 10, 0);
    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<10>(1, 9, 1);
    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<10>(2, 8, 2);
    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<10>(3, 7, 3);
    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<10>(4, 6, 4);
    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<10>(5, 5, 5);
    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<10>(6, 4, 6);
    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<10>(7, 3, 7);
    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<10>(8, 2, 8);
    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<10>(9, 1, 9);
    BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS<10>(10, 0, 10);
}

void control_points_tests()
{
    Bezier::Bezier<2> bezier2;
    BEZIER_ASSERT(bezier2.size() == 3);

    Bezier::Point &p = bezier2[0];
    BEZIER_ASSERT(p.x == 0.0f);
    BEZIER_ASSERT(p.y == 0.0f);

    p.set(1.0f, 2.0f);
    BEZIER_ASSERT(bezier2[0].x == 1.0f);
    BEZIER_ASSERT(bezier2[0].y == 2.0f);

    p.translate(1, -1);
    BEZIER_ASSERT(bezier2[0].x == 2.0f);
    BEZIER_ASSERT(bezier2[0].y == 1.0f);
}

void values_tests()
{
    Bezier::Bezier<3> bezier3 = TestBeziers::Default::CubicBezier;

    Bezier::Point val = bezier3.valueAt(0);
    BEZIER_FUZZY_ASSERT_POINT(val, 120.0, 160.0);

    val = bezier3.valueAt(1);
    BEZIER_FUZZY_ASSERT_POINT(val, 220.0, 40.0);

    val = bezier3.valueAt(0.25);
    BEZIER_FUZZY_ASSERT_POINT(val, 99.765625, 189.0625);

    val = bezier3.valueAt(0.50);
    BEZIER_FUZZY_ASSERT_POINT(val, 138.125, 197.5);

    val = bezier3.valueAt(0.75);
    BEZIER_FUZZY_ASSERT_POINT(val, 192.421875, 157.1875);

    val = bezier3.valueAt(-0.35f);
    BEZIER_FUZZY_ASSERT_POINT(val, 327.983124, 138.212509);

    val = bezier3.valueAt(1.5);
    BEZIER_FUZZY_ASSERT_POINT(val, 24.375, -537.5);

    Bezier::Bezier<2> bezier2 = TestBeziers::Default::QuadraticBezier;

    val = bezier2.valueAt(0);
    BEZIER_FUZZY_ASSERT_POINT(val, 70.0, 155.0);

    val = bezier2.valueAt(1);
    BEZIER_FUZZY_ASSERT_POINT(val, 100.0, 75.0);

    val = bezier2.valueAt(0.5);
    BEZIER_FUZZY_ASSERT_POINT(val, 52.5, 112.5);

    val = bezier2.valueAt(-1.0);
    BEZIER_FUZZY_ASSERT_POINT(val, 300.0, 255.0);

    val = bezier2.valueAt(2.0);
    BEZIER_FUZZY_ASSERT_POINT(val, 390.0, 15.0);
}

void length_tests()
{
    // Using similar curves as https://pomax.github.io/bezierinfo/#arclengthapprox for testing.
    Bezier::Bezier<2> b2({ {  70, 250 }, { 20, 110 }, { 200,  80 } });            // Real length: 256.25
    Bezier::Bezier<3> b3({ { 120, 160 }, { 35, 200 }, { 220, 260 }, {220, 40} }); // Real length: 272.87

    BEZIER_FUZZY_ASSERT(b2.length(0),    0.0f);
    BEZIER_FUZZY_ASSERT(b2.length(1),    214.009338f);
    BEZIER_FUZZY_ASSERT(b2.length(2),    248.073410f);
    BEZIER_FUZZY_ASSERT(b2.length(3),    252.609528f);
    BEZIER_FUZZY_ASSERT(b2.length(5),    254.948517f);
    BEZIER_FUZZY_ASSERT(b2.length(10),   255.927200f);
    BEZIER_FUZZY_ASSERT(b2.length(15),   256.107788f);
    BEZIER_FUZZY_ASSERT(b2.length(20),   256.171021f);
    BEZIER_FUZZY_ASSERT(b2.length(50),   256.239044f);
    BEZIER_FUZZY_ASSERT(b2.length(100),  256.248718f);
    BEZIER_FUZZY_ASSERT(b2.length(),     256.248718f); // Same as 100 (default number of intervals)
    BEZIER_FUZZY_ASSERT(b2.length(200),  256.251160f);
    BEZIER_FUZZY_ASSERT(b2.length(500),  256.249878f);
    BEZIER_FUZZY_ASSERT(b2.length(1000), 256.248779f);

    BEZIER_FUZZY_ASSERT(b3.length(0),    0.0f);
    BEZIER_FUZZY_ASSERT(b3.length(1),    156.204987f);
    BEZIER_FUZZY_ASSERT(b3.length(2),    219.160416f);
    BEZIER_FUZZY_ASSERT(b3.length(3),    251.716125f);
    BEZIER_FUZZY_ASSERT(b3.length(5),    266.579285f);
    BEZIER_FUZZY_ASSERT(b3.length(10),   271.217773f);
    BEZIER_FUZZY_ASSERT(b3.length(15),   272.134460f);
    BEZIER_FUZZY_ASSERT(b3.length(20),   272.456390f);
    BEZIER_FUZZY_ASSERT(b3.length(50),   272.803558f);
    BEZIER_FUZZY_ASSERT(b3.length(100),  272.853027f);
    BEZIER_FUZZY_ASSERT(b3.length(),     272.853027f); // Same as 100 (default number of intervals)
    BEZIER_FUZZY_ASSERT(b3.length(200),  272.865356f);
    BEZIER_FUZZY_ASSERT(b3.length(500),  272.865540f);
    BEZIER_FUZZY_ASSERT(b3.length(1000), 272.863708f);
}

// TODO: Test splitting on more curves.
void split_tests()
{
    Bezier::Bezier<3> b3({ { 120, 160 }, { 35, 200 }, { 220, 260 }, {220, 40} });
    BEZIER_ASSERT(b3.order() == 3);

    auto split_05 = b3.split();
    BEZIER_FUZZY_ASSERT_POINT(split_05.left[0], 120.0, 160.0);
    BEZIER_FUZZY_ASSERT_POINT(split_05.left[1], 77.5, 180.0);
    BEZIER_FUZZY_ASSERT_POINT(split_05.left[2], 102.5, 205.0);
    BEZIER_FUZZY_ASSERT_POINT(split_05.left[3], 138.125, 197.5);
    BEZIER_FUZZY_ASSERT_POINT(split_05.right[0], 220.0, 40.0);
    BEZIER_FUZZY_ASSERT_POINT(split_05.right[1], 220.0, 150.0);
    BEZIER_FUZZY_ASSERT_POINT(split_05.right[2], 173.75, 190.0);
    BEZIER_FUZZY_ASSERT_POINT(split_05.right[3], 138.125, 197.5);
    BEZIER_ASSERT(split_05.left.order() == b3.order());
    BEZIER_ASSERT(split_05.right.order() == b3.order());

    auto split_08 = b3.split(0.8f);
    BEZIER_FUZZY_ASSERT_POINT(split_08.left[0], 120.0, 160.0);
    BEZIER_FUZZY_ASSERT_POINT(split_08.left[1], 52.0, 192.0);
    BEZIER_FUZZY_ASSERT_POINT(split_08.left[2], 156.8, 236.8);
    BEZIER_FUZZY_ASSERT_POINT(split_08.left[3], 201.44, 140.8);
    BEZIER_FUZZY_ASSERT_POINT(split_08.right[0], 220.0, 40.0);
    BEZIER_FUZZY_ASSERT_POINT(split_08.right[1], 220.0, 84.0);
    BEZIER_FUZZY_ASSERT_POINT(split_08.right[2], 212.6, 116.8);
    BEZIER_FUZZY_ASSERT_POINT(split_08.right[3], 201.44, 140.8);
    BEZIER_ASSERT(split_08.left.order() == b3.order());
    BEZIER_ASSERT(split_08.right.order() == b3.order());

    auto split_033 = b3.split(0.33f);
    BEZIER_FUZZY_ASSERT_POINT(split_033.left[0], 120.0, 160.0);
    BEZIER_FUZZY_ASSERT_POINT(split_033.left[1], 91.950, 173.2);
    BEZIER_FUZZY_ASSERT_POINT(split_033.left[2], 93.303, 188.578);
    BEZIER_FUZZY_ASSERT_POINT(split_033.left[3], 107.7077, 195.3529);
    BEZIER_FUZZY_ASSERT_POINT(split_033.right[0], 220.0, 40.0);
    BEZIER_FUZZY_ASSERT_POINT(split_033.right[1], 220.0, 187.4);
    BEZIER_FUZZY_ASSERT_POINT(split_033.right[2], 136.9535, 209.108);
    BEZIER_FUZZY_ASSERT_POINT(split_033.right[3], 107.7077, 195.3529);
    BEZIER_ASSERT(split_033.left.order() == b3.order());
    BEZIER_ASSERT(split_033.right.order() == b3.order());
}

void arch_point_tests()
{
    float t;
    Bezier::Point point;

    Bezier::Bezier<2> a({ {80, 20}, {50, 10}, {95, 95} });
    t = a.archMidPoint();
    point = a.valueAt(t);
    BEZIER_FUZZY_ASSERT(t, 0.71246f);
    BEZIER_FUZZY_ASSERT_POINT(point, 75.32214, 53.97224);

    Bezier::Bezier<2> b({ {70, 250}, {20, 110}, {250, 63} });
    t = b.archMidPoint();
    point = b.valueAt(t);
    BEZIER_FUZZY_ASSERT(t, 0.59447f);
    BEZIER_FUZZY_ASSERT_POINT(point, 109.50371, 116.41393);

    Bezier::Bezier<3> c({ {210, 160}, {35, 200}, {220, 260}, {220, 60} });
    t = c.archMidPoint();
    point = c.valueAt(t);
    BEZIER_FUZZY_ASSERT(t, 0.66141f);
    BEZIER_FUZZY_ASSERT_POINT(point, 177.52785, 184.60152);

    Bezier::Bezier<3> d({ {170, 90}, {100, 200}, {40, 40}, {230, 135} });
    t = d.archMidPoint();
    point = d.valueAt(t);
    BEZIER_FUZZY_ASSERT(t, 0.702915192f);
    BEZIER_FUZZY_ASSERT_POINT(point, 120.56327, 104.08348);

    // ---

    Bezier::Bezier<3> cubicBezier({ { 120, 160 }, { 35, 200 }, { 220, 260 }, {220, 40} });

    t = cubicBezier.archMidPoint();
    point = cubicBezier.valueAt(t);
    BEZIER_FUZZY_ASSERT(t, 0.70718f);
    BEZIER_FUZZY_ASSERT_POINT(point, 183.83701, 168.76790);

    // High epsilon, high max search depth.
    t = cubicBezier.archMidPoint(1e-8f, 1000);
    point = cubicBezier.valueAt(t);
    BEZIER_FUZZY_ASSERT(t, 0.70718f);
    BEZIER_FUZZY_ASSERT_POINT(point, 183.83712, 168.76769);

    // High epsilon, low max search depth. Will not be precise.
    t = cubicBezier.archMidPoint(1e-8f, 10);
    point = cubicBezier.valueAt(t);
    BEZIER_FUZZY_ASSERT(t, 0.70752f);
    BEZIER_FUZZY_ASSERT_POINT(point, 183.90683, 168.68553);

    // Low epsilon, high max search depth. Will not be precise.
    t = cubicBezier.archMidPoint(0.1f, 1000);
    point = cubicBezier.valueAt(t);
    BEZIER_FUZZY_ASSERT(t, 0.70703f);
    BEZIER_FUZZY_ASSERT_POINT(point, 183.80527, 168.80531);

    // Low epsilon, low max search depth. Will not be precise.
    t = cubicBezier.archMidPoint(0.1f, 10);
    point = cubicBezier.valueAt(t);
    BEZIER_FUZZY_ASSERT(t, 0.70703f);
    BEZIER_FUZZY_ASSERT_POINT(point, 183.80527, 168.80531);
}

// The test that exists in README.md, because it should be correct.
void readme_tests()
{
    // Create a cubic bezier with 4 points. Visualized at https://www.desmos.com/calculator/fivneeogmh
    Bezier::Bezier<3> cubicBezier({ {120, 160}, {35, 200}, {220, 260}, {220, 40} });

    // Get coordinates on the curve from a value between 0 and 1 (values outside this range are also valid because of the way bezier curves are defined).
    Bezier::Point p;
    p = cubicBezier.valueAt(0);     // (120, 60)
    BEZIER_FUZZY_ASSERT_POINT(p, 120.0, 160.0);
    p = cubicBezier.valueAt(0.5);   // (138.125, 197.5)
    BEZIER_FUZZY_ASSERT_POINT(p, 138.125, 197.5);

    // Get coordinate values for a single axis. Currently only supports 2D.
    float value;
    value = cubicBezier.valueAt(1, 0);    // 220 (x-coordinate at t = 1)
    BEZIER_FUZZY_ASSERT(value, 220.0f);
    value = cubicBezier.valueAt(0.75, 1); // 157.1875 (y-coordinate at t = 0.75)
    BEZIER_FUZZY_ASSERT(value, 157.1875f);
    value = cubicBezier.length();         // 272.85 (Arc length of the bezier curve)
    BEZIER_FUZZY_ASSERT(value, 272.853f);

    // Translate and rotate Bezier curves.
    Bezier::Bezier<3> copy = cubicBezier;
    copy.translate(10, 15);      // Translate 10 in x-direction, 15 in y-direction
    BEZIER_FUZZY_ASSERT(copy.valueAt(0, 0), 130.0f);
    BEZIER_FUZZY_ASSERT(copy.valueAt(0, 1), 175.0f);
    copy.rotate(0.5);             // Rotate 0.5 radians around the origin
    copy.rotate(3.14f, {-5, 20}); // Rotate 3.14 radians around (-5, 20)

    // Get normals along the bezier curve.
    Bezier::Normal normal = cubicBezier.normalAt(0.75); // Get normalized normal at t = 0.75. Add false as second argument to disable normalization.
    float angle = normal.angle();       // Angle in radians
    float angleDeg = normal.angleDeg(); // Angle in degrees
    BEZIER_FUZZY_ASSERT(normal.x, 0.83892852f);
    BEZIER_FUZZY_ASSERT(normal.y, 0.544241607f);
    BEZIER_FUZZY_ASSERT(angle, 0.575484872f);
    BEZIER_FUZZY_ASSERT(angleDeg, 32.9728546f);

    // Get tangents along the bezier curve.
    Bezier::Tangent tangent = cubicBezier.tangentAt(0.25); // Get normalized tangent at t = 0.25. Add false as second argument to disable normalization.
    angle = tangent.angle();       // Angle in radians
    angleDeg = tangent.angleDeg(); // Angle in degrees
    BEZIER_FUZZY_ASSERT(tangent.x, 0.567925274f);
    BEZIER_FUZZY_ASSERT(tangent.y, 0.823080122f);
    BEZIER_FUZZY_ASSERT(angle, 0.966813385f);
    BEZIER_FUZZY_ASSERT(angleDeg, 55.3943253f);

    // Get derivatives of the Bezier curve, resulting in a Bezier curve of one order less.
    Bezier::Bezier<2> db  = cubicBezier.derivative(); // First derivative
    Bezier::Bezier<1> ddb = db.derivative();          // Second derivative
    BEZIER_FUZZY_ASSERT_POINT(db[0], -255.0, 120.0);
    BEZIER_FUZZY_ASSERT_POINT(db[1], 555.0, 180.0);
    BEZIER_FUZZY_ASSERT_POINT(db[2], 0.0, -660.0);
    BEZIER_FUZZY_ASSERT_POINT(ddb[0], 1620.0, 120.0);
    BEZIER_FUZZY_ASSERT_POINT(ddb[1], -1110.0, -1680.0);

    // Get extreme values of the Bezier curves.
    Bezier::ExtremeValues xVals = cubicBezier.derivativeZero();  // Contains 3 extreme value locations: t = 0.186811984, t = 1.0 and t = 0.437850952
    BEZIER_ASSERT(xVals.size() == 3);
    BEZIER_FUZZY_ASSERT(xVals[0].t, 0.186811984f);
    BEZIER_FUZZY_ASSERT(xVals[1].t, 1.0f);
    BEZIER_FUZZY_ASSERT(xVals[2].t, 0.437850952f);
    Bezier::ExtremeValue const& xVal = xVals[0];                 // Contains t value and axis for the first extreme value
    Bezier::Point xValCoord = cubicBezier.valueAt(xVal.t);       // Get the coordinates for the first extreme value (97.6645355, 182.55565)
    BEZIER_FUZZY_ASSERT(xValCoord.x, 97.6645355f);
    BEZIER_FUZZY_ASSERT(xValCoord.y, 182.555649f);
    Bezier::ExtremePoints xPoints = cubicBezier.extremePoints(); // Or get all the extreme points directly (includes 0 and 1)
    BEZIER_FUZZY_ASSERT_POINT(xPoints[0], 97.6645355, 182.55565);
    BEZIER_FUZZY_ASSERT_POINT(xPoints[1], 220.0, 40.0);
    BEZIER_FUZZY_ASSERT_POINT(xPoints[2], 125.442337, 198.86235);
    BEZIER_FUZZY_ASSERT_POINT(xPoints[3], 120.0, 160.0);

    // Get bounding boxes of the Bezier curves.
    Bezier::AABB aabb = cubicBezier.aabb();             // Axis Aligned Bounding Box
    BEZIER_ASSERT(aabb.size() == 4);
    BEZIER_FUZZY_ASSERT(aabb.minX(), 97.6645355f);
    BEZIER_FUZZY_ASSERT(aabb.maxX(), 220.0f);
    BEZIER_FUZZY_ASSERT(aabb.minY(), 40.0f);
    BEZIER_FUZZY_ASSERT(aabb.maxY(), 198.86235f);
    BEZIER_FUZZY_ASSERT(aabb.width(), 122.335464f);
    BEZIER_FUZZY_ASSERT(aabb.height(), 158.86235f);
    BEZIER_FUZZY_ASSERT(aabb.area(), 19434.5f);
    aabb = cubicBezier.aabb(xPoints);                   // Or get from extreme points (if you already have them) to reduce calculation time
    BEZIER_ASSERT(aabb.size() == 4);
    BEZIER_FUZZY_ASSERT(aabb.minX(), 97.6645355f);
    BEZIER_FUZZY_ASSERT(aabb.maxX(), 220.0f);
    BEZIER_FUZZY_ASSERT(aabb.minY(), 40.0f);
    BEZIER_FUZZY_ASSERT(aabb.maxY(), 198.86235f);
    BEZIER_FUZZY_ASSERT(aabb.width(), 122.335464f);
    BEZIER_FUZZY_ASSERT(aabb.height(), 158.86235f);
    BEZIER_FUZZY_ASSERT(aabb.area(), 19434.5f);
    Bezier::TightBoundingBox tbb = cubicBezier.tbb();   // Tight bounding box
    BEZIER_ASSERT(tbb.size() == 4);
    BEZIER_FUZZY_ASSERT(tbb.minX(), 92.568962f);
    BEZIER_FUZZY_ASSERT(tbb.maxX(), 261.989441f);
    BEZIER_FUZZY_ASSERT(tbb.minY(), 36.2565613f);
    BEZIER_FUZZY_ASSERT(tbb.maxY(), 222.517883f);
    BEZIER_FUZZY_ASSERT(tbb.width(), 60.5054359f);
    BEZIER_FUZZY_ASSERT(tbb.height(), 192.036713f);
    BEZIER_FUZZY_ASSERT(tbb.area(), 11619.2646f);

    // Split the Bezier curve at desired points. The left and right parts are new bezier curves
    // of the same order as the original curve.
    auto split = cubicBezier.split(0.5f);
    BEZIER_FUZZY_ASSERT_POINT(split.left[0], 120.0, 160.0);
    BEZIER_FUZZY_ASSERT_POINT(split.left[1], 77.5, 180.0);
    BEZIER_FUZZY_ASSERT_POINT(split.left[2], 102.5, 205.0);
    BEZIER_FUZZY_ASSERT_POINT(split.left[3], 138.125, 197.5);
    BEZIER_FUZZY_ASSERT_POINT(split.right[0], 220.0, 40.0);
    BEZIER_FUZZY_ASSERT_POINT(split.right[1], 220.0, 150.0);
    BEZIER_FUZZY_ASSERT_POINT(split.right[2], 173.75, 190.0);
    BEZIER_FUZZY_ASSERT_POINT(split.right[3], 138.125, 197.5);
    BEZIER_ASSERT(split.left.order() == cubicBezier.order());
    BEZIER_ASSERT(split.right.order() == cubicBezier.order());
    auto left  = split.left;  // Left part of the split
    auto right = split.right; // Right part of the split

    // Find the mid point on the curve by arch length.
    float tAtMidPoint = cubicBezier.archMidPoint();
    Bezier::Point midPoint = cubicBezier.valueAt(tAtMidPoint);
    BEZIER_FUZZY_ASSERT(tAtMidPoint, 0.70718f);
    BEZIER_FUZZY_ASSERT_POINT(midPoint, 183.83701f, 168.76790f);
}

int main()
{
    printf("Starting tests.\n");

    printf("Running binomial_tests().\n");         binomial_tests();
    printf("Running polynomial_tests().\n");       polynomial_tests();
    printf("Running control_points_tests().\n");   control_points_tests();
    printf("Running values_tests().\n");           values_tests();
    printf("Running length_tests().\n");           length_tests();
    printf("Running split_tests().\n");            split_tests();
    printf("Running arch_point_tests().\n");       arch_point_tests();
    printf("Running readme_tests().\n");           readme_tests();

    printf("All tests completed.\n");
    return 0;
}

