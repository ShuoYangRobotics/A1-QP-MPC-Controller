#pragma once
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cassert>

#include <bezier.h>

constexpr float BEZIER_FUZZY_ASSERT_EPSILON = 0.0001f;

void BEZIER_ASSERT(bool x)
{
    static int exitCode = 1;
    exitCode++;
    if (!x)
    {
        assert(false);
        printf("BEZIER_ASSERT failed! Exit code = %d\n.", exitCode);
        exit(exitCode);
    }
}

template<size_t N>
void BEZIER_ASSERT_BINOMIAL_COEFFICIENT(int pos, int val)
{
    BEZIER_ASSERT(Bezier::Bezier<N>::binomialCoefficients[pos] == val);
}

template<size_t N>
void BEZIER_ASSERT_POLYNOMIAL_COEFFICIENTS(int pos, int a, int b)
{
    BEZIER_ASSERT(Bezier::Bezier<N>::polynomialCoefficients[pos].one_minus_t  == a);
    BEZIER_ASSERT(Bezier::Bezier<N>::polynomialCoefficients[pos].t            == b);
}

template<typename T>
void BEZIER_FUZZY_ASSERT(T val, T correctVal)
{
    printf("Called BEZIER_FUZZY_ASSERT with non-implemented type.\n");
    assert(false);
    exit(-1);
}

template<>
void BEZIER_FUZZY_ASSERT(float val, float correctVal)
{
    printf("BEZIER_FUZZY_ASSERT: %.6f (Correct value = %.6f)\n", val, correctVal);
    BEZIER_ASSERT(val <= correctVal + BEZIER_FUZZY_ASSERT_EPSILON);
    BEZIER_ASSERT(val >= correctVal - BEZIER_FUZZY_ASSERT_EPSILON);
}

template<>
void BEZIER_FUZZY_ASSERT(double val, double correctVal)
{
    printf("BEZIER_FUZZY_ASSERT: %.6f (Correct value = %.6f)\n", val, correctVal);
    BEZIER_ASSERT(val <= correctVal + BEZIER_FUZZY_ASSERT_EPSILON);
    BEZIER_ASSERT(val >= correctVal - BEZIER_FUZZY_ASSERT_EPSILON);
}

template<typename T>
void BEZIER_FUZZY_ASSERT_POINT(const Bezier::Point &point, T x2, T y2)
{
    printf("Called BEZIER_FUZZY_ASSERT_POINT with non-implemented type.\n");
    assert(false);
    exit(-2);
}

template<>
void BEZIER_FUZZY_ASSERT_POINT(const Bezier::Point &point, float x2, float y2)
{
    BEZIER_FUZZY_ASSERT(point.x, x2);
    BEZIER_FUZZY_ASSERT(point.y, y2);
}

template<>
void BEZIER_FUZZY_ASSERT_POINT(const Bezier::Point &point, double x2, double y2)
{
    BEZIER_FUZZY_ASSERT(point.x, float(x2));
    BEZIER_FUZZY_ASSERT(point.y, float(y2));
}

