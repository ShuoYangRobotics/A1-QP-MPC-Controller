include_directories(${BEZIER_INCLUDE_DIR})
set(BEZIER "${BEZIER_INCLUDE_DIR}/bezier.h")

add_executable(basic-tests basic-tests.cpp assert.h test_beziers.h ${BEZIER})
add_executable(derivative-tests derivative-tests.cpp assert.h test_beziers.h ${BEZIER})

add_test(NAME basic-tests COMMAND basic-tests)
add_test(NAME derivative-tests COMMAND derivative-tests)

