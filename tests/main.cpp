#include <cstdio>

#include "tests.h"

#define TESTS(RC_VAR) \
  TEST_GEN(test_events, RC_VAR) \
  TEST_GEN(test_serial_handler, RC_VAR)

#define TEST_GEN(FUNC, RC_VAR) \
  RC_VAR = FUNC(); \
  if (RC_VAR == 0) { printf(#FUNC " passed!\n"); } \
  else { printf(#FUNC " failed (%d)\n", RC_VAR); }

int main() {
  int rc = 0;
  TESTS(rc)
}