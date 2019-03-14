# Dependencies
WITH_GTEST := 1
WITH_EIGEN := 1

# We need to generate a header
EXTRA_DEPS := test_headers

# For build speed
DEBUG := 1

WITH_MATPLOTLIBCPP := 1

include ../make_common/default.mk

test_headers:
	$(shell sh make_tests_include_header.sh)
