WITH_I2C_ROBOT := 1
WITH_MATPLOTLIBCPP := 1

# Some versions of gcc are really stubborn about letting you ignore a return
# value from certain functions, even when you want to.
#
# See: https://stackoverflow.com/questions/7271939/warning-ignoring-return-value-of-scanf-declared-with-attribute-warn-unused-r
CXXFLAGS += -Wno-unused-result
include ../../make_common/default.mk
