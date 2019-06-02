WITH_GAZEBO := 1
WITH_MATPLOTLIBCPP := 1
include ../../make_common/bob_robotics.mk

PLUGIN_NAME := differential_drive_plugin
EXECUTABLE_NAME := simulated_tank_gazebo
SOURCES     := $(EXECUTABLE_NAME).cc
EXECUTABLES := $(foreach exec,$(basename $(SOURCES)),$(OBJECT_PATH)$(exec))
DEPS        := $(foreach dep,$(basename $(SOURCES)),$(OBJECT_PATH)$(dep).d)

.PHONY: all clean $(EXTRA_DEPS)

all: lib$(PLUGIN_NAME).so $(EXECUTABLES)

-include lib$(PLUGIN_NAME).d $(DEPS)

lib$(PLUGIN_NAME).so: $(PLUGIN_NAME).cc lib$(PLUGIN_NAME).d
	$(CXX) -o $@ -shared -fPIC $< $(CXXFLAGS) $(LINK_FLAGS)

$(EXECUTABLES): %: %.cc $(DEPS) $(EXTRA_DEPS)
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

%.d: ;

clean:
	rm -f lib$(PLUGIN_NAME).so lib$(PLUGIN_NAME).d $(EXECUTABLES) $(DEPS)