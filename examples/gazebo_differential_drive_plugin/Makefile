WITH_GAZEBO := 1
include ../../make_common/bob_robotics.mk

NAME := differential_drive_plugin

.PHONY: all clean

all: lib$(NAME).so 

-include lib$(NAME).d

lib$(NAME).so: $(NAME).cc lib$(NAME).d
	$(CXX) -o $@ -shared -fPIC $< $(CXXFLAGS) $(LINK_FLAGS)

%.d: ;

clean:
	rm -f lib$(NAME).so 
