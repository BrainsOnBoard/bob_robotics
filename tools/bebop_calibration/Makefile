LIBBEBOP_PATH := ../../libbebop
WITH_LIBBEBOP := 1
include ../../common/flags.mk
CXXFLAGS += -I../..

.PHONY: all clean libbebop

all: flat_trim

-include flat_trim.d

flat_trim: flat_trim.cc flat_trim.d libbebop
	$(CXX) -o flat_trim flat_trim.cc $(CXXFLAGS) $(LINK_FLAGS)

flat_trim.d: ;

libbebop:
	$(MAKE) -C $(LIBBEBOP_PATH)

clean:
	rm -f flat_trim flat_trim.d
