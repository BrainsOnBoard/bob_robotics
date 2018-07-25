#!/bin/bash
# This is a simple script to build the source code documentation with doxygen.

BRDIR="$(dirname $0)"
if doxygen "$BRDIR/doxygen/Doxyfile"; then
	echo -e "
\e[32mDocumentation successfully built. You can now open $BRDIR/doxygen/html/index.html in your browser."
fi
