#!/bin/sh

# PRISM directory to link to
if [ "$PRISM_DIR" = "" ]; then
	PRISM_DIR="/home/fatma/Data/PhD/code/prism_ws/prism-svn-2/prism-svn/prism"
fi

# Class to run
if [ "$PRISM_MAINCLASS" = "" ]; then
	PRISM_MAINCLASS=demos.testMATHTS
fi

# Set up CLASSPATH:
# We look in both the top-level and the prism sub-directory
# (currently svn/git repos and downloaded distributions differ in structure)
PRISM_CLASSPATH=classes:"$PRISM_DIR":"$PRISM_DIR"/classes:"$PRISM_DIR"/lib/*:"$PRISM_DIR"/prism:"$PRISM_DIR"/prism/classes:"$PRISM_DIR"/prism/lib/*

# Set up pointers to libraries
# As above, we look in both the top-level and the prism sub-directory
PRISM_LIB_PATH="$PRISM_DIR"/lib:"$PRISM_DIR"/prism/lib
if [[ "$OSTYPE" == "darwin"* ]]; then
	export DYLD_LIBRARY_PATH="$PRISM_LIB_PATH"
else
	export LD_LIBRARY_PATH="$PRISM_LIB_PATH"
fi

# Command to launch Java
if [ "$PRISM_JAVA" = "" ]; then
	# On OS X, we want to avoiding calling java from the /usr/bin link
	# since it causes problems with dynamic linking (DYLD_LIBRARY_PATH)
	if [ -x /usr/libexec/java_home ]; then
		PRISM_JAVA=`/usr/libexec/java_home`"/bin/java"
	else
		PRISM_JAVA=java
	fi
fi

# Run PRISM through Java
"$PRISM_JAVA" -Djava.library.path="$PRISM_LIB_PATH" -classpath "$PRISM_CLASSPATH" "-Xss50m" $PRISM_MAINCLASS "$@"
