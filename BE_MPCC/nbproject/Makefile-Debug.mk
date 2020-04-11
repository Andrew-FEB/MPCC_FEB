#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-Linux
CND_DLIB_EXT=so
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/car.o \
	${OBJECTDIR}/cone.o \
	${OBJECTDIR}/main.o \
	${OBJECTDIR}/simulator.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=-m64
CXXFLAGS=-m64

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=../Dada/MPCC/mpcc_c_build_1/mpcc_optimizer/target/release/libmpcc_optimizer.so

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/be_mpcc
	${CP} ../Dada/MPCC/mpcc_c_build_1/mpcc_optimizer/target/release/libmpcc_optimizer.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/be_mpcc: ../Dada/MPCC/mpcc_c_build_1/mpcc_optimizer/target/release/libmpcc_optimizer.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/be_mpcc: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/be_mpcc ${OBJECTFILES} ${LDLIBSOPTIONS}

${OBJECTDIR}/car.o: car.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -Wall -I../Dada/MPCC/mpcc_c_build_1/mpcc_optimizer -I/opt/ros/melodic -std=c++14 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/car.o car.cpp

${OBJECTDIR}/cone.o: cone.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -Wall -I../Dada/MPCC/mpcc_c_build_1/mpcc_optimizer -I/opt/ros/melodic -std=c++14 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/cone.o cone.cpp

${OBJECTDIR}/main.o: main.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -Wall -I../Dada/MPCC/mpcc_c_build_1/mpcc_optimizer -I/opt/ros/melodic -std=c++14 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/main.o main.cpp

${OBJECTDIR}/simulator.o: simulator.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -Wall -I../Dada/MPCC/mpcc_c_build_1/mpcc_optimizer -I/opt/ros/melodic -std=c++14 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/simulator.o simulator.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} -r ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libmpcc_optimizer.so
	${RM} ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/be_mpcc

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
