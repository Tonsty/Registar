TEMPLATE = app
TARGET= overlap_estimation
CONFIG += debug_and_release debug_and_release_target warn_off 
DEPENDPATH += .
include(overlap_estimation.pri)
HEADERS += ../Tang2014/graph.h \
			../Tang2014/common.h \
			../Tang2014/pairregistration.h \
			../Tang2014/globalregistration.h \
			../Tang2014/scan.h \
			../Tang2014/loop.h \
			../Tang2014/link.h \
			../Williams2001/SRoMCPS.h

SOURCES += ../Tang2014/graph.cpp \
			../Tang2014/pairregistration.cpp \
			../Tang2014/tang2014_globalregistration.cpp \
			main.cpp \
			../Tang2014/scan.cpp \
			../Tang2014/loop.cpp \
			../Tang2014/link.cpp \
			../Williams2001/SRoMCPS.cpp



 
