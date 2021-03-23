TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

LIBRARY_RTEMS = sis3316
DBD += sis3316.dbd


sis3316_SRCS += sis3316Port.cpp
sis3316_SRCS += sis3316Ops.cpp
sis3316_SRCS += sis3316_adc.cpp

#sis3316_LIBS += pvDatabase pvAccess pvAccessCA pvData nt Com
sis3316_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

