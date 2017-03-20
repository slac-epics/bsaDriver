TOP=../..

include $(TOP)/configure/CONFIG

USR_CXXFLAGS += -std=c++0x
USR_CXXFLAGS += -DHAVE_YAML

#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#========================================

INCLUDES += -I$(YAML_INCLUDE)
INCLUDES += -I$(BOOST_INCLUDE)

## this part is just for yaml loader driver
INCLUDES += -I$(CPSW_FRAMEWORK_INCLUDE)

INCLUDES += -I$(BSA_INCLUDE)

INC += bsaAsynDriver.h
DBD += bsaAsynDriver.dbd


LIBRARY_IOC = bsaDriver

bsaDriver_DBD += bsaAsynDriver.dbd
bsaDriver_SRCS += bsaAsynDriver.cpp

boost_DIR = $(BOOST_LIB)
bsaDriver_LIBS += boost


yaml-cpp_DIR += $(YAML_LIB)
bsaDriver_LIBS += yaml-cpp

cpsw_DIR = $(CPSW_FRAMEWORK_LIB)
bsaDriver_LIBS += cpsw

bsa_DIR = $(BSA_LIB)
bsaDriver_LIBS += bsa

bsaDriver_LIBS += asyn
bsaDriver_LIBS += yamlLoader
bsaDriver_LIBS += $(EPICS_BASE_IOC_LIBS)





#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
#
