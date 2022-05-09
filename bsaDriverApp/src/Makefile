TOP=../..

include $(TOP)/configure/CONFIG

USR_CXXFLAGS += -DHAVE_YAML

#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#========================================

INCLUDES += $(addprefix -I,$(YAML_INCLUDE))
INCLUDES += $(addprefix -I,$(BOOST_INCLUDE))

## this part is just for yaml loader driver
INCLUDES += $(addprefix -I,$(CPSW_FRAMEWORK_INCLUDE))

INCLUDES += $(addprefix -I,$(BSA_INCLUDE))

INC += bsaAsynDriver.h
INC += bsssAsynDriver.h
INC += bsasAsynDriver.h
DBD += bsaAsynDriver.dbd


LIBRARY_IOC = bsaDriver

bsaDriver_DBD += bsaAsynDriver.dbd
bsaDriver_SRCS += bldStream.cpp
bsaDriver_SRCS += bsaAsynDriver.cpp
bsaDriver_SRCS += bsssAsynDriver.cpp
bsaDriver_SRCS += bsasAsynDriver.cpp

cpsw_DIR = $(CPSW_FRAMEWORK_LIB)
bsaDriver_LIBS += cpsw

bsa_DIR = $(BSA_LIB)
bsaDriver_LIBS += bsa

bsaDriver_LIBS += asyn
bsaDriver_LIBS += yamlLoader
bsaDriver_LIBS += pvxs
bsaDriver_LIbS += pvxsIoc
bsaDriver_LIBS += $(EPICS_BASE_IOC_LIBS)





#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
#
