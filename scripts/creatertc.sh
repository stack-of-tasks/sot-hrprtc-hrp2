#!/bin/bash

# Create the component for Stack-Of-Tasks 
rm -f Makefile. README. user_config.vsprops *vc* Comp* copyprops.bat .h .cpp .yaml

# Input ports
# q
# rpy
# torque
# forceLF
# forceRF
# forceLH
# forceRH

# Output ports
# zmpRef
# qRef 
# accRef

/opt/grx/bin/rtc-template -bcxx \
--module-name='rtc-stack-of-tasks' \
--module-desc='Module for controlling humanoid robot' \
--module-version='0.1' \
--module-vendor='LAAS, CNRS UPR 8001', \
--module-category=Generic \
--module-comp-type=DataFlowComponent \
--module-act-type=PERIODIC \
--module-max-inst=10 \
--inport=RTC:TimedDoubleSeq \
--inport=RTC:TimedDoubleSeq \
--inport=RTC:TimedDoubleSeq \
--inport=RTC:TimedDoubleSeq \
--inport=RTC:TimedDoubleSeq \
--inport=RTC:TimedDoubleSeq \
--inport=RTC:TimedDoubleSeq \
--inport=RTC:TimedDoubleSeq \
--outport=RTC:TimedDoubleSeq \
--outport=RTC:TimedDoubleSeq \
--outport=RTC:TimedDoubleSeq \

 
##--consumer=Control:StackOfTasks\
##--consumer-idl=OGMap3DService.idl

# --service=AskAPlan:askaplan0:FootStepPlanner \
# --service-idl=FootStepPlanner.idl \
# --inport=VoxelMap:OpenHRP::OGMap3D \
