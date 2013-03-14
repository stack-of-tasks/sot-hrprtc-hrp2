#!/bin/bash

lhostname=`hostname`

echo $lhostname

rtstop /localhost/$lhostname.host_cxt/RtcStackOfTasks0.rtc


