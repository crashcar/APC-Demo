#
# OMNeT++/OMNEST Makefile for $(LIB_PREFIX)VENTOS
#
# This file was generated with the command:
#  opp_makemake --make-so -f --deep -O out -I. -I../libs/exprtk -I../libs/UFT -I../libs/rapidxml-1.13 -lboost_system -lboost_filesystem -lboost_serialization -lcurl -lshark_debug -lblas -lgmssl -lcrypto -lipopt -d loggingWindow -XloggingWindow
#

# Name of target to be created (-o option)
TARGET = $(LIB_PREFIX)VENTOS$(D)$(SHARED_LIB_SUFFIX)
TARGET_DIR = .

# C++ include paths (with -I)
INCLUDE_PATH = -I. -I../libs/exprtk -I../libs/UFT -I../libs/rapidxml-1.13

# Additional object and library files to link with
EXTRA_OBJS =

# Additional libraries (-L, -l options)
LIBS =  -lboost_system -lboost_filesystem -lboost_serialization -lcurl -lshark_debug -lblas -lgmssl -lcrypto -lipopt

# Output directory
PROJECT_OUTPUT_DIR = ../out
PROJECTRELATIVE_PATH = src
O = $(PROJECT_OUTPUT_DIR)/$(CONFIGNAME)/$(PROJECTRELATIVE_PATH)

# Object files for local .cc, .msg and .sm files
OBJS = \
    $O/MIXIM_veins/annotations/AnnotationDummy.o \
    $O/MIXIM_veins/annotations/AnnotationManager.o \
    $O/MIXIM_veins/connectionManager/BaseConnectionManager.o \
    $O/MIXIM_veins/connectionManager/ConnectionManager.o \
    $O/MIXIM_veins/connectionManager/NicEntryDebug.o \
    $O/MIXIM_veins/connectionManager/NicEntryDirect.o \
    $O/MIXIM_veins/nic/mac/Mac1609_4.o \
    $O/MIXIM_veins/nic/mac/Mac1609_4_EDCA.o \
    $O/MIXIM_veins/nic/mac/NetwToMacControlInfo.o \
    $O/MIXIM_veins/nic/phy/ChannelAccess.o \
    $O/MIXIM_veins/nic/phy/ChannelInfo.o \
    $O/MIXIM_veins/nic/phy/ChannelState.o \
    $O/MIXIM_veins/nic/phy/MappingBase.o \
    $O/MIXIM_veins/nic/phy/MappingUtils.o \
    $O/MIXIM_veins/nic/phy/PhyLayer80211p.o \
    $O/MIXIM_veins/nic/phy/PhyUtils.o \
    $O/MIXIM_veins/nic/phy/analogueModel/BreakpointPathlossModel.o \
    $O/MIXIM_veins/nic/phy/analogueModel/JakesFading.o \
    $O/MIXIM_veins/nic/phy/analogueModel/LogNormalShadowing.o \
    $O/MIXIM_veins/nic/phy/analogueModel/NakagamiFading.o \
    $O/MIXIM_veins/nic/phy/analogueModel/PERModel.o \
    $O/MIXIM_veins/nic/phy/analogueModel/SimpleObstacleShadowing.o \
    $O/MIXIM_veins/nic/phy/analogueModel/SimplePathlossModel.o \
    $O/MIXIM_veins/nic/phy/analogueModel/TwoRayInterferenceModel.o \
    $O/MIXIM_veins/nic/phy/decider/BaseDecider.o \
    $O/MIXIM_veins/nic/phy/decider/Decider80211p.o \
    $O/MIXIM_veins/nic/phy/decider/NistErrorRate.o \
    $O/MIXIM_veins/nic/phy/decider/SNRThresholdDecider.o \
    $O/MIXIM_veins/obstacle/Obstacle.o \
    $O/MIXIM_veins/obstacle/ObstacleControl.o \
    $O/addNode/AddNode.o \
    $O/addNode/gui.o \
    $O/addNode/TrafficControl.o \
    $O/addNode/xmlUtil.o \
    $O/baseAppl/01_BaseModule.o \
    $O/baseAppl/02_BaseLayer.o \
    $O/baseAppl/03_BaseApplLayer.o \
    $O/baseAppl/04_BaseWaveApplLayer.o \
    $O/baseAppl/ApplToPhyControlInfo.o \
    $O/gettingStarted/tutorial.o \
    $O/global/BaseWorldUtility.o \
    $O/global/Color.o \
    $O/global/gnuplot.o \
    $O/global/LoopDetectors.o \
    $O/global/Statistics.o \
    $O/global/utility.o \
    $O/logging/vglog.o \
    $O/logging/vlog.o \
    $O/mobility/BaseMobility.o \
    $O/mobility/ConstSpeedMobility.o \
    $O/mobility/Coord.o \
    $O/mobility/LinearMobility.o \
    $O/mobility/TraCIMobility.o \
    $O/nodes/rsu/01_Beacon.o \
    $O/nodes/rsu/02_Monitor.o \
    $O/nodes/rsu/03_Classify.o \
    $O/nodes/rsu/04_AID.o \
    $O/nodes/rsu/05_CRL.o \
    $O/nodes/rsu/06_Intersection.o \
    $O/nodes/rsu/07_CA.o \
    $O/nodes/rsu/Manager.o \
    $O/nodes/rsu/optimizeSpeedAcc.o \
    $O/nodes/vehicle/01_Beacon.o \
    $O/nodes/vehicle/02_DynamicRouting.o \
    $O/nodes/vehicle/03_AID.o \
    $O/nodes/vehicle/04_Platoon.o \
    $O/nodes/vehicle/05_PlatoonMg.o \
    $O/nodes/vehicle/Manager.o \
    $O/router/Edge.o \
    $O/router/EdgeCosts.o \
    $O/router/Hypertree.o \
    $O/router/Net.o \
    $O/router/Node.o \
    $O/router/Router.o \
    $O/router/Vehicle.o \
    $O/traci/TraCIBuffer.o \
    $O/traci/TraCICommands.o \
    $O/traci/TraCIConnection.o \
    $O/traci/TraCIScenarioManagerInet.o \
    $O/traci/TraCIStart.o \
    $O/trafficLight/01_Base.o \
    $O/trafficLight/02_IntersectionQueue.o \
    $O/trafficLight/03_IntersectionDemand.o \
    $O/trafficLight/04_IntersectionDelay.o \
    $O/trafficLight/05_AllowedMoves.o \
    $O/trafficLight/Manager.o \
    $O/trafficLight/TSC/00_TLStateRecord.o \
    $O/trafficLight/TSC/01_Fixed.o \
    $O/trafficLight/TSC/02_Adaptive_Webster.o \
    $O/trafficLight/TSC/03_TrafficActuated.o \
    $O/trafficLight/TSC/04_LQF_NoStarv.o \
    $O/trafficLight/TSC/05_OJF.o \
    $O/trafficLight/TSC/06_LQF_MWM.o \
    $O/trafficLight/TSC/07_LQF_MWM_Aging.o \
    $O/trafficLight/TSC/08_FMSC.o \
    $O/trafficLight/TSC/09_Router.o \
    $O/msg/AirFrame_m.o \
    $O/msg/AirFrame11p_m.o \
    $O/msg/BeaconBicycle_m.o \
    $O/msg/BeaconObstacle_m.o \
    $O/msg/BeaconPedestrian_m.o \
    $O/msg/BeaconRSU_m.o \
    $O/msg/BeaconVehicle_m.o \
    $O/msg/BorderMsg_m.o \
    $O/msg/CRL_Piece_m.o \
    $O/msg/dataMsg_m.o \
    $O/msg/DummyMsg_m.o \
    $O/msg/KeyMsg_m.o \
    $O/msg/LaneChangeMsg_m.o \
    $O/msg/Mac80211Pkt_m.o \
    $O/msg/MacPkt_m.o \
    $O/msg/MacToPhyCSR_m.o \
    $O/msg/PhyToMacReport_m.o \
    $O/msg/PlatoonMsg_m.o \
    $O/msg/PltCtrl_m.o \
    $O/msg/PltInfo_m.o \
    $O/msg/RouterMsg_m.o \
    $O/msg/SystemMsg_m.o \
    $O/msg/WaveShortMessage_m.o

# Message files
MSGFILES = \
    msg/AirFrame.msg \
    msg/AirFrame11p.msg \
    msg/BeaconBicycle.msg \
    msg/BeaconObstacle.msg \
    msg/BeaconPedestrian.msg \
    msg/BeaconRSU.msg \
    msg/BeaconVehicle.msg \
    msg/BorderMsg.msg \
    msg/CRL_Piece.msg \
    msg/dataMsg.msg \
    msg/DummyMsg.msg \
    msg/KeyMsg.msg \
    msg/LaneChangeMsg.msg \
    msg/Mac80211Pkt.msg \
    msg/MacPkt.msg \
    msg/MacToPhyCSR.msg \
    msg/PhyToMacReport.msg \
    msg/PlatoonMsg.msg \
    msg/PltCtrl.msg \
    msg/PltInfo.msg \
    msg/RouterMsg.msg \
    msg/SystemMsg.msg \
    msg/WaveShortMessage.msg

# SM files
SMFILES =

#------------------------------------------------------------------------------

# Pull in OMNeT++ configuration (Makefile.inc)

ifneq ("$(OMNETPP_CONFIGFILE)","")
CONFIGFILE = $(OMNETPP_CONFIGFILE)
else
ifneq ("$(OMNETPP_ROOT)","")
CONFIGFILE = $(OMNETPP_ROOT)/Makefile.inc
else
CONFIGFILE = $(shell opp_configfilepath)
endif
endif

ifeq ("$(wildcard $(CONFIGFILE))","")
$(error Config file '$(CONFIGFILE)' does not exist -- add the OMNeT++ bin directory to the path so that opp_configfilepath can be found, or set the OMNETPP_CONFIGFILE variable to point to Makefile.inc)
endif

include $(CONFIGFILE)

# Simulation kernel and user interface libraries
OMNETPP_LIBS = -loppenvir$D $(KERNEL_LIBS) $(SYS_LIBS)

COPTS = $(CFLAGS) $(IMPORT_DEFINES)  $(INCLUDE_PATH) -I$(OMNETPP_INCL_DIR)
MSGCOPTS = $(INCLUDE_PATH)
SMCOPTS =

# we want to recompile everything if COPTS changes,
# so we store COPTS into $COPTS_FILE and have object
# files depend on it (except when "make depend" was called)
COPTS_FILE = $O/.last-copts
ifneq ("$(COPTS)","$(shell cat $(COPTS_FILE) 2>/dev/null || echo '')")
$(shell $(MKPATH) "$O" && echo "$(COPTS)" >$(COPTS_FILE))
endif

#------------------------------------------------------------------------------
# User-supplied makefile fragment(s)
# >>>
# inserted from file 'makefrag':
CFLAGS += -fopenmp
LDFLAGS +=  -fopenmp

# <<<
#------------------------------------------------------------------------------

# Main target
all: $(TARGET_DIR)/$(TARGET)

$(TARGET_DIR)/% :: $O/%
	@mkdir -p $(TARGET_DIR)
	$(Q)$(LN) $< $@
ifeq ($(TOOLCHAIN_NAME),clangc2)
	$(Q)-$(LN) $(<:%.dll=%.lib) $(@:%.dll=%.lib)
endif

$O/$(TARGET): $(OBJS) submakedirs $(wildcard $(EXTRA_OBJS)) Makefile $(CONFIGFILE)
	@$(MKPATH) $O
	@echo Creating shared library: $@
	$(Q)$(SHLIB_LD) -o $O/$(TARGET) $(OBJS) $(EXTRA_OBJS) $(AS_NEEDED_OFF) $(WHOLE_ARCHIVE_ON) $(LIBS) $(WHOLE_ARCHIVE_OFF) $(OMNETPP_LIBS) $(LDFLAGS)
	$(Q)$(SHLIB_POSTPROCESS) $O/$(TARGET)

submakedirs:  loggingWindow_dir

.PHONY: all clean cleanall depend msgheaders smheaders  loggingWindow
loggingWindow: loggingWindow_dir

loggingWindow_dir:
	cd loggingWindow && $(MAKE) all

.SUFFIXES: .cc

$O/%.o: %.cc $(COPTS_FILE) | msgheaders smheaders
	@$(MKPATH) $(dir $@)
	$(qecho) "$<"
	$(Q)$(CXX) -c $(CXXFLAGS) $(COPTS) -o $@ $<

%_m.cc %_m.h: %.msg
	$(qecho) MSGC: $<
	$(Q)$(MSGC) -s _m.cc -MD -MP -MF $O/$(basename $@).d $(MSGCOPTS) $?

%_sm.cc %_sm.h: %.sm
	$(qecho) SMC: $<
	$(Q)$(SMC) -c++ -suffix cc $(SMCOPTS) $?

msgheaders: $(MSGFILES:.msg=_m.h)
	$(Q)cd loggingWindow && $(MAKE) msgheaders

smheaders: $(SMFILES:.sm=_sm.h)
	$(Q)cd loggingWindow && $(MAKE) smheaders

clean:
	$(qecho) Cleaning $(TARGET)
	$(Q)-rm -rf $O
	$(Q)-rm -f $(TARGET_DIR)/$(TARGET)
	$(Q)-rm -f $(TARGET_DIR)/$(TARGET:%.dll=%.lib)
	$(Q)-rm -f $(call opp_rwildcard, . , *_m.cc *_m.h *_sm.cc *_sm.h)
	-$(Q)cd loggingWindow && $(MAKE) clean

cleanall:
	$(Q)$(MAKE) -s clean MODE=release
	$(Q)$(MAKE) -s clean MODE=debug
	$(Q)-rm -rf $(PROJECT_OUTPUT_DIR)

# include all dependencies
-include $(OBJS:%.o=%.d)
