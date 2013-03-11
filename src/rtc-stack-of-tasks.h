// -*- C++ -*-
/*!
 * @file  rtc-stack-of-tasks.h * @brief Module for controlling humanoid robot * @date  $Date$ 
 *
 * $Id$ 
 */
#ifndef RTC_STACK_OF_TASKS_H
#define RTC_STACK_OF_TASKS_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

# include <sot/core/abstract-sot-external-interface.hh>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;
namespace dgsot=dynamicgraph::sot;

class RtcStackOfTasks  : public RTC::DataFlowComponentBase
{
 public:
  RtcStackOfTasks(RTC::Manager* manager);
  ~RtcStackOfTasks();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry() 
 virtual RTC::ReturnCode_t onInitialize();

  // The finalize action (on ALIVE->END transition)
  // formaer rtc_exiting_entry()
  // virtual RTC::ReturnCode_t onFinalize();

  // The startup action when ExecutionContext startup
  // former rtc_starting_entry()
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  // The shutdown action when ExecutionContext stop
  // former rtc_stopping_entry()
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  // The activated action (Active state entry action)
  // former rtc_active_entry()
  // virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  // The deactivated action (Active state exit action)
  // former rtc_active_exit()
  // virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  // The execution action that is invoked periodically
  // former rtc_active_do()
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  // The aborting action when main logic error occurred.
  // former rtc_aborting_entry()
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  // The error action in ERROR state
  // former rtc_error_do()
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  // The reset action that is invoked resetting
  // This is same but different the former rtc_init_entry()
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  // The state update action that is invoked after onExecute() action
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  // The action that is invoked when execution context's rate is changed
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);

  void fillSensors(std::map<std::string,dgsot::SensorValues> &sensorsIn);

 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedDoubleSeq m_angleEncoder;
  InPort<TimedDoubleSeq> m_angleEncoderIn;
  TimedDoubleSeq m_torques;
  InPort<TimedDoubleSeq> m_torquesIn;
  TimedDoubleSeq m_baseAtt;
  InPort<TimedDoubleSeq> m_baseAttIn;
  TimedDoubleSeq m_rpy;
  InPort<TimedDoubleSeq> m_rpyIn;
  TimedDoubleSeq m_forcesLF;
  InPort<TimedDoubleSeq> m_forcesLFIn;
  TimedDoubleSeq m_forcesRF;
  InPort<TimedDoubleSeq> m_forcesRFIn;
  TimedDoubleSeq m_forcesLH;
  InPort<TimedDoubleSeq> m_forcesLHIn;
  TimedDoubleSeq m_forcesRH;
  InPort<TimedDoubleSeq> m_forcesRHIn;

  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  TimedDoubleSeq m_zmpRef;
  OutPort<TimedDoubleSeq> m_zmpRefOut;
  TimedDoubleSeq m_qRef;
  OutPort<TimedDoubleSeq> m_qRefOut;
  TimedDoubleSeq m_accRef;
  OutPort<TimedDoubleSeq> m_accRefOut;

  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">

  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">

  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">

  // </rtc-template>

  /// \brief Load the overall SoT structure.
  void LoadSot();

  /// \brief Capture the current time in the argument.
  /// \param t store current time
  void captureTime (timeval& t);

  /// \brief Log time spent between t0 and t1 in the next
  /// free case of timeArray.
  ///
  /// \param t0 begin time
  /// \param t1 end time
  void logTime (const timeval& t0, const timeval& t1);

  /// \brief Write logged times in a file.
  void writeLog ();
  
  void readControl(std::map<std::string,dgsot::ControlValues> &controlValues);

 private:

  /// \brief the sot-hrp2 controller
  dgsot::AbstractSotExternalInterface * m_sotController;

  /// \brief Name of the controller to load
  std::string libname_;
  /// Map of sensor readings
  std::map<std::string,dgsot::SensorValues> sensorsIn_;
  /// Map of control values
  std::map<std::string,dgsot::ControlValues> controlValues_;
  /// Angular values read by encoders
  std::vector <double> angleEncoder_;
  /// Angular values sent to motors
  std::vector<double> angleControl_;
  /// Forces read by force sensors
  std::vector<double> forces_;
  /// Torques
  std::vector<double> torques_;
  /// Attitude of the robot computed by extended Kalman filter.
  std::vector<double> baseAtt_;

  /// \brief Timestamp matching the beginning of the control
  /// loop.
  timeval t0_;
  /// \brief Timestamp matching the end of the control loop.
  timeval t1_;

  /// \brief Size of the array logging time spent in control loop.
  static const unsigned int TIME_ARRAY_SIZE = 100000;
  
  /// \brief Log time spend during control loops.
  double timeArray_[TIME_ARRAY_SIZE];
  /// \brief First unfilled item in timeArray.
  unsigned int timeIndex_;
  
};


extern "C"
{
  DLL_EXPORT void RtcStackOfTasksInit(RTC::Manager* manager);
};

#endif // RTC_STACK_OF_TASKS_H

