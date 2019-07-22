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
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>

# include <sot/core/abstract-sot-external-interface.hh>

#include <boost/thread.hpp>
#include <boost/array.hpp>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;
namespace dgsot=dynamicgraph::sot;

/** \brief Config variables
 */
struct robot_config_t
{
  /// \brief Name of the controller to load
  std::string libname;
  /// \brief Robot number of DoFs
  int nb_dofs;
  /// \brief Number of force sensors
  int nb_force_sensors;
  
};

/** \brief Roll-Pitch-Yaw vector */
struct RpyVector
{
  double roll;
  double pitch;
  double yaw;
};

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
  virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  // The shutdown action when ExecutionContext stop
  // former rtc_stopping_entry()
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  // The activated action (Active state entry action)
  // former rtc_active_entry()
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

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
  /// \brief Config variables 
  robot_config_t robot_config_;

  /// \brief Starting the controller.
  bool started_;
  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedDoubleSeq m_angleEncoder;
  InPort<TimedDoubleSeq> m_angleEncoderIn;
  TimedDoubleSeq m_torques;
  InPort<TimedDoubleSeq> m_torquesIn;
  TimedDoubleSeq m_baseAtt;
  InPort<TimedDoubleSeq> m_baseAttIn;
  TimedDoubleSeq m_accelerometer_0;
  InPort<TimedDoubleSeq> m_accelerometer_0In;
  TimedDoubleSeq m_gyrometer_0;
  InPort<TimedDoubleSeq> m_gyrometer_0In;
  TimedDoubleSeq m_qInit;

  // Kalman filter position.
  TimedDoubleSeq m_rpy;
  InPort<TimedDoubleSeq> m_rpyIn;

  // Forces
  TimedDoubleSeq m_forcesLF;
  InPort<TimedDoubleSeq> m_forcesLFIn;
  TimedDoubleSeq m_forcesRF;
  InPort<TimedDoubleSeq> m_forcesRFIn;
  TimedDoubleSeq m_forcesLH;
  InPort<TimedDoubleSeq> m_forcesLHIn;
  TimedDoubleSeq m_forcesRH;
  InPort<TimedDoubleSeq> m_forcesRHIn;

  // Initial state
  TimedDoubleSeq m_angleInit;
  InPort<TimedDoubleSeq> m_angleInitIn;
  TimedDoubleSeq m_positionInit;
  InPort<TimedDoubleSeq> m_positionInitIn;
  TimedDoubleSeq m_rpyInit;
  InPort<TimedDoubleSeq> m_rpyInitIn;

  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  TimedDoubleSeq m_zmpRef;
  OutPort<TimedDoubleSeq> m_zmpRefOut;
  TimedDoubleSeq m_qRef;
  OutPort<TimedDoubleSeq> m_qRefOut;
  TimedDoubleSeq m_pRef;
  OutPort<TimedDoubleSeq> m_pRefOut;
  TimedDoubleSeq m_rpyRef;
  OutPort<TimedDoubleSeq> m_rpyRefOut;
  TimedAcceleration3D m_accRef;
  OutPort<TimedAcceleration3D> m_accRefOut;

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

  /// \brief Update force sensor fields for SoT
  void fillInForceSensor(InPort<TimedDoubleSeq> &aForcePortIn,
                         TimedDoubleSeq &aForceData,
                         std::vector<double> &lforce,
                         unsigned int index);

  /// \brief Update angles for SoT.
  void fillAngles(std::map<std::string,dgsot::SensorValues> & 
                  sensorsIn,
                  bool initPort);
  
  /// \brief From rotation to RPY
  void fromRotationToRpy(double *R, RpyVector &aRpyVector);

  /// \brief Read config variables
  void readConfig();

  /// \brief Load the parameter file and the sot library
  void loadAndStart();

  /// \brief Save all the log gathered in files.
  /// Note: This method is called in the destructor. To call it,
  /// make sure to rtexit this component (in your terminal):
  /// rtexit /your_host/sot.rtc
  void saveLog() const;

  /// \brief the sot-hrp2 controller
  dgsot::AbstractSotExternalInterface * m_sotController;

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
  /// Accelerometer information.
  std::vector<double> accelerometer_;
  /// Gyrometer information.
  std::vector<double> gyrometer_;  

  /// \brief Timestamp matching the beginning of the control
  /// loop.
  timeval t0_;
  /// \brief Timestamp matching the end of the control loop.
  timeval t1_;

  /// \brief Log time spend during control loops.
  boost::array<double, 100000> timeArray_;

  /// \brief First unfilled item in timeArray.
  unsigned int timeIndex_;
  
  RTC::Manager *manager_;

  bool initialize_library_;

  /// \brief thread used for the loading of the sot.
  /// This process cannot be realized in the control loop since it can break it
  boost::shared_ptr<boost::thread> startupThread_;
};


extern "C"
{
  DLL_EXPORT void RtcStackOfTasksInit(RTC::Manager* manager);
}

#endif // RTC_STACK_OF_TASKS_H

