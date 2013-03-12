// -*- C++ -*-
/*!
 * @file  RtcStackOfTasks.cpp * @brief Module for controlling humanoid robot * $Date$ 
 *
 * $Id$ 
 */
#include "rtc-stack-of-tasks.h"
#include <dlfcn.h>

#if DEBUG
#define ODEBUG(x) std::cout << x << std::endl
#else
#define ODEBUG(x)
#endif
#define ODEBUG3(x) std::cout << x << std::endl

// Module specification
// <rtc-template block="module_spec">
static const char* RtcStackOfTasks_spec[] =
  {
    "implementation_id", "RtcStackOfTasks",
    "type_name",         "RtcStackOfTasks",
    "description",       "Module for controlling humanoid robot",
    "version",           "0.1",
    "vendor",            "LAAS, CNRS UPR 8001,",
    "category",          "Generic",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };
// </rtc-template>

RtcStackOfTasks::RtcStackOfTasks(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_angleEncoderIn("angleEncoder", m_angleEncoder),
    m_torquesIn("torques", m_torques),
    m_baseAttIn("baseAtt", m_baseAtt),
    m_rpyIn("rpy", m_rpy),
    m_forcesLFIn("forcesLF", m_forcesLF),
    m_forcesRFIn("forcesRF", m_forcesRF),
    m_forcesLHIn("forcesLH", m_forcesLH),
    m_forcesRHIn("forcesRH", m_forcesRH),
    m_zmpRefOut("zmpRef", m_zmpRef),
    m_qRefOut("qRef", m_qRef),
    m_accRefOut("accRef", m_accRef)

    // </rtc-template>
{
  coil::Properties config = manager->getConfig();
  libname_ = config.getProperty("sot.libname");
  ODEBUG3("The library to be loaded: " << libname_) ;
}

RtcStackOfTasks::~RtcStackOfTasks()
{
}

void RtcStackOfTasks::LoadSot()
{
  ODEBUG3("LoadSot - Start");
  // Load the SotHRP2Controller library.
  void * SotHRP2ControllerLibrary = dlopen(libname_.c_str(),
                                           RTLD_GLOBAL | RTLD_NOW);
  if (!SotHRP2ControllerLibrary) {
    std::cerr << "Cannot load library: " << dlerror() << '\n';
    return ;
  }
  
  // reset errors
  dlerror();
  
  // Load the symbols.
  createSotExternalInterface_t * createHRP2Controller =
    (createSotExternalInterface_t *) dlsym(SotHRP2ControllerLibrary, 
                                           "createSotExternalInterface");
  const char* dlsym_error = dlerror();
  if (dlsym_error) {
    std::cerr << "Cannot load symbol create: " << dlsym_error << '\n';
    return ;
  }
  
  destroySotExternalInterface_t * destroyHRP2Controller =
    (destroySotExternalInterface_t *) dlsym(SotHRP2ControllerLibrary, 
                                            "destroySotExternalInterface");
  dlsym_error = dlerror();
  if (dlsym_error) {
    std::cerr << "Cannot load symbol create: " << dlsym_error << '\n';
    return ;
  }
  
  // Create hrp2-controller
  m_sotController = createHRP2Controller();

  ODEBUG3("LoadSot - End");  
}
RTC::ReturnCode_t RtcStackOfTasks::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("angleEncoder", m_angleEncoderIn);
  addInPort("torques", m_torquesIn);
  addInPort("baseAtt", m_baseAttIn);
  addInPort("rpy", m_rpyIn);
  addInPort("forcesLF", m_forcesLFIn);
  addInPort("forcesRF", m_forcesRFIn);
  addInPort("forcesLH", m_forcesLHIn);
  addInPort("forcesRH", m_forcesRHIn);

  // Set OutPort buffer
  addOutPort("zmpRef", m_zmpRefOut);
  addOutPort("qRef", m_qRefOut);
  addOutPort("accRef", m_accRefOut);
  
  // Load Stack of Tasks.
  LoadSot();

  // Set service provider to Ports

  // Set service consumers to Ports

  // Set CORBA Service Ports

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  // </rtc-template>
  return RTC::RTC_OK;
}

void 
RtcStackOfTasks::fillSensors(std::map<std::string,dgsot::SensorValues> & sensorsIn)
{
  // Update joint values.
  if (m_angleEncoderIn.isNew())
    {
      m_angleEncoderIn.read();
      
      sensorsIn["joints"].setName("angle");
      for(unsigned int i=0;i<m_angleEncoder.data.length();i++)
        angleEncoder_[i] = m_angleEncoder.data[i];
      sensorsIn["joints"].setValues(angleEncoder_);
    }
  
  // Update forces
  sensorsIn["forces"].setName("force");
  unsigned int i=0;
  if (m_forcesLFIn.isNew())
    {
      m_forcesLFIn.read();
      
      for (unsigned int j = 0; j < m_forcesLF.data.length(); ++j)
        forces_[i*6+j] = m_forcesLF.data[j];
    }
  
  i=1;
  if (m_forcesRFIn.isNew())
    {
      m_forcesRFIn.read();
      
      for (unsigned int j = 0; j < m_forcesRF.data.length(); ++j)
        forces_[i*6+j] = m_forcesRF.data[j];
    }

  i=3;
  if (m_forcesLHIn.isNew())
    {
      m_forcesLHIn.read();
      
      for (unsigned int j = 0; j < m_forcesLH.data.length(); ++j)
        forces_[i*6+j] = m_forcesLF.data[j];
    }
  
  i=4;
  if (m_forcesRFIn.isNew())
    {
      m_forcesRFIn.read();
      
      for (unsigned int j = 0; j < m_forcesRH.data.length(); ++j)
        forces_[i*6+j] = m_forcesRH.data[j];
    }
  
  sensorsIn["forces"].setValues(forces_);
  
  // Update torque
  sensorsIn["torques"].setName("torque");
  for (unsigned int j = 0; j < m_torques.data.length(); ++j)
    torques_[j] = m_torques.data[j];
  sensorsIn["torques"].setValues(torques_);
  
  // Update attitude
  sensorsIn["attitude"].setName("attitude");
  for (unsigned int j = 0; j < m_baseAtt.data.length(); ++j)
    baseAtt_ [j] = m_baseAtt.data[j];
  sensorsIn["attitude"].setValues (baseAtt_);
}

void 
RtcStackOfTasks::readControl(std::map<std::string,dgsot::ControlValues> &controlValues)
{
  // Update joint values.
  angleControl_ = controlValues["joints"].getValues();
  
  m_qRef.data.length(angleControl_.size());
  for(unsigned int i=0;i<angleControl_.size();i++)
    { m_qRef.data[i] = angleControl_[i]; }
    
  // Update forces
  const std::vector<double>& zmp (controlValues["zmp"].getValues());
  m_zmpRef.data.length(zmp.size());
  for(unsigned int i=0;i<3;i++)
    m_zmpRef.data[i] = zmp[i];
  
  // Update torque
  const std::vector<double>& baseff =
    controlValues["baseff"].getValues();

  m_baseAtt.data.length(baseff.size());
  for (int j = 0; j < 3; ++j)
    m_baseAtt.data[j] = baseff[j*4+3];
  
  for(unsigned int i=0;i<3;++i)
    for (int j = 0; j < 3; ++j)
      m_baseAtt.data[i*3+j] = baseff[i*4+j];
}

void
RtcStackOfTasks::captureTime (timeval& t)
{
  gettimeofday (&t, NULL);
}

void
RtcStackOfTasks::logTime (const timeval& t0, const timeval& t1)
{
  double dt =
    (t1.tv_sec - t0.tv_sec) * 1000.
    + (t1.tv_usec - t0.tv_usec + 0.) / 1000.;
  
  if (timeIndex_ < TIME_ARRAY_SIZE)
    timeArray_[timeIndex_++] = dt;
}

/*
RTC::ReturnCode_t RtcStackOfTasks::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t RtcStackOfTasks::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t RtcStackOfTasks::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t RtcStackOfTasks::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t RtcStackOfTasks::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t RtcStackOfTasks::onExecute(RTC::UniqueId ec_id)
{
  // 
  // Log control loop start time.
  captureTime (t0_);
  
  fillSensors(sensorsIn_);
  try
    {
      m_sotController->setupSetSensors(sensorsIn_);
      m_sotController->getControl(controlValues_);
    } 
  catch (std::exception &e) {  std::cout << e.what() <<std::endl;throw e; }
  readControl(controlValues_);

  // Log control loop end time and compute time spent.
  captureTime (t1_);
  logTime (t0_, t1_);
  return RTC::RTC_OK;
}
/*
RTC::ReturnCode_t RtcStackOfTasks::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t RtcStackOfTasks::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t RtcStackOfTasks::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t RtcStackOfTasks::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t RtcStackOfTasks::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void RtcStackOfTasksInit(RTC::Manager* manager)
  {
    std::cout << "RtcStackOfTasksInit - start " << std::endl;
    coil::Properties profile(RtcStackOfTasks_spec);
    manager->registerFactory(profile,
                             RTC::Create<RtcStackOfTasks>,
                             RTC::Delete<RtcStackOfTasks>);
    std::cout << "RtcStackOfTasksInit - end " << std::endl;
  }
  
};



