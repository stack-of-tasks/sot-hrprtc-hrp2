// -*- C++ -*-
/*!
 * @file  RtcStackOfTasks.cpp * @brief Module for controlling humanoid robot * $Date$ 
 *
 * $Id$ 
 */
#include "rtc-stack-of-tasks.h"
#include <dlfcn.h>
#include <cmath>
#include <fstream>
#include <exception>
#include <iomanip>

#if DEBUG
#define ODEBUG(x) std::cout << x << std::endl
#else
#define ODEBUG(x)
#endif
#define ODEBUG3(x) std::cout << x << std::endl

#define DBGFILE "/tmp/rtc-stack-of-tasks-comp"
#define RESETDEBUG5() { std::ofstream DebugFile; \
    DebugFile.open(DBGFILE,std::ofstream::out);  \
DebugFile.close();}
#define ODEBUG5FULL(x) { std::ofstream DebugFile; \
    DebugFile.open(DBGFILE,std::ofstream::app);   \
DebugFile << __FILE__ << ":" \
          << __FUNCTION__ << "(#" \
          << __LINE__ << "):" << x << std::endl; \
          DebugFile.close();}
#define ODEBUG5(x) { std::ofstream DebugFile; \
    DebugFile.open(DBGFILE,std::ofstream::app); \
DebugFile << x << std::endl; \
          DebugFile.close();}


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
    "conf.default.sot_libname",       "librobot.so",
    "conf.default.robot_nb_dofs",      "0",
    "conf.default.robot_nb_force_sensors", "0",
    "conf.default.is_enabled", "0",
    ""
  };
// </rtc-template>

RtcStackOfTasks::RtcStackOfTasks(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_angleEncoderIn("angleEncoder", m_angleEncoder),
    m_torquesIn("torques", m_torques),
    m_baseAttIn("baseAtt", m_baseAtt),
    m_accelerometer_0In("accelerometer_0",m_accelerometer_0),
    m_gyrometer_0In("gyrometer_0",m_gyrometer_0),
    m_rpyIn("rpy", m_rpy),
    m_forcesLFIn("forcesLF", m_forcesLF),
    m_forcesRFIn("forcesRF", m_forcesRF),
    m_forcesLHIn("forcesLH", m_forcesLH),
    m_forcesRHIn("forcesRH", m_forcesRH),
    m_angleInitIn("qInit",m_angleInit),
    m_positionInitIn("pInit",m_positionInit),
    m_rpyInitIn("rpyInit",m_rpyInit),
    m_zmpRefOut("zmpRef", m_zmpRef),
    m_qRefOut("qRef", m_qRef),
    m_pRefOut("pRef", m_pRef),
    m_rpyRefOut("rpyRef", m_rpyRef),
    m_accRefOut("accRef", m_accRef),
    timeIndex_(0),
    manager_(manager),
    initialize_library_(false),
    startupThread_()
    // </rtc-template>
{
  RESETDEBUG5();
  m_rpyRef.data.length(3);
  m_pRef.data.length(3);
  m_zmpRef.data.length(3);
}

RtcStackOfTasks::~RtcStackOfTasks()
{
  saveLog();
}

void RtcStackOfTasks::saveLog() const
{
  std::string filename ("/tmp/rtc-log-time.txt");
  std::ofstream logTime (filename.c_str());
  if(logTime.is_open())
  {
    for(unsigned i=0;i<std::min(timeIndex_, timeArray_.size()); ++i)
      logTime << i << "   " << "   " << timeArray_[i] << std::endl;
    logTime.close();
  }
  else
  {
    ODEBUG5("Unable to open '" << filename <<"' to save the log'");
  }
}

void RtcStackOfTasks::readConfig()
{
  ODEBUG5("The library to be loaded: " << robot_config_.libname) ;
  ODEBUG5("Nb dofs:" << robot_config_.nb_dofs);
  ODEBUG5("Nb force sensors:" << robot_config_.nb_force_sensors);
  m_qRef.data.length(robot_config_.nb_dofs);  
}

void RtcStackOfTasks::LoadSot()
{
  char * sLD_LIBRARY_PATH;
  sLD_LIBRARY_PATH=getenv("LD_LIBRARY_PATH");
  ODEBUG5("LoadSot - Start " << sLD_LIBRARY_PATH);
  char * sPYTHONPATH;
  sPYTHONPATH=getenv("PYTHONPATH");
  ODEBUG5("PYTHONPATH:" << sPYTHONPATH );
  sPYTHONPATH=getenv("PYTHON_PATH");
  ODEBUG5("PYTHON_PATH:" << sPYTHONPATH);

  // Load the SotHRP2Controller library.
  void * SotHRP2ControllerLibrary = dlopen(robot_config_.libname.c_str(),
                                           RTLD_GLOBAL | RTLD_NOW);
  if (!SotHRP2ControllerLibrary) {
    ODEBUG5("Cannot load library: " << dlerror() );
    return ;
  }
  ODEBUG5("Success in loading the library:" << robot_config_.libname);
  // reset errors
  dlerror();
  
  // Load the symbols.
  createSotExternalInterface_t * createHRP2Controller =
    (createSotExternalInterface_t *) dlsym(SotHRP2ControllerLibrary, 
                                           "createSotExternalInterface");
  ODEBUG5("createHRPController call "<< std::hex
          << std::setbase(10));
  const char* dlsym_error = dlerror();
  if (dlsym_error) {
    ODEBUG5("Cannot load symbol create: " << dlsym_error );
    return ;
  }
  ODEBUG5("Success in getting the controller factory");
  
  // Create hrp2-controller
  try 
    {
      ODEBUG5("exception handled createHRP2Controller call "<< std::hex 
              << std::setbase(10));
      m_sotController = createHRP2Controller();
      ODEBUG5("After createHRP2Controller.");

    } 
  catch (std::exception &e)
    {
      ODEBUG5("Exception: " << e.what());
    }
  ODEBUG5("LoadSot - End");
}

RTC::ReturnCode_t RtcStackOfTasks::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("angleEncoder", m_angleEncoderIn);
  addInPort("torques", m_torquesIn);
  addInPort("baseAtt", m_baseAttIn);
  addInPort("accelerometer_0",m_accelerometer_0In);
  addInPort("gyrometer_0",m_gyrometer_0In);
  addInPort("rpy", m_rpyIn);
  addInPort("forcesLF", m_forcesLFIn);
  addInPort("forcesRF", m_forcesRFIn);
  addInPort("forcesLH", m_forcesLHIn);
  addInPort("forcesRH", m_forcesRHIn);
  addInPort("qInit",m_angleInitIn);
  addInPort("pInit",m_positionInitIn);
  addInPort("rpyInit",m_rpyInitIn);

  // Set OutPort buffer
  addOutPort("zmpRef", m_zmpRefOut);
  addOutPort("qRef", m_qRefOut);
  addOutPort("pRef", m_pRefOut);
  addOutPort("accRef", m_accRefOut);
  addOutPort("rpyRef", m_rpyRefOut);

  // Set service provider to Ports

  // Set service consumers to Ports

  // Set CORBA Service Ports

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("sot_libname", robot_config_.libname, "libtherobot.so");
  bindParameter("robot_nb_dofs",robot_config_.nb_dofs, "0");
  bindParameter("robot_nb_force_sensors",robot_config_.nb_force_sensors, "0");
  bindParameter("is_enabled",started_,"0");

  // </rtc-template>

  // Initialize angleEncoder_ to zero.
  angleEncoder_.resize(robot_config_.nb_dofs);
  for(int i=0;i<robot_config_.nb_dofs;i++)
    angleEncoder_[i] = 0.0;

  return RTC::RTC_OK;
}

void RtcStackOfTasks::
fillInForceSensor(InPort<TimedDoubleSeq> &aForcePortIn,
                  TimedDoubleSeq &aForceData,
                  std::vector<double> &lforce,
                  unsigned int index)
{
  if (aForcePortIn.isNew())
    {
      aForcePortIn.read();      
      for (unsigned int j = 0; j < aForceData.data.length(); ++j)
        lforce[index*6+j] = aForceData.data[j];
    }
  else
    {
      for (unsigned int j = 0; j < 6; ++j)
        lforce[index*6+j] = 0.0;
    }

}

void RtcStackOfTasks::fillAngles(std::map<std::string,dgsot::SensorValues> & 
                                 sensorsIn,
                                 bool initPort)
{
  InPort<TimedDoubleSeq> * langlePortIn=&m_angleEncoderIn;
  TimedDoubleSeq * langlePort=&m_angleEncoder;
  if (initPort)
    {
      ODEBUG("Read Init Port");
      langlePortIn = &m_angleInitIn;
      langlePort = &m_angleInit;
    }
  // Update joint values.
  sensorsIn["joints"].setName("angle");
  if (langlePortIn->isNew())
    {
      langlePortIn->read();
      
      angleEncoder_.resize(langlePort->data.length());
      for(unsigned int i=0;i<langlePort->data.length();i++)
        {
          angleEncoder_[i] = langlePort->data[i];
          if (initPort)
            {
              ODEBUG("qInit:["<<i << "]= " << langlePort->data[i]);
            }
        }
    }
  sensorsIn["joints"].setValues(angleEncoder_);
}

void RtcStackOfTasks::fromRotationToRpy(double *R, RpyVector &aRpyVector)
{
  double alpha,beta,gamma;
  
  beta = atan2(-R[2*3+0], sqrt(R[0]*R[0] + R[1*3]*R[1*3]));
  if (beta==M_PI/2.0)
    {
      alpha = 0;
      gamma = atan2(R[1], R[1*3+1]);
    }
  else
    {
      if (beta==-M_PI/2)
        {
          alpha = 0.0;
          gamma = -atan2(R[1], R[0]);
        }
      else
        {
          alpha = atan2(R[1*3]/cos(beta), R[0]/cos(beta));
          gamma = atan2(R[2*3+1]/cos(beta), R[2*3+2]/cos(beta));
        }
    }

  aRpyVector.roll = gamma;
  aRpyVector.pitch = beta;
  aRpyVector.yaw = alpha;
}

void 
RtcStackOfTasks::fillSensors(std::map<std::string,dgsot::SensorValues> & 
                             sensorsIn)
{
  fillAngles(sensorsIn_,false);

  // Update forces
  // TODO: Make sensors port add automatically
  // They should be create by a CORBA service.
  sensorsIn["forces"].setName("force");
  InPort<TimedDoubleSeq> * forcesIn[4];
  forcesIn[0]=&m_forcesRFIn; forcesIn[1]=&m_forcesLFIn;
  forcesIn[2]=&m_forcesRHIn; forcesIn[3]=&m_forcesLHIn;
  TimedDoubleSeq *forcesData[4];
  forcesData[0]=&m_forcesRF; forcesData[1]=& m_forcesLF; 
  forcesData[2]=&m_forcesRH; forcesData[3]=& m_forcesLH;
  forces_.resize(4*6);
  for(unsigned int i=0;i<4;i++)
    fillInForceSensor(*forcesIn[i],*forcesData[i], forces_,i);
  sensorsIn["forces"].setValues(forces_);
  
  // Update torque
  sensorsIn["torques"].setName("torque");
  if (m_torquesIn.isNew())
    {
      
      torques_.resize(m_torques.data.length());
      for (unsigned int j = 0; j < m_torques.data.length(); ++j)
        torques_[j] = m_torques.data[j];
    }
  else 
    {
      torques_.resize(robot_config_.nb_dofs);
      for(unsigned int i=0;i<(unsigned int)robot_config_.nb_dofs;i++)
        torques_[i] = 0.0;
    }
  sensorsIn["torques"].setValues(torques_);

  // Update attitude
  sensorsIn["attitude"].setName("attitude");
  if (m_baseAttIn.isNew())
    {
      baseAtt_.resize(m_baseAtt.data.length());
      for (unsigned int j = 0; j < m_baseAtt.data.length(); ++j)
        baseAtt_ [j] = m_baseAtt.data[j];
    }
  else
    {
      baseAtt_.resize(9);
      for(unsigned int i=0;i<3;i++)
        {
          for(unsigned int j=0;j<3;j++)
            {
              if (i==j)
                baseAtt_[i*3+j]=1.0;
              else 
                baseAtt_[i*3+j]=0.0;
            }
        }
    }
  sensorsIn["attitude"].setValues (baseAtt_);

  // Update accelerometer
  sensorsIn["accelerometer_0"].setName("accelerometer_0");
  if (m_accelerometer_0In.isNew())
    {
      m_accelerometer_0In.read();
      accelerometer_.resize(m_accelerometer_0.data.length());
      for (unsigned int j = 0; j < m_accelerometer_0.data.length(); ++j)
        accelerometer_[j] = m_accelerometer_0.data[j];
    }
  else 
    {
      accelerometer_.resize(3);
      for(unsigned int i=0;i<3;i++)
        accelerometer_[i] = 0.0;
    }  
  sensorsIn["accelerometer_0"].setValues(accelerometer_);
  
  // Update gyrometer
  sensorsIn["gyrometer_0"].setName("gyrometer_0");
  if (m_gyrometer_0In.isNew())
    {
      m_gyrometer_0In.read();
      gyrometer_.resize(m_gyrometer_0.data.length());
      for (unsigned int j = 0; j < m_gyrometer_0.data.length(); ++j)
        gyrometer_[j] = m_gyrometer_0.data[j];
    }
  else 
    {
      gyrometer_.resize(3);
      for(unsigned int i=0;i<3;i++)
        gyrometer_[i] = 0.0;
    }  
  sensorsIn["gyrometer_0"].setValues(gyrometer_);
}

void 
RtcStackOfTasks::readControl(std::map<std::string,dgsot::ControlValues> &controlValues)
{
  double R[9];

  RTC::Time tm;
    
  coil::TimeValue coiltm(coil::gettimeofday());
  tm.sec = coiltm.sec();
  tm.nsec = coiltm.usec()*1000;
  
  // Update joint values.
  angleControl_ = controlValues["joints"].getValues();
  
  for(unsigned int i=0;i<angleControl_.size();i++)
    { 
      m_qRef.data[i] = angleControl_[i]; 
      ODEBUG("m_qRef["<<i<<"]=" << m_qRef.data[i]);
    }
  if (angleControl_.size()<(unsigned int)robot_config_.nb_dofs)
    {
      for(unsigned int i=angleControl_.size();
          i<(unsigned int)robot_config_.nb_dofs
            ;i++)
        {
          m_qRef.data[i] = 0.0;
          ODEBUG("m_qRef["<<i<<"]=" << m_qRef.data[i]);
        }
    }
  m_qRef.tm = tm;
  m_qRefOut.write();
  
  // Update torque
  if(controlValues.find("baseff") != controlValues.end())
  {
  const std::vector<double>& baseff =
    controlValues["baseff"].getValues();
  
  m_pRef.data[0] = baseff[3]; 
  m_pRef.data[1] = baseff[7]; 
  m_pRef.data[2] = baseff[11]; 
  m_pRef.tm = tm;
  m_pRefOut.write();

  ODEBUG("m_pRef" 
          << m_pRef.data[0] << " "
          << m_pRef.data[1] << " "
          << m_pRef.data[2] << " " );  
  for(unsigned int i=0;i<3;++i)
    for (int j = 0; j < 3; ++j)
      R[i*3+j] = baseff[i*4+j];
  
  RpyVector arpyv;
  fromRotationToRpy(R,arpyv);
  
  m_rpyRef.data[0] = arpyv.roll;
  m_rpyRef.data[1] = arpyv.pitch;
  m_rpyRef.data[2] = arpyv.yaw;
  m_rpyRef.tm = tm;
  m_rpyRefOut.write();

  ODEBUG("m_rpyRef =" 
          << m_rpyRef.data[0] << " " 
          << m_rpyRef.data[1] << " " 
          << m_rpyRef.data[2] << " " );  
  }

  // Update forces
  if(controlValues.find("zmp") != controlValues.end())
  {
  const std::vector<double>& zmp (controlValues["zmp"].getValues());
  m_zmpRef.data[0] = zmp[0];
  m_zmpRef.data[1] = zmp[1];
  m_zmpRef.data[2] = zmp[2];

  ODEBUG("m_zmpRef: " << m_zmpRef.data[0] << " "
          << m_zmpRef.data[1] << " "
          << m_zmpRef.data[2] << " ");

  m_zmpRef.tm = tm;
  m_zmpRefOut.write();
  }
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
    (t1.tv_sec - t0.tv_sec)
    + (t1.tv_usec - t0.tv_usec + 0.) / 1e6;

  if (timeIndex_ < timeArray_.size())
    timeArray_[timeIndex_++] = dt;
}

/*
RTC::ReturnCode_t RtcStackOfTasks::onFinalize()
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t RtcStackOfTasks::onStartup(RTC::UniqueId /* ec_id*/)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RtcStackOfTasks::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t RtcStackOfTasks::onActivated(RTC::UniqueId /* ec_id */)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RtcStackOfTasks::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

void RtcStackOfTasks::loadAndStart()
{
  readConfig();
  LoadSot();
  if (m_angleInitIn.isNew())
    fillAngles(sensorsIn_,true);
  initialize_library_ = true;
}


RTC::ReturnCode_t RtcStackOfTasks::onExecute(RTC::UniqueId /* ec_id */)
{
  ODEBUG("onExecute - start");
  ODEBUG("Active configuration set:");

  // start the initialization thread
  if (initialize_library_ == false && !startupThread_)
  {
    startupThread_.reset(new boost::thread(&RtcStackOfTasks::loadAndStart, this));
  }
  // destroy the initialization thread when done
  else if (initialize_library_ == true  && startupThread_)
  {
    startupThread_->join();
    startupThread_.reset();
  }

  if (!started_)
    {
      ODEBUG("started_ property not set.");
      return RTC::RTC_OK;      
    }

  ODEBUG(m_configsets.getActiveId());
  // 
  // Log control loop start time.
  captureTime (t0_);
  
  fillSensors(sensorsIn_);
  try
    {
      m_sotController->setupSetSensors(sensorsIn_);
      m_sotController->getControl(controlValues_);
    } 
  catch (std::exception &e) 
    {  ODEBUG5("Exception on Execute: " << e.what());throw e; }
  ODEBUG("Before reading control");
  readControl(controlValues_);
  ODEBUG("After reading control");

  // Log control loop end time and compute time spent.
  captureTime (t1_);
  logTime (t0_, t1_);
  ODEBUG("onExecute - end");
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



