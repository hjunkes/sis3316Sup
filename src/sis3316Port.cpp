/** @file sis3316Port.cpp
 *  @brief The glue for asyn port operations
 *
 * Implements the mechanics of an Asyn port.  Translate port operations
 * into calls to member functions defined in sis3316Ops.cpp
 */
#include "sis3316Port.h"

#include <stdexcept>
#include <sstream>
#include <algorithm>

#include <math.h>
#include <string.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsExit.h>
#include "devLib.h"
#include <bsp/VME.h>

#define NUM_PARAMS    (&LAST_CMD - &FIRST_CMD + 1)

static inline
std::string namer(const char *A, const char *B)
{
   std::string ret(A);

   return(ret + B);
}

/* C function to call sample task from epicsThreadCreate*/
static void sampleTaskC(void *userPvt)
{
   Sis3316Port *pPvt = (Sis3316Port *)userPvt;

   pPvt->sampleTask();
}

/* C function to call update task from epicsThreadCreate*/
static void updateTaskC(void *userPvt)
{
   Sis3316Port *pPvt = (Sis3316Port *)userPvt;

   pPvt->updateTask();
}

Sis3316Port::Sis3316Port(const char *portName, epicsUInt32 a,
                         epicsUInt16 intVector, epicsUInt16 intLevel,
                         epicsUInt32 sampleLength, epicsUInt16 average, epicsUInt16 nrOfEvents)
   : asynPortDriver(portName, SIS3316_NUM_CHANNELS,
                    asynInt32Mask | asynInt16ArrayMask | asynFloat64Mask | asynFloat64ArrayMask |
                    asynOctetMask | asynDrvUserMask,
                    asynInt32Mask | asynInt16ArrayMask | asynFloat64Mask | asynFloat64ArrayMask,
                    0, // this device does not block and is not a multi device
                    1, // autoconnect
                    0, // prio
                    0) // default stack size
   , vmename(0)
   , a32addr(a)
   , base(0)
   , intVector(intVector)
   , intLevel(intLevel)
   , sampleLength(sampleLength)
   , average(average)
   , nrOfEvents(nrOfEvents)
   , zombie(false)

{
   try {
      // for debugging asyn
      //pasynTrace->setTraceMask(pasynUserSelf, ASYN_TRACE_ERROR | ASYN_TRACEIO_DEVICE | ASYN_TRACEIO_DRIVER | ASYN_TRACE_WARNING);
      //pasynTrace->setTraceInfoMask(pasynUserSelf, ASYN_TRACEINFO_TIME | ASYN_TRACEINFO_SOURCE);

      std::ostringstream vname;
      vname << "SIS3316 " << portName;

      vmename = epicsStrDup(vname.str().c_str());
      if (!vmename)
      {
         throw std::bad_alloc();
      }

      /* static hardware parameters */
      AERR(createParam("Module ID", asynParamInt32, &P_module_id));
      AERR(createParam("Revision Number", asynParamInt32, &P_revision_number));
      AERR(createParam("HW Version", asynParamInt32, &P_hw_version));
      AERR(createParam("Board Temp", asynParamFloat64, &P_temp));
      AERR(createParam("Serial Number", asynParamInt32, &P_serial_number));
      AERR(createParam("Gain", asynParamInt32, &P_gain));
      AERR(createParam("Termination", asynParamInt32, &P_termination));
      AERR(createParam("Offset", asynParamFloat64, &P_offset));
      AERR(createParam("Auto Trigger", asynParamInt32, &P_self_trigger));
      AERR(createParam("Trigger Wait", asynParamFloat64, &P_trigger_wait));
      AERR(createParam("Number of events", asynParamInt32, &P_number_of_events));
      AERR(createParam("Timestamp Clear", asynParamInt32, &P_timestamp_clear));
      AERR(createParam("Read Data", asynParamFloat64Array, &P_read_channel));
      AERR(createParam("Commit changes", asynParamInt32, &P_update));

      /*
       * map to SIS StandardPorts. Should be made in st.cmd
       * just before sisConfigure ???
       * I do not knwo why I am doing this ...
       */

      //BSP_VMEOutboundPortCfg(0,0x0D,0x41000000, 0x91000000,0x00010000);
      BSP_VMEOutboundPortCfg(0, 0x0D, a32addr, 0x91000000, 0x00400000);
      //BSP_VMEOutboundPortCfg(3,0x0f,0x21100000, 0x91100000,0x00400000);
      BSP_VMEOutboundPortsShow(stdout);
      // BSP_VMEResetBus(); resets itself :-(

      if (devRegisterAddress(vmename, atVMEA32, a32addr, 0x5000, (volatile void **)&base))
      {
         asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                   "%s: Failed to map A32 0x%04x\n", portName, a32addr);
         throw std::runtime_error("Failed to register");
      }

      //printf("Info: base = %p (Control STatus : %p)\n", base, (base+U32_SIS3316_CONTROL_STATUS/4));
      //devAddressMap();

      volatile epicsUInt32 x;
      /* devProbe must be adpated to Rtems 5 bspext? */
      if (devReadProbe(4, base + U32_SIS3316_CONTROL_STATUS / 4, (void *)&x))
      {
         asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                   "%s: No responce from card at %04x\n", portName, a32addr + U32_SIS3316_CONTROL_STATUS);
         throw std::runtime_error("No responce");
      }

      x = READ(base, SIS3316_MOD_ID);
      if ((x & 0xffff0000) != 0x33160000)
      {
         asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                   "%s: Invalid Module id %04x\n", portName, x);
         throw std::runtime_error("Wrong MOD_ID");
      }

      asynPrint(pasynUserSelf, ASYN_TRACEIO_DEVICE, "%s: Call init()\n", portName);

      init();

      {
         epicsThreadId tId;
         sampleDone = false;
         /* Create the thread that samples data from the digitizer */
         tId = epicsThreadCreate("sampleTask",
                                 epicsThreadPriorityMedium + 7,
                                 epicsThreadGetStackSize(epicsThreadStackMedium),
                                 (EPICSTHREADFUNC)sampleTaskC,
                                 this);
         if (tId == NULL)
         {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s: Can't create sampleTask \n", portName);
            throw std::runtime_error("Can't create sampleTask");
         }


         if (1)
         {
            updateDone = false;
            // Create the thread which  updates values to asyn
            tId = epicsThreadCreate("updateTask",
                                    epicsThreadPriorityMedium + 6,
                                    epicsThreadGetStackSize(epicsThreadStackMedium),
                                    (EPICSTHREADFUNC)updateTaskC,
                                    this);
            printf(" updateTask Id = 0x%lx\n", (long int)tId);
            if (tId == NULL)
            {
               asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                         "%s: Can't create updateTask \n", portName);
               throw std::runtime_error("Can't create updateTask");
            }
         }


         //doUpdate.signal(); // do initial update done by sample task
         doSample.signal(); // do initial update
      }

      epicsAtExit(&shutdown, (void *)this);
   } catch (std::exception& e) {
      // lock() not held (would deadlock)
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s: Failed to initialize port\nError: %s\n",
                portName, e.what());
      stop();
   }
}

Sis3316Port::~Sis3316Port()
{
   stop();
}

void Sis3316Port::shutdown(void *raw)
{
   Sis3316Port *port = (Sis3316Port *)raw;

   port->stop();
}

void
Sis3316Port::stop()
{
   lock();

   if (zombie)
   {
      return;
   }
   zombie = true;

   if (base)
   {
      devUnregisterAddress(atVMEA32, a32addr, vmename);
   }
   free(vmename);

   updateDone = true;
   sampleDone = true;
   unlock();
   doUpdate.signal();
   doSample.signal();
}

asynStatus Sis3316Port::connect(asynUser *pasynUser)
{
   if (!zombie)
   {
      return(asynPortDriver::connect(pasynUser));
   }
   asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: Port not initialized\n", portName);
   return(asynError);
}

asynStatus Sis3316Port::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
   try {
      int        addr;
      asynStatus ret      = asynSuccess;
      int        function = pasynUser->reason;

      AERR(getAddress(pasynUser, &addr));

      asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: %s\n", portName, BOOST_CURRENT_FUNCTION);

      if (function == P_termination)
      {
         if (sis3316_adc1->setTermination(addr, value) < 0)
         {
            ret = asynError;
         }
      }
      else if (function == P_gain)
      {
         if (sis3316_adc1->setGain(addr, value) < 0)
         {
            ret = asynError;
         }
      }
      else if (function == P_number_of_events)
      {
         ; //nofEvents = value; // later reconfigure ...
      }
      else if (function == P_timestamp_clear)
      {
         clearTimestamp();
      }
      else if (function == P_update)
      {
         // schedule update
         doUpdate.signal();
      }
      else if (function == P_self_trigger)
      {
         selfTrigger = value;
         if (value > 0)
         {
            WRITE(base, SIS3316_KEY_TRIGGER, 0);
         }
      }
      else
      {
         asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: Param %s is read-only\n",
                   portName, retParamName(addr, function));
         return(asynError);
      }

      if (ret == asynSuccess)
      {
         asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s: Set %s = %d\n",
                   portName, retParamName(addr, function), value);
      }

      AERR(setIntegerParam(addr, function, value));
      AERR(callParamCallbacks(addr));

      return(ret);
   } CATCHALL(pasynUser)
}

asynStatus Sis3316Port::readInt16Array(asynUser *pasynUser, epicsInt16 *value, size_t nElements, size_t *nIn)
{
   try {
      int addr;
      int function = pasynUser->reason;

      AERR(getAddress(pasynUser, &addr));

      asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: %s\n", portName, BOOST_CURRENT_FUNCTION);


      asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: Parameter %s is not a int16 array or unknown\n",
                portName, retParamName(addr, function));
      return(asynError);


      asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s: Read %s\n",
                portName, retParamName(addr, function));

      return(asynSuccess);
   } CATCHALL(pasynUser)
}

asynStatus Sis3316Port::readInt32(asynUser *pasynUser, epicsInt32 *value)
{
   try {
      int        addr;
      asynStatus ret      = asynSuccess;
      int        function = pasynUser->reason;

      AERR(getAddress(pasynUser, &addr));

      asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: %s\n", portName, BOOST_CURRENT_FUNCTION);

      if (function == P_serial_number)
      {
         *value = sis3316_adc1->serial_number;
      }
      else if (function == P_module_id)
      {
         *value = sis3316_adc1->module_id;
      }
      else if (function == P_revision_number)
      {
         *value = sis3316_adc1->revision_number;
      }
      else if (function == P_gain)
      {
         *value = (sis3316_adc1->adc_gain_termination_ch_array[addr]) & 0x3;
      }
      else if (function == P_termination)
      {
         *value = ((sis3316_adc1->adc_gain_termination_ch_array[addr]) >> 2) & 0x1;
      }
      else if (function == P_number_of_events)
      {
         *value = sis3316_adc1->nofEvents;
      }
      else if (function == P_self_trigger)
      {
         *value = selfTrigger;
      }
      else
      {
         asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: Param %s can not be read\n",
                   portName, retParamName(addr, function));
         return(asynError);
      }

      if (ret == asynSuccess)
      {
         asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s: Read %s = 0x%x\n",
                   portName, retParamName(addr, function), *value);
      }

      AERR(setIntegerParam(addr, function, *value));
      AERR(callParamCallbacks(addr));

      return(ret);
   } CATCHALL(pasynUser)
}

asynStatus Sis3316Port::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
   try {
      int        addr;
      asynStatus ret      = asynSuccess;
      int        function = pasynUser->reason;

      AERR(getAddress(pasynUser, &addr));

      asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: %s\n", portName, BOOST_CURRENT_FUNCTION);

      if (function == P_offset)
      {
         unsigned short v;
         v = (unsigned short)(65535. * value);
         if (sis3316_adc1->setDac(addr, v) < 0)
         {
            ret = asynError;
         }
      }
      else if (function == P_trigger_wait)
      {
         if (value >= 0.01)
         {
            triggerWait = value;
         }
         else
         {
            value = triggerWait;
         }
      }
      else
      {
         asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: Param %s is read-only\n",
                   portName, retParamName(addr, function));
         return(asynError);
      }

      if (ret == asynSuccess)
      {
         asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s: Set %s = %f\n",
                   portName, retParamName(addr, function), value);
      }

      AERR(setDoubleParam(addr, function, value));
      AERR(callParamCallbacks(addr));

      return(ret);
   } CATCHALL(pasynUser)
}

asynStatus Sis3316Port::readFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements, size_t *nIn)
{
   try {
      int addr;
      int function = pasynUser->reason;

      AERR(getAddress(pasynUser, &addr));

      asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: %s\n", portName, BOOST_CURRENT_FUNCTION);

      if (function == P_read_channel)
      {
         if (addr < SIS3316_NUM_CHANNELS)
         {
            unsigned int i;
            for (i = 0; i < vCh[addr].size() && i < nElements; i++)
            {
               value[i] = vCh[addr][i];
            }
            *nIn = i;
            vCh[addr].clear();
         }
      }
      else
      {
         asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: Parameter %s is not a Float64 array or unknown\n",
                   portName, retParamName(addr, function));
         return(asynError);
      }

      asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s: Read %s\n",
                portName, retParamName(addr, function));

      return(asynSuccess);
   } CATCHALL(pasynUser)
}

asynStatus Sis3316Port::readFloat64(asynUser *pasynUser, epicsFloat64 *value)
{
   try {
      int        addr;
      asynStatus ret      = asynSuccess;
      int        function = pasynUser->reason;

      AERR(getAddress(pasynUser, &addr));

      asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s: %s\n", portName, BOOST_CURRENT_FUNCTION);

      if (function == P_temp)
      {
         *value  = (double)(float)(READ(base, SIS3316_TEMP) & 0x3ff);
         *value /= 4;
      }
      else if (function == P_offset)
      {
         unsigned short data;
         if (sis3316_adc1->readDac(addr, &data) < 0)
         {
            ret = asynError;
         }
         else
         {
            *value = (double)data / 65536.;
         }
      }
      else if (function == P_trigger_wait)
      {
         *value = triggerWait;
      }
      else
      {
         asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s: Param %s not defined in readFloat64\n",
                   portName, retParamName(addr, function));
         return(asynError);
      }

      if (ret == asynSuccess)
      {
         asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s: Read %s = %g\n",
                   portName, retParamName(addr, function), *value);
      }

      AERR(setDoubleParam(addr, function, *value));
      AERR(callParamCallbacks(addr));

      return(ret);
   } CATCHALL(pasynUser)
}

AsynUserSIS::AsynUserSIS(const char *portName, int addr)
{
   usr = pasynManager->createAsynUser(&AsynUserSIS::procCB, NULL);
   if (!usr)
   {
      throw std::bad_alloc();
   }
   try{
      AERR(pasynManager->connectDevice(usr, portName, addr));

      usr->userPvt = (void *)this;
   }catch (...) {
      pasynManager->freeAsynUser(usr);
      throw;
   }
}

AsynUserSIS::AsynUserSIS(asynUser *other)
   : usr(other)
{
}

AsynUserSIS::~AsynUserSIS()
{
   if (!usr)
   {
      return;
   }
   pasynManager->freeAsynUser(usr);
}

const char *AsynUserSIS::name() throw()
{
   const char *ret = 0;

   if (pasynManager->getPortName(usr, &ret) != asynSuccess)
   {
      return("<not connected>");
   }
   return(ret);
}

asynUser *
AsynUserSIS::release()
{
   asynUser *t = usr;

   usr = 0;
   return(t);
}

void
AsynUserSIS::process()
{
   asynPrint(usr, ASYN_TRACE_FLOW, "%s: %s\n", name(), BOOST_CURRENT_FUNCTION);
}

void AsynUserSIS::procCB(asynUser *usr)
{
   AsynUserSIS *inst = (AsynUserSIS *)usr->userPvt;

   try {
      inst->process();
   } catch (std::exception& e) {
      asynPrint(usr, ASYN_TRACE_ERROR, "%s: %s unhandled exception '%s'\n",
                inst->name(), BOOST_CURRENT_FUNCTION, e.what());
   }
}

extern "C"
void drvAsynSIS3316Configure(const char *portName, epicsUInt32 a,
                             epicsUInt16 intVector, epicsUInt16 intLevel,
                             epicsUInt32 sampleLength, epicsUInt16 average,
                             epicsUInt16 nrOfEvents)
{
   try {
      // ugly, but ports can't be deleted anyway...
      new Sis3316Port(portName, a, intVector, intLevel, sampleLength, average, nrOfEvents);
   } catch (std::exception& e) {
      printf("Failed to create\n%s\n", e.what());
   }
}

#include <iocsh.h>
#include <epicsExport.h>

static const iocshArg drvAsynSIS3316ConfigureArg0 = { "port", iocshArgString };
static const iocshArg drvAsynSIS3316ConfigureArg1 = { "A32 address", iocshArgInt };
static const iocshArg drvAsynSIS3316ConfigureArg2 = { "Interrupt Vector", iocshArgInt };
static const iocshArg drvAsynSIS3316ConfigureArg3 = { "Interrupt Level", iocshArgInt };
static const iocshArg drvAsynSIS3316ConfigureArg4 = { "Sample Length", iocshArgInt };
static const iocshArg drvAsynSIS3316ConfigureArg5 = { "Average", iocshArgInt };
static const iocshArg drvAsynSIS3316ConfigureArg6 = { "Nr of Events", iocshArgInt };

static const iocshArg *const drvAsynSIS3316ConfigureArgs[] =
{ &drvAsynSIS3316ConfigureArg0, &drvAsynSIS3316ConfigureArg1, &drvAsynSIS3316ConfigureArg2,
  &drvAsynSIS3316ConfigureArg3, &drvAsynSIS3316ConfigureArg4, &drvAsynSIS3316ConfigureArg5, &drvAsynSIS3316ConfigureArg6 };
static const iocshFuncDef drvAsynSIS3316ConfigureFuncDef = {
   "drvAsynSIS3316Configure", 7, drvAsynSIS3316ConfigureArgs
    #ifdef IOCSHFUNCDEF_HAS_USAGE
   ,                          "configure SIS3316 port, a32 address, int vector, int level, sample length, average, nr of events"
    #endif
};
static void drvAsynSIS3316ConfigureCallFunc(const iocshArgBuf *args)
{
   drvAsynSIS3316Configure(args[0].sval, args[1].ival, args[2].ival,
                           args[3].ival, args[4].ival, args[5].ival, args[6].ival);
}

static
void sis3316Register(void)
{
   iocshRegister(&drvAsynSIS3316ConfigureFuncDef, drvAsynSIS3316ConfigureCallFunc);
}

epicsExportRegistrar(sis3316Register);
