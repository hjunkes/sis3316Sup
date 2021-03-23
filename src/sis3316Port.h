#ifndef SIS3316PORT_H
#define SIS3316PORT_H

#include <string>
#include <vector>
#include <stdexcept>

#include <cstdio>
#include "current_function.hpp"

#include <epicsTypes.h>
#include <epicsThread.h>
#include <epicsGuard.h>

#include <epicsRingBytes.h>

#include <asynPortDriver.h>
#include "sis3316_registers.h"
#include "sis3316_adc.h"

#include <pv/pvaClient.h>

class Sis3316Port : public asynPortDriver
{
public:
   Sis3316Port(const char *portName, epicsUInt32 a32addr,
               epicsUInt16 intVector, epicsUInt16 intLevel, epicsUInt32 sampleLength,
               epicsUInt16 average, epicsUInt16 nrOfEvents);
   virtual ~Sis3316Port();

   virtual void report(FILE *fp, int details);
   virtual asynStatus connect(asynUser *pasynUser);

   virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
   virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
   virtual asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);
   virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
   virtual asynStatus readFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements, size_t *nIn);

/*
 *  virtual asynStatus writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements);
 *  virtual asynStatus writeInt16Array(asynUser *pasynUser, epicsInt16 *value, size_t nElements);
 */
   virtual asynStatus readInt16Array(asynUser *pasynUser, epicsInt16 *value, size_t nElements, size_t *nIn);
   void intFunc();
   void updateTask();
   void sampleTask();

protected:
   // Parameter IDs must be continious
#define FIRST_CMD    P_mfr
   int P_module_id;
   int P_revision_number;
   int P_hw_version;
   int P_temp;
   int P_serial_number;
   int P_number_of_events;
   int P_gain;
   int P_termination;
   int P_offset;
   int P_self_trigger;
   int P_trigger_wait;
   int P_timestamp_clear;
   int P_read_channel;
   int P_update;
#define LAST_CMD    P_update

   char *vmename;
   epicsUInt32 a32addr;
   volatile epicsUInt32 *base;

   epicsUInt16 intVector;
   epicsUInt16 intLevel;
   epicsUInt32 sampleLength;
   epicsUInt16 average;
   epicsUInt16 nrOfEvents;

   double float_data;
   double timeStamp[SIS3316_NUM_CHANNELS];
   std::vector <double> vCh[SIS3316_NUM_CHANNELS];
   std::vector <double> sendData[SIS3316_NUM_CHANNELS];

   int dataCounter;
   unsigned int selfTrigger = 0;
   double triggerWait       = 1.0;

   const char *retParamName(int idx = 0)
   {
      return(retParamName(0, idx));
   }

   const char *retParamName(int list, int idx)
   {
      const char *ret = 0;

      if (getParamName(list, idx, &ret) != asynSuccess)
      {
         return("<invalid>");
      }
      return(ret);
   }

private:
   static void shutdown(void *raw);
   void stop();
   void init();

   bool updateDone;
   epicsEvent doUpdate;

   static double fmult()
   {
      return(0.009313);
   }                                           //Hz per LSB

   static double amult()
   {
      return(65536.0 / 20.0);
   }

   static double omult()
   {
      return(32767.0 / 10.0);
   }

   int setGain(epicsInt32 ch, epicsUInt8 gain);
   int setTermination(epicsInt32 ch, epicsUInt8 termination);
   void softTrigger();
   void clearTimestamp();
   void printInfoBlock();


   epicsEvent *intOccured;
   bool sampleDone;
   epicsEvent doSample;

   void setControlStatus(epicsUInt32 val);

   bool zombie;  // brains...
   sis3316_adc *sis3316_adc1;
};

/* translate c++ exceptions into asyn errors */

#define CATCHALL(USR)             catch (std::exception& e) {                                       \
      asynPrint(USR, ASYN_TRACE_ERROR, "%s: %s: %s\n", portName, BOOST_CURRENT_FUNCTION, e.what()); \
      return(asynError); }

#define CATCHALL_CONTINUE(USR)    catch (std::exception& e) {                                       \
      asynPrint(USR, ASYN_TRACE_ERROR, "%s: %s: %s\n", portName, BOOST_CURRENT_FUNCTION, e.what()); \
      return; }

/* and vis versa */
#define AERR(RET)                 do { if ((RET) != asynSuccess) {                                                                               \
                                          std::ostringstream msg; msg << portName << ": " << BOOST_CURRENT_FUNCTION << ": on line " << __LINE__; \
                                          throw std::runtime_error(msg.str()); } } while (0)

//! Container for asynUser
class AsynUserSIS {
   asynUser *usr;

   AsynUserSIS(const AsynUserSIS&);
   AsynUserSIS& operator=(const AsynUserSIS&);

public:
   AsynUserSIS(const char *port, int addr = -1);
   AsynUserSIS(asynUser *other);

   virtual ~AsynUserSIS();

   asynUser *release();

   const char *name() throw();

   operator asynUser *()
   {
      return(usr);
   }

   asynUser *operator->()
   {
      return(usr);
   }

   asynUser& operator*()
   {
      return(*usr);
   }

   virtual void process();

private:
   static void procCB(asynUser *);
};

class portLocker
{
   Sis3316Port *port_;
   bool locked_;
public:
   portLocker(Sis3316Port& p) : port_(&p), locked_(false)
   {
      lock();
   }

   ~portLocker()
   {
      try {
         unlock();
      }catch (std::exception& e) {
         fprintf(stderr, "%s: %s\n",
                 port_->portName, e.what());
      }
   }

   void lock()
   {
      if (locked_)
      {
         throw std::logic_error("Already locked");
      }
      if (port_->lock() != asynSuccess)
      {
         throw std::runtime_error("Failed to lock");
      }
      locked_ = true;
   }

   void unlock()
   {
      if (!locked_)
      {
         throw std::logic_error("Not locked");
      }
      if (port_->unlock() != asynSuccess)
      {
         throw std::runtime_error("Failed to unlock");
      }
      locked_ = false;
   }
};

#endif // SIS3316PORT_H
