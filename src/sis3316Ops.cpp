#include "sis3316Port.h"

#include <stdexcept>
#include <sstream>
#include <algorithm>
#include <limits>

#include <math.h>

#include <epicsTime.h>
#include <cantProceed.h>
#include <epicsAlgorithm.h>

#ifdef __rtems__
#include <os/RTEMS/epicsMath.h>
#include <errlog.h>
#include <devLib.h>
#include <bsp/vme_am_defs.h>
#include <devLibVME.h>
#endif

//#include "iChunk.h"


static void intFuncC(void *drvPvt);

void Sis3316Port::report(FILE *fp, int details)
{
   // 0 - minimal details
   // 1 - Register dump
   // 2 - asyn port connection info
   // 3 - asynPortDriver parameter list
   asynPortDriver::report(fp, std::max(0, details - 1));
   if (zombie)
   {
      return;
   }
   if (details >= 1)
   {  // FPGA registers
      for (unsigned int off = 0; off < 0x440; off += 4)
      {
         if (off < 0x100 || (off >= 0x400))
         {    // gap btw 0x100 and 0x400
            if (off % 16 == 0)
            {
               fprintf(fp, "%03x : ", off);
            }

            epicsUInt32 val = *(base + off / 4);
            fprintf(fp, "%08x ", val);

            if (off % 16 == 12)
            {
               fprintf(fp, "\n");
            }
         }
      }
      fprintf(fp, "\n");
   }
   if (details >= 2)
   {
      for (unsigned int off = 0x1000; off < 0x5000; off += 4)
      {   // FPGA channel reg.
         if (off % 16 == 0)
         {
            fprintf(fp, "%03x : ", off);
         }

         epicsUInt32 val = *(base + off / 4);
         fprintf(fp, "%08x ", val);

         if (off % 16 == 12)
         {
            fprintf(fp, "\n");
         }
      }
      fprintf(fp, "\n");
   }
}

void Sis3316Port::init()
{
   epicsUInt32 data;
   epicsInt32  r;

   AsynUserSIS tmp(portName);

   asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s: Switch on LED 1\n", portName);
   // connect interrupt
   intOccured = new epicsEvent();

   WRITE(base, SIS3316_CONTROL_STATUS, 0x00000002);
   sis3316_adc1 = new sis3316_adc(base, a32addr);
   if (sis3316_adc1->adc_125MHz_flag == 1)
   {
      asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s: ADC: SIS3316-125MHz-16bit\n", portName);
   }
   else
   {
      asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s: ADC: SIS3316-250MHz-14bit\n", portName);
   }

   WRITE(base, SIS3316_KEY_DISARM, 1);  // disarm sample logic

   // LED 1,2 on/off
   // LEDs in non Application Mode on startup
   WRITE(base, INTERFACE_ACCESS_ARBITRATION_CONTROL, 0x00000001);
   WRITE(base, SIS3316_CONTROL_STATUS, 0x00000010);
   asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s: Switch LED 1 off, LED 2 on\n", portName);
   WRITE(base, SIS3316_CONTROL_STATUS, 0x00020004);

   // adc DAC offset setup
   for (int i_ch = 0; i_ch < SIS3316_NUM_CHANNELS; i_ch++)
   {
      sis3316_adc1->adc_dac_offset_ch_array[i_ch] = 0x8000;    // middle: 5V range -> -/+2.5V; 2V range -> -/+1V
   }
   r = sis3316_adc1->write_all_adc_dac_offsets();
   if (r)
   {
      asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "%s: Error write_all_adc_dac_offsets: return_code = 0x%08x \n", portName, r);
   }

   // channel Gain/Termination setup
   for (int i_ch = 0; i_ch < SIS3316_NUM_CHANNELS; i_ch++)
   {
      sis3316_adc1->adc_gain_termination_ch_array[i_ch] = GAIN_2V;     // (2V Range and 50 Ohm termination)
   }
   r = sis3316_adc1->write_all_gain_termination_values();
   if (r)
   {
      asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "%s: Error write_all_gain_termination_values: return_code = 0x%08x \n", portName, r);
   }

   sis3316_adc1->selectCO(0); // clock out on C0
   for (int i = 0; i < SIS3316_NUM_CHANNELS; i++)
   {
      r = sis3316_adc1->internal_trigger_generation_setup(i, 0, 0, 0);
      if (r)
      {
         asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "%s: internal_trigger_generation_setup(%d, 0,0,0) returns %d\n", portName, i, r);
      }
      r = sis3316_adc1->eventConfig(i, 0x08);   // external trigger enable bit, no polarity invert
   }
   asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s: Switch LED 1 off, LED 2 on\n", portName);
   WRITE(base, SIS3316_CONTROL_STATUS, 0x00020004);



   sis3316_adc1->sampleLength = sampleLength;
   sis3316_adc1->average      = average;
   sis3316_adc1->nofEvents    = nrOfEvents;
   sis3316_adc1->headerLength = 3;  // hardcoded ... TODO
   sis3316_adc1->eventLength  = sis3316_adc1->headerLength + sis3316_adc1->sampleLength / 2;

   sis3316_adc1->addressThreshold = sis3316_adc1->nofEvents * sis3316_adc1->eventLength - 1;

   unsigned int triggerGateWindowLength = sis3316_adc1->sampleLength;
   epicsUInt8   format = 0;

   for (int i = 0; i < SIS3316_NUM_CHANNELS; i++)
   {
      r = sis3316_adc1->dataFormat(i, format);
      if (r)
      {
         asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "%s: dataFormat(%d, %d) returns %d\n", portName, i, format, r);
      }
      r = sis3316_adc1->setMawTestBuffer(i, 0, 0);
      if (r)
      {
         asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "%s: setMawTestBuffer(%d, %d, %d) returns %d\n", portName, i, 0, 0, r);
      }
   }

   for (int i = 0; i < SIS3316_NUM_FPGA_ADC_GROUPS; i++)
   {
      r = sis3316_adc1->setTriggerGateWindowLength(i, ((triggerGateWindowLength - 2) & 0x0ffff));
      if (r)
      {
         asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "%s: setTriggerGateWindowLength(%d, %d) returns %d\n", portName, i, triggerGateWindowLength - 2, r);
      }
      r = sis3316_adc1->setBufferSample(i, sis3316_adc1->sampleLength, 0);
      if (r)
      {
         asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "%s: setBufferSample(%d, %d, %d) returns %d\n", portName, i, sis3316_adc1->sampleLength, 0, r);
      }
      r = sis3316_adc1->setPreTriggerDelay(i, 400);
      if (r)
      {
         asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "%s: setPreTriggerDelay(%d, %d) returns %d\n\n", portName, i, 400, r);
      }
      r = sis3316_adc1->pileupConfig(i, 400, 400);
      if (r)
      {
         asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "%s: pileupConfig(%d, %d, %d) returns %d\n", portName, i, 400, 400, r);
      }
      r = sis3316_adc1->setAddressThreshold(i, sis3316_adc1->addressThreshold);
      if (r)
      {
         asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "%s: setAddressThreshold(%d, %d) returns %d\n", portName, i, sis3316_adc1->addressThreshold, r);
      }
   }

   r = sis3316_adc1->inputT1Trigger(ENABLE_T1_TRIGGER);
   if (r)
   {
      asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "%s: inputT1Trigger(%d) returns %d\n", portName, ENABLE_T1_TRIGGER, r);
   }
   r = sis3316_adc1->setAcquisitionControl(8, 0); //Disable External Trigger function as Trigger Enable
   if (r)
   {
      asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "%s: setAcquisitionControl(%d, %d) returns %d\n", portName, 8, 1, r);
   }
   r = sis3316_adc1->setAcquisitionControl(10, 1); //enable external timestamp clear function
   if (r)
   {
      asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "%s: setAcquisitionControl(%d, %d) returns %d\n", portName, 10, 1, r);
   }
   r = sis3316_adc1->setAcquisitionControl(15, 1); //external trigger function disabled if busy
   if (r)
   {
      asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "%s: setAcquisitionControl(%d, %d) returns %d\n", portName, 15, 1, r);
   }

   WRITE(base, SIS3316_IRQ_CONTROL, 0x00cc0000);  // disable all int sources

   sis3316_adc1->interruptVector = intVector;
   devConnectInterruptVME(sis3316_adc1->interruptVector, &intFuncC, (void *)this);
   //if( BSP_install_isr(sis3316_adc1->interruptVector, (BSP_VME_ISR_t) intFuncC, (void *)this))
   // printf("ERROR: Unable to connect to interrupt vector %d\n", sis3316_adc1->interruptVector);


   /* Write interrupt level to hardware */
   data  = READ(base, SIS3316_IRQ_CONFIG);
   data &= ~0x700;  // irq level bits
   data |= (intLevel & 0x0007) << 8;
   // data = 0x0200; // IRQ-Level 2
   data += sis3316_adc1->interruptVector;
   //data |= 0x1000; // ROAK, does not work ??
   printf(" int dat = 0x%08X\n", data);

   WRITE(base, SIS3316_IRQ_CONFIG, data);
   devEnableInterruptLevelVME((intLevel & 0x0007));

   asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s: Switch LED 1 on, LED 2 off\n", portName);
   WRITE(base, SIS3316_CONTROL_STATUS, 0x00060000);

   //sis3316_adc1->selectTO(25); // external trigger to ADC-FPGA
   //sis3316_adc1->selectUO(2); // sample logic busy

   asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s: Start read out loop\n", portName);

   data = READ(base, SIS3316_ADC_PREVIOUS_BANK_SAMPLE_ADDRESS_REG(0, 0));
   if ((data & 0x1000000) == 0x1000000)    //bank2 flag is set ?
   {
      asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s: bank2 flag is set\n", portName);
      sis3316_adc1->bank1_armed_flag = 1;
      WRITE(base, SIS3316_KEY_DISARM_ARM_BANK1, 1);   // arm bank1
   }
   else
   {
      asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s: bank2 flag is not set\n", portName);
      sis3316_adc1->bank1_armed_flag = 0;
      WRITE(base, SIS3316_KEY_DISARM_ARM_BANK2, 1);   // arm bank2
   }

   /* initialize static parameters */
   AERR(setIntegerParam(P_module_id, sis3316_adc1->module_id));
   AERR(setIntegerParam(P_revision_number, sis3316_adc1->revision_number));
   AERR(setIntegerParam(P_hw_version, sis3316_adc1->hw_version));
   AERR(setIntegerParam(P_self_trigger, 0));  // no auto trigger
   AERR(setDoubleParam(P_trigger_wait, 1.0)); // 1.0s wait for trigger
   AERR(setDoubleParam(P_temp, (double)(float)(READ(base, SIS3316_TEMP) & 0x3ff) / 4.));
}

void Sis3316Port::printInfoBlock()
{
   epicsUInt32 data, aSample, pSample, thresh;

   data = READ(base, SIS3316_AQUISITION_CONTROL);
   printf("InfoBlock\n Acquisition Control: 0x%08X\n", data);

   for (int i = 0; i < 4 /*SIS3316_NUM_CHANNELS */; i++)
   {
      aSample = READ(base, SIS3316_ADC_ACTUAL_SAMPLE_ADDRESS_REG((i >> 2), (i & 3)));
      pSample = READ(base, SIS3316_ADC_PREVIOUS_BANK_SAMPLE_ADDRESS_REG((i >> 2), (i & 3)));
      thresh  = READ(base, SIS3316_ADC_ADDRESS_THRESHOLD_REG(i >> 2));
      printf("Channel %d:\n", i);
      printf("\tACTUAL_SAMPLE_ADDRESS_REG: 0x%08X\tPREVIOUS_SAMPLE_ADDRESS_REG: 0x%08X\tTHRESHOLD: 0x%08X\n",
             aSample, pSample, thresh);
   }
}

void Sis3316Port::sampleTask()
{
   try {
      epicsTime start, end;
      asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s: sample thread starting\n", portName);

      //portLocker guard(*this);

      double sample_volt_factor = (5.0 / 64535.); // +- 5 Volt

      dataCounter = 0;
      selfTrigger = 0;
      triggerWait = 1.0;

      WRITE(base, SIS3316_IRQ_CONTROL, 0x00cc0000); // disable int src's'
      printf("Sample thread ready waiting for signal ...\n ");
      doSample.wait();
      epicsThreadSleep(5.0);                     // give the epicsInittasktime to finish
      WRITE(base, SIS3316_IRQ_CONTROL, 0x4);     // enable edge Threshold ints
      sis3316_adc1->enableInterrupts();
      sis3316_adc1->setAcquisitionControl(8, 1); //External Trigger function as Trigger Enable

      while (true)
      {
         {
            //guard.unlock();

            bool stat;
            do
            {
               stat = intOccured->wait(triggerWait);
               //if (!stat) printf(" wait timeout : %d\n", stat);
               if (selfTrigger)
               {
                  WRITE(base, SIS3316_KEY_TRIGGER, 0);
               }
            } while (!stat);
         }
         try {
            if (sampleDone)
            {
               break;
            }

            //asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s: sample\n", portName);

            /*
             * sAcqCtrlReg = READ( base, SIS3316_AQUISITION_CONTROL);
             * //printf("Sample Info : Acquisition Control: 0x%08X\n", sAcqCtrlReg);
             * //printInfoBlock();
             *
             * if((sAcqCtrlReg & 0x80000) == 0x0) {
             * if (selfTrigger) {
             *     printf(" self trigger...\n");
             *     WRITE(base,SIS3316_KEY_TRIGGER, 0);
             *  }
             * } else {
             */


            if (sis3316_adc1->bank1_armed_flag == 1)
            {
               WRITE(base, SIS3316_KEY_DISARM_ARM_BANK2, 0); // arm bank2
               sis3316_adc1->bank1_armed_flag = 0;
               //printf("arm bank2...\n");
               //asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s: SIS3316_KEY_DISARN_ARM_BANK2 \n", portName);
            }
            else
            {
               WRITE(base, SIS3316_KEY_DISARM_ARM_BANK1, 0); // arm bank1
               sis3316_adc1->bank1_armed_flag = 1;
               //printf("arm bank1...\n");
               //asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s: SIS3316_KEY_DISARN_ARM_BANK1 \n", portName);
            }
            //WRITE(base,SIS3316_KEY_TRIGGER, 0);

            //asynPrint(pasynUserSelf, ASYN_TRACEIO_DEVICE, "%s: No of events: %d\n", portName, sis3316_adc1->nofEvents);
            int rc;
            // channelNr extern defined for cry visil?
            for (int channel = 0; channel < SIS3316_NUM_CHANNELS; channel++)
            {
               rc = sis3316_adc1->read_Channel_PreviousBankDataBuffer(channel);
               if (rc)
               {
                  //asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "%s: rc code from bank1 (if 1) %d read_Channel %d = %d (nr of words: %d)\n",
                  //                                            portName, sis3316_adc1->bank1_armed_flag, rc, channel, sis3316_adc1->nofData[channel]);
                  printf("%s: rc code from bank1 (if 1) %d read_Channel %d = %d (nr of words: %d)\n",
                         portName, sis3316_adc1->bank1_armed_flag, rc, channel, sis3316_adc1->nofData[channel]);
               }
               if (!sis3316_adc1->nofData[channel])
               {
                  asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "%s: no data for channel %d\n", portName, channel);
               }
            }

            // printf(" all channels read \n");

            /* check for equal data  size
             * for (int channel = 1; channel < SIS3316_NUM_CHANNELS; channel++)
             * {
             * if (sis3316_adc1->nofData[0] != sis3316_adc1->nofData[channel])
             * {
             *  printf (" Warning: different # of channel samples (channel %d [%d] != channel 0 [%d])\n",
             *    channel, sis3316_adc1->nofData[channel], sis3316_adc1->nofData[0]);
             * }
             * }
             */
            for (unsigned int ns = 0; ns < sis3316_adc1->nofEvents; ns++)
            {
               for (int channel = 0; channel < SIS3316_NUM_CHANNELS; channel++)
               {
                  epicsUInt32  tsLow, tsHigh, check;
                  epicsUInt32 *pData;
                  epicsUInt32  nofData = 0;
                  pData = (epicsUInt32 *)sis3316_adc1->m_data[channel].data();
                  //printf( "Data pointer for channel %d: %p\n", channel, pData);
                  //printf (" ns * nofData : %d\n", ns * (nofData + sis3316_adc1->headerLength));
                  check = pData[2 + ns * (nofData + sis3316_adc1->headerLength)];
                  if (((check & 0xf0000000) >> 28) != 0xe)
                  {
                     printf("(channel %d) cs != 0x0e (0x%08x)\n", channel, check);
                     printf(" !! at event nr %d (addr: 0x%08x)\n", ns, 2 + ns * (nofData + sis3316_adc1->headerLength));
                     // check that we have valid data
                     printf(" channel %d: 0x%08x\n", channel, pData[2 + ns * (nofData + sis3316_adc1->headerLength)]);
                  }
                  else
                  {
                     // assume all timestamps are the same for the channels sampled
                     tsHigh             = (pData[0 + ns * (nofData + sis3316_adc1->headerLength)] & 0xffff0000) >> 16;
                     tsLow              = pData[1 + ns * (nofData + sis3316_adc1->headerLength)];
                     timeStamp[channel] = ((4294967296.0 * tsHigh) + (tsLow)) * sis3316_adc1->timeBase;
                     //printf(" timeStamp channel %d = %f\n", channel, timeStamp[channel]);
                     nofData = check & 0x03ffffff; //  must not change in one set!
                     if (nofData != sis3316_adc1->nofData[channel] - sis3316_adc1->headerLength)
                     {
                        printf("Warning nofData = %d != nofData[%d] (%d)  \n", nofData, channel, sis3316_adc1->nofData[channel]);
                     }
                     if (nofData)
                     {
                        epicsUInt16 *tempBuf;
                        tempBuf = (epicsUInt16 *)(pData + 3 + ns * (nofData + sis3316_adc1->headerLength));
                        vCh[channel].clear();
                        unsigned int val;
                        double       dx;
                        for (unsigned int l = 0; l < (nofData * 2);)
                        {
                           unsigned int k;
                           val = tempBuf[l];

                           /* as long average > 1 && average%2 == 0
                            * the "sample order" does'nt matter.
                            * otherwise samples must be swapped */
                           for (k = 1; k < sis3316_adc1->average; k++)
                           {
                              val += tempBuf[l + k];
                           }
                           val /= sis3316_adc1->average;
                           val  = val << 2;

                           dx = (double)val * sample_volt_factor - 2.5;
                           vCh[channel].push_back(dx);

                           dataCounter += k;
                           l           += k;
                           //printf(" l = %d\n",l);
                        }
                     }
                  }
                  //printf(" vCh size for channel %d: %d\n",channel,vCh[channel].size());
               }
            }
            // printf(" signal update...\n");
            doUpdate.signal();
            /*Wait for update done ...*/
            //epicsThreadSleep(1.0); // just for testing
            //end = epicsTime::getCurrent();

            //asynPrint(pasynUserSelf, ASYN_TRACEIO_DEVICE, "%s: sample thread taking data took %f seconds\n", portName, end - start);
            //printf( "sample thread taking data took %f seconds\n", end - start);
            //printf("vector sizes : %d, %d, %d, %d\n", struckDataNTTable->data_ts.size(), struckDataNTTable->data_x.size(), struckDataNTTable->data_y.size(), struckDataNTTable->data_y.size());



            //} // if triggered

            // AERR(callParamCallbacks());
         } CATCHALL_CONTINUE(pasynUserSelf)
      }

      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s: sample thread stopping\n", portName);
   } catch (std::exception& e) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s: sample thread died with: %s\n",
                portName, e.what());
   }
}

static void intFuncC(void *drvPvt)
{
   Sis3316Port *pSis3316Port = (Sis3316Port *)drvPvt;

   pSis3316Port->intFunc();
}

void Sis3316Port::intFunc()
{
   //disable interrupt source
   WRITE(base, SIS3316_IRQ_CONTROL, 0x00cc0000);
   intOccured->signal();
   //reenable Interrupt
   //WRITE(base, SIS3316_IRQ_CONTROL, 0x4); // enable edge Threshold ints
   //printk (" hallo ... 0x%08x (0x%08x)\n", READ(base, SIS3316_IRQ_CONTROL), READ(base, SIS3316_IRQ_CONFIG));
   //reenable Interrupt source
   WRITE(base, SIS3316_IRQ_CONTROL, 0x4);
}

void Sis3316Port::updateTask()
{
   try {
      //epicsTime start, end;
      //asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s: update thread starting\n", portName);

/*
 *  // allocate local storage so that the port lock can be released.
 *  std::vector<epicsInt16> requested[AWG_NUM_CHANNELS], actual[AWG_NUM_CHANNELS];
 *  std::vector<bool> update(AWG_NUM_CHANNELS, false),
 *                    isinit(AWG_NUM_CHANNELS, false);
 *
 *  for(size_t i=0; i<AWG_NUM_CHANNELS; i++) {
 *      requested[i].resize(MAX_WAVEFORM_SIZE, 0);
 *      actual[i].resize(MAX_WAVEFORM_SIZE, 0);
 *  }
 */


      //epicsFloat64 *tempBuf;
      //unsigned int bytesUsed, bytesToSend, bytesThanCanBeSent;
      //unsigned int bufLen = 16000 * sizeof(epicsFloat64);

      //tempBuf = (epicsFloat64 *)mallocMustSucceed(bufLen, "tempBuf malloc failed");

//    long long counter = 0;


      /*
       * string provider("pva");
       * string request("");
       * string pv ("CRYVISIL:STM:FASTSCAN:IMAGE_CHUNK");
       * ClientPutPtr ClPut;
       *
       * PvaClientPtr pva = PvaClient::get(provider);
       *
       * ClPut = ClientPut::create(pva,pv,provider,request);
       * while (!(ClPut->getConnected())) {
       * cout << pv << " channel not connected  ... wait ...\n";
       * epicsThreadSleep(1.0);
       * }// wait for connected .....
       */
      //portLocker guard(*this);

      while (true)
      {
         {
            //guard.unlock();
            if (!doUpdate.tryWait())
            {
               doUpdate.wait();
            }
            //start = epicsTime::getCurrent();
            //guard.lock();
         }
         try {
            if (updateDone)
            {
               break;
            }

            unsigned int dataToSend[SIS3316_NUM_CHANNELS];
            //asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s: update\n", portName);
            for (unsigned int i = 0; i < SIS3316_NUM_CHANNELS; i++) // TODO just activated channel
            {
               dataToSend[i] = vCh[i].size();
               sendData[i].resize(dataToSend[i]);
               for (unsigned int x = 0; x < dataToSend[i]; x++)
               {
                  sendData[i][x] = vCh[i][x];
               }
               vCh[i].clear();

               // printf(" !!! will send %d for channel %d\n", dataToSend, i);
               //asynPrint(pasynUserSelf, ASYN_TRACEIO_DEVICE, "%s: Size of channel %d: %d\n", portName, i, dataToSend);
               //bytesThanCanBeSent = epicsMin (bytesUsed, bufLen);

               if (dataToSend[i])
               {
                  //printf( /* asynPrint(pasynUserSelf, ASYN_TRACEIO_DEVICE, */ "%s: Will send %d float64 with function P_read_channe;: \n", portName, dataToSend);
                  AERR(doCallbacksFloat64Array(sendData[i].data(), dataToSend[i], P_read_channel, i));
               }
            }

            // end = epicsTime::getCurrent();
            //asynPrint(pasynUserSelf, ASYN_TRACEIO_DEVICE, "%s: update thread taking data took %f seconds\n", portName, end - start);

            // AERR(callParamCallbacks());
         } CATCHALL_CONTINUE(pasynUserSelf)
      }

      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s: update thread stopping\n", portName);
      printf("update thread stopping\n");
   } catch (std::exception& e) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s: update thread died with: %s\n",
                portName, e.what());
      printf("update thread died\n");
   }
}

void Sis3316Port::clearTimestamp()
{
   asynPrint(pasynUserSelf, ASYN_TRACEIO_DEVICE, "%s: Clear Timestamp ... now ...\n", portName);
   WRITE(base, SIS3316_KEY_TIMESTAMP_CLR, 1);
}

void Sis3316Port::softTrigger()
{
   asynPrint(pasynUserSelf, ASYN_TRACEIO_DEVICE, "%s: ... Soft Trigger ... now ...\n", portName);
   WRITE(base, SIS3316_KEY_TRIGGER, 1);
}
