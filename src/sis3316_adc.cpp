/* ---------------------------
 *
 * a very first imeplementation derived from struck sources
 */
#include "sis3316_adc.h"

#include <stdio.h>
#include <string.h>
#include <cantProceed.h>
#include <fcntl.h>

#include "epicsThread.h"
#include "epicsTime.h"

#include <devLib.h>

#ifdef __rtems__
#include <bsp/VMEDMA.h>
#include <bsp/vme_am_defs.h>
#endif

static void dmaCallbackC(void *drvPtr);

sis3316_adc::sis3316_adc(volatile epicsUInt32 *baseaddress, epicsUInt32 hwAddr) :
   base(baseaddress),
   a32addr(hwAddr),
   eventLength(SIS3316_MAX_DMA_SAMPLES),
   m_data(SIS3316_NUM_CHANNELS, std::vector <epicsUInt32>(SIS3316_MAX_DMA_SAMPLES, 0))
{
   int          return_code;
   unsigned int data;
   unsigned int i_ch;

   WRITE(base, SIS3316_KEY_RESET, 1); // reset to power state
   epicsThreadSleep(1.0);             // to be sure ...
   WRITE(base, INTERFACE_ACCESS_ARBITRATION_CONTROL, 0x00000001);
   //printf(" sis3316_adc: base = 0x%x\n", (unsigned int)base);
   // asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s: sis3316_adc base = 0x08X\n", portName, base);
   WRITE(base, SIS3316_IRQ_CONTROL, 0x00cc0000); // disable int src's'
   disableInterrupts();

   //get_frequency(0,freqSI570_calibrated_value_125MHz );
   adc_125MHz_flag = 0; //set to 250 MHz

   // ADC chip setup via SPI
   return_code = this->adc_spi_setup();
   if (return_code != 0)
   {
      printf("Error adc_spi_setup: return_code = 0x%08x \n", return_code);
   }
   //printf("adc_spi_setup: adc_125MHz_flag = %d \n", adc_125MHz_flag);

   //printf(" check check DCM_RESET......\n");
   int ok = 0;
   for (int i = 0; i < SIS3316_NUM_FPGA_ADC_GROUPS; i++)
   {
      //printf(" SIS3316_ADC_FPGA_STATUS(%d): 0x%08X\n", i, READ(base,SIS3316_ADC_FPGA_STATUS(i)));
      ok += (READ(base, SIS3316_ADC_FPGA_STATUS(i)) & 0x0100000);
   }
   if (ok != 0x400000)
   {
      if (return_code)
      {
         printf(" not all DCMs resetted , should we give up?\n");
      }
   }
   WRITE(base, SIS3316_KEY_TIMESTAMP_CLR, 1); // clear timestamp counter

   unsigned int clock_N1div, clock_HSdiv, r;
   printf(" Boards 125 MHz : %d\n", adc_125MHz_flag);

   if (adc_125MHz_flag == 0)  //250Mhz
   {
      clock_N1div     = 4;
      clock_HSdiv     = 5;
      iob_delay_value = 0x1002;
      timeBase        = 4; // 4ns
   }
   else
   {
      clock_N1div     = 8;
      clock_HSdiv     = 5;
      iob_delay_value = 0x1008;
      timeBase        = 8; // 8ns
   }

   /* There should be a command to change frequencies
    * printf(" Set to 62. MHz ... test test\n");
    * clock_N1div = 16;
    * clock_HSdiv = 5;
    * iob_delay_value = 0x1060;
    * timeBase = 4; // 16ns
    */

   ok = 0;
   for (int i = 0; i < SIS3316_NUM_FPGA_ADC_GROUPS; i++)
   {
      /* to "see" that we have no VME-issue. Seen with MVME6100 as VMEmaster and SIS3316 */
      printf(" SIS3316_ADC_FPGA_STATUS(%d): 0x%08X\n", i, READ(base, SIS3316_ADC_FPGA_STATUS(i)));
      ok += (READ(base, SIS3316_ADC_FPGA_STATUS(i)) & 0x0100000);
   }
   if (ok != 0x400000)
   {
      printf(" not all DCMs resetted ... will reset once\n");
      WRITE(base, SIS3316_KEY_ADC_CLOCK_DCM_RESET, 0);
   }

   //Clear Link Error Latch bits, manual page 89
   for (int i = 0; i < SIS3316_NUM_FPGA_ADC_GROUPS; i++)
   {
      WRITE(base, SIS3316_ADC_INP_TAP_DLY(i), 0x400);
   }

   //Clear error latch bits, manual page 87
   WRITE(base, VME_FPGA_LINK_ADC_PROT_STATUS, 0xE0E0E0E0);
   epicsUInt32 status;
   //Check that latched error bits (after power up) are cleared, manual page 114
   for (int i = 0; i < SIS3316_NUM_FPGA_ADC_GROUPS; i++)
   {
      status  = READ(base, SIS3316_ADC_FPGA_STATUS(i));
      status &= 0x000000FF;
      if (status != 0x18)
      {
         printf(" Unknown ADC_FPGA_STATUS 0x%x (expected 0x18)\n", status);
      }
   }

   r = change_frequency_HSdiv_N1div(0, clock_HSdiv, clock_N1div);
   if (r)
   {
      printf(" change_frequency_HSdiv_N1div(0, %d, %d) returns %d\n", clock_HSdiv, clock_N1div, r);
   }
   r = configure_adc_fpga_iob_delays(iob_delay_value);
   if (r)
   {
      printf(" configure_adc_fpga_iob_delay ( %d ) returns %d\n", iob_delay_value, r);
   }
   printf(" Switch LED 2 off, LED 1 on\n");
   WRITE(base, SIS3316_CONTROL_STATUS, 0x00040002);

   // write Header ID registers
   for (int i = 0; i < SIS3316_NUM_FPGA_ADC_GROUPS; i++)
   {
      setHeaderId(i, ((uintptr_t)base & 0xff000000));
   }
   // adc DAC offset configuration
   return_code = configure_all_adc_dac_offsets();
   if (return_code != 0)
   {
      printf("Error configure_all_adc_dac_offsets: return_code = %d \n", return_code);
   }
   // adc DAC offset setup
   for (i_ch = 0; i_ch < SIS3316_NUM_CHANNELS; i_ch++)
   {
      adc_dac_offset_ch_array[i_ch] = 0x8000;  // middle: 5V range -> -/+2.5V; 2V range -> -/+1V
   }
   return_code = write_all_adc_dac_offsets();
   if (return_code != 0)
   {
      printf("Error write_all_adc_dac_offsets: return_code = %d \n", return_code);
   }
   // channel Gain/Termination setup
   for (i_ch = 0; i_ch < SIS3316_NUM_CHANNELS; i_ch++)
   {
      adc_gain_termination_ch_array[i_ch] = GAIN_2V;  // (2V Range and 50Ohm termination)
   }
   return_code = this->write_all_gain_termination_values();
   if (return_code != 0)
   {
      printf("Error write_all_gain_termination_values: return_code = %d \n", return_code);
   }

   this->adc_spi_reg_enable_adc_outputs();  // necessary after reset

   data = READ(base, SIS3316_SERIAL_NUMBER_REG);
   if (data & 0x10000)
   {
      printf("Error serial number not valid\n");
      serial_number = 4711;
   }
   else
   {
      serial_number = (READ(base, SIS3316_SERIAL_NUMBER_REG) & 0x0000ffff);
   }
   for (int i = 0; i < SIS3316_NUM_FPGA_ADC_GROUPS; i++)
   {
      adc_fpga_version[i] = READ(base, SIS3316_ADC_FPGA_FIRMWARE_REG(i)); // read CH1_4
   }
   module_id       = (READ(base, SIS3316_MOD_ID) & 0xFFFF0000) >> 16;
   revision_number = (READ(base, SIS3316_MOD_ID) & 0x0000FFFF);
   hw_version      = READ(base, SIS3316_HW_VERSION);

   if (1)
   {
      printf("sis3316:\n fpga[0]_version:0x%x, fpga[1]_version:0x%x, fpga[2]_version:0x%x, fpga[3]_version:0x%x\n",
             adc_fpga_version[0], adc_fpga_version[1], adc_fpga_version[2], adc_fpga_version[3]);
      printf("module_id: 0x%x, revision_number: 0x%x, hw_version: 0x%x\n", module_id, revision_number, hw_version);
   }

   dmaDoneEventId_ = epicsEventCreate(epicsEventEmpty);

  #ifdef __rtems__
   if (BSP_VMEDmaSetup(1, BSP_VMEDMA_OPT_THROUGHPUT /*BSP_VMEDMA_OPT_LOWLATENCY*/, (VME_AM_EXT_SUP_MBLT | BSP_VMEDMA_MODE_NOINC_VME), 0))
   {
      printf("ERROR: Unable to setup DMA\n");
   }

   if (BSP_VMEDmaInstallISR(1, dmaCallbackC, (void *)this))
   {
      printf("ERROR: Unable to install DMA ISR\n");
   }
 #endif
}

/* DMA handling*/
static void dmaCallbackC(void *drvPvt)
{
   sis3316_adc *pSis3316 = (sis3316_adc *)drvPvt;

   pSis3316->dmaCallback();
}

void sis3316_adc::dmaCallback()
{
   epicsEventSignal(dmaDoneEventId_);
}

/* Interrupt handling */
void sis3316_adc::enableInterrupts()
{
   epicsUInt32 data;

   data  = READ(base, SIS3316_IRQ_CONFIG);
   data |= 0x800;
   WRITE(base, SIS3316_IRQ_CONFIG, data);
}

void sis3316_adc::disableInterrupts()
{
   epicsUInt32 data;

   data  = READ(base, SIS3316_IRQ_CONFIG);
   data &= ~0x800;
   WRITE(base, SIS3316_IRQ_CONFIG, data);
   // SIS3316IRQ_ENABLE
}

int sis3316_adc::update_firmware(char *path, int offset, void (*cb)(int percentage))
{
   int   rc;
   FILE *fp;
   char *buf;
   int   fileSize;
   int   percent, percent_old;

   if (path == NULL)
   {
      return(-100);
   }

   fp = fopen(path, "rb");
   if (fp == NULL)
   {
      return(-101);
   }

   fseek(fp, 0, SEEK_END);
   fileSize = ftell(fp);
   rewind(fp);

   buf = (char *)callocMustSucceed(1, fileSize, "sis3316_adc::update_firmware");
   if (buf == NULL)
   {
      return(-102);
   }

   rc = fread(buf, 1, fileSize, fp);
   if (rc != fileSize)
   {
      return(-103);
   }

   fclose(fp);

   percent = percent_old = 0;
   if (cb)
   {
      (cb)(percent);
   }

   int written = 0;
   int pageProgramSize;

   this->FlashEnableProg();

   while (written < fileSize)
   {
      // erase
      if ((written & (SIS3316_FLASH_ERASE_BLOCKSIZE - 1)) == 0)
      {
         rc = this->FlashEraseBlock((offset + written) & 0xFFFF0000);
      }

      if (fileSize >= (written + SIS3316_FLASH_PROGRAM_PAGESIZE))
      {
         pageProgramSize = SIS3316_FLASH_PROGRAM_PAGESIZE;
      }
      else
      {
         pageProgramSize = fileSize - written;
      }

      rc = this->FlashProgramPage(offset + written, buf + written, pageProgramSize);

      written += pageProgramSize;

      if (cb)
      {
         percent = written * 100 / fileSize;
         if (percent != percent_old)
         {
            (cb)(percent);
            percent_old = percent;
         }
      }
   }

   this->FlashDisableProg();

   free(buf);

   return(0);
}

int sis3316_adc::FlashEnableProg()
{
   epicsUInt32 tmp;

   tmp  = READ(base, SIS3316_SPI_FLASH_CSR);
   tmp |= (1 << ENABLE_SPI_PROG);
   WRITE(base, SIS3316_SPI_FLASH_CSR, tmp);

   return(0);
}

int sis3316_adc::FlashDisableProg()
{
   epicsUInt32 tmp;

   tmp  = READ(base, SIS3316_SPI_FLASH_CSR);
   tmp &= ~(1 << ENABLE_SPI_PROG);
   WRITE(base, SIS3316_SPI_FLASH_CSR, tmp);

   return(0);
}

int sis3316_adc::FlashEnableCS(int chip)
{
   epicsUInt32 tmp;

   tmp = READ(base, SIS3316_SPI_FLASH_CSR);

   switch (chip)
   {
   case 0:
      tmp |= (1 << CHIPSELECT_1);
      break;

   case 1:
      tmp |= (1 << CHIPSELECT_2);
      break;

   default:
      return(-100);
   }

   WRITE(base, SIS3316_SPI_FLASH_CSR, tmp);

   return(0);
}

int sis3316_adc::FlashDisableCS(int chip)
{
   epicsUInt32 tmp;

   tmp = READ(base, SIS3316_SPI_FLASH_CSR);

   switch (chip)
   {
   case 0:
      tmp &= ~(1 << CHIPSELECT_1);
      break;

   case 1:
      tmp &= ~(1 << CHIPSELECT_2);
      break;

   default:
      return(-100);
   }

   WRITE(base, SIS3316_SPI_FLASH_CSR, tmp);

   return(0);
}

int sis3316_adc::FlashWriteEnable()
{
   int rc;

   rc = this->FlashEnableCS(0);
   if (rc)
   {
      return(rc);
   }

   rc = this->FlashXfer(0x06, NULL);      // Write Enable command
   if (rc)
   {
      this->FlashDisableCS(0);
      return(rc);
   }

   rc = this->FlashDisableCS(0);
   if (rc)
   {
      return(rc);
   }

   return(0);
}

int sis3316_adc::FlashProgramPage(int address, char *data, int len)
{
   int         rc;
   char        tmp;
   epicsUInt32 utmp;

   //epicsUInt32 dmabuf[SIS3316_FLASH_PROGRAM_PAGESIZE];
   // epicsUInt32 putWords;

   if (data == NULL)
   {
      return(-100);
   }

   rc = this->FlashWriteEnable();
   if (rc)
   {
      return(rc);
   }

   // program command
   rc = this->FlashEnableCS(0);
   if (rc)
   {
      return(rc);
   }

   rc = this->FlashXfer(0x02, NULL);      // Page program command
   if (rc)
   {
      this->FlashDisableCS(0);
      return(rc);
   }
   rc = this->FlashXfer((char)(address >> 16), NULL);
   if (rc)
   {
      this->FlashDisableCS(0);
      return(rc);
   }
   rc = this->FlashXfer((char)(address >> 8), NULL);
   if (rc)
   {
      this->FlashDisableCS(0);
      return(rc);
   }
   rc = this->FlashXfer((char)address, NULL);
   if (rc)
   {
      this->FlashDisableCS(0);
      return(rc);
   }

   /*
    * // dma d32
    * for(int k = 0;k < SIS3316_FLASH_PROGRAM_PAGESIZE;k++){
    *      dmabuf[k] = (epicsUInt32)*(data + k);
    * }
    *
    * rc = i->vme_A32DMA_D32FIFO_write(this->baseaddress + SIS3316_SPI_FLASH_DATA, dmabuf, SIS3316_FLASH_PROGRAM_PAGESIZE, &putWords);
    * if(rc || putWords != SIS3316_FLASH_PROGRAM_PAGESIZE){
    *      return -101;
    * }
    */
   // busy polling
   epicsThreadSleep(0.01); // testing
   do
   {
      utmp  = READ(base, SIS3316_SPI_FLASH_CSR);
      utmp &= (1 << FLASH_LOGIC_BUSY ^ 1 << FIFO_NOT_EMPTY);
   } while (utmp != 0);

   // single cycles
   for (int i = 0; i < len && i < SIS3316_FLASH_PROGRAM_PAGESIZE; i++)
   {
      rc = this->FlashXfer(*(data + i), NULL);
      if (rc)
      {
         this->FlashDisableCS(0);
         return(rc);
      }
   }

   rc = this->FlashDisableCS(0);
   if (rc)
   {
      return(rc);
   }
   epicsThreadSleep(0.01);
   // busy polling
   do
   {
      rc = this->FlashEnableCS(0);
      if (rc)
      {
         return(rc);
      }

      rc = this->FlashXfer(0x05, NULL);           // read status register 1 command
      if (rc)
      {
         this->FlashDisableCS(0);
         return(rc);
      }
      rc = this->FlashXfer(0, &tmp);
      if (rc)
      {
         this->FlashDisableCS(0);
         return(rc);
      }

      tmp &= 1;

      rc = this->FlashDisableCS(0);
      if (rc)
      {
         return(rc);
      }
      epicsThreadSleep(0.01);
   } while (tmp);

   return(0);
}

int sis3316_adc::FlashEraseBlock(int address)
{
   int  rc;
   char tmp;

   rc = this->FlashWriteEnable();
   if (rc)
   {
      return(rc);
   }

   // erase command
   rc = this->FlashEnableCS(0);
   if (rc)
   {
      return(rc);
   }

   rc = this->FlashXfer(0xD8, NULL);      // 64kB Block erase command
   if (rc)
   {
      this->FlashDisableCS(0);
      return(rc);
   }
   rc = this->FlashXfer((char)(address >> 16), NULL);
   if (rc)
   {
      this->FlashDisableCS(0);
      return(rc);
   }
   rc = this->FlashXfer((char)(address >> 8), NULL);
   if (rc)
   {
      this->FlashDisableCS(0);
      return(rc);
   }
   rc = this->FlashXfer((char)address, NULL);
   if (rc)
   {
      this->FlashDisableCS(0);
      return(rc);
   }

   rc = this->FlashDisableCS(0);
   if (rc)
   {
      return(rc);
   }
   epicsThreadSleep(0.01);      // testing
   // busy polling
   do
   {
      rc = this->FlashEnableCS(0);
      if (rc)
      {
         this->FlashDisableCS(0);
         return(rc);
      }

      rc = this->FlashXfer(0x05, NULL);           // read status register 1 command
      if (rc)
      {
         this->FlashDisableCS(0);
         return(rc);
      }
      rc = this->FlashXfer(0, &tmp);
      if (rc)
      {
         this->FlashDisableCS(0);
         return(rc);
      }

      tmp &= 1;

      rc = this->FlashDisableCS(0);
      if (rc)
      {
         return(rc);
      }
      epicsThreadSleep(0.01);
   } while (tmp);

   return(0);
}

int sis3316_adc::FlashXfer(char in, char *out)
{
   epicsUInt32 tmp;
   char        ctmp;

   tmp = (epicsUInt32)in;
   WRITE(base, SIS3316_SPI_FLASH_DATA, tmp);

   do
   {
      tmp  = READ(base, SIS3316_SPI_FLASH_CSR);
      tmp &= (1 << FLASH_LOGIC_BUSY ^ 1 << FIFO_NOT_EMPTY);
      epicsThreadSleep(0.01);
   } while (tmp != 0);

   tmp = READ(base, SIS3316_SPI_FLASH_DATA);

   ctmp = tmp & 0xFF;
   if (out)
   {
      *out = ctmp;
   }

   return(0);
}

int sis3316_adc::FlashGetId(char *id)
{
   int rc;

   if (id == NULL)
   {
      return(-100);
   }

   rc = this->FlashEnableCS(0);
   if (rc)
   {
      this->FlashDisableCS(0);
      return(rc);
   }

   rc = this->FlashXfer(0x9F, NULL);      // JEDEC ID command
   if (rc)
   {
      this->FlashDisableCS(0);
      return(rc);
   }
   rc = this->FlashXfer(0, id);
   if (rc)
   {
      this->FlashDisableCS(0);
      return(rc);
   }
   rc = this->FlashXfer(0, id + 1);
   if (rc)
   {
      this->FlashDisableCS(0);
      return(rc);
   }
   rc = this->FlashXfer(0, id + 2);
   if (rc)
   {
      this->FlashDisableCS(0);
      return(rc);
   }

   rc = this->FlashDisableCS(0);
   if (rc)
   {
      return(rc);
   }

   return(0);
}

/****************************************************************************************************/

int sis3316_adc::si5325_clk_muliplier_write(unsigned int addr, unsigned int data)
{
   unsigned int write_data, read_data;
   unsigned int poll_counter;

   // write address
   write_data = 0x0000 + (addr & 0xff);   // write ADDR Instruction + register addr
   WRITE(base, SIS3316_NIM_CLK_MULTIPLIER_SPI_REG, write_data);
   epicsThreadSleep(0.01);

   poll_counter = 0;
   do
   {
      poll_counter++;
      read_data = READ(base, SIS3316_NIM_CLK_MULTIPLIER_SPI_REG);
   } while ((read_data & 0x80000000) && (poll_counter < SI5325_SPI_POLL_COUNTER_MAX));
   if (poll_counter == SI5325_SPI_POLL_COUNTER_MAX)
   {
      return(-2);
   }
   epicsThreadSleep(0.01);

   // write data
   write_data = 0x4000 + (data & 0xff);   // write Instruction + data
   WRITE(base, SIS3316_NIM_CLK_MULTIPLIER_SPI_REG, write_data);
   epicsThreadSleep(0.01);

   poll_counter = 0;
   do
   {
      poll_counter++;
      read_data = READ(base, SIS3316_NIM_CLK_MULTIPLIER_SPI_REG);
   } while ((read_data & 0x80000000) && (poll_counter < SI5325_SPI_POLL_COUNTER_MAX));
   if (poll_counter == SI5325_SPI_POLL_COUNTER_MAX)
   {
      return(-2);
   }

   return(0);
}

/****************************************************************************************************/

int sis3316_adc::si5325_clk_muliplier_read(unsigned int addr, unsigned int *data)
{
   unsigned int write_data, read_data;
   unsigned int poll_counter;

   // read address
   write_data = 0x0000 + (addr & 0xff);   // read ADDR Instruction + register addr
   WRITE(base, SIS3316_NIM_CLK_MULTIPLIER_SPI_REG, write_data);

   poll_counter = 0;
   do
   {
      poll_counter++;
      read_data = READ(base, SIS3316_NIM_CLK_MULTIPLIER_SPI_REG);
   } while ((read_data & 0x80000000) && (poll_counter < SI5325_SPI_POLL_COUNTER_MAX));
   if (poll_counter == SI5325_SPI_POLL_COUNTER_MAX)
   {
      return(-2);
   }
   epicsThreadSleep(0.01);

   // read data
   write_data = 0x8000;    // read Instruction + data
   WRITE(base, SIS3316_NIM_CLK_MULTIPLIER_SPI_REG, write_data);
   epicsThreadSleep(0.01);

   poll_counter = 0;
   do
   {
      poll_counter++;
      read_data = READ(base, SIS3316_NIM_CLK_MULTIPLIER_SPI_REG);
   } while ((read_data & 0x80000000) && (poll_counter < SI5325_SPI_POLL_COUNTER_MAX));
   if (poll_counter == SI5325_SPI_POLL_COUNTER_MAX)
   {
      return(-2);
   }
   //*data = (read_data & 0xff) ;
   *data = (read_data);
   return(0);
}

/****************************************************************************************************/


int sis3316_adc::si5325_clk_muliplier_internal_calibration_cmd(void)
{
   unsigned int write_data, read_data;
   unsigned int poll_counter, cal_poll_counter;

   // write address
   write_data = 0x0000 + 136;   // write ADDR Instruction + register addr
   WRITE(base, SIS3316_NIM_CLK_MULTIPLIER_SPI_REG, write_data);

   poll_counter = 0;
   do
   {
      poll_counter++;
      read_data = READ(base, SIS3316_NIM_CLK_MULTIPLIER_SPI_REG);
   } while ((read_data & 0x80000000) && (poll_counter < SI5325_SPI_POLL_COUNTER_MAX));
   if (poll_counter == SI5325_SPI_POLL_COUNTER_MAX)
   {
      return(-2);
   }

   // write data
   write_data = 0x4000 + 0x40;   // write Instruction + data
   WRITE(base, SIS3316_NIM_CLK_MULTIPLIER_SPI_REG, write_data);

   poll_counter = 0;
   do
   {
      poll_counter++;
      read_data = READ(base, SIS3316_NIM_CLK_MULTIPLIER_SPI_REG);
   } while ((read_data & 0x80000000) && (poll_counter < SI5325_SPI_POLL_COUNTER_MAX));
   if (poll_counter == SI5325_SPI_POLL_COUNTER_MAX)
   {
      return(-2);
   }

   // poll until Calibration is ready
   cal_poll_counter = 0;
   do
   {
      cal_poll_counter++;
      // read data
      write_data = 0x8000;     // read Instruction + data
      WRITE(base, SIS3316_NIM_CLK_MULTIPLIER_SPI_REG, write_data);

      poll_counter = 0;
      do
      {
         poll_counter++;
         read_data = READ(base, SIS3316_NIM_CLK_MULTIPLIER_SPI_REG);
      } while ((read_data & 0x80000000) && (poll_counter < SI5325_SPI_POLL_COUNTER_MAX));
      if (poll_counter == SI5325_SPI_POLL_COUNTER_MAX)
      {
         return(-2);
      }
   } while (((read_data & 0x40) == 0x40) && (cal_poll_counter < SI5325_SPI_CALIBRATION_READY_POLL_COUNTER_MAX));
   if (cal_poll_counter == SI5325_SPI_CALIBRATION_READY_POLL_COUNTER_MAX)
   {
      return(-3);
   }

   return(0);
}

/****************************************************************************************************/


int sis3316_adc::get_status_external_clock_multiplier(unsigned int *status)
{
   int          rc;
   unsigned int data;

   rc      = si5325_clk_muliplier_read(128, &data); //
   *status = data & 0x1;
   rc      = si5325_clk_muliplier_read(129, &data); //
   *status = *status + (data & 0x2);
   return(rc);
}

/****************************************************************************************************/

int sis3316_adc::bypass_external_clock_multiplier(void)
{
   int rc;

   rc = si5325_clk_muliplier_write(0, 0x2);      // Bypass
   rc = si5325_clk_muliplier_write(11, 0x02);    //  PowerDown clk2
   return(rc);
}

/****************************************************************************************************/

int sis3316_adc::set_external_clock_multiplier(unsigned int bw_sel, unsigned int n1_hs, unsigned int n1_clk1, unsigned int n1_clk2, unsigned int n2, unsigned int n3, unsigned int clkin1_mhz)
{
   // int rc;
   volatile unsigned int n1_val;
   volatile unsigned int n1_hs_val;
   volatile unsigned int n2_val;
   volatile unsigned int n3_val;

   // input frequency
   if ((clkin1_mhz < 10) || (clkin1_mhz > 250))
   {
      return(-2);
   }
   // bw_sel : see DSPLLsinm for setting
   if (bw_sel > 15)
   {
      return(-3);
   }
   // n1_hs
   if ((n1_hs < 4) || (n1_hs > 11))
   {
      return(-4);
   }


   // n1_clk1
   if (n1_clk1 == 0)
   {
      return(-5);
   }
   else
   {
      if ((((n1_clk1) & 0x1) == 1) && (n1_clk1 != 1))              // odd but not 1
      {
         return(-5);
      }
      if ((n1_clk1 & 0xfff00000) != 0)              // > 2**20
      {
         return(-5);
      }
   }

   // n1_clk2
   if (n1_clk2 == 0)
   {
      return(-6);
   }
   else
   {
      if ((((n1_clk2) & 0x1) == 1) && (n1_clk2 != 1))              // odd but not 1
      {
         return(-6);
      }
      if ((n1_clk2 & 0xfff00000) != 0)              // > 2**20
      {
         return(-6);
      }
   }


   // n2
   if ((n2 < 32) || (n2 > 512))
   {
      return(-7);
   }
   else
   {
      if ((n2 & 0x1) == 1)              // odd
      {
         return(-7);
      }
   }

   // n3
   if (n3 == 0)
   {
      return(-8);
   }
   else
   {
      if ((n3 & 0xfff80000) != 0)              // > 2**19
      {
         return(-8);
      }
   }


   si5325_clk_muliplier_write(0, 0x0);      // No Bypass
   si5325_clk_muliplier_write(11, 0x02);    //  PowerDown clk2

   // N3 = 1
   n3_val = n3 - 1;
   si5325_clk_muliplier_write(43, ((n3_val >> 16) & 0x7));      //  N3 bits 18:16
   si5325_clk_muliplier_write(44, ((n3_val >> 8) & 0xff));      //  N3 bits 15:8
   si5325_clk_muliplier_write(45, (n3_val & 0xff));             //  N3 bits 7:0

   n2_val = n2;
   si5325_clk_muliplier_write(40, 0x00);                           //    N2_LS bits 19:16
   si5325_clk_muliplier_write(41, ((n2_val >> 8) & 0xff));         //  N2_LS bits 15:8
   si5325_clk_muliplier_write(42, (n2_val & 0xff));                //  N2_LS bits 7:0

   n1_hs_val = n1_hs - 4;
   si5325_clk_muliplier_write(25, (n1_hs_val << 5));      //

   n1_val = n1_clk1 - 1;
   si5325_clk_muliplier_write(31, ((n1_val >> 16) & 0xf));      //  NC1_LS (low speed divider) bits 19:16
   si5325_clk_muliplier_write(32, ((n1_val >> 8) & 0xff));      //  NC1_LS (low speed divider) bits 15:8
   si5325_clk_muliplier_write(33, (n1_val & 0xff));             //  NC1_LS (low speed divider) bits 7:0

   n1_val = n1_clk2 - 1;
   si5325_clk_muliplier_write(34, ((n1_val >> 16) & 0xf));     //  NC2_LS (low speed divider) bits 19:16
   si5325_clk_muliplier_write(35, ((n1_val >> 8) & 0xff));     //  NC2_LS (low speed divider) bits 15:8
   si5325_clk_muliplier_write(36, (n1_val & 0xff));            //  NC2_LS (low speed divider) bits 7:0


   si5325_clk_muliplier_write(2, (bw_sel << 5));        //BWSEL_REG

   si5325_clk_muliplier_internal_calibration_cmd();

   return(0);
}

/************************************************************************************************************************************************/

// adc_fpga_group: 0,1,2,3
// adc_chip: 0 or 1
//				-1 : not all adc chips have the same chip ID
//				>0 : VME Error Code

int sis3316_adc::adc_spi_setup(void)
{
   int          return_code;
   unsigned int adc_chip_id;
   unsigned int data;
   unsigned     i_adc_fpga_group;
   unsigned     i_adc_chip;

   // disable ADC output
   for (i_adc_fpga_group = 0; i_adc_fpga_group < SIS3316_NUM_FPGA_ADC_GROUPS; i_adc_fpga_group++)
   {
      WRITE(base, SIS3316_ADC_SPI_CTRL_REG(i_adc_fpga_group), 0x0);
   }

   // dummy loop to access each adc chip one time after power up
   for (i_adc_fpga_group = 0; i_adc_fpga_group < SIS3316_NUM_FPGA_ADC_GROUPS; i_adc_fpga_group++)
   {
      for (i_adc_chip = 0; i_adc_chip < SIS3316_NR_ADC_PER_FPGA; i_adc_chip++)
      {
         adc_spi_read(i_adc_fpga_group, i_adc_chip, 1, &data);
      }
   }
   //reset
   for (i_adc_fpga_group = 0; i_adc_fpga_group < SIS3316_NUM_FPGA_ADC_GROUPS; i_adc_fpga_group++)
   {
      for (i_adc_chip = 0; i_adc_chip < SIS3316_NR_ADC_PER_FPGA; i_adc_chip++)
      {
         return_code = adc_spi_write(i_adc_fpga_group, i_adc_chip, 0x0, 0x24); // soft reset
         if (return_code != 0)
         {
            return(return_code);
         }
      }
      epicsThreadSleep(0.1); // after reset
   }

   return_code = adc_spi_read(0, 0, 1, &adc_chip_id);   // read chip Id from adc chips ch1/2

   printf(" adc_chip_id = 0x%x\n", adc_chip_id);

   for (i_adc_fpga_group = 0; i_adc_fpga_group < SIS3316_NUM_FPGA_ADC_GROUPS; i_adc_fpga_group++)
   {
      for (i_adc_chip = 0; i_adc_chip < SIS3316_NR_ADC_PER_FPGA; i_adc_chip++)
      {
         adc_spi_read(i_adc_fpga_group, i_adc_chip, 1, &data);
         if (data != adc_chip_id)
         {
            printf("i_adc_fpga_group = %d   i_adc_chip = %d    data = 0x%08x     adc_chip_id = 0x%08x     \n", i_adc_fpga_group, i_adc_chip, data, adc_chip_id);
            return(-1);
         }
      }
   }

   adc_125MHz_flag = 0;
   if ((adc_chip_id & 0xff) == 0x32)
   {
      adc_125MHz_flag = 1;
   }

   // reg 0x14 : Output mode
   if (adc_125MHz_flag == 0) // 250 MHz chip AD9643
   {
      data = 0x04;           //  Output inverted (bit2 = 1)
   }
   else                      // 125 MHz chip AD9268
   {
      data = 0x40;           // Output type LVDS (bit6 = 1), Output inverted (bit2 = 0) !
   }
   for (i_adc_fpga_group = 0; i_adc_fpga_group < SIS3316_NUM_FPGA_ADC_GROUPS; i_adc_fpga_group++)
   {
      for (i_adc_chip = 0; i_adc_chip < SIS3316_NR_ADC_PER_FPGA; i_adc_chip++)
      {
         adc_spi_write(i_adc_fpga_group, i_adc_chip, 0x14, data);
      }
   }

   // reg 0x18 : Reference Voltage / Input Span
   if (adc_125MHz_flag == 0) // 250 MHz chip AD9643
   {
      data = 0x0;            //  1.75V
   }
   else                      // 125 MHz chip AD9268
   {                         //data = 0x8 ;    //  1.75V
      data = 0xC0;           //  2.0V
   }
   for (i_adc_fpga_group = 0; i_adc_fpga_group < SIS3316_NUM_FPGA_ADC_GROUPS; i_adc_fpga_group++)
   {
      for (i_adc_chip = 0; i_adc_chip < SIS3316_NR_ADC_PER_FPGA; i_adc_chip++)
      {
         adc_spi_write(i_adc_fpga_group, i_adc_chip, 0x18, data);
      }
   }

   // reg 0xff : register update
   data = 0x01;         // update
   for (i_adc_fpga_group = 0; i_adc_fpga_group < SIS3316_NUM_FPGA_ADC_GROUPS; i_adc_fpga_group++)
   {
      for (i_adc_chip = 0; i_adc_chip < SIS3316_NR_ADC_PER_FPGA; i_adc_chip++)
      {
         adc_spi_write(i_adc_fpga_group, i_adc_chip, 0xff, data);
      }
   }
   if (0)
   {
      for (i_adc_fpga_group = 0; i_adc_fpga_group < SIS3316_NUM_FPGA_ADC_GROUPS; i_adc_fpga_group++)
      {
         for (i_adc_chip = 0; i_adc_chip < SIS3316_NR_ADC_PER_FPGA; i_adc_chip++)
         {
            adc_spi_read(i_adc_fpga_group, i_adc_chip, 0x14, &data);
            printf("reg 0x14 on fpga group %d, adc chip %d = 0x%x\n", i_adc_fpga_group, i_adc_chip, data);
            adc_spi_read(i_adc_fpga_group, i_adc_chip, 0x18, &data);
            printf("reg 0x18 on fpga group %d, adc chip %d = 0x%x\n", i_adc_fpga_group, i_adc_chip, data);
         }
      }
   }
   // enable ADC output
   for (i_adc_fpga_group = 0; i_adc_fpga_group < SIS3316_NUM_FPGA_ADC_GROUPS; i_adc_fpga_group++)
   {
      WRITE(base, SIS3316_ADC_SPI_CTRL_REG(i_adc_fpga_group), 0x01000000);   //  set bit 24
   }
   return(0);
}

int sis3316_adc::adc_spi_reg_enable_adc_outputs(void)
{
   unsigned i_adc_fpga_group;

   for (i_adc_fpga_group = 0; i_adc_fpga_group < SIS3316_NUM_FPGA_ADC_GROUPS; i_adc_fpga_group++)
   {
      WRITE(base, SIS3316_ADC_SPI_CTRL_REG(i_adc_fpga_group), 0x01000000);   //  set bit 24
   }
   return(0);
}

/*******************************************************************************/

// adc_fpga_group: 0,1,2,3
// adc_chip: 0 or 1
// return codes
//				-1 : invalid parameter
//				-2 : timeout
//				>0 : VME Error Code

int sis3316_adc::adc_spi_read(unsigned int adc_fpga_group, unsigned int adc_chip, unsigned int spi_addr, unsigned int *spi_data)
{
   unsigned int data;
   unsigned int uint_adc_mux_select;
   unsigned int pollcounter;

   pollcounter = 1000;

   if (adc_fpga_group >= SIS3316_NUM_FPGA_ADC_GROUPS)
   {
      return(-1);
   }
   if (adc_chip >= SIS3316_NR_ADC_PER_FPGA)
   {
      return(-1);
   }
   if (spi_addr > 0xffff)
   {
      return(-1);
   }

   if (adc_chip == 0)
   {
      uint_adc_mux_select = 0;          // adc chip ch1/ch2
   }
   else
   {
      uint_adc_mux_select = 0x400000;    // adc chip ch3/ch4
   }

   // read register to get the information of bit 24 (adc output enabled)
   data = READ(base, SIS3316_ADC_SPI_CTRL_REG(adc_fpga_group));
   data = data & 0x01000000;   // save bit 24

   data = data + 0xC0000000 + uint_adc_mux_select + ((spi_addr & 0x1fff) << 8);
   //printf("in spi read , write data : 0x%x\n", data);
   WRITE(base, SIS3316_ADC_SPI_CTRL_REG(adc_fpga_group), data);
   epicsThreadSleep(0.01); // ???
   do                      // the logic is appr. 7us busy  (20us), HPJ: busy never seen ...
   {
      data = READ(base, SIS3316_ADC_FPGA_SPI_BUSY_STATUS_REG);
      pollcounter--;
      //printf(" spi busy status: 0x%x\n",data);
   } while (((data & 0x0000000f) == 0x0000000f) && (pollcounter > 0));
   // } while (((data & 0x80000000) == 0x80000000) && (pollcounter > 0)); should be in Rev >=6

   if (pollcounter == 0)
   {
      return(-2);
   }

   // epicsThreadSleep(0.01); // ???

   data = READ(base, SIS3316_ADC_SPI_RDB(adc_fpga_group));

   *spi_data = data & 0xff;
   return(0);
}

int sis3316_adc::adc_spi_write(unsigned int adc_fpga_group, unsigned int adc_chip, unsigned int spi_addr, unsigned int spi_data)
{
   unsigned int data;
   unsigned int uint_adc_mux_select;
   unsigned int pollcounter;

   pollcounter = 1000;

   if (adc_fpga_group >= SIS3316_NUM_FPGA_ADC_GROUPS)
   {
      return(-1);
   }
   if (adc_chip >= SIS3316_NR_ADC_PER_FPGA)
   {
      return(-1);
   }
   if (spi_addr > 0xffff)
   {
      return(-1);
   }

   if (adc_chip == 0)
   {
      uint_adc_mux_select = 0;  // adc chip ch1/ch2
   }
   else
   {
      uint_adc_mux_select = 0x400000;  // adc chip ch3/ch4
   }

   // read register to get the information of bit 24 (adc output enabled)
   data = READ(base, SIS3316_ADC_SPI_CTRL_REG(adc_fpga_group));
   data = data & 0x01000000;   // save bit 24

   data = data + 0x80000000 + uint_adc_mux_select + ((spi_addr & 0xffff) << 8) + (spi_data & 0xff);
   WRITE(base, SIS3316_ADC_SPI_CTRL_REG(adc_fpga_group), data);
   epicsThreadSleep(0.01);
   do    // the logic is appr. 7us busy
   {
      data = READ(base, SIS3316_ADC_FPGA_SPI_BUSY_STATUS_REG);
      //printf("spi poll counter %d Data: 0x%x\n", pollcounter,  data);
      pollcounter--;
   } while (((data & 0x0000000f) == 0x0000000f) && (pollcounter > 0));
   // } while (((data & 0x80000000) == 0x80000000) && (pollcounter > 0)); should be in Rev >=6
   if (pollcounter == 0)
   {
      return(-2);
   }
   return(0);
}

int sis3316_adc::I2cStart(int osc)
{
   int         i;
   epicsUInt32 tmp;

   if (osc > 3)
   {
      return(-101);
   }

   // start
   WRITE(base, SIS3316_ADC_CLK_OSC_I2C_REG(osc), 1 << I2C_START);
   epicsThreadSleep(0.01);
   i = 0;
   do
   {
      // poll i2c fsm busy
      tmp = READ(base, SIS3316_ADC_CLK_OSC_I2C_REG(osc));
      i++;
   } while ((tmp & (1 << I2C_BUSY)) && (i < 1000));

   // register access problem
   if (i == 1000)
   {
      return(-100);
   }

   return(0);
}

int sis3316_adc::I2cStop(int osc)
{
   int         i;
   epicsUInt32 tmp;

   if (osc > 3)
   {
      return(-101);
   }

   // stop
   WRITE(base, SIS3316_ADC_CLK_OSC_I2C_REG(osc), 1 << I2C_STOP);
   epicsThreadSleep(0.01);
   i = 0;
   do
   {
      // poll i2c fsm busy
      tmp = READ(base, SIS3316_ADC_CLK_OSC_I2C_REG(osc));
      i++;
   } while ((tmp & (1 << I2C_BUSY)) && (i < 1000));

   // register access problem
   if (i == 1000)
   {
      return(-100);
   }

   return(0);
}

int sis3316_adc::I2cWriteByte(int osc, unsigned char data, char *ack)
{
   int         i;
   epicsUInt32 tmp;

   if (osc > 3)
   {
      return(-101);
   }

   // write byte, receive ack
   WRITE(base, SIS3316_ADC_CLK_OSC_I2C_REG(osc), 1 << I2C_WRITE ^ data);
   epicsThreadSleep(0.01);
   i = 0;
   do
   {
      // poll i2c fsm busy
      tmp = READ(base, SIS3316_ADC_CLK_OSC_I2C_REG(osc));
      tmp = READ(base, SIS3316_ADC_CLK_OSC_I2C_REG(osc));
      i++;
   } while ((tmp & (1 << I2C_BUSY)) && (i < 1000));

   // register access problem
   if (i == 1000)
   {
      return(-100);
   }

   // return ack value?
   if (ack)
   {
      // yup
      *ack = tmp & 1 << I2C_ACK ? 1 : 0;
   }

   return(0);
}

int sis3316_adc::I2cReadByte(int osc, unsigned char *data, char ack)
{
   int           i;
   epicsUInt32   tmp;
   unsigned char char_tmp;

   if (osc > 3)
   {
      return(-101);
   }

   // read byte, put ack
   tmp  = 1 << I2C_READ;
   tmp |= ack ? 1 << I2C_ACK : 0;
   WRITE(base, SIS3316_ADC_CLK_OSC_I2C_REG(osc), tmp);
   epicsThreadSleep(0.01);
   i = 0;
   do
   {
      // poll i2c fsm busy
      tmp = READ(base, SIS3316_ADC_CLK_OSC_I2C_REG(osc));
      i++;
   } while ((tmp & (1 << I2C_BUSY)) && (i < 1000));

   // register access problem
   if (i == 1000)
   {
      return(-100);
   }
   char_tmp = (unsigned char)(tmp & 0xff);
   *data    = char_tmp;

   return(0);
}

int sis3316_adc::Si570FreezeDCO(int osc)
{
   int  rc;
   char ack;

   // start
   rc = this->I2cStart(osc);
   if (rc)
   {
      this->I2cStop(osc);
      return(rc);
   }

   // address
   rc = this->I2cWriteByte(osc, OSC_ADR << 1, &ack);
   if (rc)
   {
      this->I2cStop(osc);
      return(rc);
   }

   if (!ack)
   {
      this->I2cStop(osc);
      return(-101);
   }

   // register offset
   rc = this->I2cWriteByte(osc, 0x89, &ack);
   if (rc)
   {
      this->I2cStop(osc);
      return(rc);
   }

   if (!ack)
   {
      this->I2cStop(osc);
      return(-101);
   }

   // write data
   rc = this->I2cWriteByte(osc, 0x10, &ack);
   if (rc)
   {
      this->I2cStop(osc);
      return(rc);
   }

   if (!ack)
   {
      this->I2cStop(osc);
      return(-101);
   }

   // stop
   rc = this->I2cStop(osc);
   if (rc)
   {
      return(rc);
   }

   return(0);
}

int sis3316_adc::Si570ReadDivider(int osc, unsigned char *data)
{
   int  rc;
   char ack;
   int  i;

   // start
   rc = this->I2cStart(osc);
   if (rc)
   {
      this->I2cStop(osc);
      return(rc);
   }

   // address
   rc = this->I2cWriteByte(osc, OSC_ADR << 1, &ack);
   if (rc)
   {
      this->I2cStop(osc);
      return(rc);
   }

   if (!ack)
   {
      this->I2cStop(osc);
      return(-101);
   }

   // register offset
   rc = this->I2cWriteByte(osc, 0x0D, &ack);
   if (rc)
   {
      this->I2cStop(osc);
      return(rc);
   }

   if (!ack)
   {
      this->I2cStop(osc);
      return(-101);
   }

   rc = this->I2cStart(osc);
   if (rc)
   {
      this->I2cStop(osc);
      return(rc);
   }

   // address + 1
   rc = this->I2cWriteByte(osc, (OSC_ADR << 1) + 1, &ack);
   if (rc)
   {
      this->I2cStop(osc);
      return(rc);
   }

   if (!ack)
   {
      this->I2cStop(osc);
      return(-101);
   }

   // read data
   for (i = 0; i < 6; i++)
   {
      ack = 1;
      if (i == 5)
      {
         ack = 0;
      }
      rc = this->I2cReadByte(osc, &data[i], ack);
      if (rc)
      {
         this->I2cStop(osc);
         return(rc);
      }
   }

   // stop
   rc = this->I2cStop(osc);
   if (rc)
   {
      return(rc);
   }

   return(0);
}

int sis3316_adc::Si570Divider(int osc, unsigned char *data)
{
   int  rc;
   char ack;
   int  i;

   // start
   rc = this->I2cStart(osc);
   if (rc)
   {
      this->I2cStop(osc);
      return(rc);
   }

   // address
   rc = this->I2cWriteByte(osc, OSC_ADR << 1, &ack);
   if (rc)
   {
      this->I2cStop(osc);
      return(rc);
   }

   if (!ack)
   {
      this->I2cStop(osc);
      return(-101);
   }

   // register offset
   rc = this->I2cWriteByte(osc, 0x0D, &ack);
   if (rc)
   {
      this->I2cStop(osc);
      return(rc);
   }

   if (!ack)
   {
      this->I2cStop(osc);
      return(-101);
   }

   // write data
   for (i = 0; i < 2; i++)    // war 2??
   {
      rc = this->I2cWriteByte(osc, data[i], &ack);
      if (rc)
      {
         this->I2cStop(osc);
         return(rc);
      }

      if (!ack)
      {
         this->I2cStop(osc);
         return(-101);
      }
   }

   // stop
   rc = this->I2cStop(osc);
   if (rc)
   {
      return(rc);
   }

   return(0);
}

int sis3316_adc::Si570UnfreezeDCO(int osc)
{
   int  rc;
   char ack;

   // start
   rc = this->I2cStart(osc);
   if (rc)
   {
      this->I2cStop(osc);
      return(rc);
   }

   // address
   rc = this->I2cWriteByte(osc, OSC_ADR << 1, &ack);
   if (rc)
   {
      this->I2cStop(osc);
      return(rc);
   }

   if (!ack)
   {
      this->I2cStop(osc);
      return(-101);
   }

   // register offset
   rc = this->I2cWriteByte(osc, 0x89, &ack);
   if (rc)
   {
      this->I2cStop(osc);
      return(rc);
   }

   if (!ack)
   {
      this->I2cStop(osc);
      return(-101);
   }

   // write data
   rc = this->I2cWriteByte(osc, 0x00, &ack);
   if (rc)
   {
      this->I2cStop(osc);
      return(rc);
   }

   if (!ack)
   {
      this->I2cStop(osc);
      return(-101);
   }

   // stop
   rc = this->I2cStop(osc);
   if (rc)
   {
      return(rc);
   }

   return(0);
}

int sis3316_adc::Si570NewFreq(int osc)
{
   int  rc;
   char ack;

   // start
   rc = this->I2cStart(osc);
   if (rc)
   {
      this->I2cStop(osc);
      return(rc);
   }

   // address
   rc = this->I2cWriteByte(osc, OSC_ADR << 1, &ack);
   if (rc)
   {
      this->I2cStop(osc);
      return(rc);
   }

   if (!ack)
   {
      this->I2cStop(osc);
      return(-101);
   }

   // register offset
   rc = this->I2cWriteByte(osc, 0x87, &ack);
   if (rc)
   {
      this->I2cStop(osc);
      return(rc);
   }

   if (!ack)
   {
      this->I2cStop(osc);
      return(-101);
   }

   // write data
   rc = this->I2cWriteByte(osc, 0x40, &ack);
   if (rc)
   {
      this->I2cStop(osc);
      return(rc);
   }

   if (!ack)
   {
      this->I2cStop(osc);
      return(-101);
   }

   // stop
   rc = this->I2cStop(osc);
   if (rc)
   {
      return(rc);
   }

   return(0);
}

int sis3316_adc::set_frequency(int osc, unsigned char *values)
{
   int rc;

   if (values == NULL)
   {
      return(-100);
   }
   if (osc > 3 || osc < 0)
   {
      return(-100);
   }

   rc = this->Si570FreezeDCO(osc);
   if (rc)
   {
      return(rc);
   }

   rc = this->Si570Divider(osc, values);
   if (rc)
   {
      return(rc);
   }

   rc = this->Si570UnfreezeDCO(osc);
   if (rc)
   {
      return(rc);
   }

   rc = this->Si570NewFreq(osc);
   if (rc)
   {
      return(rc);
   }
   //min 10ms wait
   epicsThreadSleep(0.2);
   // DSM Reset
   //printf(" ADC _CLOCK RESET ...........\n");
   WRITE(base, SIS3316_KEY_ADC_CLOCK_DCM_RESET, 0);

   // DCM/PLL will be stable after max. 5ms
   epicsThreadSleep(0.1);
   return(0);
}

int sis3316_adc::owReset(int *presence)
{
   epicsUInt32 data;

   WRITE(base, ONEWIRE_CSR, 1 << 10);    // reset
   epicsThreadSleep(0.1);
   do
   {
      data = READ(base, ONEWIRE_CSR);
   } while (data & 1 << 31);  // while busy

   if (presence)
   {
      *presence = (data & 1 << 0) ? 0 : 1;
   }

   return(0);
}

int sis3316_adc::owRead(unsigned char *data)
{
   epicsUInt32 reg;

   WRITE(base, ONEWIRE_CSR, 1 << 8);
   epicsThreadSleep(0.1);
   do
   {
      reg = READ(base, ONEWIRE_CSR);
   } while (reg & 1 << 31);  // while busy

   if (data)
   {
      *data = reg & 0xFF;
   }

   return(0);
}

int sis3316_adc::owWrite(unsigned char data)
{
   epicsUInt32 reg;

   WRITE(base, ONEWIRE_CSR, 1 << 9 ^ data);
   epicsThreadSleep(0.1);
   do
   {
      reg = READ(base, ONEWIRE_CSR);
   } while (reg & 1 << 31);  // while busy

   return(0);
}

int sis3316_adc::owEeReadPage(int page, unsigned char *data)
{
   int rc;
   int i;

   if (page >= 80)
   {
      return(-1);
   }
   if (data == NULL)
   {
      return(-2);
   }

   // presence
   rc = this->owReset(&i);
   if (rc)
   {
      return(rc);
   }
   if (!i)
   {
      return(-3);
   }

   // read page
   rc = this->owWrite(0xCC);      // skip rom
   if (rc)
   {
      return(rc);
   }

   rc = this->owWrite(0xF0);      // read memory
   if (rc)
   {
      return(rc);
   }

   rc = this->owWrite((page * 32) & 0xFF);      // adr lo
   if (rc)
   {
      return(rc);
   }

   rc = this->owWrite(((page * 32) >> 8) & 0xFF);      // adr hi
   if (rc)
   {
      return(rc);
   }

   for (i = 0; i < 32; i++)
   {
      rc = this->owRead(data + i);           // data in
      if (rc)
      {
         this->owReset(NULL);
         return(rc);
      }
   }

   this->owReset(NULL);

   return(0);
}

int sis3316_adc::owEeWritePage(int page, unsigned char *data)
{
   int           rc;
   int           i;
   unsigned char aLo, aHi, esReg;
   unsigned char vfyData[32];

   if (page >= 80)
   {
      return(-1);
   }
   if (data == NULL)
   {
      return(-2);
   }

   // Step 1, copy page to eeprom internal scratchpad sram

   // presence
   rc = this->owReset(&i);
   if (rc)
   {
      return(rc);
   }
   if (!i)
   {
      return(-3);
   }

   // read page
   rc = this->owWrite(0xCC);      // skip rom
   if (rc)
   {
      return(rc);
   }

   rc = this->owWrite(0x0F);      // write scratchpad
   if (rc)
   {
      return(rc);
   }

   rc = this->owWrite((page * 32) & 0xFF);      // adr lo
   if (rc)
   {
      return(rc);
   }

   rc = this->owWrite(((page * 32) >> 8) & 0xFF);      // adr hi
   if (rc)
   {
      return(rc);
   }

   for (i = 0; i < 32; i++)
   {
      rc = this->owWrite(*(data + i));           // data in
      if (rc)
      {
         this->owReset(NULL);
         return(rc);
      }
   }

   this->owReset(NULL);

   // Step 2, verify the scratchpad

   // presence
   rc = this->owReset(&i);
   if (rc)
   {
      return(rc);
   }
   if (!i)
   {
      return(-3);
   }

   // read page
   rc = this->owWrite(0xCC);      // skip rom
   if (rc)
   {
      return(rc);
   }

   rc = this->owWrite(0xAA);      // read scratchpad
   if (rc)
   {
      return(rc);
   }

   rc = this->owRead(&aLo);      // adr lo
   if (rc)
   {
      return(rc);
   }

   rc = this->owRead(&aHi);      // adr hi
   if (rc)
   {
      return(rc);
   }

   rc = this->owRead(&esReg);      // es reg
   if (rc)
   {
      return(rc);
   }

   for (i = 0; i < 32; i++)
   {
      rc = this->owRead(vfyData + i);           // data in
      if (rc)
      {
         this->owReset(NULL);
         return(rc);
      }
   }

   this->owRead(NULL);     // crc16
   this->owRead(NULL);     // crc16

   this->owReset(NULL);

   // Step 3, copy scratchpad to eeprom array

   // presence
   rc = this->owReset(&i);
   if (rc)
   {
      return(rc);
   }
   if (!i)
   {
      return(-3);
   }

   // read page
   rc = this->owWrite(0xCC);      // skip rom
   if (rc)
   {
      return(rc);
   }

   rc = this->owWrite(0x55);      // copy scratchpad
   if (rc)
   {
      return(rc);
   }

   rc = this->owWrite(aLo);      // adr lo
   if (rc)
   {
      return(rc);
   }

   rc = this->owWrite(aHi);      // adr hi
   if (rc)
   {
      return(rc);
   }

   rc = this->owWrite(esReg);      // es reg
   if (rc)
   {
      return(rc);
   }

   epicsThreadSleep(0.1);

   this->owReset(NULL);

   return(0);
}

int sis3316_adc::read_ee(int offset, int len, unsigned char *data)
{
   int           rc;
   int           pageNum, localLen = len, pageOffs, localOffs = offset;
   int           copyLen;
   unsigned char page[32];

   if (data == NULL)
   {
      return(-1);
   }
   if (offset + len > 2560)
   {
      return(-2);
   }
   if (len == 0)
   {
      return(-3);
   }

   // page loop
   // read
   while (localLen)
   {
      // page to start
      pageNum = localOffs / 32;
      // offset within first page
      pageOffs = localOffs % 32;

      // read
      rc = this->owEeReadPage(pageNum, page);
      if (rc)
      {
         return(rc);
      }

      if ((pageOffs + localLen) > 32)
      {
         copyLen = 32 - pageOffs;
      }
      else
      {
         copyLen = localLen;
      }

      // copy back
      memcpy(data, page + pageOffs, copyLen);

      // adjust
      localLen  -= copyLen;
      localOffs += copyLen;
      data      += copyLen;
   }

   return(0);
}

int sis3316_adc::write_ee(int offset, int len, unsigned char *data)
{
   int           rc;
   int           pageNum, localLen = len, pageOffs, localOffs = offset;
   int           copyLen;
   unsigned char page[32];

   if (data == NULL)
   {
      return(-1);
   }
   if (offset + len > 2560)
   {
      return(-2);
   }
   if (len == 0)
   {
      return(-3);
   }

   // page loop
   // read-modify-write
   while (localLen)
   {
      // page to start
      pageNum = localOffs / 32;
      // offset within first page
      pageOffs = localOffs % 32;

      // read
      rc = this->owEeReadPage(pageNum, page);
      if (rc)
      {
         return(rc);
      }

      if ((pageOffs + localLen) > 32)
      {
         copyLen = 32 - pageOffs;
      }
      else
      {
         copyLen = localLen;
      }

      // modify
      memcpy(page + pageOffs, data, copyLen);

      // write
      rc = this->owEeWritePage(pageNum, page);
      if (rc)
      {
         return(0);
      }

      // adjust
      localLen  -= copyLen;
      localOffs += copyLen;
      data      += copyLen;
   }

   return(0);
}

int sis3316_adc::ow_id_ee(unsigned char *data)
{
   int rc;
   int i;

   // presence
   rc = this->owReset(&i);
   if (rc)
   {
      return(rc);
   }
   if (!i)
   {
      return(-1);
   }

   // read rom cmd
   rc = this->owWrite(0x33);
   if (rc)
   {
      return(rc);
   }

   // read data
   for (i = 0; i < 8; i++)
   {
      rc = this->owRead(data + i);
      if (rc)
      {
         return(rc);
      }
   }

   return(0);
}

int sis3316_adc::change_frequency_HSdiv_N1div(int osc, unsigned hs_div_val, unsigned n1_div_val)
{
   int           rc;
   unsigned      i;
   unsigned      N1div;
   unsigned      HSdiv;
   unsigned      HSdiv_reg[6];
   unsigned      HSdiv_val[6];
   unsigned char freqSI570_high_speed_rd_value[6];
   unsigned char freqSI570_high_speed_wr_value[6];

   if (osc > 3 || osc < 0)
   {
      return(-100);
   }

   HSdiv_reg[0] = 0;
   HSdiv_val[0] = 4;

   HSdiv_reg[1] = 1;
   HSdiv_val[1] = 5;

   HSdiv_reg[2] = 2;
   HSdiv_val[2] = 6;

   HSdiv_reg[3] = 3;
   HSdiv_val[3] = 7;

   HSdiv_reg[4] = 5;
   HSdiv_val[4] = 9;

   HSdiv_reg[5] = 7;
   HSdiv_val[5] = 11;

   HSdiv = 0xff;
   for (i = 0; i < 6; i++)
   {
      if (HSdiv_val[i] == hs_div_val)
      {
         HSdiv = HSdiv_reg[i];
      }
   }
   if (HSdiv > 11)
   {
      return(-101);
   }

   // gt than 127 or odd then return
   if ((n1_div_val > 127) || ((n1_div_val & 0x01) == 1) || (n1_div_val == 0))
   {
      return(-102);
   }
   N1div = n1_div_val - 1;

   rc = this->Si570ReadDivider(osc, freqSI570_high_speed_rd_value);
   if (rc)
   {
      printf("Si570ReadDivider = %d \n", rc);
      return(rc);
   }

   /* else {
    * for (int i = 0; i<6; i++)
    *  printf("freqSI570_high_speed_rd_value[%d] = 0x%x\n", i, freqSI570_high_speed_rd_value[i]);
    * } */
   freqSI570_high_speed_wr_value[0] = ((HSdiv & 0x7) << 5) + ((N1div & 0x7c) >> 2);
   freqSI570_high_speed_wr_value[1] = ((N1div & 0x3) << 6) + (freqSI570_high_speed_rd_value[1] & 0x3F);
   freqSI570_high_speed_wr_value[2] = freqSI570_high_speed_rd_value[2];
   freqSI570_high_speed_wr_value[3] = freqSI570_high_speed_rd_value[3];
   freqSI570_high_speed_wr_value[4] = freqSI570_high_speed_rd_value[4];
   freqSI570_high_speed_wr_value[5] = freqSI570_high_speed_rd_value[5];

   freqSI570_high_speed_wr_value[0] = 0x20;
   freqSI570_high_speed_wr_value[1] = 0xc2;
   freqSI570_high_speed_wr_value[2] = 0xbb;
   freqSI570_high_speed_wr_value[3] = 0xfb;
   freqSI570_high_speed_wr_value[4] = 0x85;
   freqSI570_high_speed_wr_value[5] = 0x11;

   rc = this->set_frequency(osc, freqSI570_high_speed_wr_value);
   if (rc)
   {
      printf("set_frequency = %d \n", rc);
      return(rc);
   }
   return(0);
}

int sis3316_adc::get_frequency(int osc, unsigned char *values)
{
   int rc;

   if (values == NULL)
   {
      return(-100);
   }
   if (osc > 3 || osc < 0)
   {
      return(-100);
   }
   rc = this->Si570ReadDivider(osc, values);
   if (rc)
   {
      return(rc);
   }
   return(0);
}

int sis3316_adc::configure_adc_fpga_iob_delays(unsigned int iob_delay_value)
{
   unsigned int i_adc_fpga_group;

   // Calibrate IOB _delay Logic
   for (i_adc_fpga_group = 0; i_adc_fpga_group < SIS3316_NUM_FPGA_ADC_GROUPS; i_adc_fpga_group++)
   {
      WRITE(base, SIS3316_ADC_INP_TAP_DLY(i_adc_fpga_group), 0xf00);
   }
   epicsThreadSleep(0.3);


   // set IOB _delay Logic
   for (i_adc_fpga_group = 0; i_adc_fpga_group < SIS3316_NUM_FPGA_ADC_GROUPS; i_adc_fpga_group++)
   {
      WRITE(base, SIS3316_ADC_INP_TAP_DLY(i_adc_fpga_group), 0x300 + iob_delay_value);
   }
   epicsThreadSleep(0.2);
   return(0);
}

int sis3316_adc::setDac(int channel, unsigned short value)
{
   epicsUInt32 vmeData;
   int         return_code;

   if (channel < 0 || channel >= SIS3316_NUM_CHANNELS)
   {
      return(-1);
   }
   //enable internal reference
   WRITE(base, SIS3316_ADC_DAC_OFFSET_CTRL_REG(channel >> 2), 0x88F00001);
   return_code = poll_on_adc_dac_offset_busy();
   if (return_code != 0)
   {
      return(return_code);
   }
   WRITE(base, SIS3316_ADC_DAC_OFFSET_CTRL_REG(channel >> 2), 0xC0000000);  // CMD: DAC LDAC (load)
   return_code = this->poll_on_adc_dac_offset_busy();
   if (return_code != 0)
   {
      return(return_code);
   }

   // 0x90000000 == Write Command/Address/Data to DAC and generate “DAC LDAC” (load)
   vmeData = 0x90000000 + (3 << 24) + ((channel & 3) << 20) + (((epicsUInt32)value) << 4);
   //printf("setDac : vmeData = 0x%08x\n", vmeData);

   WRITE(base, SIS3316_ADC_DAC_OFFSET_CTRL_REG(channel >> 2), vmeData);
   return_code = this->poll_on_adc_dac_offset_busy();
   if (return_code != 0)
   {
      return(return_code);
   }

   adc_dac_offset_ch_array[channel] = value;

   return(0);
}

int sis3316_adc::readDac(int channel, unsigned short *value)
{
//    epicsUInt32 vmeData;
   if (channel < 0 || channel >= SIS3316_NUM_CHANNELS)
   {
      return(-1);
   }

   /* did not know what this register is for? page 93
    * vmeData = READ(base, SIS3316_ADC_FPGA_OFFSET_RDB(channel>>2));
    * value = (unsigned short)vmeData & 0xFFFF;
    *
    * if ( *value != dacOffsets[channel] ) {
    *  printf(" Warning readDac (0x%x) differs from dacOffsets[%d] (0x%x)\n", *value, channel, dacOffsets[channel]);
    * }
    */
   *value = adc_dac_offset_ch_array[channel];  // to be tested ...
   return(0);
}

/*
 * p 79
 */
// general set bit !!!
int sis3316_adc::inputT1Trigger(epicsUInt32 state)
{
   epicsUInt32 v;

   v = READ(base, SIS3316_NIM_INPUT_CONTROL_REG);
   if (state)
   {
      v |= NIM_INPUT_TI_AS_TRIGGER_ENABLE;
   }
   else
   {
      v &= ~NIM_INPUT_TI_AS_TRIGGER_ENABLE;
   }
   WRITE(base, SIS3316_NIM_INPUT_CONTROL_REG, v);
   return(0);
}

/*
 * p 113 new manual
 */
int sis3316_adc::selectCO(epicsUInt8 bit)
{
   WRITE(base, SIS3316_LEMO_OUT_CO_SELECT_REG, 1 << bit);
   return(0);
}

/*
 * p 114 new manual
 */
int sis3316_adc::selectTO(epicsUInt8 bit)
{
   WRITE(base, SIS3316_LEMO_OUT_TO_SELECT_REG, 1 << bit);
   return(0);
}

/*
 * p 115 new manual
 */
int sis3316_adc::selectUO(epicsUInt8 bit)
{
   WRITE(base, SIS3316_LEMO_OUT_UO_SELECT_REG, 1 << bit);
   return(0);
}

/*
 * p 80
 */

int sis3316_adc::setAcquisitionControl(epicsUInt32 bit, epicsUInt32 state)
{
   epicsUInt32 v;

   v = READ(base, SIS3316_AQUISITION_CONTROL);
   if (state)
   {
      v |= 1 << bit;
   }
   else
   {
      v &= ~(1 << bit);
   }
   WRITE(base, SIS3316_AQUISITION_CONTROL, v);
   return(0);
}

// old not to use
int sis3316_adc::setGain(int channel, unsigned char value)
{
   epicsUInt32 vmeData;

   if (channel < 0 || channel >= SIS3316_NUM_CHANNELS)
   {
      return(-1);
   }

   vmeData = READ(base, SIS3316_ADC_ANALOG_CTRL_REG(channel >> 2));

   vmeData &= ~(3 << (8 * (channel & 3)));
   vmeData |= (value << (8 * (channel & 3)));
   WRITE(base, SIS3316_ADC_ANALOG_CTRL_REG(channel >> 2), vmeData);

   adc_gain_termination_ch_array[channel]  = adc_gain_termination_ch_array[channel] & ~GAIN_MASK;
   adc_gain_termination_ch_array[channel] += value;

   return(0);
}

// old not to use
int sis3316_adc::setTermination(int channel, unsigned char value)
{
   epicsUInt32 vmeData;

   if (channel < 0 || channel >= SIS3316_NUM_CHANNELS)
   {
      return(-1);
   }
   vmeData  = READ(base, SIS3316_ADC_ANALOG_CTRL_REG(channel >> 2));
   vmeData &= ~(1 << ((8 * (channel & 3)) + 2));
   if (value)
   {
      vmeData |= (1 << ((8 * (channel & 3)) + 2)); // disable 50R
   }
   WRITE(base, SIS3316_ADC_ANALOG_CTRL_REG(channel >> 2), vmeData);
   adc_gain_termination_ch_array[channel]  = adc_gain_termination_ch_array[channel] & ~DIS_50_OHM;
   adc_gain_termination_ch_array[channel] |= value << 3;
   return(0);
}

int sis3316_adc::internal_trigger_generation_setup(int channel,
                                                   unsigned int uint_trigger_threshold_reg, unsigned int uint_he_trigger_threshold_reg,
                                                   unsigned int uint_trigger_setup_reg)
{
   if (channel < 0 || channel >= SIS3316_NUM_CHANNELS)
   {
      return(0x900);
   }
   // clear FIR Trigger Setup -> a following Setup will reset the logic
   WRITE(base, SIS3316_ADC_FIR_TRIGGER_SETUP_REG((channel >> 2), channel), 0);
   // disable all ch_sum
   WRITE(base, SIS3316_ADC_FIR_TRIGGER_THRESHOLD_REG((channel >> 2), channel), 0);

   WRITE(base, SIS3316_ADC_FIR_HIGH_ENERGY_THRESHOLD_REG((channel >> 2), channel), uint_he_trigger_threshold_reg);

   WRITE(base, SIS3316_ADC_FIR_TRIGGER_THRESHOLD_REG((channel >> 2), channel), uint_trigger_threshold_reg);

   WRITE(base, SIS3316_ADC_FIR_TRIGGER_SETUP_REG((channel >> 2), channel), uint_trigger_setup_reg);

   return(0);
}

/*
 * p 97
 */
int sis3316_adc::setHeaderId(int group, epicsUInt32 value)
{
   epicsUInt32 v;

   if (group < 0 || group >= SIS3316_NUM_FPGA_ADC_GROUPS)
   {
      return(-1);
   }
   v  = value;
   v += (group) << 22;
   WRITE(base, SIS3316_ADC_CHANNEL_HEADER_REG(group), v);
   return(0);
}

/*
 * p 99
 */
int sis3316_adc::setTriggerGateWindowLength(int group, unsigned short length)
{
   if (group < 0 || group >= SIS3316_NUM_FPGA_ADC_GROUPS)
   {
      return(-1);
   }
   if (length % 2)
   {
      return(-2);
   }
   WRITE(base, SIS3316_ADC_TRIGGER_GATE_WINDOW_LENGTH(group), length);
   return(0);
}

/*
 * p 100
 */
int sis3316_adc::setBufferSample(int group, unsigned int sampleLength, unsigned int startIndex)
{
   epicsUInt32 v;

   if (group < 0 || group > SIS3316_NUM_FPGA_ADC_GROUPS)
   {
      return(-1);
   }
   if ((sampleLength % 2) || (startIndex % 2))
   {
      return(-2);
   }

   v = ((sampleLength & 0xffff) << 16) + (startIndex & 0xffff);
   //printf(" Raw data buffer: 0x%0X\n", v);
   WRITE(base, SIS3316_ADC_RAW_DATA_BUFFER_CONFIG_REG(group), v);
   if (sampleLength > 0xffff)
   {
      WRITE(base, SIS3316_ADC_EXTENDED_RAW_DATA_BUFFER_CONFIG_REG(group), sampleLength);
   }
   else
   {
      WRITE(base, SIS3316_ADC_EXTENDED_RAW_DATA_BUFFER_CONFIG_REG(group), 0);
   }
   return(0);
}

/*
 * p 101
 */
int sis3316_adc::pileupConfig(int group, unsigned short rePileupWin, unsigned int pileupWin)
{
   epicsUInt32 v;

   if (group < 0 || group > SIS3316_NUM_FPGA_ADC_GROUPS)
   {
      return(-1);
   }
   if ((pileupWin % 2) || (rePileupWin % 2))
   {
      return(-2);
   }
   v = (rePileupWin << 16) + pileupWin;
   WRITE(base, SIS3316_ADC_PILEUP_CONFIG_REG(group), v);
   return(0);
}

/*
 * p 102
 */
int sis3316_adc::setPreTriggerDelay(int group, unsigned short delay)
{
   epicsUInt32 v;

   if (group < 0 || group >= SIS3316_NUM_FPGA_ADC_GROUPS)
   {
      return(-1);
   }
   if ((delay % 2) || (delay > MAX_PRE_TRIGGER_DELAY))
   {
      return(-2);
   }
   v = READ(base, SIS3316_ADC_PRE_TRIGGER_DELAY_REG(group));
   // bit 15 Additional Delay of Fir Trigger??
   v &= 0xfffff800;
   v += delay;
   WRITE(base, SIS3316_ADC_PRE_TRIGGER_DELAY_REG(group), v);
   return(0);
}

/*
 * p 103
 */
int sis3316_adc::setDataFormat(int channel, unsigned char saveMode)
{
   epicsUInt32 v;

   if (channel < 0 || channel >= SIS3316_NUM_CHANNELS)
   {
      return(-1);
   }

   if (saveMode > 0x10)
   {
      return(-2);
   }

   v = (saveMode & 0x17) << (8 * (channel & 3));
   WRITE(base, SIS3316_ADC_DATAFORMAT_CONFIG_REG(channel >> 2), v);
   return(0);
}

/*
 * p 104
 */
int sis3316_adc::setMawTestBuffer(int channel, unsigned short bLength, unsigned short bPreTriggerDelay)
{
   epicsUInt32 v;

   if (channel < 0 || channel >= SIS3316_NUM_CHANNELS)
   {
      return(-1);
   }
   if ((bLength % 2) || (bLength > 1022))
   {
      return(-2);
   }
   // 0 seems also to be ok
   if ((bPreTriggerDelay % 2) || (bPreTriggerDelay > 1022))
   {
      return(-3);
   }
   v = (bPreTriggerDelay << 16) + bLength;

   WRITE(base, SIS3316_ADC_TEST_BUFFER_CONFIG_REG(channel >> 2), v);
   return(0);
}

/*
 * p 107
 */
int sis3316_adc::firTriggerSetup(int channel, unsigned short pulse, unsigned short gapTime, unsigned short peakingTime)
{
   epicsUInt32 v;

   if (channel < 0 || channel >= SIS3316_NUM_CHANNELS)
   {
      return(-1);
   }
   if ((pulse % 2) || (gapTime % 2) || (peakingTime % 2))
   {
      return(-2);
   }
   if ((pulse > 256) || (gapTime > 510) || (peakingTime > 510))
   {
      return(-3);
   }
   v = (pulse << 24) + (gapTime << 12) + peakingTime;

   WRITE(base, SIS3316_ADC_FIR_TRIGGER_SETUP_REG((channel >> 2), (channel & 3)), v);
   return(0);
}

int sis3316_adc::sumFirTriggerSetup(int group, unsigned short pulse, unsigned short gapTime, unsigned short peakingTime)
{
   epicsUInt32 v;

   if (group < 0 || group >= SIS3316_NUM_FPGA_ADC_GROUPS)
   {
      return(-1);
   }
   if ((pulse % 2) || (gapTime % 2) || (peakingTime % 2))
   {
      return(-2);
   }
   if ((pulse > 256) || (gapTime > 510) || (peakingTime > 510))
   {
      return(-3);
   }
   v = (pulse << 24) + (gapTime << 12) + peakingTime;

   WRITE(base, SIS3316_ADC_FIR_TRIGGER_SETUP_REG(group, FIR_SUM_REG), v);
   return(0);
}

/*
 * p 132, new manual
 */
int sis3316_adc::eventConfig(int channel, epicsUInt8 state)
{
   epicsUInt32 v;

   if (channel < 0 || channel >= SIS3316_NUM_CHANNELS)
   {
      return(-1);
   }
   v = READ(base, SIS3316_ADC_EVENT_CONFIG_REG((channel >> 2)));
   //printf ("Read channel % d: event config = 0x%08x\n", channel, v);
   switch (channel & 3)
   {
   case 0: v &= 0xffffff00; v |= state; break;

   case 1: v &= 0xffff00ff; v |= state << 8; break;

   case 2: v &= 0xff00ffff; v |= state << 16; break;

   case 3: v &= 0x00ffffff; v |= state << 24; break;

   default: printf(" Houps ...\n"); return(-1);
   }
   //printf ("write channel % d: event config = 0x%08x\n", channel, v);
   WRITE(base, SIS3316_ADC_EVENT_CONFIG_REG((channel >> 2)), v);
   return(0);
}

int sis3316_adc::dataFormat(int channel, epicsUInt8 format)
{
   epicsUInt32 v;

   if (channel < 0 || channel >= SIS3316_NUM_CHANNELS)
   {
      return(-1);
   }
   v = READ(base, SIS3316_ADC_DATAFORMAT_CONFIG_REG((channel >> 2)));
   //printf ("Read channel % d: event config = 0x%08x\n", channel, v);
   switch (channel & 3)
   {
   case 0: v &= 0xffffff00; v |= format; break;

   case 1: v &= 0xffff00ff; v |= format << 8; break;

   case 2: v &= 0xff00ffff; v |= format << 16; break;

   case 3: v &= 0x00ffffff; v |= format << 24; break;

   default: printf(" Houps ...\n"); return(-1);
   }
   //printf ("write channel % d: event config = 0x%08x\n", channel, v);
   WRITE(base, SIS3316_ADC_DATAFORMAT_CONFIG_REG((channel >> 2)), v);
   return(0);
}

/*
 * p 108
 */
int sis3316_adc::firTrigger(int channel, epicsUInt32 state)
{
   epicsUInt32 v;

   if (channel < 0 || channel >= SIS3316_NUM_CHANNELS)
   {
      return(-1);
   }
   v = READ(base, SIS3316_ADC_FIR_TRIGGER_THRESHOLD_REG((channel >> 2), (channel & 3)));
   if (state)
   {
      v |= FIR_TRIGGER_ENABLE;
   }
   else
   {
      v &= ~FIR_TRIGGER_ENABLE;
   }
   WRITE(base, SIS3316_ADC_FIR_TRIGGER_THRESHOLD_REG((channel >> 2), (channel & 3)), v);
   return(0);
}

int sis3316_adc::firSumTrigger(int group, epicsUInt32 state)
{
   epicsUInt32 v;

   if (group < 0 || group >= SIS3316_NUM_FPGA_ADC_GROUPS)
   {
      return(-1);
   }
   v = READ(base, SIS3316_ADC_FIR_TRIGGER_THRESHOLD_REG(group, FIR_SUM_REG));
   if (state)
   {
      v |= FIR_TRIGGER_ENABLE;
   }
   else
   {
      v &= ~FIR_TRIGGER_ENABLE;
   }
   WRITE(base, SIS3316_ADC_FIR_TRIGGER_THRESHOLD_REG(group, FIR_SUM_REG), v);
   return(0);
}

/*
 * p 110
 */
int sis3316_adc::setHighEnergyThreshold(int channel, epicsUInt32 value)
{
   if (channel < 0 || channel >= SIS3316_NUM_CHANNELS)
   {
      return(-1);
   }
   WRITE(base, SIS3316_ADC_FIR_TRIGGER_THRESHOLD_REG((channel >> 2), (channel & 3)), value);
   return(0);
}

int sis3316_adc::setSumHighEnergyThreshold(int group, epicsUInt32 value)
{
   if (group < 0 || group >= SIS3316_NUM_FPGA_ADC_GROUPS)
   {
      return(-1);
   }
   WRITE(base, SIS3316_ADC_FIR_TRIGGER_THRESHOLD_REG(group, FIR_SUM_REG), value);
   return(0);
}

/*
 * p 98
 */
int sis3316_adc::setAddressThreshold(int group, epicsUInt32 threshold)
{
   if (group < 0 || group >= SIS3316_NUM_FPGA_ADC_GROUPS)
   {
      return(-1);
   }
   if (threshold >= 0x01000000)
   {
      return(-2);
   }
   threshold += 0x80000000;  // suppress saving following hits/events
   WRITE(base, SIS3316_ADC_ADDRESS_THRESHOLD_REG(group), threshold);
   return(0);
}

/****************************************************************************************************/

int sis3316_adc::read_Channel_PreviousBankDataBuffer(unsigned int channel_no)
{
   unsigned int data, reset;
   epicsUInt32 *vmePhysAddr;
   unsigned int previous_bank_addr_value;
   epicsUInt32  nrOfDataCollected;
   unsigned int req_nof_32bit_words;
   unsigned int memory_bank_offset_addr;
   unsigned int max_poll_counter;

   if (channel_no >= SIS3316_NUM_CHANNELS)
   {
      return(-1);
   }


   reset = 0x00000000;    // Reset
   WRITE(base, SIS3316_DATA_TRANSFER_CTRL_REG((channel_no >> 2) & 0x3), reset);

   // read previous Bank sample address poll until it is valid.
   max_poll_counter    = 1000;
   nofData[channel_no] = 0;
   do
   {
      previous_bank_addr_value = READ(base, SIS3316_ADC_PREVIOUS_BANK_SAMPLE_ADDRESS_REG(((channel_no >> 2) & 0x03), (channel_no & 0x3)));
      max_poll_counter--;
      if (!max_poll_counter)
      {
         return(-2);
      }
   } while (((previous_bank_addr_value & 0x1000000) >> 24) != bank1_armed_flag);   // previous Bank sample address is valid if bit 24 is equal bank2_read_flag
   //printf(" bank2_read_flag = %d (poll_counter : %d)\n", bank1_armed_flag, max_poll_counter);
   nrOfDataCollected = previous_bank_addr_value & 0xffffff;
   //printf("Channel No : %d ,  nrOfDataCollected : 0x%08x\n", channel_no, nrOfDataCollected);

   if (!nrOfDataCollected)
   { // no data sampled !
      return(0);
   }

   memory_bank_offset_addr = 0x00000000; // see struck manual p. 72
   if (bank1_armed_flag)
   {
      memory_bank_offset_addr += 0x01000000;                    // Bank2 offset
   }
   if (channel_no & 0x1)
   {
      memory_bank_offset_addr += 0x02000000;                   // channel 1, 3, ... 15
   }
   if (channel_no & 0x2)
   {
      memory_bank_offset_addr += 0x10000000;                    // channel 2,3 , 6,7 ...
   }
   // start readout of nrOfDataCollected
   data = 0x80000000 + memory_bank_offset_addr;

   WRITE(base, SIS3316_DATA_TRANSFER_CTRL_REG((channel_no >> 2) & 0x3), data);

   vmePhysAddr = (epicsUInt32 *)SIS3316_FPGA_ADC_MEM_BASE + ((((channel_no >> 2) & 0x3) * SIS3316_FPGA_ADC_MEM_OFFSET) / sizeof(uint32_t));
   // should be taken from devRegister
   vmePhysAddr += a32addr / sizeof(uint32_t); // add base;
   // vmePhysAddr +=  0x91000000/sizeof(uint32_t);

   req_nof_32bit_words = nrOfDataCollected;

   int rc, dmaStatus;
   //epicsTime now;
   //now = epicsTime::getCurrent();
   //now.show(0);

   m_data[channel_no].resize(nrOfDataCollected, 0x00420042);

   uint32_t alreadyCopied = 0;
   uint32_t toBeCopied    = 0;

   //printf(" channel_no %d,data :0x%x, physAddr : 0x%p\n", channel_no, data, vmePhysAddr);
   do
   {
      if (req_nof_32bit_words - alreadyCopied > MAX_DMA_LENGTH)
      {
         toBeCopied = MAX_DMA_LENGTH;
      }
      else
      {
         toBeCopied = req_nof_32bit_words - alreadyCopied;
      }
      //printf(" alreadyCopied %ld, toBeCopied %d\n", alreadyCopied, toBeCopied);
      //epicsEventShow(dmaDoneEventId_,3);
      rc = BSP_VMEDmaStart(1, (uintptr_t)(m_data[channel_no].data() + alreadyCopied), (uintptr_t)vmePhysAddr, toBeCopied * sizeof(uint32_t));
      if (rc)
      {
         printf(" return code VMEDmaStart : %d, request for %d words(32bit)\n", rc, toBeCopied);
      }

      /*
       * epicsUInt32 *pData = (epicsUInt32*)m_data[channel_no].data();
       * pData += alreadyCopied;
       * for(unsigned int i = 0; i < toBeCopied; i++) {
       * pData[i] = vmePhysAddr[0];
       * }
       * //bcopyLongs((char*)(m_data[channel_no].data() + alreadyCopied), (char*)vmePhysAddr,toBeCopied * sizeof(uint32_t) );
       *
       * uint32_t *p = m_data[channel_no].data();
       *
       * printf(" data ptr= %p\n",p);
       * for(int i =0;i < 10; i++){
       * printf(" data [%d] = 0x%x\n", i, p[i]);
       * }
       */
      (void)epicsEventWait(dmaDoneEventId_);
      dmaStatus = BSP_VMEDmaStatus(1);
      if (dmaStatus)
      {
         printf(" Warning : dma_status = %d (for %d requested 32bit words)\n", dmaStatus, toBeCopied);
         printf("Transfer Status Reg.: 0x%08x\n", READ(base, SIS3316_DATA_TRANSFER_STATUS_REG(channel_no >> 2)));
      }
      else
      {
         alreadyCopied += toBeCopied;
      }
      //printf(" alreadyCopied : %d,req_nof_32bit_words : %d \n",alreadyCopied, req_nof_32bit_words);
   } while (alreadyCopied < req_nof_32bit_words);
   //now = epicsTime::getCurrent();
   //now.show(0);

   nofData[channel_no] = req_nof_32bit_words;

   reset = 0x00000000;  // Reset
   WRITE(base, SIS3316_DATA_TRANSFER_CTRL_REG((channel_no >> 2) & 0x3), reset);

   return(0);
}

int sis3316_adc::poll_on_adc_dac_offset_busy(void)
{
   unsigned int data;
   unsigned int poll_counter;

   poll_counter = 1000;
   epicsThreadSleep(0.01);
   do
   {
      poll_counter--;
      data = READ(base, SIS3316_ADC_FPGA_SPI_BUSY_STATUS_REG);
      //printf("poll_on_adc_dac_offset_busy: 0x%x\n",data);
   } while (((data & 0xf) != 0) && (poll_counter > 0));
   if (poll_counter == 0)
   {
      printf("Error poll_on_adc_dac_offset_busy: poll_counter = 0x%08x \n", poll_counter);
      return(0x900);
   }
   return(0);
}

int sis3316_adc::configure_all_adc_dac_offsets(void)
{
   int          return_code;
   unsigned int i_adc_fpga_group;

   for (i_adc_fpga_group = 0; i_adc_fpga_group < SIS3316_NUM_FPGA_ADC_GROUPS; i_adc_fpga_group++)
   {
      WRITE(base, SIS3316_ADC_DAC_OFFSET_CTRL_REG(i_adc_fpga_group), 0x88F00001);   // Standalone mode, set Internal Reference
      return_code = this->poll_on_adc_dac_offset_busy();
      if (return_code != 0)
      {
         return(return_code);
      }
      WRITE(base, SIS3316_ADC_DAC_OFFSET_CTRL_REG(i_adc_fpga_group), 0xC0000000);     // CMD: DAC LDAC (load)
      return_code = this->poll_on_adc_dac_offset_busy();
      if (return_code != 0)
      {
         return(return_code);
      }
   }
   return(0);
}

int sis3316_adc::write_all_adc_dac_offsets(void)
{
   int          return_code;
   unsigned int dac_offset;
   unsigned int i_adc_fpga_group;

   for (i_adc_fpga_group = 0; i_adc_fpga_group < SIS3316_NUM_FPGA_ADC_GROUPS; i_adc_fpga_group++)
   {
      dac_offset = this->adc_dac_offset_ch_array[(i_adc_fpga_group * 4) + 0];
      WRITE(base, SIS3316_ADC_DAC_OFFSET_CTRL_REG(i_adc_fpga_group), 0x80000000 + 0x2000000 + 0x000000 + ((dac_offset & 0xffff) << 4));     // clear error Latch bits
      return_code = poll_on_adc_dac_offset_busy();
      if (return_code != 0)
      {
         return(return_code);
      }

      dac_offset = this->adc_dac_offset_ch_array[(i_adc_fpga_group * 4) + 1];
      WRITE(base, SIS3316_ADC_DAC_OFFSET_CTRL_REG(i_adc_fpga_group), 0x80000000 + 0x2000000 + 0x100000 + ((dac_offset & 0xffff) << 4));     // clear error Latch bits
      return_code = poll_on_adc_dac_offset_busy();
      if (return_code != 0)
      {
         return(return_code);
      }

      dac_offset = this->adc_dac_offset_ch_array[(i_adc_fpga_group * 4) + 2];
      WRITE(base, SIS3316_ADC_DAC_OFFSET_CTRL_REG(i_adc_fpga_group), 0x80000000 + 0x2000000 + 0x200000 + ((dac_offset & 0xffff) << 4));     // clear error Latch bits
      return_code = poll_on_adc_dac_offset_busy();
      if (return_code != 0)
      {
         return(return_code);
      }

      dac_offset = this->adc_dac_offset_ch_array[(i_adc_fpga_group * 4) + 3];
      WRITE(base, SIS3316_ADC_DAC_OFFSET_CTRL_REG(i_adc_fpga_group), 0x80000000 + 0x2000000 + 0x300000 + ((dac_offset & 0xffff) << 4)); // clear error Latch bits

      WRITE(base, SIS3316_ADC_DAC_OFFSET_CTRL_REG(i_adc_fpga_group), 0xC0000000);                                                       // CMD LDAC

      return_code = poll_on_adc_dac_offset_busy();
      if (return_code != 0)
      {
         return(return_code);
      }
   }

   return(0);
}

int sis3316_adc::write_all_gain_termination_values(void)
{
   unsigned int gain_termination_reg_value;
   unsigned int i_adc_fpga_group;

   for (i_adc_fpga_group = 0; i_adc_fpga_group < 4; i_adc_fpga_group++)
   {
      gain_termination_reg_value  = (adc_gain_termination_ch_array[(i_adc_fpga_group * 4) + 0] & 0xff);         // ch 1, 5, 9, 1
      gain_termination_reg_value += ((adc_gain_termination_ch_array[(i_adc_fpga_group * 4) + 1] & 0xff) << 8);  // ch 2, 6, 10, 14
      gain_termination_reg_value += ((adc_gain_termination_ch_array[(i_adc_fpga_group * 4) + 2] & 0xff) << 16); // ch 3, 7, 11, 15
      gain_termination_reg_value += ((adc_gain_termination_ch_array[(i_adc_fpga_group * 4) + 3] & 0xff) << 24); // ch 4, 8, 12, 16
      //printf(" write gain termination  0x%x to reg %d\n",gain_termination_reg_value,i_adc_fpga_group );
      WRITE(base, SIS3316_ADC_ANALOG_CTRL_REG(i_adc_fpga_group), gain_termination_reg_value);
   }
   return(0);
}

sis3316_adc::~sis3316_adc(void)
{
}
