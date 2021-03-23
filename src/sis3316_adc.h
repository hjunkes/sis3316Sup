#ifndef SIS3316_ADC_H
#define SIS3316_ADC_H

#include <string>
#include <vector>
#include <stdexcept>

#include <cstdio>
#include <epicsEvent.h>

#include <epicsTypes.h>
#include "sis3316_registers.h"

#define INT_CLOCK_SOURCE_250MHz     0
#define INT_CLOCK_SOURCE_125MHz     1
#define INT_CLOCK_SOURCE_62_5MHz    2

class sis3316_adc
{
private:

   int FlashEnableProg(void);
   int FlashDisableProg(void);
   int FlashEnableCS(int chip);
   int FlashDisableCS(int chip);
   int FlashWriteEnable(void);
   int FlashProgramPage(int address, char *data, int len);
   int FlashEraseBlock(int address);

   int FlashXfer(char in, char *out);

   //BSP_VMEDmaListDescriptor head;
   void *head;
   epicsEventId dmaDoneEventId_;

   int adc_spi_setup(void);
   int adc_spi_read(unsigned int adc_fpga_group, unsigned int adc_chip, unsigned int spi_addr, unsigned int *spi_data);
   int adc_spi_write(unsigned int adc_fpga_group, unsigned int adc_chip, unsigned int spi_addr, unsigned int spi_data);
   int poll_on_adc_dac_offset_busy(void);
   int I2cStart(int osc);
   int I2cStop(int osc);
   int I2cWriteByte(int osc, unsigned char data, char *ack);
   int I2cReadByte(int osc, unsigned char *data, char ack);
   int Si570FreezeDCO(int osc);
   int Si570ReadDivider(int osc, unsigned char *data);
   int Si570Divider(int osc, unsigned char *data);
   int Si570UnfreezeDCO(int osc);
   int Si570NewFreq(int osc);

   int si5325_clk_muliplier_write(unsigned int addr, unsigned int data);
   int si5325_clk_muliplier_read(unsigned int addr, unsigned int *data);
   int si5325_clk_muliplier_internal_calibration_cmd(void);

   int owReset(int *presence);
   int owRead(unsigned char *data);
   int owWrite(unsigned char data);
   int owEeReadPage(int page, unsigned char *data);
   int owEeWritePage(int page, unsigned char *data);

public:
   volatile epicsUInt32 *base;
   epicsUInt32 a32addr;
   bool enabled;
   sis3316_adc(volatile epicsUInt32 *baseaddress, epicsUInt32 a32addr);
   int update_firmware(char *path, int offset, void (*cb)(int percentage));

   //unsigned char freqPreset62_5MHz[6];
   //unsigned char freqPreset125MHz[6];
   //unsigned char freqPreset250MHz[6];
   //unsigned char freqSI570_calibrated_value_125MHz[6];
   unsigned int iob_delay_value;
   unsigned int adc_125MHz_flag;
   unsigned int timeBase;      //ns per sample
   unsigned int bank1_armed_flag;


   int interruptVector;
   int interruptLevel;


   unsigned int sampleLength;
   unsigned int headerLength;
   unsigned int eventLength;
   unsigned int nofEvents;
   unsigned int addressThreshold;
   unsigned int average;

   std::vector <std::vector <epicsUInt32> > m_data;
   unsigned int nofData[SIS3316_NUM_CHANNELS];
   void dmaCallback();

   int configure_all_adc_dac_offsets(void);
   int write_all_adc_dac_offsets(void);
   int write_all_gain_termination_values(void);
   int internal_trigger_generation_setup(int channel, unsigned int uint_trigger_threshold_reg, unsigned int uint_he_trigger_threshold_reg,
                                         unsigned int uint_trigger_setup_reg);
   int dataFormat(int channel, epicsUInt8 format);
   int change_frequency_HSdiv_N1div(int osc, unsigned hs_div_val, unsigned n1_div_val);
   int set_frequency(int osc, unsigned char *values);
   int get_frequency(int osc, unsigned char *values);
   int configure_adc_fpga_iob_delays(unsigned int iob_delay_value);
   int adc_spi_reg_enable_adc_outputs(void);
   int FlashGetId(char *id);
   int set_external_clock_multiplier(unsigned int bw_sel, unsigned int n1_hs, unsigned int n1_clk1, unsigned int n1_clk2, unsigned int n2, unsigned int n3, unsigned int clkin1_mhz);
   int bypass_external_clock_multiplier(void);
   int get_status_external_clock_multiplier(unsigned int *status);
   int read_ee(int offset, int len, unsigned char *data);
   int write_ee(int offset, int len, unsigned char *data);
   int ow_id_ee(unsigned char *data);

   unsigned int clockSourceChoice;
   unsigned short adc_dac_offset_ch_array[SIS3316_NUM_CHANNELS];
   unsigned char adc_gain_termination_ch_array[SIS3316_NUM_CHANNELS];

   void enableInterrupts();
   void disableInterrupts();

   unsigned int serial_number;
   unsigned int adc_fpga_version[SIS3316_NUM_FPGA_ADC_GROUPS];
   unsigned int module_id;
   unsigned int revision_number;
   unsigned int hw_version;

   // dac
   int setDac(int channel, unsigned short value);
   int readDac(int channel, unsigned short *value);
   int inputT1Trigger(epicsUInt32 state);
   int selectCO(epicsUInt8 bit);
   int selectTO(epicsUInt8 bit);
   int selectUO(epicsUInt8 bit);
   int setGain(int channel, unsigned char value);
   int setTermination(int channel, unsigned char value);
   int setHeaderId(int group, epicsUInt32 value);
   int firTrigger(int channel, epicsUInt32 state);
   int eventConfig(int channel, epicsUInt8 state);
   int firSumTrigger(int group, epicsUInt32 state);
   int setHighEnergyThreshold(int channel, epicsUInt32 value);
   int setSumHighEnergyThreshold(int roup, epicsUInt32 value);
   int firTriggerSetup(int channel, unsigned short pulse, unsigned short gapTime, unsigned short peakingTime);
   int setTriggerGateWindowLength(int group, unsigned short length);
   int setBufferSample(int group, unsigned int sampleLength, unsigned int startIndex);
   int pileupConfig(int group, unsigned short rePileupWin, unsigned int pileupWin);
   int setPreTriggerDelay(int group, unsigned short delay);
   int setDataFormat(int channel, unsigned char saveMode);
   int setMawTestBuffer(int channel, unsigned short bLength, unsigned short bPreTriggerDelay);
   int sumFirTriggerSetup(int group, unsigned short pulse, unsigned short gapTime, unsigned short peakingTime);
   int setAddressThreshold(int group, epicsUInt32 threshold);
   int setAcquisitionControl(epicsUInt32 bit, epicsUInt32 state);
   int read_Channel_PreviousBankDataBuffer(unsigned int channel_no);

   ~sis3316_adc(void);
};
#endif /* SIS3316_ADC_H */
