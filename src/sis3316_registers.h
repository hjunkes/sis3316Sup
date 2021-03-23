#ifndef SIS3316_REGISTERS_H
#define SIS3316_REGISTERS_H

/* From SIS3316 16 Channel VME Digitizer
 * User Manual.  April 11 2013
 *
 * struck innovative systems.
 */

#include <stdio.h>
#ifdef __rtems__
#include <bsp.h>
#include <libcpu/byteorder.h>
#endif

/* Registers */

#define U32_SIS3316_CONTROL_STATUS                  0x0000
#define U32_SIS3316_MOD_ID                          0x0004
#define U32_SIS3316_HW_VERSION                      0x001c
#define U32_SIS3316_TEMP                            0x0020

#define U32_SIS3316_SERIAL_NUMBER_REG               0x0028

#define U32_SIS3316_IRQ_CONFIG                      0x08
#define U32_SIS3316_IRQ_CONTROL                     0x0C

#define SIS3316_NUM_CHANNELS                        16
#define SIS3316_ALL_CHANNEL                         -1
#define SIS3316_NUM_FPGA_ADC_GROUPS                 4
#define SIS3316_NR_ADC_PER_FPGA                     2

#define SIS3316_MAX_DMA_SAMPLES                     5000000 // start with
#define MAX_DMA_LENGTH                              1024 * 256

#define U32_SIS3316_ADC_INP_TAP_DLY_N               0x01000
#define U32_SIS3316_ADC_INP_TAP_DLY(N)    (U32_SIS3316_ADC_INP_TAP_DLY_N + 0x01000 * (N))

#define U32_VME_FPGA_LINK_ADC_PROT_STATUS           0x00a0
#define U32_INTERFACE_ACCESS_ARBITRATION_CONTROL    0x10

/*
 #define U32_ADC_FPGA_GAIN_TERM_N 0x01004
 #define U32_ADC_FPGA_GAIN_TERM(N) (U32_ADC_FPGA_GAIN_TERM_N + 0x01000*(N))
 *
 #define U32_ADC_FPGA_OFFSET_N 0x01008
 #define U32_ADC_FPGA_OFFSET(N) (U32_ADC_FPGA_OFFSET_N + 0x01000*(N))
 */


#define U32_ONEWIRE_CSR                                  0x24

#define U32_SIS3316_ADC_FPGA_BOOT_CSR                    0x30
#define U32_SIS3316_SPI_FLASH_CSR                        0x34
 #define ENABLE_SPI_PROG                                 0
 #define CHIPSELECT_1                                    1
 #define CHIPSELECT_2                                    2
 #define FIFO_NOT_EMPTY                                  14
 #define FLASH_LOGIC_BUSY                                31

#define U32_SIS3316_SPI_FLASH_DATA                       0x38
#define SIS3316_FLASH_PROGRAM_PAGESIZE                   256
#define SIS3316_FLASH_ERASE_BLOCKSIZE                    65536

#define U32_SIS3316_ADC_CLK_OSC_I2C_REG_N                0x40
#define U32_SIS3316_ADC_CLK_OSC_I2C_REG(N)    (U32_SIS3316_ADC_CLK_OSC_I2C_REG_N + 4 * (N))

#define U32_SIS3316_NIM_CLK_MULTIPLIER_SPI_REG           0x54
#define SI5325_SPI_POLL_COUNTER_MAX                      100
#define SI5325_SPI_CALIBRATION_READY_POLL_COUNTER_MAX    1000

#define U32_SIS3316_NIM_INPUT_CONTROL_REG                0x5C
 #define NIM_INPUT_TI_AS_TRIGGER_ENABLE                  0x10
 #define ENABLE_T1_TRIGGER                               1
 #define DISABLE_T1_TRIGGER                              0

#define U32_SIS3316_AQUISITION_CONTROL                   0x60

#define U32_SIS3316_LEMO_OUT_CO_SELECT_REG               0x70
#define U32_SIS3316_LEMO_OUT_TO_SELECT_REG               0x74
#define U32_SIS3316_LEMO_OUT_UO_SELECT_REG               0x78

#define U32_SIS3316_DATA_TRANSFER_STATUS_REG_N           0x90
#define U32_SIS3316_DATA_TRANSFER_STATUS_REG(N)        (U32_SIS3316_DATA_TRANSFER_STATUS_REG_N + 4 * (N))

#define U32_SIS3316_DATA_TRANSFER_CTRL_REG_N             0x80
#define U32_SIS3316_DATA_TRANSFER_CTRL_REG(N)          (U32_SIS3316_DATA_TRANSFER_CTRL_REG_N + 4 * (N))

#define U32_SIS3316_ADC_INTERNAL_TRIGGER_COUNTER_N       0xC0
#define U32_SIS3316_ADC_INTERNAL_TRIGGER_COUNTER(N)    (U32_SIS3316_ADC_INTERNAL_TRIGGER_COUNTER_N + 4 * (N))

#define U32_SIS3316_ADC_ANALOG_CTRL_REG_N                0x01004
#define U32_SIS3316_ADC_ANALOG_CTRL_REG(N)             (U32_SIS3316_ADC_ANALOG_CTRL_REG_N + 0x01000 * (N))
 #define GAIN_MASK                                       0x3
 #define GAIN_5V                                         0
 #define GAIN_2V                                         1
 #define GAIN_1_9V                                       2
 #define GAIN_X_1_9V                                     3
 #define DIS_50_OHM                                      0x04

#define U32_SIS3316_ADC_DAC_OFFSET_CTRL_REG_N            0x01008
#define U32_SIS3316_ADC_DAC_OFFSET_CTRL_REG(N)      (U32_SIS3316_ADC_DAC_OFFSET_CTRL_REG_N + 0x01000 * (N))

#define U32_SIS3316_ADC_SPI_CTRL_REG_N                   0x0100C
#define U32_SIS3316_ADC_SPI_CTRL_REG(N)             (U32_SIS3316_ADC_SPI_CTRL_REG_N + 0x01000 * (N))

#define U32_SIS3316_ADC_EVENT_CONFIG_REG_N               0x01010
#define U32_SIS3316_ADC_EVENT_CONFIG_REG(N)         (U32_SIS3316_ADC_EVENT_CONFIG_REG_N + 0x01000 * (N))

#define U32_SIS3316_ADC_CHANNEL_HEADER_REG_N             0x01014
#define U32_SIS3316_ADC_CHANNEL_HEADER_REG(N)       (U32_SIS3316_ADC_CHANNEL_HEADER_REG_N + 0x01000 * (N))

#define U32_SIS3316_ADC_ADDRESS_THRESHOLD_REG_N          0x01018
#define U32_SIS3316_ADC_ADDRESS_THRESHOLD_REG(N)    (U32_SIS3316_ADC_ADDRESS_THRESHOLD_REG_N + 0x01000 * (N))

#define U32_SIS3316_ADC_DATAFORMAT_CONFIG_REG_N          0x01030
#define U32_SIS3316_ADC_DATAFORMAT_CONFIG_REG(N)    (U32_SIS3316_ADC_DATAFORMAT_CONFIG_REG_N + 0x01000 * (N))

// 5th is FIR_SUM
 #define FIR_SUM_REG                                             4
#define U32_SIS3316_ADC_FIR_TRIGGER_SETUP_REG_N                  0x01040
#define U32_SIS3316_ADC_FIR_TRIGGER_SETUP_REG(N, C)        (U32_SIS3316_ADC_FIR_TRIGGER_SETUP_REG_N + 0x01000 * (N)+0x10 * (C))

#define U32_SIS3316_ADC_FIR_TRIGGER_THRESHOLD_REG_N              0x01044
#define U32_SIS3316_ADC_FIR_TRIGGER_THRESHOLD_REG(N, C)    (U32_SIS3316_ADC_FIR_TRIGGER_THRESHOLD_REG_N + 0x01000 * (N)+0x10 * (C))
 #define FIR_TRIGGER_ENABLE                                      0x80000000

 #define ENABLE_FIR_TRIGGER                                      1
 #define DISABLE_FIR_TRIGGER                                     0

#define U32_SIS3316_ADC_FIR_HIGH_ENERGY_THRESHOLD_REG_N          0x01048
#define U32_SIS3316_ADC_FIR_HIGH_ENERGY_THRESHOLD_REG(N, C)       (U32_SIS3316_ADC_FIR_HIGH_ENERGY_THRESHOLD_REG_N + 0x01000 * (N)+0x10 * (C))

#define U32_SIS3316_ADC_TRIGGER_GATE_WINDOW_LENGTH_N             0x0101C
#define U32_SIS3316_ADC_TRIGGER_GATE_WINDOW_LENGTH(N)             (U32_SIS3316_ADC_TRIGGER_GATE_WINDOW_LENGTH_N + 0x01000 * (N))

#define U32_SIS3316_ADC_RAW_DATA_BUFFER_CONFIG_REG_N             0x01020
#define U32_SIS3316_ADC_RAW_DATA_BUFFER_CONFIG_REG(N)             (U32_SIS3316_ADC_RAW_DATA_BUFFER_CONFIG_REG_N + 0x01000 * (N))

#define U32_SIS3316_ADC_EXTENDED_RAW_DATA_BUFFER_CONFIG_REG_N    0x01098
#define U32_SIS3316_ADC_EXTENDED_RAW_DATA_BUFFER_CONFIG_REG(N)    (U32_SIS3316_ADC_EXTENDED_RAW_DATA_BUFFER_CONFIG_REG_N + 0x01000 * (N))

#define U32_SIS3316_ADC_PILEUP_CONFIG_REG_N                      0x01024
#define U32_SIS3316_ADC_PILEUP_CONFIG_REG(N)                      (U32_SIS3316_ADC_PILEUP_CONFIG_REG_N + 0x01000 * (N))

#define U32_SIS3316_ADC_PRE_TRIGGER_DELAY_REG_N                  0x01028
#define U32_SIS3316_ADC_PRE_TRIGGER_DELAY_REG(N)                  (U32_SIS3316_ADC_PRE_TRIGGER_DELAY_REG_N + 0x01000 * (N))
 #define MAX_PRE_TRIGGER_DELAY                                   16378

#define U32_SIS3316_ADC_DATAFORMAT_CONFIG_REG_N                  0x01030
#define U32_SIS3316_ADC_DATAFORMAT_CONFIG_REG(N)                  (U32_SIS3316_ADC_DATAFORMAT_CONFIG_REG_N + 0x01000 * (N))

#define U32_SIS3316_ADC_TEST_BUFFER_CONFIG_REG_N                 0x01034
#define U32_SIS3316_ADC_TEST_BUFFER_CONFIG_REG(N)                 (U32_SIS3316_ADC_TEST_BUFFER_CONFIG_REG_N + 0x01000 * (N))

#define U32_SIS3316_ADC_FPGA_FIRMWARE_REG_N                      0x01100
#define U32_SIS3316_ADC_FPGA_FIRMWARE_REG(N)                      (U32_SIS3316_ADC_FPGA_FIRMWARE_REG_N + 0x01000 * (N))

#define U32_SIS3316_ADC_FPGA_STATUS_N                            0x01104
#define U32_SIS3316_ADC_FPGA_STATUS(N)                            (U32_SIS3316_ADC_FPGA_STATUS_N + 0x01000 * (N))

#define U32_SIS3316_ADC_FPGA_OFFSET_RDB_N                        0x01108
#define U32_SIS3316_ADC_FPGA_OFFSET_RDB(N)                        (U32_SIS3316_ADC_FPGA_OFFSET_RDB_N + 0x01000 * (N))

#define U32_SIS3316_ADC_SPI_RDB_N                                0x0110C
#define U32_SIS3316_ADC_SPI_RDB(N)                                (U32_SIS3316_ADC_SPI_RDB_N + 0x01000 * (N))

#define U32_SIS3316_ADC_ACTUAL_SAMPLE_ADDRESS_REG_N              0x01110
#define U32_SIS3316_ADC_ACTUAL_SAMPLE_ADDRESS_REG(N, C)           (U32_SIS3316_ADC_ACTUAL_SAMPLE_ADDRESS_REG_N + 0x01000 * (N)+4 * (C))

#define U32_SIS3316_ADC_PREVIOUS_BANK_SAMPLE_ADDRESS_REG_N       0x01120
#define U32_SIS3316_ADC_PREVIOUS_BANK_SAMPLE_ADDRESS_REG(N, C)    (U32_SIS3316_ADC_PREVIOUS_BANK_SAMPLE_ADDRESS_REG_N + 0x01000 * (N)+4 * (C))

#define SIS3316_FPGA_ADC_MEM_BASE                                0x100000
#define SIS3316_FPGA_ADC_MEM_OFFSET                              0x100000


 #define I2C_ACK                                                 8
 #define I2C_START                                               9
 #define I2C_REP_START                                           10
 #define I2C_STOP                                                11
 #define I2C_WRITE                                               12
 #define I2C_READ                                                13
 #define I2C_BUSY                                                31


#define U32_SIS3316_ADC_FPGA_SPI_BUSY_STATUS_REG                 0xA4
#define OSC_ADR                                                  0x55

#define U32_SIS3316_KEY_RESET                                    0x400
#define U32_SIS3316_KEY_DISARM                                   0x414
#define U32_SIS3316_KEY_TRIGGER                                  0x418
#define U32_SIS3316_KEY_TIMESTAMP_CLR                            0x41c
#define U32_SIS3316_KEY_DISARM_ARM_BANK1                         0x420
#define U32_SIS3316_KEY_DISARM_ARM_BANK2                         0x424

#define U32_SIS3316_KEY_ADC_CLOCK_DCM_RESET                      0x438

#define AWG_DDS_A                                                0
#define AWG_DDS_B                                                1
#define AWG_DDS_C                                                2
#define AWG_DDS_D                                                3

#define AWG_AMPL_W                                               0
#define AWG_AMPL_X                                               1
#define AWG_AMPL_Y                                               2
#define AWG_AMPL_Z                                               3

#define U16_FHN                                                  0x100
#define U16_FLN                                                  0x102
#define U16_FH(N)     (U16_FHN + 0x10 * (N))
#define U16_FL(N)     (U16_FLN + 0x10 * (N))

#define U16_AMPN          0x140
#define U16_AMP(N)    (U16_AMPN + 0x2 * (N))

#define U16_OFSN          0x148
#define U16_OFS(N)    (U16_OFSN + 0x2 * (N))

#define U16_SRCN          0x160
#define U16_SRC(N)    (U16_SRCN + 0x20 * (N))

#define U16_DIVN          0x162
#define U16_DIV(N)    (U16_DIVN + 0x20 * (N))

#define U16_PTRN          0x164
#define U16_PTR(N)    (U16_PTRN + 0x20 * (N))

#define U16_WAVN          0x166
#define U16_WAV(N)    (U16_WAVN + 0x20 * (N))

#define U16_SIZN          0x168
#define U16_SIZ(N)    (U16_SIZN + 0x20 * (N))

#define U16_BASN          0x16a
#define U16_BAS(N)    (U16_BASN + 0x20 * (N))

#define U16_PHAN          0x16c
#define U16_PHA(N)    (U16_PAHN + 0x20 * (N))

#define U16_FLTN          0x16e
#define U16_FLT(N)    (U16_FLTN + 0x20 * (N))

#define U16_JMPN          0x170
#define U16_JMP(N)    (U16_JMPN + 0x20 * (N))

#define U16_TRGN          0x172
#define U16_TRG(N)    (U16_TRGN + 0x20 * (N))

#define U16_JPHN          0x174
#define U16_JPH(N)    (U16_JPHN + 0x20 * (N))

#define U16_JBAN          0x176
#define U16_JBA(N)    (U16_JPBA + 0x20 * (N))

#define U16_SNPN          0x178
#define U16_SNP(N)    (U16_SNPN + 0x20 * (N))

#define U16_BCCN          0x17a
#define U16_BCC(N)    (U16_BCCN + 0x20 * (N))

#define U16_BCXN          0x17c
#define U16_BCX(N)    (U16_BCXN + 0x20 * (N))

#define U16_SUMW          0x17e
#define U16_SUMX          0x19e
#define U16_SUMY          0x1be
#define U16_SUMZ          0x1de

#define U16_TPASS_MSW     0x1f6
#define U16_TPASS_LSW     0x1f8
#define U16_TERROR_MSW    0x1fa
#define U16_TERROR_LSW    0x1fc
#define U16_TSTOP         0x1fe
#define TSTOP_MAGIC_NR    0x137

# define ADDR_cad_mask(N)    ((N) ? 0xfff0 : 0x000f)
# define ADDR_cad_shft(N)    ((N) ? 4 : 0)

/* # of 16-bit words */
#define BUFFER_SIZE    128

#define MACRO_SIZE     112 // Params(111) + macro

#define DIV_MIN        0
#define DIV_MAX        255

#define SIZ_MAX        10
#define SIZ_BLOCK      64
#define SIZ_1024       4

#ifdef __rtems__
#define READ(base, reg)          in_be32((volatile uint32_t *)((base) + (U32_ ## reg) / sizeof(uint32_t)))
#define WRITE(base, reg, val)    out_be32((volatile uint32_t *)((base) + (U32_ ## reg) / sizeof(uint32_t)), val)
#endif
#ifndef __rtems__
#define READ(base, reg)          printf("READ (%p, 0x%x)\n", base, (U32_ ## reg))
#define WRITE(base, reg, val)    printf("WRITE (%p, 0x%x, 0x%x)\n", base, (U32_ ## reg), val)
#endif

#endif // SIS3316_REGISTERS_H
