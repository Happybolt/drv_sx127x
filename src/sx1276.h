#ifndef __SX1276_H
#define __SX1276_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sx1276.reg.lora.h"


//#define SX1276_DEBUG

#ifdef SX1276_DEBUG
typedef struct{
	uint32_t timeout_send;
	uint32_t timeout_recieved;
}SX1276_Debug;

void SX1276_InitDebug(SX1276_Debug *_sx_debug);


#endif

#define SX1276_NUMBER_MODE	((uint8_t)SX1276_MODE_CAD + 1)

#ifndef SX1276_MODE_MONITOR_STATE
#define SX1276_MODE_MONITOR_STATE 0
#endif

#define SX1276_IS_ENABLE_MM (SX1276_MODE_MONITOR_STATE != 0)

typedef enum {
	SX1276_MODE_SLEEP 			= 0,
	SX1276_MODE_STANDBY 		= 1,
	SX1276_MODE_FSTX				= 2,
	SX1276_MODE_FSRX				= 3,
	SX1276_MODE_TX 					= 4,
	SX1276_MODE_RX_CONT			= 5,
	SX1276_MODE_RX_SINGLE		= 6, // only LoRa
	SX1276_MODE_CAD 				= 7, // only LoRa

}SX1276_Mode;

typedef enum{
	SX1276_STATE_RESET		 							=					0x00U,
	SX1276_STATE_READY									=					0x01U,
	SX1276_STATE_BUSY										=					0x02U,
	SX1276_STATE_TX_RUNNING							=					0x03U,
	SX1276_STATE_RX_RUNNING							=					0x04U,
	SX1276_STATE_CAD								=					0x05U,
	SX1276_STATE_FATAL_ERROR						=					0x06U
}SX1276_State;

typedef enum{
	SX1276_SELECT,
	SX1276_DESELECT
}SX1276_Action;

typedef enum{
	SX1276_MODEM_FSK,
	SX1276_MODEM_LORA
}SX1276_TypeModem;

typedef enum{
	SX1276_HF,
	SX1276_LF
}SX1276_FrequencyMode;

typedef enum{
	SX1276_HEADER_EXPLICIT,
	SX1276_HEADER_IMPLICIT
}SX1276_HeaderMode;

typedef enum{
        SX1276_AMPLIFER_PA_BOOST,
        SX1276_AMPLIFER_RFO
}SX1276_TypeAmplifier;

typedef bool (*sx1276_set_rst)(void *_context, SX1276_Action _action);
typedef bool (*sx1276_transmit_spi)(void *_context, uint8_t *_buf, size_t _size);
typedef bool (*sx1276_receive_spi)(void *_context, uint8_t *_buf, size_t _size);
typedef bool (*sx1276_transmit_receive_spi)(void *_context, uint8_t *_buf_tx, size_t _size_tx, uint8_t *_buf_rx, size_t _size_rx);
typedef bool (*sx1276_set_nss)(void *_context, SX1276_Action _action);
typedef void (*sx1276_atomic_block)(SX1276_Action _action);

typedef uint32_t (*get_time_ms)(void);

#if SX1276_IS_ENABLE_MM
typedef struct
{
	uint32_t timeSetMode;
	uint32_t timeInMode[SX1276_NUMBER_MODE];
}SX1276_ModeMonitor;
#endif

struct SX1276_Definition_{
	sx1276_set_rst 							rst;
	sx1276_transmit_spi 				tx;
	sx1276_receive_spi					rx;
	sx1276_set_nss							nss;
	get_time_ms									get_time;
	sx1276_atomic_block					atomicb;
};

typedef void (*sx1276_received_clbk)(void *_context, uint8_t *_buf,	size_t _size, int8_t _rssi, int8_t _snr);
typedef void (*sx1276_switch_tx_clbk)(void *_context);
typedef void (*sx1276_switch_rx_clbk)(void *_context);

struct SX1276_Clbk_{
	sx1276_received_clbk 	received;
	sx1276_switch_tx_clbk	switchTx;
	sx1276_switch_rx_clbk	switchRx;
};



typedef struct {
	int												freq_current;
	
	SX1276_TypeModem					modem;
	SX1276_State							state;
	SX1276_FrequencyMode			freq_mode;
	SX1276_HeaderMode					header_mode;

	volatile SX1276_Mode mode;
#if SX1276_IS_ENABLE_MM
	SX1276_ModeMonitor *mm;
#endif
	struct SX1276_Definition_ definit;
	struct SX1276_Clbk_				clbk;
	unsigned int  timeStartRxHeader;
	unsigned int  timeDetectionHeader;
        SX1276_TypeAmplifier amplifer;
	void *context;
}SX1276_Descr;


/*
  _tx_rx_spi - deprecated / not used
*/
bool SX1276_Init(SX1276_Descr *_sx,sx1276_set_rst _rst, sx1276_transmit_spi _tx_spi,
									sx1276_receive_spi	_rx_spi, sx1276_set_nss _nss,get_time_ms _get_time,
									sx1276_transmit_receive_spi _tx_rx_spi,SX1276_TypeModem _modem,
									SX1276_FrequencyMode _mode,void *_context, sx1276_atomic_block _atomicb,
                                                                        SX1276_TypeAmplifier _amplifer,
                                                                        bool _toDoRest);

void SX1276_SetClbk(SX1276_Descr *_sx, const struct SX1276_Clbk_ *_clbk);


void SX1276_InterruptDio0(SX1276_Descr *_sx); // call in handler
void SX1276_InterruptDio1(SX1276_Descr *_sx); // call in handler
void SX1276_InterruptDio2(SX1276_Descr *_sx); // call in handler
void SX1276_InterruptDio3(SX1276_Descr *_sx); // call in handler
void SX1276_InterruptDio4(SX1276_Descr *_sx); // call in handler
void SX1276_InterruptDio5(SX1276_Descr *_sx); // call in handler

bool SX1276_SetMode(SX1276_Descr *_sx,SX1276_Mode _mode);
bool SX1276_SetModem(SX1276_Descr *_sx, SX1276_TypeModem _modem);
bool SX1276_SetFreq(SX1276_Descr *_sx, uint32_t _freq);
bool SX1276_SetOutputPower(SX1276_Descr *_sx, uint8_t _precent);
bool SX1276_MaxCurrent(SX1276_Descr *_sx, bool _protection_on, uint8_t _max_current);
bool SX1276_SetLnaGain(SX1276_Descr *_sx, SX1276_LoraLnaGain _gain, bool _on_boost_hf);
bool SX1276_SetBandwidth(SX1276_Descr *_sx, SX1276_BandwidthLr _bw);
bool SX1276_SetCodingRate(SX1276_Descr *_sx, SX1276_CodingRate _cr);
bool SX1276_SetHeaderMode(SX1276_Descr *_sx, SX1276_HeaderMode _mode);
bool SX1276_SetSpreadingFactor(SX1276_Descr *_sx, SX1276_SpreadingFactor_E _sf);
bool SX1276_SetRxPayloadCrc(SX1276_Descr *_sx, bool _crc_on);
bool SX1276_SetPreambleLength(SX1276_Descr *_sx, uint16_t _lenght);
bool SX1276_SetInvertIQ(SX1276_Descr *_sx, bool _invert_mode);
bool SX1276_SetSyncWord(SX1276_Descr *_sx, uint8_t _value);
bool SX1276_SetModeWaitPacket(SX1276_Descr *_sx);
bool SX1276_SetPayloadLength(SX1276_Descr *_sx, uint8_t _lenght);
bool SX1276_BeginCalibrationRxChain(SX1276_Descr *_sx);
double SX1276_GetDistance(int rssi, int txPower);

bool SX1276_Send(SX1276_Descr *_sx,uint8_t *_buf, size_t _size, uint32_t _timeout);
bool SX1276_SendIT(SX1276_Descr *_sx,uint8_t *_buf, size_t _size);

bool SX1276_IsChannelFree(SX1276_Descr *_sx);
bool SX1276_IsReceivingData(SX1276_Descr *_sx);
bool SX1276_IsSignalDetected(SX1276_Descr *_sx);

SX1276_State SX1276_GetState(SX1276_Descr *_sx);
SX1276_Mode  SX1276_GetMode(SX1276_Descr *_sx);
int16_t SX1276_ReadRssi(SX1276_Descr *_sx);

bool SX1276_RdRegIrq(SX1276_Descr *_sx, uint8_t *_value);
bool SX1276_ReadDetectionThreshold(SX1276_Descr *_sx, uint8_t *_value);
bool SX1276_ReadModemStatus(SX1276_Descr *_sx, uint8_t *_value);
bool SX1276_ReadRssiWideband(SX1276_Descr *_sx, uint8_t *_value);

float  SX1276_GetPrecentModeActivity(SX1276_Descr *_sx, SX1276_Mode _mode);
#if SX1276_IS_ENABLE_MM
void SX1276_ConnectModeMonitor(SX1276_Descr *_sx, SX1276_ModeMonitor *_monitor);
#endif

#endif
