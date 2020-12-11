#include "sx1276.h"
#include "sx1276.reg.fsk.h"
#include "sx1276.reg.lora.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "math.h"

#define TIME_EXPECTATIONS 100
#define FSTEP 			61.03515625 // Frequency synthesizer step
#define XTAL_FREQ   (32000000)

#define SIZE_FIFO		256
#define SIZE_SYNC		1

#define FIFO_FIRST_ADDR		0x00U
#define FIFO_LAST_ADDR		0xFFU

#define RSSI_OFFSET_HF -157
#define RSSI_OFFSET_LF -164

#define SIZE_REG		sizeof(uint8_t) 
#define MASK_FULL_VAL_REG		0xFF

#define LOCK_SX(__DESCR__,__RETURN__)												\
	do{																												\
			if((__DESCR__)->state != 	SX1276_STATE_READY)					\
				return (__RETURN__);																\
			(__DESCR__)->state = SX1276_STATE_BUSY;								\
	}while(0);

#define UNLOCK_SX(__DESCR__,__RETURN__)											\
	do{																												\
			(__DESCR__)->state = SX1276_STATE_READY;							\
				return (__RETURN__);																\
	}while(0);

#define UNLOCK(__DESCR__)											\
	do{																												\
			(__DESCR__)->state = SX1276_STATE_READY;							\
	}while(0);

#define __CH(__FUNCT__) 											\
		do{																				\
			if(!(__FUNCT__)){	return false;}					\
		}while(0)

#define __IS_TIMEOUT_OUT(__sx__,__START__)												  \
		((__sx__->definit.get_time() - __START__) > TIME_EXPECTATIONS)			


#define __SAFE_CLBK(__CLBK__, __DRV__)	\
	if((__CLBK__) != NULL)					\
	{										\
		__CLBK__((__DRV__)->context);		\
	}

typedef struct{
	bool is_successfully;
	uint8_t value_reg;
}ReadRegResult;

typedef struct{
	int16_t rssi;
	int16_t snr;
	bool is_successfully;
}InfLastRxPacket;
 

typedef enum{
	SX_IRQ_FLAG_RX_TIMEOUT							=					RFLR_IRQFLAGS_RXTIMEOUT,
	SX_IRQ_FLAG_RX_DONE									=					RFLR_IRQFLAGS_RXDONE,
	SX_IRQ_FLAG_RX_PAYLOAD_CRC_ERROR		=					RFLR_IRQFLAGS_PAYLOADCRCERROR,
	SX_IRQ_FLAG_RX_VALID_HEADER					=					RFLR_IRQFLAGS_VALIDHEADER,
	SX_IRQ_FLAG_RX_TX_DONE							=					RFLR_IRQFLAGS_TXDONE,
	SX_IRQ_FLAG_RX_CADDONE							=					RFLR_IRQFLAGS_CADDONE,
	SX_IRQ_FLAG_RX_FHSS_CHANGED_CNABBEL	=					RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL,
	SX_IRQ_FLAG_RX_CAD_DETECTED					=					RFLR_IRQFLAGS_CADDETECTED
}ValueIrqFlags;

typedef enum{
	SX_MODEM_CLEAR												=					0x01 	<<	4,
	SX_HEADER_INFO_VALID									=					0x01	<<	3,
	SX_RX_ON_GOING												=					0x01	<<	2,
	SX_SIGNAL_SYNCHRONIZED								=					0x01	<<	1,
	SX_SIGNAL_DETECTED										=					0x01	<<	0
}ModemStatus;

uint8_t GetValueRegMode(SX1276_Mode _mode){
	switch(_mode){
		case SX1276_MODE_SLEEP:
			return RFLR_OPMODE_SLEEP;
		case SX1276_MODE_STANDBY:
			return RFLR_OPMODE_STANDBY;
		case SX1276_MODE_FSTX:
			return RFLR_OPMODE_SYNTHESIZER_TX;
		case SX1276_MODE_FSRX:
			return RFLR_OPMODE_SYNTHESIZER_RX;  
		case SX1276_MODE_TX:
			return RFLR_OPMODE_TRANSMITTER;
		case SX1276_MODE_RX_CONT:
			return RFLR_OPMODE_RECEIVER;
		case SX1276_MODE_RX_SINGLE:
			return RFLR_OPMODE_RECEIVER_SINGLE;
		case SX1276_MODE_CAD:
			return RFLR_OPMODE_CAD;
	}
	return RFLR_OPMODE_MASK;
}

static SX1276_Mode GetModeValueReg(uint8_t _valueReg){
	switch(_valueReg){
		case RFLR_OPMODE_SLEEP:
			return SX1276_MODE_SLEEP;
		case RFLR_OPMODE_STANDBY:
			return SX1276_MODE_STANDBY;
		case RFLR_OPMODE_SYNTHESIZER_TX:
			return SX1276_MODE_FSTX;
		case RFLR_OPMODE_SYNTHESIZER_RX:
			return SX1276_MODE_FSRX;
		case RFLR_OPMODE_TRANSMITTER:
			return SX1276_MODE_TX;
		case RFLR_OPMODE_RECEIVER:
			return SX1276_MODE_RX_CONT;
		case RFLR_OPMODE_RECEIVER_SINGLE:
			return SX1276_MODE_RX_SINGLE;
		case RFLR_OPMODE_CAD:
			return SX1276_MODE_CAD;
	}
	return RFLR_OPMODE_MASK;
}

void ControlModeMonitor(SX1276_Descr *_sx, SX1276_Mode _old, SX1276_Mode _new);


#ifdef SX1276_DEBUG
	SX1276_Debug *debug;
void SX1276_InitDebug(SX1276_Debug *_sx_debug){
	debug = _sx_debug;
}
#endif
static bool RdRegister(SX1276_Descr *_sx, uint8_t _reg, uint8_t *_value){
	uint32_t time_start = _sx->definit.get_time();
	bool res = true;

        _sx->definit.nss(_sx->context,SX1276_SELECT);

	while(!_sx->definit.tx(_sx->context,&_reg,sizeof(_reg)) || 
              !_sx->definit.rx(_sx->context,(uint8_t*)_value,sizeof(*_value)))
        {
          if(__IS_TIMEOUT_OUT(_sx,time_start))
          {
            res = false;
            break;
          }
        }
        _sx->definit.nss(_sx->context,SX1276_DESELECT);

	return res;
}


static bool WrRegister(SX1276_Descr *_sx, uint8_t _reg, uint8_t _value){
	uint32_t time_start = _sx->definit.get_time();
	uint8_t tx_buf[2] = { _reg | 0x80, _value};
	bool res = true;

	_sx->definit.nss(_sx->context,SX1276_SELECT);
	while(!_sx->definit.tx(_sx->context,tx_buf,sizeof(tx_buf)))
	{
		if(__IS_TIMEOUT_OUT(_sx,time_start))
		{
			res = false;
			break;
		}

	}
	_sx->definit.nss(_sx->context,SX1276_DESELECT);
	return res;
}

static bool WriteFifo(SX1276_Descr *_sx, uint8_t *_buf,size_t _size){

	_sx->definit.nss(_sx->context,SX1276_SELECT);
	uint8_t reg = REG_LR_FIFO | 0x80;
	
	bool res = 	_sx->definit.tx(_sx->context,(uint8_t*)&reg,sizeof(reg))	&&
							_sx->definit.tx(_sx->context,(uint8_t*)_buf,_size);

	_sx->definit.nss(_sx->context,SX1276_DESELECT);
	return res;
}
static bool ReadFifo(SX1276_Descr *_sx, uint8_t _addr, uint8_t *_buf,size_t _size){
	_sx->definit.nss(_sx->context,SX1276_SELECT);

	bool res	=	_sx->definit.tx(_sx->context,(uint8_t*)&(_addr),sizeof(_addr))	&& 
							_sx->definit.rx(_sx->context,_buf,_size);
	
	_sx->definit.nss(_sx->context,SX1276_DESELECT);
	return res;
}
static bool ReplaceRegister(SX1276_Descr *_sx, uint8_t _reg, uint8_t _value, uint8_t _mask){
	uint8_t value_reg;
	
	return 	RdRegister(_sx,_reg,(uint8_t*)&value_reg) 			&&
			WrRegister(_sx,_reg,(value_reg & _mask) | _value);
	
}

ReadRegResult RdValueRegister(SX1276_Descr *_sx, uint8_t _reg, uint8_t _mask){
	ReadRegResult res = {0};
	res.is_successfully = RdRegister(_sx,_reg,(uint8_t*)&res.value_reg);
	res.value_reg &= _mask;
	return res;
}



static bool IsConnectModul(SX1276_Descr *_sx){
	uint8_t rx_buf = 0;
	__CH(RdRegister(_sx,REG_LR_VERSION,(uint8_t*)&rx_buf));

	if(rx_buf != 0 && rx_buf != 0xFF)
		return true;

	return false;
}

static bool Reset(SX1276_Descr *_sx){
	__CH(_sx->definit.rst(_sx->context, SX1276_SELECT));
	uint32_t time_start = _sx->definit.get_time();
	while((_sx->definit.get_time() - time_start) < 10);

	__CH(_sx->definit.rst(_sx->context, SX1276_DESELECT));
	time_start = _sx->definit.get_time();
	while((_sx->definit.get_time() - time_start) < 20); // by datasheet 10 ms

	return true;
}



static bool SetMode(SX1276_Descr *_sx, SX1276_Mode _mode){

	if(_mode == SX1276_MODE_RX_CONT	||
			_mode == SX1276_MODE_RX_SINGLE	||
			_mode == SX1276_MODE_FSRX )
	{
		__SAFE_CLBK(_sx->clbk.switchRx, _sx);
	}else if(_mode == SX1276_MODE_TX || _mode == SX1276_MODE_FSTX)
	{
		__SAFE_CLBK(_sx->clbk.switchTx, _sx);
	}

	_sx->timeDetectionHeader = 0;
	_sx->timeStartRxHeader = 0;
	int res =  ReplaceRegister(_sx,REG_LR_OPMODE,GetValueRegMode(_mode),RFLR_OPMODE_MASK);
	if(res)
	{
		ControlModeMonitor(_sx, _sx->mode, _mode);
		_sx->mode = _mode;
	}
	return res;
}

static SX1276_Mode RdMode(SX1276_Descr *_sx)
{
	SX1276_Mode mode = SX1276_MODE_SLEEP;
	uint8_t valueReg;
	if(RdRegister(_sx, REG_LR_OPMODE, &valueReg))
	{
		mode = GetModeValueReg(valueReg & ~RFLR_OPMODE_MASK);
	}
	return mode;
}

static bool SetModem(SX1276_Descr *_sx, SX1276_TypeModem _modem){

	if(_modem == SX1276_MODEM_LORA){
		ReadRegResult reg_opmode = RdValueRegister(_sx,REG_LR_OPMODE,RFLR_OPMODE_LONGRANGEMODE_MASK);
		__CH(reg_opmode.is_successfully);
		
		if(reg_opmode.value_reg != SX1276_MODE_SLEEP)
			__CH(SetMode(_sx, SX1276_MODE_SLEEP));
		
		__CH(ReplaceRegister(_sx,REG_LR_OPMODE,RFLR_OPMODE_LONGRANGEMODE_ON,RFLR_OPMODE_LONGRANGEMODE_MASK));

		if(reg_opmode.value_reg != SX1276_MODE_SLEEP)
			return SetMode(_sx,(SX1276_Mode)(reg_opmode.value_reg &(~RFLR_OPMODE_LONGRANGEMODE_MASK)));
	}

	return true;
}

static bool SetFreq(SX1276_Descr *_sx, uint32_t _freq){
	uint32_t freq = (int)((double)_freq / ( double )FSTEP);
	__CH(WrRegister(_sx,REG_LR_FRFMSB,(uint8_t)((freq >> 16) & 0xFF)));
	__CH(WrRegister(_sx,REG_LR_FRFMID,(uint8_t)((freq >> 8) & 0xFF)));
	__CH(WrRegister(_sx,REG_LR_FRFLSB,(uint8_t)((freq >> 0) & 0xFF)));
	_sx->freq_current = _freq;
	return true;
}

static bool SetOutputPowerdBm(SX1276_Descr *_sx, int8_t _power_dBm){ // (-4 dBm) --- (20 dBm)
	uint8_t reg_pa;

	if(_power_dBm > 23)
		_power_dBm = 23;
	if(_power_dBm < 5)
		_power_dBm = 5;

	// For RH_RF95_PA_DAC_ENABLE, manual says '+20dBm on PA_BOOST when OutputPower=0xf'
	// RH_RF95_PA_DAC_ENABLE actually adds about 3dBm to all power levels. We will us it
	// for 21, 22 and 23dBm
	if(_power_dBm > 20)
	{
		__CH(ReplaceRegister(_sx,REG_LR_PADAC,RFLR_PADAC_20DBM_ON,RFLR_PADAC_20DBM_MASK));
		_power_dBm -= 3;
	}else{
		__CH(ReplaceRegister(_sx,REG_LR_PADAC,RFLR_PADAC_20DBM_OFF,RFLR_PADAC_20DBM_MASK));
	}
	// RFM95/96/97/98 does not have RFO pins connected to anything. Only PA_BOOST
	// pin is connected, so must use PA_BOOST
	// Pout = 2 + OutputPower.
	// The documentation is pretty confusing on this topic: PaSelect says the max power is 20dBm,
	// but OutputPower claims it would be 17dBm.
	// My measurements show 20dBm is correct
	reg_pa = RFLR_PACONFIG_PASELECT_PABOOST | (_power_dBm -5);

	return WrRegister(_sx,REG_LR_PACONFIG,reg_pa);
}


bool SetRegOcp(SX1276_Descr *_sx, bool _ocp_enabled, uint8_t _max_current){ // 45 - 240 mA
	if(_max_current < 45)
		_max_current = 45;
	else
		if(_max_current > 240)
			_max_current = 240;

	uint8_t value_trim = 0;
	if( 15 >= _max_current)
		value_trim = (_max_current - 45)/5;
	else
		value_trim = (_max_current + 30)/10;

	return WrRegister(_sx, REG_LR_OCP,(uint8_t)(_ocp_enabled << 5) | value_trim);
}




bool SetRegLna(SX1276_Descr *_sx, SX1276_LoraLnaGain _gain, bool _on_boost_hf){
	uint8_t value_lna = _gain |	RFLR_LNA_BOOST_LF_DEFAULT;
	value_lna |= _on_boost_hf ? RFLR_LNA_BOOST_HF_ON : RFLR_LNA_BOOST_HF_OFF;
	return WrRegister(_sx,REG_LR_LNA,value_lna);
}

bool SetRegFifoAddrPtr(SX1276_Descr *_sx,uint8_t _ptr_fifo){
	return WrRegister(_sx,REG_LR_FIFOADDRPTR,_ptr_fifo);
}

bool SetRegFifoTxBaseAddr(SX1276_Descr *_sx,uint8_t _ptr_fifo){
	return WrRegister(_sx,REG_LR_FIFOTXBASEADDR,_ptr_fifo);
}

bool SetRegFifoRxBaseAddr(SX1276_Descr *_sx,uint8_t _ptr_fifo){
	return WrRegister(_sx,REG_LR_FIFORXBASEADDR,_ptr_fifo);
}
bool SetRegIrqFlagsMask(SX1276_Descr *_sx, uint8_t _bit_mask){
	return WrRegister(_sx,REG_LR_IRQFLAGSMASK,_bit_mask);
}

bool SetBandwidth(SX1276_Descr *_sx, SX1276_BandwidthLr _bw){
	return ReplaceRegister(_sx,REG_LR_MODEMCONFIG1, _bw, RFLR_MODEMCONFIG1_BW_MASK);
}

bool SetCodingRate(SX1276_Descr *_sx, SX1276_CodingRate _cr){
	return ReplaceRegister(_sx,REG_LR_MODEMCONFIG1, _cr, RFLR_MODEMCONFIG1_CODINGRATE_MASK);
}

bool SetHeaderMode(SX1276_Descr *_sx, SX1276_HeaderMode _mode){
	uint8_t value_reg_header_mode = _mode ==  SX1276_HEADER_EXPLICIT	
																? RFLR_MODEMCONFIG1_IMPLICITHEADER_OFF	:
																	RFLR_MODEMCONFIG1_IMPLICITHEADER_ON;

	return ReplaceRegister(_sx,REG_LR_MODEMCONFIG1, value_reg_header_mode, RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK);
}

bool SetSpreadingFactor(SX1276_Descr *_sx, SX1276_SpreadingFactor_E _sf){
	return ReplaceRegister(_sx,REG_LR_MODEMCONFIG2, _sf, RFLR_MODEMCONFIG2_SF_MASK);
}

bool SetContinuousMode(SX1276_Descr *_sx, bool _mode_on){
	uint8_t cont_mode = _mode_on ?	RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_ON:
																	RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_OFF;
	
	return ReplaceRegister(_sx,REG_LR_MODEMCONFIG2, cont_mode, RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_MASK);
}

bool SetRxPayloadCrc(SX1276_Descr *_sx, bool _crc_on){
	uint8_t crc = _crc_on ? RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON	:
													RFLR_MODEMCONFIG2_RXPAYLOADCRC_OFF;

	return ReplaceRegister(_sx,REG_LR_MODEMCONFIG2, crc, RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK);
}

bool SetSymbTimeout(SX1276_Descr *_sx, uint16_t _timeout){ // To DO  Rs = BW/2^SF  - > Ts = 1 / Rs  - > TimeOut = SymbTimeout * Ts
// uint32_t rs = 0;
	return false; 
}

bool SetPreambleLength(SX1276_Descr *_sx, uint16_t _lenght){
	return 	WrRegister(_sx,REG_LR_PREAMBLEMSB, (_lenght >> 8))		&&
					WrRegister(_sx,REG_LR_PREAMBLELSB, (_lenght & 0xFF));
}

bool SetPayloadLength(SX1276_Descr *_sx, uint8_t _lenght){
	return WrRegister(_sx,REG_LR_PAYLOADLENGTH,_lenght);
}

bool SetPayloadMaxLength(SX1276_Descr *_sx, uint8_t _lenght){
	return WrRegister(_sx,REG_LR_PAYLOADMAXLENGTH,_lenght);
}

bool SetFreqHoppingPeriod(SX1276_Descr *_sx, uint8_t _period){
	return WrRegister(_sx,REG_LR_HOPPERIOD,_period);
}

bool SetLowDataRateOptimize(SX1276_Descr *_sx, bool _enable){
	uint8_t optimize = _enable ? 	RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_ON :
																RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_OFF;

	return ReplaceRegister(_sx,REG_LR_MODEMCONFIG3,optimize,RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK);
}

bool SetAgcAutoOn(SX1276_Descr *_sx, bool _enable){
	uint8_t agc = _enable ? RFLR_MODEMCONFIG3_AGCAUTO_ON:
													RFLR_MODEMCONFIG3_AGCAUTO_OFF;

	return ReplaceRegister(_sx,REG_LR_MODEMCONFIG3,agc,RFLR_MODEMCONFIG3_AGCAUTO_MASK);
}

bool SetPpmCorrection(SX1276_Descr *_sx, uint8_t _value){
	return WrRegister(_sx,REG_LR_PPMCORRECTION,_value);
}

bool SetInvertIQ(SX1276_Descr *_sx, bool _invert_mode){
	uint8_t invert = _invert_mode ? RFLR_INVERTIQ_ON : RFLR_INVERTIQ_OFF;
	return ReplaceRegister(_sx,REG_LR_INVERTIQ,invert,RFLR_INVERTIQ_MASK);
}

bool SetSyncWord(SX1276_Descr *_sx, uint8_t _value){
	SX1276_Mode originalMode = _sx->mode;
	return 	SetMode(_sx, SX1276_MODE_SLEEP)		&&
			WrRegister(_sx,REG_LR_SYNCWORD,_value)	&&
			SetMode(_sx, originalMode);
}

bool SetMappingDio(SX1276_Descr *_sx, SX1276_MappingDio0 _map){
	return ReplaceRegister(_sx,REG_LR_DIOMAPPING1,_map,RFLR_DIOMAPPING1_DIO0_MASK);
}

/*Clear*/
bool ClearReqIrqFlag(SX1276_Descr *_sx, ValueIrqFlags _flag){
	return WrRegister(_sx,REG_LR_IRQFLAGS, _flag);
} 
/*Rd*/
uint8_t RdRegFifoRxCurrentAddr(SX1276_Descr *_sx){
	ReadRegResult res =  RdValueRegister(_sx,REG_LR_FIFORXCURRENTADDR,MASK_FULL_VAL_REG);
	return res.is_successfully ? res.value_reg : 0;
}


bool IsIrqFlag(SX1276_Descr *_sx,ValueIrqFlags _flag){
	ReadRegResult res = RdValueRegister(_sx,REG_LR_IRQFLAGS,_flag);
	return res.is_successfully ? (bool)res.value_reg : false;
}


uint8_t RdRxNumbBytes(SX1276_Descr *_sx){
	ReadRegResult res = RdValueRegister(_sx, REG_LR_RXNBBYTES, 0xFF);
	return res.is_successfully ? res.value_reg : false;
}

uint32_t RdNumbValidHeaders(SX1276_Descr *_sx){
	ReadRegResult res = {0};
	uint32_t numb_valid_hd = 0;

	for(uint8_t idx = 0 ;idx < sizeof(SX1276_RegLrValidHeaderCnt); idx++){
		res = RdValueRegister(_sx,SX1276_RegLrValidHeaderCnt[idx],MASK_FULL_VAL_REG);
		if(!res.is_successfully)
			return 0;
		numb_valid_hd = res.value_reg << (24 - (idx * sizeof(uint8_t)));
	}

	return numb_valid_hd;
}

uint8_t RdRxCodingRate(SX1276_Descr *_sx){
	ReadRegResult res = RdValueRegister(_sx, REG_LR_MODEMSTAT, RFLR_MODEMSTAT_RX_CR_MASK);
	return res.is_successfully ? res.value_reg : 0;
}

bool IsStatusModem(SX1276_Descr *_sx, ModemStatus _status){
	ReadRegResult res = RdValueRegister(_sx, REG_LR_MODEMSTAT, RFLR_MODEMSTAT_MODEM_STATUS_MASK);
	return res.is_successfully ? res.value_reg & _status : false;
}
int8_t RdRxPacketSnr(SX1276_Descr *_sx){
	ReadRegResult res = RdValueRegister(_sx, REG_LR_PKTSNRVALUE, MASK_FULL_VAL_REG);

	if(!res.is_successfully)
		return 0;

	res.value_reg = res.value_reg & 0x80 ? 	- ((~res.value_reg + 1) >> 2) :
																					res.value_reg >> 2;

	return (int8_t)res.value_reg;
}


int16_t RdRxPacketRssi(SX1276_Descr *_sx){
	int rssi_offset = _sx->freq_mode == SX1276_HF ? RSSI_OFFSET_HF : RSSI_OFFSET_LF;
	ReadRegResult res = RdValueRegister(_sx, REG_LR_PKTRSSIVALUE, MASK_FULL_VAL_REG);
	return res.is_successfully ? (rssi_offset+ res.value_reg + (res.value_reg >> 4)) : 0;
}

int16_t RdValueRssi(SX1276_Descr *_sx){
	int rssi_offset = _sx->freq_mode == SX1276_HF ? RSSI_OFFSET_HF : RSSI_OFFSET_LF;
	ReadRegResult res = RdValueRegister(_sx, REG_LR_RSSIVALUE, MASK_FULL_VAL_REG);
	
	if(!res.is_successfully)
		return ~0;

	int8_t snr = RdRxPacketSnr(_sx);
	
	return snr < 0 ? 	rssi_offset+ res.value_reg + (res.value_reg >> 4) + snr :
										rssi_offset+ res.value_reg + (res.value_reg >> 4);
}

InfLastRxPacket RdInfLastRxPacket(SX1276_Descr *_sx){
	InfLastRxPacket inf = {.snr = 0,.rssi = 0,.is_successfully = false};
	ReadRegResult res_snr = RdValueRegister(_sx, REG_LR_PKTSNRVALUE, MASK_FULL_VAL_REG);
	ReadRegResult res_rssi = RdValueRegister(_sx, REG_LR_PKTRSSIVALUE, MASK_FULL_VAL_REG);
	
	
	if(res_snr.is_successfully && res_rssi.is_successfully){
//		inf.snr = res_snr.value_reg & 0x80 ? 	(- ((((~res_snr.value_reg) + 1)&0xFF) >> 2)) :
//																					(res_snr.value_reg&0xFF) >> 2;
		inf.snr = ((int8_t)res_snr.value_reg) / 4;

		int rssi_offset = _sx->freq_mode == SX1276_HF ? RSSI_OFFSET_HF : RSSI_OFFSET_LF;
		inf.rssi= inf.snr < 0 ? (rssi_offset+ (int16_t)res_rssi.value_reg + ((int8_t)res_snr.value_reg * 0.25)):
								(rssi_offset+ ((16.0/15.0) * (int16_t)res_rssi.value_reg));

		inf.is_successfully = true;
	}

	return inf;
}
bool RdIsPllFailed(SX1276_Descr *_sx){
	ReadRegResult res = RdValueRegister(_sx, REG_LR_HOPCHANNEL,  RFLR_HOPCHANNEL_PLL_LOCK_TIMEOUT_MASK);
	return res.is_successfully ? (bool)res.value_reg : false;
}

bool RdIsCrcOnHeader(SX1276_Descr *_sx){
	ReadRegResult res = RdValueRegister(_sx, REG_LR_HOPCHANNEL,  RFLR_HOPCHANNEL_CRCONPAYLOAD_MASK );
	return res.is_successfully ? (bool)res.value_reg : false;
}

uint8_t RdFhssPresentChannel(SX1276_Descr *_sx){
	ReadRegResult res = RdValueRegister(_sx, REG_LR_HOPCHANNEL,  RFLR_HOPCHANNEL_CHANNEL_MASK );
	return res.is_successfully ? res.value_reg : 0;
}

uint8_t RdFifoRxLastByteAddrPtr(SX1276_Descr *_sx){
	ReadRegResult res = RdValueRegister(_sx, REG_LR_FIFORXBYTEADDR,  MASK_FULL_VAL_REG);
	return res.is_successfully ? res.value_reg : 0;
}

uint8_t RdPpmCorrection(SX1276_Descr *_sx){
	ReadRegResult res = RdValueRegister(_sx, REG_LR_PPMCORRECTION,  MASK_FULL_VAL_REG);
	return res.is_successfully ? res.value_reg : 0;
}

uint32_t RdFreqError(SX1276_Descr *_sx){
	uint32_t freq_error = 0;
	for(uint8_t idx = 0; idx < sizeof(SX1276_RegLrFreqError); idx++){
		ReadRegResult res = RdValueRegister(_sx, SX1276_RegLrFreqError[idx],  MASK_FULL_VAL_REG);
		if(!res.is_successfully)
			return 0;
		freq_error |= res.value_reg << (24 - (idx * sizeof(uint8_t)));
	}	
	return freq_error;
}

int16_t RdRssiWideband(SX1276_Descr *_sx){
	ReadRegResult res = RdValueRegister(_sx, REG_LR_RSSIWIDEBAND,  MASK_FULL_VAL_REG);
	return res.is_successfully ? - res.value_reg : 0;
}	

uint8_t RdRegFifoAddrPtrData(SX1276_Descr *_sx){
	ReadRegResult res = RdValueRegister(_sx, REG_LR_FIFOADDRPTR, MASK_FULL_VAL_REG);
		return res.is_successfully ? res.value_reg : 0;
}
// Common
volatile uint8_t bufRq[4];
bool SetModeWaitPacket(SX1276_Descr *_sx){

	RdRegister(_sx, REG_LR_DIOMAPPING1, (uint8_t*)&bufRq[0]);
	RdRegister(_sx, REG_LR_DIOMAPPING2, (uint8_t*)&bufRq[1]);
	RdRegister(_sx, REG_LR_OPMODE, (uint8_t*)&bufRq[2]);

	bool res = SetMode(_sx,SX1276_MODE_STANDBY)								&&
						 SetMappingDio(_sx, SX_DIO0_MAP_PAYLOAD_READY)	&&
						 SetRegFifoAddrPtr(_sx,0)												&&
						 SetRegFifoRxBaseAddr(_sx,0)										&&
						 SetMode(_sx,SX1276_MODE_RX_CONT);
	RdRegister(_sx, REG_LR_OPMODE, (uint8_t*)&bufRq[3]);
	for(uint32_t i = 0; i < 100; i++);
	RdRegister(_sx, REG_LR_OPMODE, (uint8_t*)&bufRq[3]);

	return res;
}

bool SetModeTransmitItPacket(SX1276_Descr *_sx){


	bool res = SetMode(_sx,SX1276_MODE_STANDBY)								&&
						 SetMappingDio(_sx, SX_DIO0_MAP_PACKET_SENT)	&&
						 SetMode(_sx,SX1276_MODE_TX);
	return res;
}

bool UpdateDetectionOptimize(SX1276_Descr *_sx){
 ReadRegResult res = RdValueRegister(_sx,REG_LR_MODEMCONFIG2,RFLR_MODEMCONFIG2_SF_MASK);
	if(!res.is_successfully)
		return false;
	
	return res.value_reg == SX_SF_6 ? WrRegister(_sx,REG_LR_DETECTOPTIMIZE,RFLR_DETECTIONOPTIMIZE_SF6)	:
																		WrRegister(_sx,REG_LR_DETECTOPTIMIZE,RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12);
}

bool UpdateDetectionThreshold(SX1276_Descr *_sx){
	ReadRegResult res = RdValueRegister(_sx,REG_LR_MODEMCONFIG2,RFLR_MODEMCONFIG2_SF_MASK);
	if(!res.is_successfully)
		return false;

	return res.value_reg == SX_SF_6 ? WrRegister(_sx,REG_LR_DETECTIONTHRESHOLD,RFLR_DETECTIONTHRESH_SF6)	:
																		WrRegister(_sx,REG_LR_DETECTIONTHRESHOLD,RFLR_DETECTIONTHRESH_SF7_TO_SF12);
}

bool CalibrationRxChain(SX1276_Descr *_sx){
	uint8_t req_opmode;
	bool res = false;
	__CH(RdRegister(_sx,REG_LR_OPMODE,(uint8_t*)&req_opmode));
	if(	SetModem(_sx,SX1276_MODEM_FSK)				&&
			SetMode(_sx,SX1276_MODE_STANDBY)			&&
			SetFreq(_sx, 868300000)){

		if(ReplaceRegister(_sx,REG_IMAGECAL,RF_IMAGECAL_IMAGECAL_START,RF_IMAGECAL_IMAGECAL_MASK)){
			uint32_t time_start = _sx->definit.get_time();
			while(!__IS_TIMEOUT_OUT(_sx,time_start)){
				uint8_t value = 0;
				if(RdRegister(_sx,REG_IMAGECAL,(uint8_t*)&value)){
					if((value & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_DONE){
						res = true;
						break;
					}
				}
			}
		}

	}
	return res && SetFreq(_sx,_sx->freq_current)	&& WrRegister(_sx,REG_LR_OPMODE,req_opmode);
}

//static bool ClearFIFO (SX1276_Descr *_sx){
//	uint8_t rxBuf = 0;
//	bool res;
//	uint32_t stime = _sx->definit.get_time();
//
//	res = 	SetMode(_sx,SX1276_MODE_STANDBY) &&
//			WrRegister(_sx, REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN);
//	if(res)
//	{
//		do{
//			if(!RdRegister(_sx, REG_IRQFLAGS2, (uint8_t*)&rxBuf))
//				rxBuf = RF_IRQFLAGS2_FIFOOVERRUN;
//			if(_sx->definit.get_time() - stime > 20){
//				res = false;
//				break;
//			}
//		}
//		while(rxBuf&RF_IRQFLAGS2_FIFOOVERRUN);
//	}
//
//	return res;
//}


// Api
bool SX1276_Init(SX1276_Descr *_sx,sx1276_set_rst _rst, sx1276_transmit_spi _tx_spi,
									sx1276_receive_spi	_rx_spi, sx1276_set_nss _nss,get_time_ms _get_time,
									sx1276_transmit_receive_spi _tx_rx_spi,SX1276_TypeModem _modem,
									SX1276_FrequencyMode _mode,void *_context, sx1276_atomic_block _atomicb,
                                                                         bool _toDoRest){

	_sx->definit.nss 				= _nss;
	_sx->definit.rst				=	_rst;
	_sx->definit.rx					=	_rx_spi;
	_sx->definit.tx					=	_tx_spi;
	_sx->definit.get_time 	= _get_time;
	_sx->definit.atomicb = _atomicb;
	_sx->modem 							= _modem;
	_sx->freq_mode 					= _mode;
	_sx->header_mode				=	SX1276_HEADER_EXPLICIT; // def after reset
	__CH(((!_toDoRest || Reset(_sx)) && IsConnectModul(_sx) /*&& CalibrationRxChain(_sx)*/));
	SetRegFifoRxBaseAddr(_sx,0);
	SetRegFifoTxBaseAddr(_sx,0);
	UNLOCK_SX(_sx,true); 
}

void SX1276_SetClbk(SX1276_Descr *_sx, const struct SX1276_Clbk_ *_clbk){
	memcpy(&_sx->clbk,_clbk, sizeof(struct SX1276_Clbk_));
}

bool SX1276_SetMode(SX1276_Descr *_sx,SX1276_Mode _mode){
	LOCK_SX(_sx,false);

	bool res = SetMode(_sx,_mode);

	UNLOCK_SX(_sx,res);
}

bool SX1276_SetModem(SX1276_Descr *_sx, SX1276_TypeModem _modem){
	LOCK_SX(_sx,false);
	
	bool res = SetModem(_sx, _modem);

	UNLOCK_SX(_sx,res);
}

bool SX1276_SetFreq(SX1276_Descr *_sx, uint32_t _freq){
	LOCK_SX(_sx,false);

	bool res = SetFreq(_sx,_freq);

	UNLOCK_SX(_sx,res);
}
//23-5
bool SX1276_SetOutputPower(SX1276_Descr *_sx, uint8_t _precent){
	LOCK_SX(_sx,false);
	float db = ((float)(18.0 *((float)_precent) / 100.0) + 5.0);
	//int8_t db = (int8_t)((int8_t)((int8_t)((double)25 / (double)100) * _precent) - 5);
	bool res = SetOutputPowerdBm(_sx,(int8_t)db);

	UNLOCK_SX(_sx,res);
}

bool SX1276_MaxCurrent(SX1276_Descr *_sx, bool _protection_on, uint8_t _max_current){
	LOCK_SX(_sx,false);

	bool res = SetRegOcp(_sx,_protection_on,_max_current);

	UNLOCK_SX(_sx,res);
}

bool SX1276_SetLnaGain(SX1276_Descr *_sx, SX1276_LoraLnaGain _gain, bool _on_boost_hf){
	LOCK_SX(_sx,false);

	bool res = SetRegLna(_sx, _gain,_on_boost_hf);

	UNLOCK_SX(_sx,res);
}

bool SX1276_SetBandwidth(SX1276_Descr *_sx, SX1276_BandwidthLr _bw){
	LOCK_SX(_sx,false);

	bool res = SetBandwidth(_sx,  _bw);

	UNLOCK_SX(_sx,res);
}

bool SX1276_SetCodingRate(SX1276_Descr *_sx, SX1276_CodingRate _cr){
	LOCK_SX(_sx,false);

	bool res = SetCodingRate(_sx, _cr);

	UNLOCK_SX(_sx,res);
}

bool SX1276_SetHeaderMode(SX1276_Descr *_sx, SX1276_HeaderMode _mode){
	LOCK_SX(_sx,false);
	
	bool res =  SetHeaderMode(_sx,_mode);

	UNLOCK_SX(_sx,res);
}

bool SX1276_SetSpreadingFactor(SX1276_Descr *_sx, SX1276_SpreadingFactor_E _sf){
	LOCK_SX(_sx,false);

	bool res = SetSpreadingFactor(_sx,_sf);
	
	UNLOCK_SX(_sx,res);
}

bool SX1276_SetRxPayloadCrc(SX1276_Descr *_sx, bool _crc_on){
	LOCK_SX(_sx,false);

	bool res = SetRxPayloadCrc(_sx,_crc_on);

	UNLOCK_SX(_sx,res);
}

bool SX1276_SetPreambleLength(SX1276_Descr *_sx, uint16_t _lenght){
	LOCK_SX(_sx,false);
	
	bool res = SetPreambleLength(_sx,_lenght);

	UNLOCK_SX(_sx,res);
}

bool SX1276_SetPayloadMaxLength(SX1276_Descr *_sx, uint8_t _lenght){
	LOCK_SX(_sx,false);

	bool res = SetPayloadMaxLength(_sx,_lenght);

	UNLOCK_SX(_sx,res);
}

bool SX1276_SetInvertIQ(SX1276_Descr *_sx, bool _invert_mode){
	LOCK_SX(_sx,false);

	bool res = SetInvertIQ(_sx,_invert_mode);

	UNLOCK_SX(_sx,res);
}

bool SX1276_SetSyncWord(SX1276_Descr *_sx, uint8_t _value){
	LOCK_SX(_sx,false);

	bool res =	SetSyncWord(_sx,_value);

	UNLOCK_SX(_sx,res);
}

bool SX1276_SetModeWaitPacket(SX1276_Descr *_sx){
	LOCK_SX(_sx,false);

	bool res = SetModeWaitPacket(_sx);
	
	UNLOCK_SX(_sx,res);
}
bool SX1276_SetPayloadLength(SX1276_Descr *_sx, uint8_t _lenght){
	return SetPayloadLength(_sx, _lenght);
}
bool SX1276_BeginCalibrationRxChain(SX1276_Descr *_sx){
	LOCK_SX(_sx,false);
	
	bool res  = CalibrationRxChain(_sx);
	
	UNLOCK_SX(_sx,res);
}
double SX1276_GetDistance(int rssi, int txPower) {
    /*
     * RSSI = TxPower - 10 * n * lg(d)
     * n = 2 (in free space)
     * 
     * d = 10 ^ ((TxPower - RSSI) / (10 * n))
     */
 
    return pow(10, ((double) txPower - rssi) / (10 * 2));
//	return 0;
}

bool IsChannelFree(SX1276_Descr *_sx)
{
	bool isChFree = true;
	_sx->state = SX1276_STATE_CAD;

	if(	SetMode(_sx,SX1276_MODE_STANDBY) &&
		SetMappingDio(_sx, SX_DIO0_MAP_CAD_DONE) &&
		SetMode(_sx,SX1276_MODE_CAD) )
	{
		while(_sx->mode == SX1276_MODE_CAD);
		isChFree = !IsIrqFlag(_sx, SX_IRQ_FLAG_RX_CAD_DETECTED);
	}

	ClearReqIrqFlag(_sx,(ValueIrqFlags)MASK_FULL_VAL_REG);
	SetMode(_sx,SX1276_MODE_STANDBY);
	_sx->state = SX1276_STATE_BUSY;
	return isChFree;
}

bool SX1276_IsChannelFree(SX1276_Descr *_sx)
{
	LOCK_SX(_sx,false);
	bool res;

	SX1276_Mode inMode =  SX1276_GetMode(_sx);
	res = IsChannelFree(_sx);
	SetMode(_sx, inMode);

	UNLOCK_SX(_sx,res);
}

bool SX1276_IsReceivingData(SX1276_Descr *_sx)
{
	if(_sx->definit.atomicb == NULL)
		return false;

	_sx->definit.atomicb(SX1276_SELECT);

	bool res;
	if(_sx->state == SX1276_STATE_READY)
	{
		_sx->state = SX1276_STATE_BUSY;
		if(_sx->timeDetectionHeader == 0 ||
		 _sx->definit.get_time() - _sx->timeDetectionHeader > 20)
		{
			if( (_sx->timeStartRxHeader == 0 						||
			_sx->definit.get_time() - _sx->timeStartRxHeader < 300)	&&
			IsIrqFlag(_sx, SX_IRQ_FLAG_RX_VALID_HEADER) )
			{
				if(_sx->timeStartRxHeader == 0)
					_sx->timeStartRxHeader =  _sx->definit.get_time();

				_sx->timeDetectionHeader = _sx->definit.get_time();
			}
			else
			{
				_sx->timeDetectionHeader = 0;
				_sx->timeStartRxHeader = 0;
			}
		}


		_sx->state = SX1276_STATE_READY;
	}
	res = _sx->timeDetectionHeader != 0;
	_sx->definit.atomicb(SX1276_DESELECT);

	return res;
}

bool SX1276_IsSignalDetected(SX1276_Descr *_sx)
{
	_sx->definit.atomicb(SX1276_SELECT);
	bool signal = false;

	if(_sx->state == SX1276_STATE_READY)
	{
		_sx->state = SX1276_STATE_BUSY;

		ReadRegResult reg_irq = RdValueRegister(_sx,REG_LR_MODEMSTAT,MASK_FULL_VAL_REG);
		if(reg_irq.is_successfully)
		{
			signal = (bool)(reg_irq.value_reg & 0x01);
		}

		_sx->state = SX1276_STATE_READY;
	}
	_sx->definit.atomicb(SX1276_DESELECT);
	return signal;
}
bool SX1276_RdRegIrq(SX1276_Descr *_sx, uint8_t *_value)
{
	LOCK_SX(_sx,false);
	bool res = false;

	ReadRegResult reg_irq = RdValueRegister(_sx,REG_LR_IRQFLAGS,MASK_FULL_VAL_REG);
	if(reg_irq.is_successfully)
	{
		res = true;
		*_value = reg_irq.value_reg;
	}

	UNLOCK_SX(_sx,res);
}
#ifdef SX1276_DEBUG
	uint32_t deb_time_start_send = 0;
#endif
bool SX1276_Send(SX1276_Descr *_sx,uint8_t *_buf, size_t _size, uint32_t _timeout){
	LOCK_SX(_sx,false);

	uint32_t time_start = _sx->definit.get_time();
	bool res = false;
	if(_size < SIZE_FIFO - SIZE_SYNC){

#ifdef SX1276_DEBUG
	deb_time_start_send = _sx->definit.get_time();
#endif
	//ClearFIFO(_sx);
	//SetMode(_sx,SX1276_MODE_TX);
		res = 	(SetMode(_sx,SX1276_MODE_STANDBY) 													&&
						SetRegFifoTxBaseAddr(_sx,0x00)												&&
						SetRegFifoAddrPtr(_sx,0x00)													&&
						WriteFifo(_sx,_buf,_size)													&&
						SetPayloadLength(_sx, _size)												&&
						SetMode(_sx,SX1276_MODE_TX));
		

	

		

		while(res && !IsIrqFlag(_sx,SX_IRQ_FLAG_RX_TX_DONE)){
			res = !((_sx->definit.get_time() - time_start) > _timeout);
		}
		
	#ifdef SX1276_DEBUG
		debug->timeout_send = _sx->definit.get_time() - deb_time_start_send;
	#endif
		res	=	res	&&	ClearReqIrqFlag(_sx,SX_IRQ_FLAG_RX_TX_DONE) &&
						SetMode(_sx,SX1276_MODE_STANDBY);
	}
	UNLOCK_SX(_sx,res);
}


bool SX1276_SendIT(SX1276_Descr *_sx,uint8_t *_buf, size_t _size){
	bool res = false;
	if(_size < SIZE_FIFO - SIZE_SYNC){
		LOCK_SX(_sx,false);

		if(SetMode(_sx,SX1276_MODE_STANDBY))
		{
			uint32_t sTime = _sx->definit.get_time();
			SX1276_Mode mode = SX1276_MODE_SLEEP;

			do
			{
				if(_sx->definit.get_time() - sTime > 5)
					break;
				mode = RdMode(_sx);
			}while(mode != SX1276_MODE_STANDBY);

			if(mode == SX1276_MODE_STANDBY)
			{
				res = 		(	SetRegFifoTxBaseAddr(_sx,0x00)												&&
								SetRegFifoAddrPtr(_sx,0x00)													&&
								WriteFifo(_sx,_buf,_size)													&&
								SetPayloadLength(_sx, _size) );

				if(res)
				{
					if(_sx->definit.atomicb != NULL)
						_sx->definit.atomicb(SX1276_SELECT);
					res = SetModeTransmitItPacket(_sx);

					UNLOCK(_sx);

					if(_sx->definit.atomicb != NULL)
						_sx->definit.atomicb(SX1276_DESELECT);

				}
			}
		}
		if(!res)
			UNLOCK(_sx);
	}
	return res;
}


#ifdef SX1276_DEBUG
	uint32_t deb_time_start_recieved;
#endif
static void ProcessRecivedData(SX1276_Descr *_sx)
{
#ifdef SX1276_DEBUG
		deb_time_start_recieved = _sx->definit.get_time();
#endif

	if(SetMode(_sx,SX1276_MODE_STANDBY))
	{
		ReadRegResult reg_irq = RdValueRegister(_sx,REG_LR_IRQFLAGS,MASK_FULL_VAL_REG);
		if(reg_irq.is_successfully 																&&
		  (reg_irq.value_reg & RFLR_IRQFLAGS_RXDONE)							&&
		  (reg_irq.value_reg & RFLR_IRQFLAGS_VALIDHEADER)					&&
		  !(reg_irq.value_reg & RFLR_IRQFLAGS_PAYLOADCRCERROR)){
			uint8_t payload = RdRxNumbBytes(_sx);
			uint8_t addr_data = RdRegFifoAddrPtrData(_sx);
			uint8_t buf[payload];

			if(payload != 0 && buf != NULL && ReadFifo(_sx,addr_data,buf,payload)){
				InfLastRxPacket inf =  RdInfLastRxPacket(_sx);
				if(inf.is_successfully){
					_sx->clbk.received(_sx->context,buf,payload,inf.rssi,inf.snr);
				}
			}
		}
	}

#ifdef SX1276_DEBUG
		debug->timeout_recieved = _sx->definit.get_time() - deb_time_start_recieved;
#endif
}


void SX1276_InterruptDio0(SX1276_Descr *_sx){
	if(_sx->state == SX1276_STATE_READY)
	{
		_sx->state = SX1276_STATE_BUSY;

		switch(_sx->mode)
		{
			case SX1276_MODE_TX: // must be put into receive mode and clear the interrupt flags. It will be after the switch.
			break;				// no processing required
			case SX1276_MODE_RX_CONT:
			case SX1276_MODE_RX_SINGLE:
				ProcessRecivedData(_sx);
			break;
			default: //error
				break;
		}


		ClearReqIrqFlag(_sx,(ValueIrqFlags)(SX_IRQ_FLAG_RX_DONE | SX_IRQ_FLAG_RX_PAYLOAD_CRC_ERROR | SX_IRQ_FLAG_RX_VALID_HEADER));
		SetModeWaitPacket(_sx);

		_sx->state = SX1276_STATE_READY;
	} else if(_sx->state == SX1276_STATE_CAD) {
		SetMode(_sx,SX1276_MODE_STANDBY);
		_sx->mode = SX1276_MODE_STANDBY;
	}
}


SX1276_State SX1276_GetState(SX1276_Descr *_sx)
{
	return _sx->state;
}
SX1276_Mode  SX1276_GetMode(SX1276_Descr *_sx)
{
	return _sx->mode;
}

int16_t SX1276_ReadRssi(SX1276_Descr *_sx)
{
	LOCK_SX(_sx,0xFFFF);
	int16_t res = RdValueRssi(_sx);
	UNLOCK(_sx);
	return res;
}

bool SX1276_ReadDetectionThreshold(SX1276_Descr *_sx, uint8_t *_value)
{
	LOCK_SX(_sx,false);
	bool res;

	ReadRegResult reg_irq = RdValueRegister(_sx,REG_LR_DETECTIONTHRESHOLD,MASK_FULL_VAL_REG);
	if(reg_irq.is_successfully)
	{
		res = true;
		*_value = reg_irq.value_reg;
	}
	UNLOCK_SX(_sx,res);
}

bool SX1276_ReadModemStatus(SX1276_Descr *_sx, uint8_t *_value)
{
	LOCK_SX(_sx,false);
	bool res;

	ReadRegResult reg_irq = RdValueRegister(_sx,REG_LR_MODEMSTAT,MASK_FULL_VAL_REG);
	if(reg_irq.is_successfully)
	{
		res = true;
		*_value = reg_irq.value_reg;
	}
	UNLOCK_SX(_sx,res);
}

bool SX1276_ReadRssiWideband(SX1276_Descr *_sx, uint8_t *_value)
{
	LOCK_SX(_sx,false);
	bool res;

	ReadRegResult reg_irq = RdValueRegister(_sx,REG_LR_RSSIWIDEBAND,MASK_FULL_VAL_REG);
	if(reg_irq.is_successfully)
	{
		res = true;
		*_value = reg_irq.value_reg;
	}
	UNLOCK_SX(_sx,res);
}

#if SX1276_IS_ENABLE_MM
void SX1276_ConnectModeMonitor(SX1276_Descr *_sx, SX1276_ModeMonitor *_monitor)
{
	if(_sx != NULL)
		_sx->mm = _monitor;
}
#endif
uint32_t SX1276_GetTimeInMode(SX1276_Descr *_sx, SX1276_Mode _mode)
{
#if SX1276_IS_ENABLE_MM
	return _sx->mm == NULL ? 0 : _sx->mm->timeInMode[_mode];
#else
	return 0;
#endif
}
float  SX1276_GetPrecentModeActivity(SX1276_Descr *_sx, SX1276_Mode _mode)
{
#if SX1276_IS_ENABLE_MM
	uint64_t value  = 0;
	float ret = 0;
	if(_sx != NULL && _sx->mm != NULL)
	{
		for(uint32_t idx = 0; idx < SX1276_NUMBER_MODE; idx++)
		{
			value += _sx->mm->timeInMode[idx];
		}
	}
	ret = (float)((100 * (float)_sx->mm->timeInMode[_mode]) / value);
	return ret;
#else
	return 0;
#endif
}
void SX1276_ResetModeMonitor(SX1276_Descr *_sx)
{
#if SX1276_IS_ENABLE_MM
	if(_sx != NULL && _sx->mm != NULL)
	{
		memset(_sx->mm, 0, sizeof(SX1276_ModeMonitor));
	}
#endif
}
void ControlModeMonitor(SX1276_Descr *_sx, SX1276_Mode _old, SX1276_Mode _new)
{
#if SX1276_IS_ENABLE_MM
	(void)_new;
	if(_sx->mm != NULL)
	{
		uint32_t timeStamp = _sx->definit.get_time();
		_sx->mm->timeInMode[_old] += (timeStamp - _sx->mm->timeSetMode);
		_sx->mm->timeSetMode = timeStamp;
	}
#else
#endif
}

