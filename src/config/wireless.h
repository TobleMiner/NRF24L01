#ifndef __WIRELESS_CFG__
#define __WIRELESS_CFG__
	//Main config
	#define WIRELESS_CHANNEL	42		//Initial com channel, range 0 ... 125
	#define WIRELESS_BAUDRATE	1000	//in kbps. Enter either 1000 or 2000. everything else will lead to a compiler error as it is not supported by the NRF24L01 series wireless modules
	#define WIRELESS_PACK_LEN	32		//Packet length in bytes used for data transmissions. Value must be inside [0 ... 32]. If set to 0 dynamic payload length is enabled
	#define WIRELESS_EN_TX_IRQ	TRUE
	#define WIRELESS_EN_RX_IRQ	TRUE
	#define WIRELESS_EN_RT_IRQ	TRUE
	
	//TX config
	#define WIRELESS_TX_ENABLED TRUE
	#define WIRELESS_TX_PWR		0		//TX power in dBm. Must be either 0, -6, -12 or -18
	
	//RX config
	#define WIRELESS_RX_ENABLED TRUE
#endif