//A few standard checks
#ifndef F_CPU
	#error "To use this wireless lib F_CPU must be defined"
#endif

//Wireless stuff
#if WIRELESS_CHANNEL > 125 || WIRELESS_CHANNEL < 0
	#error "Invalid wireless channel. Your wireless channel must be inside [0 ... 125]"
#endif

#if WIRELESS_BAUDRATE != 1000 && WIRELESS_BAUDRATE != 2000
	#error "Invalid wireless baudrate. Must be either 1000 [kbps] or 2000 [kbps]"
#endif

#if WIRELESS_PACK_LEN < 0 || WIRELESS_PACK_LEN > 32
	#error "Invalid wireless paket length. Must be inside [0 ... 32]"
#endif

#if WIRELESS_TX_PWR < -18 || WIRELESS_TX_PWR > 0 || WIRELESS_TX_PWR % 6 != 0
	#error "Invalid wireless TX power. Must be either -18 [dBm], -12 [dBm], -6 [dBm] or 0 [dBm]"
#endif

#if WIRELESS_TX_ENABLED == TRUE && WIRELESS_RX_ENABLED == TRUE
	#pragma message "Keep in mind that transmitter and receiver can not both really be used at the same time but the receiver is disabled while transmitting."
#endif