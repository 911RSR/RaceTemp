// Parts of this code are based on
// -- ublox documentation: Chapter "32 UBX Protocol" of document UBX-13003221 (M8) or UBX-21022436 (M9)
// -- DMA FIFO: ST AN3109

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "cmsis_os.h"
#include "UBX.h"
#include "usart.h"
#include "FreeRTOS.h"


//#include "stm32f1xx_hal.h"
//#include "stm32f1xx_hal_usart.h"
//#include "stm32f1xx_hal_stream.h"

#define UBX_SYNC1     0xB5
#define UBX_SYNC2     0x62

#define UBX_NAV_CLASS     0x01
#define UBX_NAV_POSLLH    0x02
#define UBX_NAV_STATUS    0x03
#define UBX_NAV_DOP       0x04
#define UBX_NAV_SOL       0x06
#define UBX_NAV_PVT       0x07
#define UBX_NAV_PVT_U4	0xB5620107
#define UBX_NAV_VELNED    0x12
#define UBX_NAV_TIMEUTC   0x21
#define UBX_NAV_SVINFO    0x30
#define UBX_NAV_SBAS      0x32
#define UBX_NAV_SAT       0x35
#define UBX_NAV_SLAS      0x35

#define UBX_RXM_CLASS     0x02
#define UBX_RXM_RAW       0x10
#define UBX_RXM_SFRB      0x11

#define UBX_ACK_CLASS     0x05
#define UBX_ACK_ACK	   	  0x01
#define UBX_ACK_NAK	   	  0x00

#define UBX_CFG_CLASS     0x06
#define UBX_CFG_PRT       0x00
#define UBX_CFG_MSG       0x01
#define UBX_CFG_TP        0x07
#define UBX_CFG_RATE      0x08
#define UBX_CFG_CFG       0x09
#define UBX_CFG_SBAS      0x16
#define UBX_CFG_NAV5      0x24
#define UBX_CFG_GNSS      0x3E

#define UBX_SBAS_AUTO     0x00000000
#define UBX_SBAS_WAAS     0x0004E004
#define UBX_SBAS_EGNOS    0x00000851
#define UBX_SBAS_MSAS     0x00020200
#define UBX_SBAS_GAGAN    0x00000108

#define UBX_DYN_PORTABLE   0
#define UBX_DYN_STATIONARY 2
#define UBX_DYN_PED        3
#define UBX_DYN_AUTOMOTIVE 4
#define UBX_DYN_SEA        5
#define UBX_DYN_AIR1G      6
#define UBX_DYN_AIR2G      7
#define UBX_DYN_AIR4G      8

#define UBX_MON_CLASS     0x0a
#define UBX_MON_VER       0x04
#define UBX_MON_HW        0x09

#define UBX_AID_CLASS     0x0b
#define UBX_AID_REQ       0x00

#define UBX_TIM_CLASS     0x0d
#define UBX_TIM_TP        0x01

#define UBX_GNSSID_GPS     0
#define UBX_GNSSID_SBAS    1
#define UBX_GNSSID_BEIDOU  3
#define UBX_GNSSID_QZSS    5
#define UBX_GNSSID_GLONASS 6


// a few defines for UBX protocol with UBX-CFG-VALSET, UBX-CFGVALGET, UBX-CFG-VALDEL
// VALSET/GET is not fully implemented here -- still using some of the legacy CFG
#define UBX_CFG_VALSET	  	0x8A
#define UBX_CFG_VALGET	  	0x8B
#define CFG_MSGOUT_UBX_NAV_PVT_UART1 0x20910007
#define CFG_MSGOUT_UBX_NAV_SAT_UART1 0x20910016
#define CFG_RATE_MEAS			0x30210001
#define CFG_RATE_NAV			0x30210002
#define CFG_RATE_TIMEREF		0x30210003
#define CFG_UART1_BAUDRATE		0x40520001
#define CFG_UART1_STOPBITS		0x20520002
#define CFG_UART1_DATABITS		0x20520003
#define CFG_SBAS_USE_TESTMODE	0x10360002
#define CFG_SBAS_USE_RANGING	0x10360003
#define CFG_SBAS_USE_DIFFCORR	0x10360004
#define CFG_SBAS_USE_INTEGRITY	0x10360005
#define CFG_SBAS_PRNSCANMASK	0x50360006
#define CFG_NAVSPG_DYNMODEL		0x20110021

struct ubx_cfg_valset_t
{
	U1 version;			// Message version (0x00 for simplified or 0x01 for advanced)
	X1 layers;			// bit0=RAM, bit1=BBR, bit2 = Flash
	U1 reserved[2];     //
	// Configuration data -- up to 64 key and value pairs, but check also UBX_MAX_PAYLOAD!
} __attribute__((packed, aligned(4)));


struct ubx_cfg_prt_t // deprecated -- use VALSET in stead
{
	U1 portID;
	U1 reserved1;
	U2 txReady;           // Bit field txReady (threshold, pin, polarity, enabler TX ready feature)
	U4 mode;              // little endian
	U4 baudRate;          // Possible UART baud rates: 4800, 9600 (default?), 19200, 38400 (default?), 57600, 115200, 230400, 460800
	U2 inProtoMask;       // Bit field, 7 = Rtcm + Nmea + Ubx
	U2 outProtoMask;      // Bit field  1 = Ubx only
	U2 flags;
	U1 reserved[2];
} __attribute__((packed, aligned(4))) ;


/*struct ubx_cfg_nav5_t  // deprecated -- use VALSET in stead
{
	X2 mask;        // Parameters bit mask. Only the masked parameters will be applied.
	U1 dynModel;    // 0..12, e.g. 4=Automotive
	U1 fixMode;     // Position fixing mode: 1= 2D only, 2= 3D only, 3= auto 2D/3D
	I4 fixedAlt;    // [0.01m] Fixed altitude (mean sea level) for 2D fix mode
	U4 fixedAltVar; // [0.0001 m^2] Fixed altitude variance for 2D mode
	I1 minElev;     // [deg] Minimum elevation for a GNSS satellite to 	be used in NAV
	U1 drLimit;		// [s] Reserved
	U2 pDop;
	U2 tDop;
	U2 pAcc;
	U2 tAcc;
	U1 staticHoldThresh;
	U1 dgnssTimeout;
	U1 cnoThreshNumSVs;
	U1 cnoThresh;
	U1 reserved1[2];
	U2 staticHoldMaxDist;
	U1 utcStandard;
	U1 reserved2[5];
} __attribute__((packed, aligned(4)));*/

struct ubx_cfg_sbas_t  // deprecated -- use VALSET in stead
{
	X1 mode;
	X1 usage;
	U1 maxSBAS;
	X1 scanmode2;
	X4 scanmode1;
} __attribute__((packed, aligned(4)));


/*struct ubx_cfg_rate_t // deprecated -- use VALSET in stead
{
	U2 measRate;
	U2 navRate;
	U2 timeRef;
} __attribute__((packed, aligned(4)));*/

/*struct ubx_cfg_msg_t
// This message is deprecated in protocol versions greater than 23.01.
// Use UBX-CFG-VALSET, UBX-CFGVALGET, UBX-CFG-VALDEL instead.
{
	U1 msgClass;       // Message Class
	U1 msgID;          // Message Identifier
	U1 rate;
} __attribute__((packed, aligned(4)));*/


//
// Send UBX message to GNS-module. This is used to change settings or poll messages
// It must contain length info as defined in "ubx->msg->length"
// Checksum is calculated and added
//
void ubx_send_msg( struct ubx_msg_t *msg )
{
    U1 *c, ck_a = 0x00U, ck_b = 0x00U;
    c = & msg->class;
    for (int i=0; i < msg->length+4; i++)  // +4 for class, id and length
    {
        ck_a += *c++;
        ck_b += ck_a;
    }
    *c++=ck_a;
    *c=ck_b;
    c = &msg->synch1;
    while (c < &msg->payload[0] + msg->length+2 )
    {
    	//while ( ! LL_USART_IsActiveFlag_TXE_TXFNF(GNS_dev) ) ;  // wait till GNS_dev is ready
    	while ( ! LL_USART_IsActiveFlag_TXE( GNS_dev ) ) ;  // wait till GNS_dev is ready
    	LL_USART_TransmitData8( GNS_dev, *c );
    	c++;
    }
    vTaskDelay(1/portTICK_PERIOD_MS);   // is this needed ?  it looks better in the logic analyzer -- allows time for ACK (reply)
    //HAL_Delay(1);
    // ToDo: Receive/check ACK-ACK or ACK-NAK  (remember a timout)
}

//
// Send UBX message to GNS-module. This is used to change settings or poll messages
// It must contain length info as defined in "ubx->msg->length"
// Checksum is calculated and added
//
U1 ubx_send( struct ubx_handle_t *ubx, uint32_t timeout )
{
	//struct ubx_msg_t *msg = &ubx->msg;
	ubx_send_msg( &ubx->msg );
    //vTaskDelay(1/portTICK_PERIOD_MS);   // is this needed ?  it looks better in the logic analyzer -- allows time for ACK (reply)
    //HAL_Delay(1);
    // ToDo: Receive/check ACK-ACK or ACK-NAK
	timeout += HAL_GetTick();
    while ( HAL_GetTick() < timeout && !( (ubx->msg.class == UBX_ACK_CLASS) && (ubx->msg.state == 0) ) ) // wait for UBX_ACK or timeout
    {
    	ubx_receive2( ubx );
//    	vTaskDelay(1/portTICK_PERIOD_MS);
    }
    ubx->msg.class=0;
    return ubx->msg.id; // returned value should be id of UBX_ACK_ACK (unless timeout or a NAK occurred)
}


/**
 * \brief Parse UBX message from GNS-module.
 * (Copy new data into UBX->msg struct)
 */
void ubx_parse( struct ubx_handle_t *ubx, const U1 * buf, size_t len )
{
    //const U1* d = buf;
	struct ubx_msg_t *msg = &(ubx->msg);  // short hand alias
	while ( len > 0 )
	{
		// for debug:
//		while ( ! LL_USART_IsActiveFlag_TXE( GNS_dev ) ) ;  // wait till GNS_dev is ready
//		LL_USART_TransmitData8( GNS_dev, *d );
//		while ( ! LL_USART_IsActiveFlag_TXE( GNS_dev ) ) ;  // wait till GNS_dev is ready
//		LL_USART_TransmitData8( GNS_dev, (U1) msg->state );
//		while ( ! LL_USART_IsActiveFlag_TXE( GNS_dev ) ) ;  // wait till GNS_dev is ready
//		LL_USART_TransmitData8( GNS_dev, (U1) len );
		// :debug


	    switch (msg->state)
		{
		case 0:  // looking for UBX_SYNC1.  Note the GNSS might also send
			msg->c = &( msg->synch1 );  // point to first position
			if ( *buf == UBX_SYNC1 )
			{
				msg->state = 1; // UBX header found
			}
			break;
		case 1:   // looking for UBX_SYNC2
			if ( *buf == UBX_SYNC2 )
			{
				msg->state = 2; // UBX header found
				msg->ck_a = 0;  // Initialize the checksum
				msg->ck_b = 0;
				msg->length = UBX_MAX_PAYLOAD;
			}
			else
			{
				msg->state=0;
			}
			break;
		case 2: // ---------------- message body (class, ID, length and payload)
			msg->ck_a += *buf; // update the checksum
			msg->ck_b += msg->ck_a;
			if ( msg->c >= &msg->payload[0] + msg->length - 1 )
			{
				msg->state = 3;  // finished reading message body
			}
			break;
		case 3: // ---------------- checksum ck_a
			msg->state = (*buf == msg->ck_a)? 4:0;   // ToDo: Debug this!
			//msg->state = 4;
			break;
		case 4: // ---------------- checksum ck_b
			msg->state=0;
			if ( (*buf == msg->ck_b))
			{
				//debug: echo the ck bytes for debug via serial console or logic analyzer
//				while ( ! LL_USART_IsActiveFlag_TXE( GNS_dev ) );
//				LL_USART_TransmitData8( GNS_dev, msg->ck_a );
//				while ( ! LL_USART_IsActiveFlag_TXE( GNS_dev ) );
//				LL_USART_TransmitData8( GNS_dev, msg->ck_b );
				//:debug

				switch (msg->class)
				{
					case UBX_NAV_CLASS:
						switch (msg->id)
						{
							case UBX_NAV_PVT: // save this message for later...
								ubx->nav_pvt_flag=0;
								memcpy((void*) &ubx->nav_pvt, (void*) &msg->payload, sizeof(struct ubx_nav_pvt_t) );
								ubx->nav_pvt_flag=1;
								break;
							case UBX_NAV_SAT: // save this message for later...
								ubx->nav_sat_flag=0;
								memcpy((void*) &ubx->nav_sat, (void*) &msg->payload, sizeof(struct ubx_nav_sat_t) );
								ubx->nav_sat_flag=1;
								break;
						}
					break;
				}
			}
			break;
		}
		*msg->c = *buf; // save the received character
		msg->c++;       // point to next location in msg
		buf++;		    // point to next location in buf
		len--;
	}
}


/**
 * \brief           Check for new data received via DMA
 */
void ubx_receive(struct ubx_handle_t * ubx)
{
    struct ubx_msg_t *msg = &(ubx->msg);
    U1 *buf = ubx->rx_buf;
    size_t pos = UBX_RX_BUF_SIZE - LL_DMA_GetDataLength( ubx->rx_dma_master, LL_DMA_STREAM_7 );
    size_t old_pos = ubx->rx_pos;
	ubx->rx_pos = pos;   // Save current position for next call
	if (pos == old_pos){   // No new data.  Is the line idle?
	} else if (pos > old_pos) {    		// New data is contiguous in buffer
		memcpy( (void*) &msg->synch1, &buf[old_pos], pos - old_pos );
	} else if ( pos < old_pos ) {  	// New data is split at tail and head of buffer
		memcpy( (void*) &msg->synch1, &buf[old_pos], UBX_RX_BUF_SIZE - old_pos );
		memcpy( (void*) &msg->synch1 + UBX_RX_BUF_SIZE - old_pos, &buf[0], pos);
	}
	if ( *((U4*) &msg->synch1) == UBX_NAV_PVT_U4 ){  // ToDo: Check if message is complete first!
		ubx->nav_pvt_flag = 1;
		memcpy((void*) &ubx->nav_pvt, (void*) &msg->payload, sizeof(struct ubx_nav_pvt_t) );
	}
}


/**
 * \brief           Check for new data received via DMA
 *
 * Not doing reads fast enough will cause DMA to overflow and loose data.
 */
void ubx_receive2(struct ubx_handle_t * ubx)
{
    U1 *buf = ubx->rx_buf;
    size_t pos = UBX_RX_BUF_SIZE - LL_DMA_GetDataLength( ubx->rx_dma_master, LL_DMA_STREAM_7 );
    size_t old_pos = ubx->rx_pos;  // The saved position from last call is now old
	ubx->rx_pos = pos;   // Save current position as old for next call
	if ( pos > old_pos ) {    		// New data is contiguous in buffer
		ubx_parse( ubx, &buf[old_pos], pos - old_pos );
	} else if ( pos < old_pos ) {  	// New data is split at tail and head of buffer
		ubx_parse( ubx, &buf[old_pos], UBX_RX_BUF_SIZE - old_pos );
		ubx_parse( ubx, &buf[0], pos );
	}
}


/**
 * \brief Navigation configuration
 * ToDo: Consider using uCenter (and saving it there in stead)
 */
void ubx_init( struct ubx_handle_t * ubx )
{
	ubx->rx_dma_master = DMA1;
	//ubx->rx_dma_channel = UBX_RX_DMA_CHANNEL;
	ubx->rx_pos = 0;
	ubx->msg.c = &ubx->msg.synch1;
	ubx->nav_pvt_flag = 0;
	ubx->nav_sat_flag = 0;
	ubx->msg.state = 0;

	LL_USART_Disable( GNS_dev );
	LL_DMA_SetPeriphAddress( ubx->rx_dma_master, LL_DMA_STREAM_7, LL_USART_DMA_GetRegAddr( GNS_dev ));
	LL_DMA_SetMemoryAddress( ubx->rx_dma_master, LL_DMA_STREAM_7, (uint32_t) ubx->rx_buf);
    LL_DMA_SetDataLength( ubx->rx_dma_master, LL_DMA_STREAM_7, UBX_RX_BUF_SIZE );
    //NVIC_DisableIRQ( UBX_RX_DMA_MASTER_Channel3_IRQn );  // Disable DMA interrupts -- enabled in dma.c by CubeMX
    //  reset HISR LISR
    //ubx->rx_dma_master->LISR =
	//LL_DMA_ClearFlag_TE3( UBX_RX_DMA_MASTER );
    //LL_USART_EnableIT_IDLE( GNS_dev );
	LL_USART_Enable( GNS_dev );


	// TODO: Replace the Delay! Better wait for and check startup message from GNS module
	vTaskDelay( 1000/portTICK_PERIOD_MS );
	//HAL_Delay(1000);
	//printf("  Navi module connected at baudrate 38400\n\r");

	// Initialise our ubx buffer for sending CFG messages
	// ToDo: Disable reception of messages (or use separate buffers for TX and RX)
	ubx->msg.synch1=UBX_SYNC1;
	ubx->msg.synch2=UBX_SYNC2;

	// Increase UART baud rate, configure what protocols to use
	ubx->msg.class = UBX_CFG_CLASS;
	ubx->msg.id = UBX_CFG_PRT;
	ubx->msg.length = sizeof(struct ubx_cfg_prt_t);
	struct ubx_cfg_prt_t *cfg_prt;
	cfg_prt = (struct ubx_cfg_prt_t*) &ubx->msg.payload;
	cfg_prt->portID =  0x01;
	cfg_prt->reserved1 = 0x01;
	cfg_prt->txReady = 0b0000000000000000;   // Bit field txReady (threshold, pin, polarity, enabler TX ready feature)
	cfg_prt->mode = 0x000008d0;              //little endian
	//cfg_prt->baudRate = 921600;              // Possible UART baud rates: 4800, 9600, 19200, 38400 (default for NEO M9N), 57600, 115200, 230400, 460800, 921600
	cfg_prt->baudRate = 115200;              // Possible UART baud rates: 4800, 9600, 19200, 38400 (default for NEO M9N), 57600, 115200, 230400, 460800, 921600
	cfg_prt->inProtoMask = 0x0007;           // Bitfield, 7 = Rtcm + Nmea + Ubx
	cfg_prt->outProtoMask = 0x0001;          // Bitfield  1 = Ubx only
	cfg_prt->flags=0x0000;
	cfg_prt->reserved[0]=0x00;
	cfg_prt->reserved[1]=0x00;
	ubx_send( ubx, 100 );

	// Disable the USART and re-connect with new baud rate
	LL_USART_Disable( GNS_dev );
	LL_USART_SetBaudRate( GNS_dev, 48000000, LL_USART_OVERSAMPLING_16, cfg_prt->baudRate );
	//printf("  Navi module connected at baudrate %8lu", cfg_prt->baudRate );
	LL_USART_Enable( GNS_dev );
	LL_USART_EnableDMAReq_RX( GNS_dev );
	LL_DMA_EnableStream( ubx->rx_dma_master, LL_DMA_STREAM_7);
	//LL_DMA_EnableChannel( ubx->rx_dma_master, UBX_RX_DMA_CHANNEL );

	/*ubx->msg.class = UBX_CFG_CLASS;
	ubx->msg.id = UBX_CFG_NAV5; // UBX_CFG_NAV5 is deprecated in protocol versions greater than 23.01. Use UBX-CFG-VALSET,
	ubx->msg.length=sizeof(struct ubx_cfg_nav5_t);
	struct ubx_cfg_nav5_t * cfg_nav5;
	cfg_nav5 = (struct ubx_cfg_nav5_t*) &ubx->msg.payload;
	cfg_nav5->mask = 0x0001;  // set only dyn model
	cfg_nav5->dynModel=UBX_DYN_AUTOMOTIVE;
	//ubx_send_msg( &ubx->msg );
	ubx_send( ubx, 1000 );
	//printf("  Navi dyn model requested: %4u\n\r",cfg_nav5->dynModel);*/

	ubx->msg.class = UBX_CFG_CLASS;
	ubx->msg.id = UBX_CFG_SBAS; // UBX_CFG_SBAS is obsolete and superseded by UBX-CFG-GNSS for protocol versions 14.00+
	ubx->msg.length = sizeof(struct ubx_cfg_sbas_t);
	struct ubx_cfg_sbas_t* cfg_sbas;
	cfg_sbas = (struct ubx_cfg_sbas_t*) &ubx->msg.payload;
	cfg_sbas->mode = 0x01;
	cfg_sbas->usage = 0x03;
	cfg_sbas->maxSBAS = 0x03;
	cfg_sbas->scanmode2 = 0x00;
	cfg_sbas->scanmode1 = UBX_SBAS_EGNOS;
	ubx_send( ubx, 100 );
	//printf("  Navi SBAS model requested: %lx\n\r", ubx_cfg_sbas.scanmode1 );

	/*ubx->msg.id = UBX_CFG_MSG;  // UBX_CFG_MSG is deprecated in protocol versions greater than 23.01. Use UBX-CFG-VALSET instead!
	ubx->msg.length = sizeof(struct ubx_cfg_msg_t);
	struct ubx_cfg_msg_t* cfg_msg;
	cfg_msg = (struct ubx_cfg_msg_t*) &ubx->msg.payload;
	cfg_msg->msgClass = UBX_NAV_CLASS;    // Message Class
	cfg_msg->msgID = UBX_NAV_PVT;         // Message Identifier
	cfg_msg->rate = 1;                    // rate
	ubx_send_msg( &ubx->msg );*/

	//! Set the rate of measurements --- this message fails -- I get ACK_NACK response
	/*ubx->id = UBX_CFG_RATE;
	ubx->length = sizeof(struct ubx_cfg_rate_t);
	struct ubx_cfg_rate_t* cfg_rate;
	cfg_rate = (struct ubx_cfg_rate_t*) &ubx->msg;
	cfg_rate->measRate = 25; // [ms] E.g.: 100 ms results in 10 Hz, 1000 ms = 1 Hz. The minimum value is 25.
	cfg_rate->navRate = 1;
	cfg_rate->timeRef = 1;   // 1 = GPS time
	//ubx_send_msg( ubx );
	ubx_send( ubx, 300 );
	*/
	vTaskDelay(1000/portTICK_PERIOD_MS);
	ubx->msg.class = UBX_CFG_CLASS;
	ubx->msg.id = UBX_CFG_VALSET;
	struct ubx_cfg_valset_t* cfg_valset;
	cfg_valset = (struct ubx_cfg_valset_t*) &ubx->msg.payload;
	cfg_valset->version=0x00;			// Message version (0x00 for simple or 0x01 for transaction)
	cfg_valset->layers=0b00000001;	// bit0=RAM, bit1=BBR, bit2 = Flash
	cfg_valset->reserved[0]=0x00;
	cfg_valset->reserved[1]=0x00;
	U1* val_U1;
	U2* val_U2;
	//U4* val_U4;
	// ToDo: Clean up -- find a better way to make the key & value pairs
	U4* key = (U4*) (cfg_valset + 1);
	*key++ = CFG_RATE_MEAS; 					val_U2 = (U2 *) key;	*val_U2++ = 110;	key= (U4 *) val_U2;  // [ms] interval (i.e. inverse of a normal "RATE")
	*key++ = CFG_RATE_NAV; 						val_U2 = (U2 *) key;	*val_U2++ =   1;	key= (U4 *) val_U2;
	*key++ = CFG_MSGOUT_UBX_NAV_PVT_UART1; 		val_U1 = (U1 *) key;	*val_U1++ =   1;	key= (U4 *) val_U1;
	//*key++ = CFG_MSGOUT_UBX_NAV_SAT_UART1; 	val_U1 = (U1 *) key;	*val_U1++ =  10;	key= (U4 *) val_U1;
	*key++ = CFG_NAVSPG_DYNMODEL;	  			val_U1 = (U1 *) key;	*val_U1++ = UBX_DYN_AUTOMOTIVE;  key= (U4 *) val_U1;
	ubx->msg.length = (U1 *) key - (U1 *) cfg_valset;
	//ubx_send_msg( &ubx->msg );
	ubx_send( ubx, 300 );
};


// 
// NMEA output   see www.wikipedia.org/wiki/NMEA_0183
//
extern uint8_t nmea_checksum(const char *);


// $GNGGA message: 
// $GNGGA,     time,      lat,NS,      long,EW,quality,numSV,HDOP,alt,M,sep,M,diffAge,diffStation*cs<CR><LF>
// $GNGGA,hhmmss.ss,ddmm.mmmmm,n,dddmm.mmmmm,e,      q,   ss, y.y,a.a,z,g.g,z,    t.t,        iii*CC
// example:
// $GNGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,*5B
// $GNGGA,081243.300,5940.36629,N,00940.48517,E,1,05,4.14,265.83,M,40.5,M,,*40
//
void gngga(const struct ubx_nav_pvt_t *pvt, char buf[]){
   sprintf(buf,"$GPGGA,%02d%02d%02d.%03.0f,%02ld%08.5f,%s,%03ld%08.5f,%s,%1d,%02d,%3.2f,%3.1f,M,%3.1f,M,,",
        pvt->hour, pvt->min, pvt->sec,      // Time Stamp   hhmmss.ss
        pvt->nano<0?0:1.0e-6*(pvt->nano),   // the .ss part, negative nanos are removed
        labs(pvt->lat)/10000000,            // dddmm.mmmmm,N/S
        labs(pvt->lat)%10000000*60.0e-7,    //
        (pvt->lat>0) ? "N":"S",             //
        labs(pvt->lon)/10000000,            // dddmm.mmmmm,E/W
        labs(pvt->lon)%10000000*60.0e-7,    //
        (pvt->lon>0) ? "E":"W",             //
        pvt->fixType<2?0:1,                 // Fix quality 0 = No fix, 1 = GNSS 2D/3D
                                            // missing: , 2 = Differential GNSS, 4 = Real-time kinematic (RTK) fixed,
                                            // 5 = RTK float, 6 = Estimated/Dead reckoning fix
        pvt->numSV,                         // Number of satellites used in Nav solution, range 1..12
        0.001*pvt->hAcc,                    // Horizontal accuracy estimate [m].  (Missing: Horizontal Dilution of Precision)
        0.001*pvt->hMSL,                    // Altitude above mean sea level [m]
        0.001*(pvt->height - pvt->hMSL)      // is the sign correct ?
   );
   sprintf(buf + strlen(buf), "*%02X\r\n", nmea_checksum(buf+1));
}

//
// RMC 
//
void gnrmc( const struct ubx_nav_pvt_t * pvt, char buf[]){
   sprintf(buf,"$GNRMC,%02d%02d%02d.%03.0f,%s,%03ld%08.5f,%s,%03ld%08.5f,%s,%1.4f,%1.2f,%02d%02d%02d,%1.1f,E,A",
        pvt->hour, pvt->min,pvt->sec,          // Time Stamp   hhmmss.ss
        pvt->nano<0?0:1.0e-6*pvt->nano,        // the .ss part, negative nanos are removed
        (pvt->valid&&0b111== 0b111) ? "A":"V", // validity - A-ok, V-invalid
        labs(pvt->lat)/10000000,               // dddmm.mmmmm,N
		labs(pvt->lat)%10000000*60.0e-7,       //
        (pvt->lat>=0) ? "N":"S",               //
        labs(pvt->lon)/10000000,               // dddmm.mmmmm,E
        labs(pvt->lon)%10000000*60.0e-7,       //
        (pvt->lon>=0) ? "E":"W",               //
        1.943844e-3*pvt->gSpeed,               // speed (2-D) [knot] xxx.x
        1.0e-5*pvt->headMot,                   // Heading of motion(2-D) [deg]  xxx.xx
        pvt->day, pvt->month, pvt->year%100,   // ddmmyy
        1.0e-2*pvt->magDec);
	sprintf(buf + strlen(buf),"*%02X\r\n", nmea_checksum(buf+1));
}

void gnvtg(const struct ubx_nav_pvt_t * pvt, char buf[]){
   sprintf(buf,"$GPVTG,%1.2f,T,%1.2f,M,%1.4f,%1.4f,A",
        1.0e-5*pvt->headMot,                     // Course over ground (true) = Heading of motion(2-D) [deg]  xxx.xx
        1.0e-5*pvt->headMot+1.0e-2*pvt->magDec,  // Course over ground (magnetic)
        1.943844e-3*pvt->gSpeed,                 // speed (2-D) [knot]  xxx.x
        3.6e-3*pvt->gSpeed);                     // speed (2-D) [km/hr] xxx.x
        // fixed value "A"                       // Mode Indicator (NMEA v2.3 and over only)
	sprintf(buf + strlen(buf), "*%02X\r\n", nmea_checksum(buf+1));
}

void gnzda(const struct ubx_nav_pvt_t *pvt, char buf[]){
   sprintf(buf,"$GNZDA,%02d%02d%02d.%03.0f,%02d,%02d,%4d,00,00",
        pvt->hour, pvt->min, pvt->sec,     // Time Stamp   hhmmss.sss
        pvt->nano<0?0:1.0e-6*pvt->nano,    // the .sss part, negative nanos are removed
        pvt->day, pvt->month, pvt->year);
	sprintf(buf + strlen(buf), "*%02X\r\n", nmea_checksum(buf+1));
}


