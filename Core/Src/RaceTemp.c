/*
 * racetemp.c
 *
 *  Created on: 20 Oct 2019
 *      Author: DrMotor
 */

#include "RaceTemp.h"
#include "pass.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>  //sprintf
//#include "printf.h"
#include <string.h> //strlen
//#include "MPU9250.h"
#include "UBX.h"
#include "NTC.h"
#include <SPI.h>
#include "lambda_2.h"
//NTC ntc;

struct ubx_handle_t ubx;    // global message struct
uint32_t count=0;    // Message counter for RaceChrono
uint16_t NTC_raw;
float tempNTC=25.0;  // [degC] initial value = starting temperature for low pass filter
float tempTC=25.0;   // [degC] initial value = starting temperature for low pass filter
uint32_t rc3_time;
uint32_t TC_time;
uint32_t ign_diff=36000;  // initialize to >1 to avoid division by zero

//void StartTransfers(void);  // enable DMA IRQ

#define RC_TX_BUF_SIZE 1024
#define RC_RX_BUF_SIZE 1024
//static char rc_txBuf[256];  // The RC3 message it is actually approx. 138, but might be longer
static char rc_txBuf[RC_TX_BUF_SIZE]; // larger buffer for sending also AT, NMEA and NBP...
static char rc_rxBuf[RC_RX_BUF_SIZE]; // buffer for receiving messages
DMA_TypeDef * RC_dma_master = DMA2;

void delay(const float sec)
{
	vTaskDelay( configTICK_RATE_HZ * sec );
}


volatile float rpm=0;
// Interrupt service routine (ISR) for timer input capture
void RaceTemp_ignition_pulse_isr( uint32_t cnt )
{
#ifdef RPM
  LL_GPIO_TogglePin( BP_LED_GPIO_Port, BP_LED_Pin);  // toggle the LED to show ignition pulse detection
  //timer.setCount(0);  // so that we avoid overflow interrupts while engine is running
  static uint32_t last_ignition = 0UL;
  uint32_t diff = cnt - last_ignition;
  //if ( diff > 2000 ) { //  Valid only if it leads to less than 60e6/2000 = 30 000 rpm
  if ( diff > 36000UL ) { //  Valid only if it leads to less than 20 000 rpm:   12MHz * 60 sec/minute / 20000 rpm  = 36000
    last_ignition=cnt;
    ign_diff=diff;
  }
#endif
}


volatile long mag_split[5] = { 0 };  // time between magnet strips
volatile bool mag_lap_OK = false;
#ifdef MAG
  #define MAGSTRIPS 3 // do not define no more than 5 strips!
  #define MAGSTART 1  // ID# of first strip after switch on.  Start/finish-ine has ID# = MAGSTRIPS-1
void RaceTemp_magstrip_isr(  uint32_t ccr )  // Interrupt service routine (ISR)
{
  static unsigned int mag_id = MAGSTART;
  static unsigned long last_mag=0;
  static bool first_lap = true;
  unsigned long mag = micros();
  unsigned long diff = (mag >= last_mag) ? mag - last_mag : 0xffffffff-last_mag+mag;
  if ( diff > 1000000 ) { //  Only valid if less than 1 second since last pulse
    mag_split[mag_id]=diff;
    if (++mag_id == MAGSTRIPS) {
      mag_id = 0;
      if (first_lap) first_lap = false;
      else  mag_lap_OK = true;
    } else {
      mag_lap_OK = false;
    }
  }
}
#else // dummy MAG for debug
void RaceTemp_magstrip_isr( uint32_t ccr )
{
  static uint32_t avg=0x00000000;
  //uint16_t val = analogRead(MAGSTRIP);
  avg++; //+= avg>>8
  // if magstrip>
}
#endif //MAG


uint16_t adc_read(ADC_TypeDef *ADCx, uint32_t channel)
{
      ADCx->SQR3 = channel;            // Select analog input
      LL_ADC_ClearFlag_EOCS(ADCx);
      LL_ADC_ClearFlag_OVR(ADCx);
      LL_ADC_REG_StartConversionSWStart(ADCx);
      //while (!LL_ADC_IsActiveFlag_EOCS(ADCx)){}     // Wait for end of conversion
      return LL_ADC_REG_ReadConversionData12(ADCx);
}

void RaceTemp_ADC_isr()
{
	static unsigned int chan = 0;
	uint16_t ADC_raw = LL_ADC_REG_ReadConversionData12(ADC1);
	switch ( ++chan )
	{
	case 1:
		NTC_raw = ADC_raw;
		chan = 0; // only one channel is configured
		break;
	}
}

// Function for reading temperature
// See STM32 reference manual section SPI -- "Unidirectional receive-only procedure (BIDIMODE=0 and RXONLY=1)"
// and https://datasheets.maximintegrated.com/en/ds/MAX6675.pdf
//
// Note ADC time is up to 220 ms -- calling a MAX6675 at shorter interval might give invalid results
//
// ToDo: Why does the SPI clocks run 16 cycles for 8 bit data (and 32 for 16 bits)?
// This happens when SPI_InitStruct.BaudRate is less than LL_SPI_BAUDRATEPRESCALER_DIV64.
//
// HW NSS seems to be broken -- cannot get it to function like needed for this application,
// so we need to use SW NSS via GPIO.
//
float MAX6675_temp( SPI_TypeDef *spi )
{
	LL_GPIO_ResetOutputPin( MAX6675_CS_GPIO_Port, MAX6675_CS_Pin );  //Set chip select pin low, chip in use.
	delay(0.0001); // >0.1 us delay is required (MAX6675 datasheet) before starting the SPI clock
	//LL_SPI_ReceiveData16( spi ); // reset RXNE
	LL_SPI_ClearFlag_OVR( spi ); // resets both RXNE and OVR
	LL_SPI_Enable(spi);  // start the SPI clock
	while ( !LL_SPI_IsActiveFlag_RXNE(spi) );  // wait for data  ToDo: Add timeout
	//while ( LL_SPI_IsActiveFlag_BSY(spi) );  // wait while busy
	LL_SPI_Disable(spi); // stop the SPI clock
	// set NSS high to start the next ADC cycle in MAX6675
	LL_GPIO_SetOutputPin( MAX6675_CS_GPIO_Port, MAX6675_CS_Pin ); //Set chip select pin high, chip not in use.
	uint16_t RX_data = LL_SPI_ReceiveData16( spi );
	return 0.25f * ( RX_data >> 3 );
}

/**
 * \brief           Send string to USART
 * \param[in]       str: String to send
 */
void send_string(char* str) {
	size_t len = strlen(str);
    for (; len > 0; --len, ++str) {
        LL_USART_TransmitData8( RC_dev, *str );
        while (!LL_USART_IsActiveFlag_TXE(RC_dev)) {}   // Blocking mode, ToDo: consider using DMA!
    }
    *(str-len)=0; // clear the buffer, strlen=0
}


/**
 * \brief send TxBuf via DMA and USART
 */
void send_txBuf_DMA() {
	LL_DMA_DisableStream( DMA2, LL_DMA_STREAM_7 );
	LL_DMA_SetDataLength( DMA2, LL_DMA_STREAM_7, strlen(rc_txBuf) );
	LL_DMA_EnableStream( DMA2, LL_DMA_STREAM_7 );
	LL_USART_EnableDMAReq_TX( RC_dev );
}


/**
 * \brief           Send AT+CIPSEND and string to USART
 * \param[in]       str: String to send
 */
void RaceTemp_AT_CIPSEND(char* str) {
	size_t len = strlen(str);
	if ( len > 0 )
	{
		char at_com[]="AT+CIPSEND=0,xxx         ";
		//sprintf(&at_com[13],"%03u\r\n",len);
		sprintf(&at_com[13],"%u\r\n",len);
		send_string( at_com );
		//send_string("AT+CIPSEND=0,100\r\n");
		//delay(0.003);  // wait for ">"  ToDo: Read and check the response
		delay(0.001);  // wait for ">"  ToDo: Read and check the response
		//send_string(str);
		send_txBuf_DMA();
		//delay(0.02);  // wait for "OK"  ToDo: Read and check the response
	}
}

// NMEA 0183 defines a max message length of 82, including $ *checksum CR and LF
//#define NMEA_MAX_LENGTH 82
//static char nmea_buf[NMEA_MAX_LENGTH];
uint8_t nmea_checksum(const char *s) {  // s should not include the leading $-sign and the ending *-sign
    int c = 0;
    while(*s) c ^= *s++;
    return c;
}


//
// Write $RC3 message in the buffer
//
void rc3_sprintf( char buf[] )
{
	int16_t acc[3]={0}, gyro[3]={0};
	#ifdef MPU9250
		// Accelerometer and gyro -------------------------------------------------------------------------
		int16_t mag[3]={0};
		MPU9250_read(acc, gyro, mag);
  	#endif

    // Analogue input -------------------------------------------------------------------------------
	//#ifdef NTC
	//float t = NTC_temp( NTC_raw, NTC_KOSO );
	float t = NTC_temp( NTC_raw, NTC_AC );
	LL_ADC_ClearFlag_OVR(ADC1);
	LL_ADC_REG_StartConversionSWStart(ADC1); // Trigger next regular ADC conversion
	if ( (t<200.0) && (t>-50.0)) tempNTC = 0.99*tempNTC + 0.01*t; // if plausible: 1st order filter
//#define MOCK_VALUES
#ifdef MOCK_VALUES
  tempNTC=tempNTC+0.1;
  if (tempNTC>99.9) tempNTC=0.0;
#endif
  	//tempNTC=analogRead(NTC);  // without filter
  	//#endif
    //const float percent=100.0/ADCrange;
    #ifdef BRK
       float brake=analogRead(BRK)*percent;  // brake pedal sensor
    #else
        float brake=40;  // brake pedal sensor not yet connected (not allowed in karting)
    #endif
    //float throttle=analogRead(A3)*percent;  // throttle pedal sensor
    float throttle=50;  // throttle pedal sensor not yet connected (not allowed in karting)
    //float a5=0; //analogRead(A4)*percent;  // not yet connected
    //float a6=0; //analogRead(A5)*percent;  // not yet connected
    //float a7=0; //analogRead(A6)*percent;  // not yet connected
    //float a8=0; //analogRead(A0)*percent;  // not yet connected

    // lap times from magnet strips --------------------------------------------------------------------------
    static unsigned long mag_lap=0;
    if (mag_lap_OK) mag_lap = mag_split[0] + mag_split[1] + mag_split[2] + mag_split[3] + mag_split[4];

  // RaceChrono output in $RC3 format ------------------------------------------------------------------------
  /* $RC3,[time],[count],[xacc],[yacc],[zacc],[gyrox],[gyroy],[gyroz],[rpm/d1],[d2],
     [a1],[a2],[a3],[a4],[a5],[a6],[a7],[a8],[a9],[a10],[a11],[a12],[a13],[a14],[a15]*checksum
     - $ is message start character
     - RC2 and RC3 are message identifiers
     - time stamp is not used (empty). (for blended GNS support this should be a GNS synchronized realtime timestamp in NMEA 0183 format).
     - count is an overflowing line counter 0-65535. Can be left empty if GNS timestamp is provided.
     - acc fields: -1.000 = -1G, 1.000 = +1G
     - gyro fields: degrees per second, -1.000 = -1 deg/s, 1.000 = +1 deg/s
     - dx are digital channel fields, range -2000000.000 - 2000000.000
     - ax are analog channel fields, range -2000000.000 - 2000000.000
     - * is message separator character
     - NMEA 0183 type checksum, with two uppercase hexadecimal digits (one byte)
     - each line is terminated with CR plus LF
   */
    //uint32_t engine_speed = (ign_diff > 200000*96) ? 0 : 60000000*96/ign_diff; // [rpm] engine speed  (= 0 if below 60e6*96/200000 = 300 rpm)
    float engine_speed = ( ign_diff > 2400000UL) ? 0 : 7.2e8f / ign_diff; // [rpm] engine speed  (= 0 if below 60*96e6/200000 = 300 rpm)
    float gear_ratio = (ubx.nav_pvt.gSpeed < 1000) ? 0.0f : 1000.0f / 60.0f * engine_speed / ubx.nav_pvt.gSpeed; // [rev/m]
    sprintf(buf,"$RC3,,%lu,%1.3f,%1.3f,%1.3f,%1.3f,%1.3f,%1.3f,%1.1f,%i,%1.1f,%1.1f,%1.1f,%1.1f,%1.1f,%1.1f",
        //ubx.nav_pvt.hour,ubx.nav_pvt.min,ubx.nav_pvt.sec, // Time Stamp   hhmmss.ss   %02d%02d%02d.%03.0f
        //ubx.nav_pvt.nano<0?0:1.0e-6*ubx.nav_pvt.nano,     // the .ss part, negative nanos are removed
        count++,            // $RC3,time,count[-]
        1.2214e-4*acc[0],   // 8/0xffff * acc_x [g]  (range = +/-4 g)
        1.2214e-4*acc[1],   // 8/0xffff * acc_y [g]
        1.2214e-4*acc[2],   // 8/0xffff * acc_z [g]
        3.0518e-2*gyro[0],  // 2000/0xffff * gyro_x [deg/s]  (range = +/-1000 deg/s)
        3.0518e-2*gyro[1],  // 2000/0xffff * gyro_y [deg/s]
        3.0518e-2*gyro[2],  // 2000/0xffff * gyro_z [deg/s]
        engine_speed,       // d1 [rpm] engine speed
        0,	         		// d2 [-] Gear 0,1,2,3,4,5,6 (0 = neutral or unknown)
        tempNTC,            // a1 [degC] = water temp
        t,                  // a2 [degC] = water temp without filter
        throttle,           // a3 = (ratiometric Hall)
        brake,              // a4 = (ratiometric Hall)
		tempTC, 			// a5 [degC] EGT -- exhaust gas temperature
		gear_ratio );       // a6 [rev/m] engine revolution per travel over ground

    Lambda_sprintf( buf + strlen(buf) );  // a7,a8,a9 = fuel excess, battery voltage and LSU temperature

    sprintf(buf + strlen(buf), ",%1.1f,%1.1f,%1.1f,%1.1f,%1.1f",
        //1.0e-6*mag_lap,      // a10  lap time
        1.0e-6*mag_split[0],   // a11  split time 0
        1.0e-6*mag_split[1],   // a12
        1.0e-6*mag_split[2],   // a13
        1.0e-6*mag_split[3],   // a14
        1.0e-6*mag_split[4] ); // a15

    sprintf( buf + strlen(buf), "*%02X\r\n", nmea_checksum(buf+1) );

     //printf((char*) buf,strlen(buf));
     //RC.write((uint8_t*) buf,strlen(buf));
     //RC_Transmit();

     //#ifndef RPM
       // toggle the LED anyway when RPM is not defined
       LL_GPIO_TogglePin( BP_LED_GPIO_Port, BP_LED_Pin );
       //digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ^ 1);
     //#endif
}


/**
 * \brief           Check for new data received via DMA
 *
 * Not doing reads fast enough will cause DMA to overflow and loose data.
 */
/*void rc_receive()
{
    U1 *buf = rc_rxBuf;
    size_t pos = UBX_RX_BUF_SIZE - LL_DMA_GetDataLength( RC_dma_master, LL_DMA_STREAM_5 );
    size_t old_pos = RC_rx_pos;  // The saved position from last call is now old
	RC_rx_pos = pos;   // Save current position as old for next call
	if ( pos > old_pos ) {    		// New data is contiguous in buffer
		rc_parse( ubx, rc_rxBuf[old_pos], pos - old_pos );
	} else if ( pos < old_pos ) {  	// New data is split at tail and head of buffer
		rc_parse( rc_rxBuf[old_pos], sizeof(rc_rxBuf) - old_pos );
		rc_parse( rc_rxBuf[0], pos );
	}
}*/

void SetupESP8266()
{
	//ubx->rx_dma_channel = UBX_RX_DMA_CHANNEL;
	/* LL_USART_Disable( RC_dev );
	LL_DMA_SetPeriphAddress( RC_dma_master, LL_DMA_STREAM_5, LL_USART_DMA_GetRegAddr( RC_dev ));
	LL_DMA_SetMemoryAddress( RC_dma_master, LL_DMA_STREAM_5, *rc_rxBuf );
    LL_DMA_SetDataLength( RC_dma_master, LL_DMA_STREAM_5, sizeof(rc_rxBuf) );
    //LL_USART_EnableIT_IDLE( RC_dev );
	LL_USART_Enable( RC_dev ); */

	delay(0.5);  // for ESP boot and ADC stabilisation

    // HC-06 is in AT mode by default, and stays that way until a device connects with it.
	// "AT" strings to the HC-06 do not need termination by \n \r.
	// The HC-06 "should" always wake up at default baud rate 9600 bps, but it does not...
	// The HC-06 responds after a second or so, therefore is inserted delays here.
	//send_string("AT+BAUDC"); // set baud rate to 132400, should work with HC06, BT06 and others with Linvor firmware
	//send_string("AT+DEFAULT");
	//send_string("AT+BAUD9");
	//send_string("AT+BAUD8");
	//send_string( "AT+BAUD8" );
	//delay(0.1);
	//send_string( "AT+VERSION\r\n" );
	//delay(0.1);
/*	LL_USART_Disable( BT_dev );
	LL_USART_SetBaudRate( BT_dev, LL_RCC_GetUSARTClockFreq(LL_RCC_USART1_CLKSOURCE),
			LL_USART_PRESCALER_DIV1, LL_USART_OVERSAMPLING_16, 115200 );
	LL_USART_Enable( BT_dev );*/
	//while((!(LL_USART_IsActiveFlag_TEACK(BT_dev))) || (!(LL_USART_IsActiveFlag_REACK(BT_dev))));

	/* ESP8266 WiFi module: https://www.espressif.com/sites/default/files/4a-esp8266_at_instruction_set_en_v1.5.4_0.pdf
	 * AT+CWSAP_CUR=<ssid>, <pwd>, <chl>, <ecn>, [, <max conn>][, <ssid hidden>]
	 * <ssid> string, ESP8266 softAP’ SSID
	 * <pwd> string, range: 8 ~ 64 bytes ASCII
	 * <chl> channel id
	 * <ecn>
	 * 		0 OPEN
	 * 		2 WPA_PSK
	 * 		3 WPA2_PSK
	 * 		4 WPA_WPA2_PSK
	 * <max conn> maximum count of stations that allowed to connect to ESP8266 soft-AP. range: [1, 4]
	 * <ssid hidden> Broadcast SSID by default
	 * 		0 broadcast SSID of ESP8266 soft-AP
	 * 		1 do not broadcast SSID of ESP8266 soft-AP
	 */
	send_string( CWSAP );  // <---- EDIT this in pass.h
	delay(1.0);
	send_string( "ATE0\r\n" ); // Echo off
	delay(0.1);

	// ESP8266 supports baud rates up to 115200*40 = 4608000, but 1000000 seems to be more reliable (for me)
	// Version 2.2 of the ESP01 AT-firmware does not support 4608000 (?)
	const uint32_t baudrate=1000000;
	sprintf( rc_txBuf,"AT+UART_CUR=%lu,8,1,0,0\r\n",baudrate ); // 8 bits, 1 stop bit, no parity, flow control"
	send_string( rc_txBuf );

	delay(0.1);
	LL_USART_Disable( RC_dev );
	// LL_USART_SetBaudRate( BT_dev, 170000000U, LL_USART_PRESCALER_DIV1, LL_USART_OVERSAMPLING_16, 115200 );
	//LL_USART_SetBaudRate( RC_dev, 96000000U, LL_USART_OVERSAMPLING_16, 1000000 );

	// PCLK2 is used for USART1 = RC_dev.
	// ToDo: Clean up this code so that it does not need to know that PCLK2 is used for RC_dev!
	LL_RCC_ClocksTypeDef RCC_Clocks;
	LL_RCC_GetSystemClocksFreq( &RCC_Clocks );
	LL_USART_SetBaudRate( RC_dev, RCC_Clocks.PCLK2_Frequency, LL_USART_OVERSAMPLING_16, baudrate );

	LL_DMA_SetPeriphAddress( DMA2, LL_DMA_STREAM_5, LL_USART_DMA_GetRegAddr( RC_dev ) );
	LL_DMA_SetPeriphAddress( DMA2, LL_DMA_STREAM_7, LL_USART_DMA_GetRegAddr( RC_dev ) );
	LL_DMA_SetMemoryAddress( DMA2, LL_DMA_STREAM_5, (uint32_t) rc_rxBuf );
	LL_DMA_SetMemoryAddress( DMA2, LL_DMA_STREAM_7, (uint32_t) rc_txBuf );
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_7);
    LL_DMA_SetDataLength( DMA2, LL_DMA_STREAM_5, RC_RX_BUF_SIZE );
    LL_DMA_SetDataLength( DMA2, LL_DMA_STREAM_7, RC_TX_BUF_SIZE );
	LL_USART_EnableDMAReq_RX( RC_dev );
	LL_USART_EnableDMAReq_TX( RC_dev );
	LL_DMA_EnableStream( DMA2, LL_DMA_STREAM_5 );

	LL_USART_Enable( RC_dev );
	delay(0.005);
	//while((!(LL_USART_IsActiveFlag_TEACK(BT_dev))) || (!(LL_USART_IsActiveFlag_REACK(BT_dev))));
	//sprintf(rc_buf,"AT+UART=1382400,0,0\r\n");  // alternative in case AT+BAUD is not recognised (my HC06 does not respond to "AT+BAUDB\r\n")

	//send_string( "AT\r\n");
	sprintf( rc_txBuf,"AT\r\n");
	send_txBuf_DMA();
	//LL_DMA_SetDataLength( DMA2, LL_DMA_STREAM_7, strlen(rc_txBuf) );
	//LL_DMA_EnableStream( DMA2, LL_DMA_STREAM_7 );

	delay(0.1);
	//LL_DMA_DisableStream( DMA2, LL_DMA_STREAM_7 );
	//send_string( "AT+UART?\r\n");
	//delay(1.0);
	//send_string( "AT+GMR\r\n");
	//sprintf( rc_txBuf,"AT+GMR\r\n");
	sprintf( rc_txBuf,"AT+GMR\r\n" );
	send_txBuf_DMA();
	delay(0.1);

	// Request the ESP to do not store settings in flash
	sprintf( rc_txBuf,"AT+SYSSTORE=0\r\n" );
	send_txBuf_DMA();
	delay(0.1);

	/*
	 * uint32_t baudrates[]={1200, 2400, 4800, 9600, 19200, 28800, 38400, 57600, 115200, 230400, 460800, 921600, 1382400};
	for (int i=12; i<=12; i++) // min baudrate 2595 Bits/s for STM32
	{
		LL_USART_Disable( BT_dev );
		LL_USART_SetBaudRate( BT_dev, 170000000U, LL_USART_PRESCALER_DIV1, LL_USART_OVERSAMPLING_16, baudrates[i] );
		LL_USART_Enable( BT_dev );
		while((!(LL_USART_IsActiveFlag_TEACK(BT_dev))) || (!(LL_USART_IsActiveFlag_REACK(BT_dev))));
		send_string( "AT+VERSION");  // expecting reply "OK" in the logic analyser
		delay(2.000);
		send_string( "AT+VERSION\r\n");  // expecting reply "OK" in the logic analyser
		delay(2.000);
	}*/
	send_string( "AT+CWMODE=2\r\n"); // 2 = softAP mode (allow phone to connect via WiFi)
	delay(0.10);
	send_string( "AT+CIPMODE=0\r\n" );  // Prepare normal mode
	delay(0.1);
	send_string( "AT+CIFSR\r\n" );  // Query IP address
	delay(0.1);
	send_string( "AT+CIPMUX=1\r\n"); // Enable multiple connections --> no pass-through mode
	delay(0.1);
	send_string( "AT+CIPSERVER=1\r\n"); // Create a TCP server. Default port = 333
	delay(1.000);  // wait for connection

	// *** Problems with ESP01 CIPSEND ***
	// The response times are inconsistent -- sometimes > 0.1 s.  This leads to loss of data.
	// Solution to this could be to use pass-through mode...

	// ***Problems with ESP01's Pass-through mode***
	// 1. RaceChrono only connects via TCP to ESP01 in AP mode.
	// 2. WiFi TCP pass-through mode ***not*** available in ESP01 AP mode.
	// 3. WiFi ***UDP*** pass-through mode ***is*** available in ESP01 AP mode (but RaceChrono does not support UDP).
	// 4. WiFi TCP pass-through mode ***is*** available in ESP01 Station mode (but RaceChrono does not support this).
	// Preliminary conclusion: RaceChrono is not compatible with the ESP01's pass-through mode
	//
	//send_string( "AT+CIPMODE=1\r\n" );  // Prepare UART-WiFi pass-through mode
	//delay(0.005);
	//send_string( "AT+CIPSTART=1\r\n" );  // Prepare UART-WiFi pass-through mode
	//delay(0.005);
	//send_string( "AT+CIPSEND\r\n" );  // Start sending Enter Wi-Fi Pass-through mode
	//delay(0.010);
}

// Setup USART and DMA.
// An ESP32s3 is running the UART to WiFi bridge (Arduino sketch)
void SetupUARTtoWiFi()
{
	// PCLK2 is used for USART1 = RC_dev.
	// ToDo: Clean up this code so that it does not need to know that PCLK2 is used for RC_dev!
	LL_RCC_ClocksTypeDef RCC_Clocks;
	LL_RCC_GetSystemClocksFreq( &RCC_Clocks );
	// ESP32s3 supports baud rates up to 115200*40 = 4608000 (or?), but 1000000 is plenty for our use
	LL_USART_SetBaudRate( RC_dev, RCC_Clocks.PCLK2_Frequency, LL_USART_OVERSAMPLING_16, 230400 );
	LL_DMA_SetPeriphAddress( DMA2, LL_DMA_STREAM_5, LL_USART_DMA_GetRegAddr( RC_dev ) );
	LL_DMA_SetPeriphAddress( DMA2, LL_DMA_STREAM_7, LL_USART_DMA_GetRegAddr( RC_dev ) );
	LL_DMA_SetMemoryAddress( DMA2, LL_DMA_STREAM_5, (uint32_t) rc_rxBuf );
	LL_DMA_SetMemoryAddress( DMA2, LL_DMA_STREAM_7, (uint32_t) rc_txBuf );
	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_7);
    LL_DMA_SetDataLength( DMA2, LL_DMA_STREAM_5, RC_RX_BUF_SIZE );
    LL_DMA_SetDataLength( DMA2, LL_DMA_STREAM_7, RC_TX_BUF_SIZE );
	LL_USART_EnableDMAReq_RX( RC_dev );
	LL_USART_EnableDMAReq_TX( RC_dev );
	LL_DMA_EnableStream( DMA2, LL_DMA_STREAM_5 );
	LL_USART_Enable( RC_dev );
}

void RaceTemp()
{
	LL_ADC_EnableIT_EOCS(ADC1); // Enable end of conversion interrupt
	LL_ADC_Enable(ADC1);

	LL_TIM_CC_EnableChannel(TIM2,LL_TIM_CHANNEL_CH1|LL_TIM_CHANNEL_CH2);
	LL_TIM_EnableCounter(TIM2); // Start the timer for ignition probe and lap times
	LL_TIM_EnableIT_CC1(TIM2);
	LL_TIM_EnableIT_CC2(TIM2);
	LL_TIM_ClearFlag_CC1(TIM2);
	LL_TIM_ClearFlag_CC2(TIM2);

	memset(rc_txBuf, 0, sizeof(rc_txBuf)); // clear the buffer
	memset(rc_rxBuf, 0, sizeof(rc_rxBuf)); // clear the buffer

	//SetupESP8266();
	SetupUARTtoWiFi();

#ifdef UBX_NAVI
	ubx_init( &ubx );
	/*LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2, (uint32_t) &rc_buf,
			LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_TRANSMIT),
			LL_DMA_DIRECTION_MEMORY_TO_PERIPH);*/
#endif
	//LL_GPIO_SetOutputPin( MAX6675_VDD_GPIO_Port, MAX6675_VDD_Pin );  //Power up the MAX6675 for exhaust temp reading (not needed if permanently connected to VDD)

	// just testing... will the clock polarity change if sending some dummy data
	//LL_SPI_Enable( MAX6675_dev );
	//LL_SPI_TransmitData16( MAX6675_dev, 0x5555 );
	//while (MAX6675_dev->SR & SPI_SR_BSY);  // wait
	//LL_SPI_Disable( MAX6675_dev );
	//delay(0.010);

	rc_txBuf[0]=0; // set length = 0;
	rc3_time = HAL_GetTick();
	TC_time = rc3_time;
	while(1)
	{
		#ifdef UBX_NAVI
		ubx_receive2( &ubx );
		if (ubx.nav_pvt_flag == 1)  // you got a complete new ubx nav_pvt mail !!!!
		{
			ubx.nav_pvt_flag = 0;
			// GNS NMEA ------------------------------------------------------------------------------------------
			// What NMEA sentences does RaceChrono accept?  Answer from AOL:
			// RMC + GGA combinations will work.
			// GGA + VTG + ZDA  combinations will also work.
			// GSA and GSV are used only for satellite azimuth display.
			gnrmc( &ubx.nav_pvt, rc_txBuf + strlen(rc_txBuf) );
			gngga( &ubx.nav_pvt, rc_txBuf + strlen(rc_txBuf) );
			//printf(nmea_buf,strlen(nmea_buf));
		    //gnzda( &ubx.nav_pvt, nmea_buf );
		    //gnvtg( &ubx.nav_pvt, nmea_buf );
		}
		if (ubx.nav_sat_flag == 1)  // you got a complete new ubx nav_pvt mail !!!!
		{
			ubx.nav_sat_flag = 0;
			//ToDo: Write GSA and GSV
		}
		  /*while ( GNS.available() >0 ) {  // direct forwarding from GNS to USB and RC
		    char c = GNS.read();
		    USB.write(c); //echo to USB/PC for uCenter or debugging
		    //RC.write(c);  // echo to RaceChrono via BlueTooth
		  }*/
 	    	//char c;
 	    	//CDC_Receive_FS(&c, 1)
		    //HAL_UART_Transmit_DMA(GN1,&c,1)

		  #ifdef USB
		  //while ( USB.available() ) GNS.write(USB.read());  // for u-Center or debugging
		  #endif //USB
		#endif //UBX_NAVI
		//LambdaShield_calc();
		uint32_t tick = HAL_GetTick();
		if (tick >= TC_time )
		{
			TC_time += 300;  // do not set this lower than 220!  ref MAX6675 datasheet
			tempTC = MAX6675_temp( MAX6675_dev); //[degC] thermocouple temperature
		}

		// RaceChrono RC3 or TrackAddict NBP -------------------------------------------
		if ( tick >= rc3_time )
		{  // set the += value to the wanted period [ms] = 1000/(update rate [Hz]).
			rc3_time += 100;  // aol's advice: "pick update rate that is close as possible to 1/5/10/20/30/40/50/100 Hz."

			//rc3_sprintf( rc_txBuf );
			rc3_sprintf( rc_txBuf + strlen(rc_txBuf) );

		    //Lambda_nbp_sprintf( rc_txBuf + strlen(rc_txBuf) );
			send_txBuf_DMA();
			//RaceTemp_AT_CIPSEND( rc_txBuf ); // for EESP8266 AT-mode
			rc_txBuf[0]=0; // set length = 0;
		}
		//delay( 0.01 ); // ToDo: Wait for "SEND OK" from ESP8266
	}
}



