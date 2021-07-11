#include "lora.h"
#include <string.h>
#include "board.h"
#include "gpio.h"
#include "delay.h"
#include "timer.h"
#include "radio.h"
#include "stm32l4xx_hal.h"
#include <stdio.h>
#include "sx126x.h"
#include "board-config.h"
#include "sx126x-board.h"


#define USE_MODEM_FSK



// #define RF_FREQUENCY                                433000000  // Hz
// #define RF_FREQUENCY                                855800000 // Hz real fre=855820400 (26M tcxo)
//#define RF_FREQUENCY                                (855800000-20400) // Hz real fre=855800000 (26M tcxo)
#define RF_FREQUENCY                                (855800000-20400-1600) // Hz real fre=855798400 (26M tcxo) ,this is same to si4463
// #define RF_FREQUENCY                                870000000 // Hz



#define TX_OUTPUT_POWER                             14        // dBm

#if defined( USE_MODEM_LORA )

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#elif defined( USE_MODEM_FSK )

// #define FSK_FDEV                                    25000     // Hz
// #define FSK_DATARATE                                50000     // bps
// #define FSK_BANDWIDTH                               100000     // Hz
// #define FSK_AFC_BANDWIDTH                           166666     // Hz
// #define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
// #define FSK_FIX_LENGTH_PAYLOAD_ON                   false

#define FSK_FDEV                                    5000     // Hz
#define FSK_DATARATE                                20000     // bps
#define FSK_BANDWIDTH                               100000     // Hz
#define FSK_AFC_BANDWIDTH                           166666     // Hz
#define FSK_PREAMBLE_LENGTH                         8         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false
// #define FSK_FIX_LENGTH_PAYLOAD_ON                   true


#else
    #error "Please define a modem in the compiler options."
#endif

typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
}States_t;

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 64 // Define the payload size here

const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

States_t State = LOWPOWER;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

//uint8_t g_xordata[]={0x63,0xac,0xe8,0x98,0x49,0x04,0x32,0x05,0xdc,0x5d,0xd6};
//                     0x63  0xac  0xe8  0x98  0x69  0x04  0x32  0x05  0xd8
//                   0x71  0x8e  0xde  0xda  0x5b  0x62  0x50  0x87  0x4a
uint8_t g_xordata[]={0x63,0xac,0xe8,0x98,0x69,0x04,0x32,0x05,0xd8};
uint8_t xorbuf[]={0xff,0xe1 ,0x1d ,0x9a ,0xed ,0x85 ,0x33 ,0x24 ,0xea ,0x7a ,0xd2 ,0x39 ,0x70 ,0x97 ,0x57 ,0x0a ,0x54 ,0x7d ,0x2d ,0xd8 ,0x6d ,0x0d ,0xba ,0x8f ,0x67 ,0x59 ,0xc7 ,0xa2 ,0xbf ,0x34 ,0xca ,0x18 ,0x30 ,0x53 ,0x93 ,0xdf ,0x92 ,0xec ,0xa7 ,0x15 ,0x8a ,0xdc ,0xf4 ,0x86 ,0x55 ,0x4e ,0x18 ,0x21 ,0x40 ,0xc4 ,0xc4 ,0xd5 ,0xc6 ,0x91 ,0x8a ,0xcd ,0xe7 };
uint8_t xorbuf_rx[]={0x66, 0xa5, 0xc3, 0x5a, 0x24, 0xff, 0x00, 0xbd, 0x24, 0x99,\
                      0xa5, 0x7e, 0x7e, 0xbd, 0x5a, 0x7e, 0xc3, 0x99, 0xc3, 0xdb,\
                      0xbd, 0xe7, 0x7e, 0x81, 0xc3, 0x24, 0xe7, 0x42, 0x18, 0x99,\
                      0x00, 0x3c, 0x99, 0x5a, 0x24, 0xdb, 0xdb, 0x42, 0xbd, 0xdb,\
                      0xe7, 0xdb, 0xe7, 0xff, 0x3c, 0x00, 0xa5, 0x42, 0xe7, 0xe7, \
                      0x7e, 0xa5, 0x18, 0xdb, 0x7e, 0x00, 0x5a, 0x3c, 0x99, 0x7e};
/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * LED GPIO pins objects
 */
extern Gpio_t Led1;
extern Gpio_t Led2;
extern SX126x_t SX126x;
/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( void );



void OnTxDone( void )
{
    Radio.Sleep( );
    State = TX;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    State = TX_TIMEOUT;
}

void OnRxTimeout( void )
{
    Radio.Sleep( );
    State = RX_TIMEOUT;
}

void OnRxError( void )
{
    Radio.Sleep( );
    State = RX_ERROR;
}

/**
 * @brief packet init
 * 
 */
void packet_init(void)
{
    GpioInit(&Led1, LED_1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);
    GpioInit(&Led2, LED_2, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0);
    SpiInit( &SX126x.Spi, SPI_1, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, NC );
    SX126xIoInit( );
    
    // Radio initialization
    // RadioEvents.TxDone = OnTxDone;
    // RadioEvents.RxDone = OnRxDone;
    // RadioEvents.TxTimeout = OnTxTimeout;
    // RadioEvents.RxTimeout = OnRxTimeout;
    // RadioEvents.RxError = OnRxError;

    Radio.Init(&RadioEvents);

    Radio.SetChannel(RF_FREQUENCY);

#if defined(USE_MODEM_LORA)

    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

    Radio.SetMaxPayloadLength(MODEM_LORA, BUFFER_SIZE);

#elif defined(USE_MODEM_FSK)

    Radio.SetTxConfig(MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                      FSK_DATARATE, 0,
                      FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, 0, 3000);

    Radio.SetRxConfig(MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                      0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                      0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                      0, 0, false, true);

    Radio.SetMaxPayloadLength(MODEM_FSK, BUFFER_SIZE);

#else
#error "Please define a frequency band in the compiler options."
#endif
}

/**
 * @brief transmit packet
 * 
 */
void packet_tx(void)
{
    int i;
    uint16_t irqRegs;
    // uint8_t xx[10]={0x0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t xx[10]={0x12, 0x22, 0x32, 0x42, 0x52, 0x62, 0x72, 0x82, 0x92};
    uint8_t yy1[12] = {0xf6, 0xf3, 0x3f, 0xa8, 0xaf, 0xd7, 0x51, 0x56, 0x68, 0xe8, 0x8f, 0xef}; // si4463 whiten data for 0x12 to 0x92
    //uint8_t yy[12] =  {0xb7, 0xa1, 0xcc, 0x24, 0x57, 0x5e, 0x25, 0xc2, 0xf6, 0x87, 0xb8, 0x59};  // whiten serial of sx1268
    uint8_t yy[12] =  {0x18, 0x76, 0x9d, 0x72, 0xf6, 0x74, 0x87, 0xf1, 0xf6, 0x74, 0x87, 0xf1};  // whiten serial of sx1268
    uint16_t crc;
    // Indicates on a LED that the received frame is a PONG
    GpioToggle(&Led1);

    // Send the next PING frame
    // Buffer[0] = 'P';
    // Buffer[1] = 'I';
    // Buffer[2] = 'N';
    // Buffer[3] = 'G';
    Buffer[0] = 0x12;
    Buffer[1] = 0x34;
    Buffer[2] = 0x56;
    Buffer[3] = 0x78;

    // We fill the buffer with numbers for the payload
    BufferSize=4;
    for (i = 4; i < BufferSize; i++)
    {
        Buffer[i] = i - 4;
    }
    Radio.Standby();
    
    DelayMs(2);
    irqRegs=SX126xGetIrqStatus();
    SX126xClearIrqStatus(irqRegs);
    // SX126xClearIrqStatus(0xffff);

    // yy[0]=0x09;
    // memcpy(&yy[1],xx,9);

    // crc=crc16(yy,yy[0]+1);
    // yy[10]=crc>>8;
    // yy[11]=crc&0xff;
    // for ( i = 0; i < 12; i++)
    // {
    //     yy[i]^=xorbuf[i];
    // }
    // Radio.Send(yy,12);

    for ( i = 0; i < 12; i++)
    {
        yy[i]^=yy1[i];
    }
    Radio.Send(&yy[1],9);
    // Radio.Send(xx,9);
    // Radio.Send(Buffer, BufferSize);
    // SX126xClearIrqStatus(0xffff);
    i=0;
    while (1)
    {
        irqRegs=SX126xGetIrqStatus();
        // printf("the Irq status is 0x%04x\n",irqRegs);
        if((irqRegs&IRQ_TX_DONE) ==IRQ_TX_DONE)
        {
            printf("transmit finished\n");
            break;
        }
        else
        {
            // printf("--------\n");
            i++;
            HAL_Delay(10);
            if(i>=200)
            {
                printf("transmit timeout\n");
                break;
            }
        }
    }
    

}
void fix_lenth_packet_tx_crc_whiten(void)
{
     int i;
    uint16_t irqRegs;
    uint8_t xx[]={0x12, 0x22, 0x32, 0x42, 0x52, 0x62, 0x72, 0x82, 0x92};
    uint8_t yy[12];
    uint16_t crc;
    // Indicates on a LED that the received frame is a PONG
    GpioToggle(&Led1);
    
    // redirect packet parameter
    RadioStandby( );
    SX126x.PacketParams.Params.Gfsk.HeaderType =  RADIO_PACKET_FIXED_LENGTH ;//: RADIO_PACKET_VARIABLE_LENGTH;
    SX126x.PacketParams.Params.Gfsk.DcFree = RADIO_DC_FREE_OFF;
    SX126xSetPacketParams( &SX126x.PacketParams );

    // read and clear the interrupt pending bits
    DelayMs(2);
    irqRegs=SX126xGetIrqStatus();
    SX126xClearIrqStatus(irqRegs);
    

    // yy[0]=0x09;
    // memcpy(&yy[1],xx,9);

    // crc=crc16(yy,yy[0]+1);
    // yy[10]=crc>>8;
    // yy[11]=crc&0xff;

    // for ( i = 0; i < 12; i++)
    // {
    //     yy[i]^=xorbuf[i];
    // }
    // Radio.Send(yy,12);

    BufferSize=9;
    memcpy(Buffer,xx,BufferSize);
    assembly_package(Buffer,BufferSize);
    for(i=0;i<12;i++)
    {
        printf("0x%02x,",Buffer[i]);
    }
    Radio.Send(Buffer,BufferSize+3);
    //Radio.Send(yy,12);
    // Radio.Send(xx,9);
    // Radio.Send(Buffer, BufferSize);
   

    i=0;
    while (1)
    {
        irqRegs=SX126xGetIrqStatus();
        // printf("the Irq status is 0x%04x\n",irqRegs);
        if((irqRegs&IRQ_TX_DONE) ==IRQ_TX_DONE)
        {
            printf("transmit finished\n");
            break;
        }
        else
        {
            // printf("--------\n");
            i++;
            HAL_Delay(10);
            if(i>=200)
            {
                printf("transmit timeout\n");
                break;
            }
        }
    }

    //set the packet parameter to variable length packet
    // RadioStandby( );
    // SX126x.PacketParams.Params.Gfsk.HeaderType =   RADIO_PACKET_VARIABLE_LENGTH;
    // SX126x.PacketParams.Params.Gfsk.DcFree = RADIO_DC_FREEWHITENING;
    // SX126xSetPacketParams( &SX126x.PacketParams );
    

}

/**
 * @brief add the package length ot the first byte of the payload
 *        and then add crc16 the end of the payload
 * @param p 
 * @param len 
 */
void assembly_package(unsigned char  *p,unsigned char  len)
{
    uint8_t *p1;
    uint16_t crc;
    int i;
    // allocate new space to stuff
    p1= (unsigned char *)malloc(len);
    memcpy(p1,p,len);

    
    // insert the length to the struff
    p[0]=len;
    
    // stuff the origin data
    memcpy(p+1,p1,len);

    // get the crc 16 of the struf
    crc=crc16(p,len+1);


    // append the crc to the payload 
    p[len+1]=crc>>8;
    p[len+2]=crc&0xff;

    // whiten the stuff
    for ( i = 0; i < len+3; i++)
    {
        p[i]^=xorbuf[i];
    }
    
    // free the memory
    free(p1);

}
/**
 * @brief receive packet
 * 
 */
void packet_rx(void)
{
    int i;
    uint16_t irqRegs = SX126xGetIrqStatus();
    uint8_t rxbuf[255];
    uint8_t size=0;
    PacketStatus_t radiopkstatus;
    uint8_t xx[64] = {0x12, 0x22, 0x32, 0x42, 0x52, 0x62, 0x72, 0x82, 0x92};
    //uint8_t xor_xx[64]={0x66, 0xa5, 0xc3, 0x5a, 0x24, 0xff, 0x00, 0xbd, 0x24};// active whiten, the seed is 0x01ff,work normal
    

    //   uint8_t xor_xx[]={0x63, 0xac, 0xe8, 0x98, 0x09, 0x04, 0x32, 0x05, 0xd8,};//active whiten,the seed is 0x0000, work unnormal
    //   uint8_t xor_xx[]={0x63, 0xac, 0xfc, 0x98, 0x29, 0x04, 0x32, 0x05, 0xdc};//active whiten,the seed is 0x0000, work unnormal
    // uint8_t xor_xx[] = {0x63, 0xac, 0xe8, 0x98, 0x08, 0x04, 0x32, 0x05, 0xdc}; //active whiten,the seed is 0x0000, work unnormal
    // uint8_t xor_xx[] = {0x63, 0xac, 0xf8, 0x98, 0x48, 0x00, 0x32, 0x45, 0xd8}; //active whiten,the seed is 0x0000, work unnormal
                        


    // uint8_t xor_xx[]={0x63, 0xac, 0xfc, 0x98, 0x49, 0x04, 0x32, 0x05, 0xd8};// deactive whiten, the seed is 0x01ff
    // uint8_t xor_xx[]={0x63, 0xac, 0xf8, 0x98, 0x08, 0x04, 0x32, 0x05, 0xdc};// deactive whiten, the seed is 0x01ff
    // uint8_t xor_xx[]={0x63, 0xac, 0xf8, 0x98, 0x28, 0x04, 0x32, 0x45, 0xdc};// deactive whiten, the seed is 0x01ff
    // uint8_t xor_xx[]={0x63, 0xac, 0xe8, 0x98, 0x09, 0x04, 0x32, 0x04, 0xd8};// deactive whiten, the seed is 0x01ff

    //    uint8_t xor_xx[]={0x63, 0xac, 0xe8, 0x98, 0x09, 0x14, 0x32, 0x05, 0xd8};// deactive whiten, the seed is 0x0000
    //    uint8_t xor_xx[]={0x63, 0xac, 0xf8, 0x98, 0x09, 0x14, 0x32, 0x04, 0xd8};// deactive whiten, the seed is 0x0000


    //  RadioStandby( );
    SX126x.PacketParams.Params.Gfsk.HeaderType =   RADIO_PACKET_VARIABLE_LENGTH;
    SX126x.PacketParams.Params.Gfsk.DcFree = RADIO_DC_FREEWHITENING;
    SX126xSetPacketParams( &SX126x.PacketParams );

    SX126xClearIrqStatus(irqRegs);
    Radio.Rx(0);
    i=0;
    while (1)
    {
        irqRegs = SX126xGetIrqStatus();

        // printf("the Irq status is 0x%04x\n",irqRegs);
        if((irqRegs&IRQ_RX_DONE)==IRQ_RX_DONE)
        {
            // printf("receive packet successfully\r\n");
            GpioToggle(&Led2);
            SX126xGetPayload( rxbuf, &size , 255 );

            // if((size&0xf0)==0xf0)
                // size^=0xff;
            // printf("the length of receive buffer is %d\n",size);
            // if(size!=0x09)
            //     break;
            printf("receive packet successfully\r\n");
            printf("the length of receive buffer is %d\n",size);
            for (i = 0; i < size; i++)
            {
                 //printf("0x%02x  ",rxbuf[i]^g_xordata[i]);
                 printf("0x%02x  ",rxbuf[i]);
                // if(rxbuf[i]!=xx[i])
                // {
                //     printf("-------------------------------error------------------------\r\n");
                //     break;
                // }
            }
            // if(memcmp(rxbuf,xx,9)==0)
            // {
            //     printf("++++++++++++found the seed+++++++++++++++++++++++++++\n");
            //     while(1);
            // }

            // printf("\r\nthe rewhiten serial is:");

            // for (i = 0; i < size; i++)
            // {
            //      //printf("0x%02x  ",rxbuf[i]^g_xordata[i]);
            //     //  printf("0x%02x, ",rxbuf[i]^xx[i]);
            //     printf("0x%02x, ",rxbuf[i]^(i+1));
            //     // if(rxbuf[i]!=xx[i])
            //     // {
            //     //     printf("-------------------------------error------------------------\r\n");
            //     //     break;
            //     // }
            // }
            printf("\r\nthe rewhiten payload is:");

             for (i = 0; i < size; i++)
            {
                 //printf("0x%02x  ",rxbuf[i]^g_xordata[i]);
                 printf("0x%02x, ",rxbuf[i]^xorbuf_rx[i]);
                // if(rxbuf[i]!=xx[i])
                // {
                //     printf("-------------------------------error------------------------\r\n");
                //     break;
                // }
            }
            printf("\r\n");


            // printf("\r");
            SX126xGetPacketStatus(&radiopkstatus);
            //printf("the rssi is %d\n",radiopkstatus.Params.Gfsk.RssiAvg);
            //printf("the rssi is %d\n",radiopkstatus.Params.Gfsk.RssiSync);
           
            break;
        }
        else
        {
            i++;
            HAL_Delay(10);
            if(i>=500)
            {
                printf("receive timeout\r\n");
                break;
            }
        }
    }
    


}
/**
 * @brief fixed length packet receive
 * 
 */
void fix_len_packet_rx(void)
{
    int i;
    uint16_t irqRegs = SX126xGetIrqStatus();
    uint8_t rxbuf[255];
    uint8_t size=0;
    PacketStatus_t radiopkstatus;
    uint8_t xx[] = {0x12, 0x22, 0x32, 0x42, 0x52, 0x62, 0x72, 0x82, 0x92};
    //0x71  0x8e  0xde  0xda  0x5b  0x62  0x50  0x87  0x4a 
    SX126xClearIrqStatus(irqRegs);
    Radio.Rx(0);
    i=0;
    while (1)
    {
        irqRegs = SX126xGetIrqStatus();

        // printf("the Irq status is 0x%04x\n",irqRegs);
        if((irqRegs&IRQ_RX_DONE)==IRQ_RX_DONE)
        {
            printf("receive packet successfully\r\n");
            printf("the radio IRQ status is 0x%04x\n",irqRegs);

            GpioToggle(&Led2);
            SX126xGetPayload( rxbuf, &size , 255 );
            // size^=0xff;
            printf("the length of receive buffer is %d\n",size);
            for (i = 0; i < size; i++)
            {
                 //printf("0x%02x  ",rxbuf[i]^g_xordata[i]);
                 printf("0x%02x  ",rxbuf[i]);
                // if(rxbuf[i]!=xx[i])
                // {
                //     printf("-------------------------------error------------------------\r\n");
                //     break;
                // }
            }
            printf("\r\n");

            // received whiten data
            if ((rxbuf[0] & 0xf0) == 0xf0)
            {
                printf("\r\nthe dewhiten data is below:\r\n");
                for (i = 0; i < size; i++)
                {
                    //printf("0x%02x  ",rxbuf[i]^g_xordata[i]);
                    printf("0x%02x, ", rxbuf[i] ^ xorbuf[i]);
                    // if(rxbuf[i]!=xx[i])
                    // {
                    //     printf("-------------------------------error------------------------\r\n");
                    //     break;
                    // }
                }
                printf("\r\n");
            }

            // printf("\r");
            SX126xGetPacketStatus(&radiopkstatus);
            //printf("the rssi is %d\n",radiopkstatus.Params.Gfsk.RssiAvg);
            //printf("the rssi is %d\n",radiopkstatus.Params.Gfsk.RssiSync);
           
            break;
        }
        else
        {
            i++;
            HAL_Delay(10);
            if(i>=500)
            {
                printf("receive timeout\r\n");
                break;
            }
        }
    }

}
void register_test(void)
{
    //write register
    char xx[20];
    int a;
    printf("register write and read test\n");
    SX126xWriteRegister(0x06C0, 0x12);
    SX126xWriteRegister(0x06C1, 0x34);
    SX126xWriteRegister(0x06C2, 0x56);
    SX126xWriteRegister(0x06C3, 0x78);

    a=SX126xReadRegister(0x06C0);
    sprintf(xx,"the sync 0 is 0x%02x\n",a);
    printf(xx);

    a=SX126xReadRegister(0x06C1);
    sprintf(xx,"the sync 1 is 0x%02x\n",a);
    printf(xx);

    a=SX126xReadRegister(0x06C2);
    sprintf(xx,"the sync 2 is 0x%02x\n",a);
    printf(xx);

    a=SX126xReadRegister(0x06C3);
    sprintf(xx,"the sync 3 is 0x%02x\n",a);
    printf(xx);
}

/**
 * @brief crc16 for IBM
 * which can get the same result as si4463
 * the important factor is not reverse,start from the MSB
 * @param data
 * @param length 
 * @return unsigned int 
 */
unsigned int crc16(unsigned char *data, unsigned char length)
{
  int j;
  unsigned int reg_crc = 0xFFFF;
//   unsigned int reg_crc = 0x0000;
  // unsigned int reg_crc = 0x4230;
  while (length--)
  {

    reg_crc ^= (*data++<<8);
    for (j = 0; j < 8; j++)
    {
      if (reg_crc & 0x8000)
      {
        // reg_crc = (reg_crc >> 1) ^ 0xA001;
        reg_crc = (reg_crc << 1) ^ 0x8005;
      }
      else
      {
        reg_crc = reg_crc << 1;
      }
    }
  }
  reg_crc&=0xffff;
  return reg_crc;
}

/**
 * Main application entry point.
 */
int main_PingPang( void )
{
    bool isMaster = true;
    uint8_t i;

    // Target board initialization
    BoardInitMcu( );
    BoardInitPeriph( );

    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init( &RadioEvents );

    Radio.SetChannel( RF_FREQUENCY );

#if defined( USE_MODEM_LORA )

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

    Radio.SetMaxPayloadLength( MODEM_LORA, BUFFER_SIZE );

#elif defined( USE_MODEM_FSK )

    Radio.SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                                  FSK_DATARATE, 0,
                                  FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, 0, 3000 );

    Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                                  0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                                  0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                                  0, 0,false, true );

    Radio.SetMaxPayloadLength( MODEM_FSK, BUFFER_SIZE );

#else
    #error "Please define a frequency band in the compiler options."
#endif

    Radio.Rx( RX_TIMEOUT_VALUE );

    while( 1 )
    {
        switch( State )
        {
        case RX:
            if( isMaster == true )
            {
                if( BufferSize > 0 )
                {
                    if( strncmp( ( const char* )Buffer, ( const char* )PongMsg, 4 ) == 0 )
                    {
                        // Indicates on a LED that the received frame is a PONG
                        GpioToggle( &Led1 );

                        // Send the next PING frame
                        Buffer[0] = 'P';
                        Buffer[1] = 'I';
                        Buffer[2] = 'N';
                        Buffer[3] = 'G';
                        // We fill the buffer with numbers for the payload
                        for( i = 4; i < BufferSize; i++ )
                        {
                            Buffer[i] = i - 4;
                        }
                        DelayMs( 1 );
                        Radio.Send( Buffer, BufferSize );
                    }
                    else if( strncmp( ( const char* )Buffer, ( const char* )PingMsg, 4 ) == 0 )
                    { // A master already exists then become a slave
                        isMaster = false;
                        GpioToggle( &Led2 ); // Set LED off
                        Radio.Rx( RX_TIMEOUT_VALUE );
                    }
                    else // valid reception but neither a PING or a PONG message
                    {    // Set device as master ans start again
                        isMaster = true;
                        Radio.Rx( RX_TIMEOUT_VALUE );
                    }
                }
            }
            else
            {
                if( BufferSize > 0 )
                {
                    if( strncmp( ( const char* )Buffer, ( const char* )PingMsg, 4 ) == 0 )
                    {
                        // Indicates on a LED that the received frame is a PING
                        GpioToggle( &Led1 );

                        // Send the reply to the PONG string
                        Buffer[0] = 'P';
                        Buffer[1] = 'O';
                        Buffer[2] = 'N';
                        Buffer[3] = 'G';
                        // We fill the buffer with numbers for the payload
                        for( i = 4; i < BufferSize; i++ )
                        {
                            Buffer[i] = i - 4;
                        }
                        DelayMs( 1 );
                        Radio.Send( Buffer, BufferSize );
                    }
                    else // valid reception but not a PING as expected
                    {    // Set device as master and start again
                        isMaster = true;
                        Radio.Rx( RX_TIMEOUT_VALUE );
                    }
                }
            }
            State = LOWPOWER;
            break;
        case TX:
            // Indicates on a LED that we have sent a PING [Master]
            // Indicates on a LED that we have sent a PONG [Slave]
            GpioToggle( &Led2 );
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            if( isMaster == true )
            {
                // Send the next PING frame
                Buffer[0] = 'P';
                Buffer[1] = 'I';
                Buffer[2] = 'N';
                Buffer[3] = 'G';
                for( i = 4; i < BufferSize; i++ )
                {
                    Buffer[i] = i - 4;
                }
                DelayMs( 1 );
                Radio.Send( Buffer, BufferSize );
            }
            else
            {
                Radio.Rx( RX_TIMEOUT_VALUE );
            }
            State = LOWPOWER;
            break;
        case TX_TIMEOUT:
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;
        case LOWPOWER:
        default:
            // Set low power
            break;
        }

        BoardLowPowerHandler( );
        // Process Radio IRQ
        if( Radio.IrqProcess != NULL )
        {
            Radio.IrqProcess( );
        }
    }
}

void tx_cw(void)
{
    Radio.SetTxContinuousWave(RF_FREQUENCY,TX_OUTPUT_POWER,0);
}
