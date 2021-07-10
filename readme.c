void fix_lenth_packet_tx_crc_whiten(void);
void packet_rx(void);

unsigned int crc16(unsigned char *data, unsigned char length);
void assembly_package(unsigned char  *p,unsigned char  len);


uint8_t xorbuf[]={0xff,0xe1 ,0x1d ,0x9a ,0xed ,0x85 ,0x33 ,0x24 ,0xea ,0x7a ,0xd2 ,0x39 ,0x70 ,0x97 ,0x57 ,0x0a ,0x54 ,0x7d ,0x2d ,0xd8 ,0x6d ,0x0d ,0xba ,0x8f ,0x67 ,0x59 ,0xc7 ,0xa2 ,0xbf ,0x34 ,0xca ,0x18 ,0x30 ,0x53 ,0x93 ,0xdf ,0x92 ,0xec ,0xa7 ,0x15 ,0x8a ,0xdc ,0xf4 ,0x86 ,0x55 ,0x4e ,0x18 ,0x21 ,0x40 ,0xc4 ,0xc4 ,0xd5 ,0xc6 ,0x91 ,0x8a ,0xcd ,0xe7 };

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
    uint8_t xor_xx[]={0x66, 0xa5, 0xc3, 0x5a, 0x24, 0xff, 0x00, 0xbd, 0x24, 0x99,\
                      0xa5, 0x7e, 0x7e, 0xbd, 0x5a, 0x7e, 0xc3, 0x99, 0xc3, 0xdb,\
                      0xbd, 0xe7, 0x7e, 0x81, 0xc3, 0x24, 0xe7, 0x42, 0x18, 0x99,\
                      0x00, 0x3c, 0x99, 0x5a, 0x24, 0xdb, 0xdb, 0x42, 0xbd, 0xdb,\
                      0xe7, 0xdb, 0xe7, 0xff, 0x3c, 0x00, 0xa5, 0x42, 0xe7, 0xe7, \
                      0x7e, 0xa5, 0x18, 0xdb, 0x7e, 0x00, 0x5a, 0x3c, 0x99, 0x7e};

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
                 printf("0x%02x, ",rxbuf[i]^xor_xx[i]);
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

// 发射函数
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
    RadioStandby( );
    SX126x.PacketParams.Params.Gfsk.HeaderType =   RADIO_PACKET_VARIABLE_LENGTH;
    SX126x.PacketParams.Params.Gfsk.DcFree = RADIO_DC_FREEWHITENING;
    SX126xSetPacketParams( &SX126x.PacketParams );
    

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

