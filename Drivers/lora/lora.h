#ifndef __LORA__H__
#define __LORA__H__
//application
/**
 * @brief transmit continuous wave(single wave)
 * 
 */
void tx_cw(void);
void packet_init(void);

void packet_tx(void);
void fix_lenth_packet_tx_crc_whiten(void);
/**
 * @brief receive packet
 * 
 */
void packet_rx(void);
void fix_len_packet_rx(void);
void register_test(void);
unsigned int crc16(unsigned char *data, unsigned char length);
void assembly_package(unsigned char  *p,unsigned char  len);
#endif