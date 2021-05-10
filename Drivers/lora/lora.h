#ifndef __LORA__H__
#define __LORA__H__
//application
/**
 * @brief transmit continuous wave(single wave)
 * 
 */
void tx_cw(void);
void packet_init(void);
/**
 * @brief transmit packet
 * 
 */
void packet_tx(void);
/**
 * @brief receive packet
 * 
 */
void packet_rx(void);
void register_test(void);
#endif