/**
  ******************************************************************************
  * @file:      sys_command_line.c
  * @author:    Cat
  * @version:   V1.0
  * @date:      2018-1-18
  * @brief:     command line
  * @attention:
  * 
  *             V0.5: [add] colorful log print API
  *             V0.6: [add] history command support
  * 
  ******************************************************************************
  */


#include "sys_command_line.h"
//#include "./../bsp/bsp_usart.h"
#include "app_led.h"
#include "main.h"
//#include "bootloader.h"

static uint8_t cli_help(void *para, uint8_t len);
static uint8_t cli_clear(void *para, uint8_t len);
static uint8_t cli_echo(void *para, uint8_t len);
static uint8_t cli_reboot(void *para, uint8_t len);

// extern unsigned char cli_rx;

__packed typedef struct {
#define HANDLE_LEN 128

    uint8_t buff[HANDLE_LEN];
    uint8_t len;
} HANDLE_TYPE_S;

header_union_t  m_BinHeader;
OTA_STATUS_t ota_stas=OTA_STOP;
uint16_t binSize;
uint32_t binCRC;
uint16_t binReceived;
uint32_t dwPolynomial = 0x04c11db7;

uint8_t cli_echo_flag = DISABLE; /* ECHO default: disable */

RX_BUFF_TYPE cli_rx_buff; /* 128byte FIFO, saving commands from the terminal */



const char CLI_Cmd_Help[] =
    "\r\n"
    "[help]\r\n"
    " * show commands\r\n"
    "\r\n";

const char CLI_Cmd_Clear[] =
    "[cls]\r\n"
    " * clear the screen\r\n"
    "\r\n";

const char CLI_Cmd_Echo[] =
    "[echo]\r\n"
    " * echo 1: echo on\r\n"
    " * echo 0: echo off\r\n"
    "\r\n";

const char CLI_Cmd_Reboot[] =
    "[reboot]\r\n"
    " * reboot MCU\r\n"
    "\r\n";


/**
  * cmd struct
  */
const COMMAND_S CLI_Cmd[] = {
    /* cmd              cmd help            init func.      func. */
    // {"help",            CLI_Cmd_Help,       NULL,           cli_help},
    // {"cls",             CLI_Cmd_Clear,      NULL,           cli_clear},
    // {"echo",            CLI_Cmd_Echo,       NULL,           cli_echo},
    {"reboot", CLI_Cmd_Reboot, junfeng_pInit, cli_reboot},

    /* application cmd start */
    //{"led",           CLI_Cmd_LED,          CLI_LED_Init,   CLI_LED},
    {"junfeng", "junfeng is a good boy\n", junfeng_pInit, junfeng_pFun},
    // {"writeReg",           "write reg\n",          NULL,   WriteReg_pFun},
    // {"ReadReg",           "read reg\n",          NULL,   ReadReg_pFun},
    // {"ReadInputReg",           "read input reg\n",          NULL,   ReadInputReg_pFun},
    // {"otaneed",           "firmwar ota\n",          NULL,   Fireware_OTA_pFun},
    // {"nic_tx", "harmwee nic rf tx test\n", NULL, nic_tx_pFun},
    // {"nic_rx", "harmwee nic rf rx test\n", NULL, nic_rx_pFun},
    // {"nic_reset", "harmwee nic chip reset\n", NULL, nic_reset_pFun},
    {"reset_sx126x", "input the reset the sx126x command\n", NULL, cli_reset_sx126x},
    

};

// additional  20200403
uint8_t junfeng_pInit(void)
{
    printf("this is junfeng init function of cli\n");
    return 0x01;
}
uint8_t junfeng_pFun(void *p, uint8_t len)
{
    printf("this is junfeng execute function of cli\n");
    return 0x00;
}
uint8_t nic_tx_pFun(void * p, uint8_t len)
{
    nic_enter_tx();
    PRINTF("harmwee nic enter rf tx test mode\n");
    return 0x00;
}
uint8_t nic_rx_pFun(void * p, uint8_t len)
{
    nic_enter_rx();
    PRINTF("harmwee nic enter rf rx test mode\n");
    return 0x00;
}
uint8_t nic_reset_pFun(void * p, uint8_t len)
{
    nic_enter_reset();
    PRINTF("harmwee nic enter rf reset mode\n");
    return 0x00;
}

/**
 * @brief reset the sx126x
 * 
 * @param p 
 * @param len 
 * @return uint8_t 
 */
extern void packet_init(void);
extern uint8_t g_nOpMode;
uint8_t cli_reset_sx126x(void * p, uint8_t len)
{
    PRINTF("reset the sx126x\n");
    g_nOpMode=0;
    packet_init();
    return 0x00;
}
/**
  * @brief  printf the help info.
  * @param  para addr. & length
  * @retval True means OK
  */

static uint8_t cli_help(void *para, uint8_t len)
{
    uint8_t i;

    for(i = 0; i < sizeof(CLI_Cmd) / sizeof(COMMAND_S); i++) {
        if (NULL != CLI_Cmd[i].pHelp) {
            PRINTF(CLI_Cmd[i].pHelp);
        }
    }

    return TRUE;
}


/**
  * @brief  clear the screen
  * @param  para addr. & length
  * @retval True means OK
  */
static uint8_t cli_clear(void *para, uint8_t len)
{
    TERMINAL_BACK_BLACK(); /* set terminal background color: black */
    TERMINAL_FONT_GREEN(); /* set terminal display color: green */

    /* This prints the clear screen and move cursor to top-left corner control
     * characters for VT100 terminals. This means it will not work on
     * non-VT100 compliant terminals, namely Windows' cmd.exe, but should
     * work on anything unix-y. */
    TERMINAL_RESET_CURSOR();
    TERMINAL_DISPLAY_CLEAR();

    return TRUE;
}


/**
  * @brief  ECHO setting
  * @param  para addr. & length
  * @retval True means OK
  */
static uint8_t cli_echo(void *para, uint8_t len)
{
    uint8_t *pTemp;
    pTemp = (uint8_t *)para;

    if((0 < len) && (NULL != pTemp)) {
        pTemp++; /* skip a blank space*/

        if('1' == *pTemp) {
            /* ECHO on */
            cli_echo_flag = ENABLE;
            PRINTF("echo on\r\n");
        } else if('0' == *pTemp) {
            /* ECHO off */
            cli_echo_flag = DISABLE;
            PRINTF("echo off\r\n");
        } else {
            /* wrong para, return False */
            return FALSE;
        }
    }

    return TRUE;
}


/**
  * @brief  MCU reboot
  * @param  para addr. & length
  * @retval True means OK
  */
static uint8_t cli_reboot(void *para, uint8_t len)
{
    extern void Delay(__IO uint32_t nCount);
    PRINTF("\r\n[END]: System Rebooting");
    PRINTF(".");
    Delay(0x8FFFFF);
    PRINTF(".");
    Delay(0x8FFFFF);
    PRINTF(".");

    SYSTEM_REBOOT();

    return TRUE;
}


#if CLI_HISTORY

__packed typedef struct {
    char cmd[HISTORY_MAX][HANDLE_LEN];
    uint8_t count;
    uint8_t latest;
    uint8_t show;
}HISTORY_S;

static HISTORY_S history;


/**
  * @brief          add a history command
  * @param  buff:   command
  * @retval         null
  */
static void cli_history_add(char* buff)
{
    uint16_t len;
    uint8_t index = history.latest;

    if (NULL == buff) return;

    len = strlen((const char *)buff);
    if (len >= HANDLE_LEN) return;  /* command too long */

    /* find the latest one */
    if (0 != index) {
        index--;
    } else {
        index = HISTORY_MAX - 1;
    }

    if (0 != memcmp(history.cmd[index], buff, len)) {
        /* if the new one is different with the latest one, the save */
        memset((void *)history.cmd[history.latest], 0x00, HANDLE_LEN);
        memcpy((void *)history.cmd[history.latest], (const void *)buff, len);
        if (history.count < HISTORY_MAX) {
            history.count++;
        }

        history.latest++;
        if (history.latest >= HISTORY_MAX) {
            history.latest = 0;
        }
    }

    history.show = 0;
}


/**
  * @brief              display history command
  * @param  mode:       TRUE for look up, FALSE for look down
  * @param  p_history:  target history command
  * @retval             TRUE for no history found, FALSE for success
  */
static uint8_t cli_history_show(uint8_t mode, char** p_history)
{
    uint8_t err = TRUE;
    uint8_t num;
    uint8_t index;

    if (0 == history.count) return err;

    if (TRUE == mode) {
        /* look up */
        if (history.show < history.count) {
            history.show++;
        }
    } else {
        /* look down */
        if (1 < history.show) {
            history.show--;
        }
    }

    num = history.show;
    index = history.latest;
    while (num) {
        if (0 != index) {
            index--;
        } else {
            index = HISTORY_MAX - 1;
        }
        num--;
    }

    err = FALSE;
    *p_history = history.cmd[index];
    //PRINTF("history: %s \r\n", history.cmd[index]);

    return err;
}

#endif  /* CLI_HISTORY */


/**
  * @brief  command line init.
  * @param  bandrate
  * @retval null
  */
void cli_init(uint32_t baud)
{
    uint8_t i;

    memset((uint8_t *)&cli_rx_buff, 0, sizeof(RX_BUFF_TYPE));

#if CLI_HISTORY
    memset((uint8_t *)&history, 0, sizeof(history));
#endif  /* CLI_HISTORY */
    // PRINTF("    Email: 843553493@qq.com    \r\n\r\n");
    //USART_INIT(baud);

    /* init. every command */
    for(i = 0; i < sizeof(CLI_Cmd) / sizeof(COMMAND_S); i++) {
        /* need to init. or not */
        if(NULL != CLI_Cmd[i].pInit) {
            if(FALSE == CLI_Cmd[i].pInit()) {
                /* something wrong */
                PRINTF("\r\n-> FUN[%d] INIT WRONG\r\n", i);
            }
        }
    }

    PRINTF(" \r\n");
    // TERMINAL_BACK_BLACK(); /* set terminal background color: black */
    // TERMINAL_FONT_GREEN(); /* set terminal display color: green */
    // TERMINAL_DISPLAY_CLEAR();
    // TERMINAL_RESET_CURSOR();

    // PRINTF_COLOR(E_FONT_YELLOW, "-------------------------------\r\n\r\n");
    // TERMINAL_HIGH_LIGHT();
    PRINTF("    CLI version: V0.1          \r\n\r\n");
    PRINTF("    coder: Cat                 \r\n\r\n");
    PRINTF("    Email: junfeng.wei@honestar.com    \r\n\r\n");
    // TERMINAL_UN_HIGH_LIGHT();
    // PRINTF_COLOR(E_FONT_YELLOW, "-------------------------------\r\n\r\n");
}


/**
  * @brief  handle commands from the terminal
  * @param  commands
  * @retval null
  */
static void cli_rx_handle(RX_BUFF_TYPE *rx_buff)
{
    static HANDLE_TYPE_S Handle = {.len = 0};
    uint8_t i = Handle.len;
    uint8_t ParaLen;
    uint8_t *ParaAddr;
    uint8_t cmd_match = FALSE;

    /*  ---------------------------------------
        Step1: save chars from the terminal
        ---------------------------------------
     */
    // if(cli_rx==0)
    //     return;

    while (1)
    {

        if (Handle.len < HANDLE_LEN)
        { /* check the buffer */

            /* new char coming from the terminal, copy it to Handle.buff */
            if (TRUE == QUEUE_OUT((*rx_buff), Handle.buff[Handle.len]))
            {
                Handle.len++;
            }
            else
            {
                /* all chars copied to Handle.buff */

                /* display char in terminal */
                for (; i < Handle.len; i++)
                {
                    //USART_SendData(DEBUG_USARTx, Handle.buff[i]);
                    //Tx_Byte(Handle.buff[i]);
                    // Tx_Byte_To_Cat(Handle.buff[i]);
                }

                break;
            }
        }
        else
        {
            /* buffer full */
            break;
        }
    }

    /*  ---------------------------------------
        Step2: handle the commands
        ---------------------------------------
     */
    if ((1 == Handle.len) && (KEY_ENTER == Handle.buff[Handle.len - 1]))
    {
        /* KEY_ENTER -->ENTER key from terminal */
        Handle.len = 0;
    }
    else if (1 < Handle.len)
    { /* check for the length of command */
        /* a command must ending with KEY_ENTER */
        if (KEY_ENTER == Handle.buff[Handle.len - 1])
        {
            Handle.buff[Handle.len - 1] = '\0';
            // printf("the last chareacter is enter--\n");
            /* looking for a match */
            for (i = 0; i < sizeof(CLI_Cmd) / sizeof(COMMAND_S); i++)
            {
                if (0 == strncmp((const char *)Handle.buff,
                                 (void *)CLI_Cmd[i].pCmd,
                                 strlen(CLI_Cmd[i].pCmd)))
                {
                    cmd_match = TRUE;
                    ParaLen = Handle.len - strlen(CLI_Cmd[i].pCmd);   /* para. length */
                    ParaAddr = &Handle.buff[strlen(CLI_Cmd[i].pCmd)]; /* para. address */

                    if (NULL != CLI_Cmd[i].pFun)
                    {
                        // printf("call junfeng function\n");
                        /* call the func. */
                        if (CLI_Cmd[i].pFun(ParaAddr, ParaLen))
                        {
                            PRINTF("\r\n-> OK\r\n");

                            /* ECHO */
                            if (ENABLE == cli_echo_flag)
                            {
                                Handle.buff[Handle.len] = '\0';
                                //PRINTF("[echo]: %s\r\n", (const char*)Handle.buff);
                                printf("001\n");
                            }
                        }
                        else
                        {
                            PRINTF("\r\n-> PARA. ERR\r\n");
                            /* parameter wrong */
                            PRINTF(CLI_Cmd[i].pHelp);
                        }
                    }
                    else
                    {
                        /* func. is void */
                        PRINTF("\r\n-> FUNC. ERR\r\n");
                    }
                }
            }

            if (FALSE == cmd_match)
            {
                /* no matching command */
                PRINTF("\r\n-> CMD ERR, try: help\r\n\r\n");
            }

            Handle.len = 0;
        }
    }

    if(Handle.len >= HANDLE_LEN) {
        /* full, so restart the count */
        Handle.len = 0;
    }

    // cli_rx=0;
}

/**
  * @brief  handle commands from the terminal
  * @param  commands
  * @retval null
  */
static void cli_rx_handle_raw(RX_BUFF_TYPE *rx_buff)
{
    static HANDLE_TYPE_S Handle = {.len = 0};
    uint8_t i = Handle.len;
    uint8_t ParaLen;
    uint8_t *ParaAddr;
    uint8_t cmd_match = FALSE;

    /*  ---------------------------------------
        Step1: save chars from the terminal
        ---------------------------------------
     */
    while(1) {

        if(Handle.len < HANDLE_LEN) {  /* check the buffer */

            /* new char coming from the terminal, copy it to Handle.buff */
            if(TRUE == QUEUE_OUT((*rx_buff), Handle.buff[Handle.len])) {
                /* KEY_BACKSPACE -->get DELETE key from keyboard */
                // if (KEY_BACKSPACE == Handle.buff[Handle.len]) {
                //     /* buffer not empty */
                //     if (0 < Handle.len) {
                //         /* delete a char in terminal */
                //         TERMINAL_MOVE_LEFT(1);
                //         TERMINAL_CLEAR_END();
                //         Handle.len -= 1;
                //     }

                // } else {
                    //PRINTF("%02x ", Handle.buff[Handle.len]); /* debug */
                    Handle.len++;
                // }

            } else {
                /* all chars copied to Handle.buff */
#if CLI_HISTORY
                uint8_t key = 0;
                uint8_t err = 0xff;
                char *p_hist_cmd = 0;

                if (Handle.len > 2) {
                    if (0 != strstr((const char *)Handle.buff, KEY_UP)) {
                        //PRINTF("KEY_UP \r\n");
                        key = 1;
                        TERMINAL_MOVE_LEFT(Handle.len);
                        TERMINAL_CLEAR_END();
                        err = cli_history_show(TRUE, &p_hist_cmd);
                    } else if (0 != strstr((const char *)Handle.buff, KEY_DOWN)) {
                        //PRINTF("KEY_DOWN \r\n");
                        key = 2;
                        TERMINAL_MOVE_LEFT(Handle.len);
                        TERMINAL_CLEAR_END();
                        err = cli_history_show(FALSE, &p_hist_cmd);
                    } else if (0 != strstr((const char *)Handle.buff, KEY_RIGHT)) {
                        //PRINTF("KEY_RIGHT \r\n");
                        key = 3;
                    } else if (0 != strstr((const char *)Handle.buff, KEY_LEFT)) {
                        //PRINTF("KEY_LEFT \r\n");
                        key = 4;
                    }

                    if (0 != key) {
                        if (FALSE == err) {
                            memset(&Handle, 0x00, sizeof(Handle));
                            memcpy(Handle.buff, p_hist_cmd, strlen(p_hist_cmd));
                            Handle.len = strlen(p_hist_cmd);
                            Handle.buff[Handle.len] = '\0';
                            PRINTF("%s", Handle.buff);  /* display history command */
                        } else if ((TRUE == err) || (0 != key)) {
                            /* no history found */
                            TERMINAL_MOVE_LEFT(Handle.len);
                            TERMINAL_CLEAR_END();
                            memset(&Handle, 0x00, sizeof(Handle));
                        }
                    }
                }

                if ((0 == key) && (i < Handle.len)) {
#endif  /* CLI_HISTORY */
                    /* display char in terminal */
                    for (; i < Handle.len; i++) {
                        //USART_SendData(DEBUG_USARTx, Handle.buff[i]);
                        //Tx_Byte(Handle.buff[i]);
                        Tx_Byte_To_Cat(Handle.buff[i]);
                    }
#if CLI_HISTORY
                }
#endif  /* CLI_HISTORY */
                break;
            }

        } else {
            /* buffer full */
            break;
        }
    }

    /*  ---------------------------------------
        Step2: handle the commands
        ---------------------------------------
     */
    if((1 == Handle.len) && (KEY_ENTER == Handle.buff[Handle.len - 1])) {
        /* KEY_ENTER -->ENTER key from terminal */
        Handle.len = 0;
    } else if(1 < Handle.len) {  /* check for the length of command */
        /* a command must ending with KEY_ENTER */
        if(KEY_ENTER == Handle.buff[Handle.len - 1]) {
            Handle.buff[Handle.len - 1] = '\0';
            printf("the last chareacter is enter--\n");
            /* looking for a match */
            for(i = 0; i < sizeof(CLI_Cmd) / sizeof(COMMAND_S); i++) {
                if(0 == strncmp((const char *)Handle.buff,
                                (void *)CLI_Cmd[i].pCmd,
                                strlen(CLI_Cmd[i].pCmd))) {
                    cmd_match = TRUE;
                    ParaLen = Handle.len - strlen(CLI_Cmd[i].pCmd);   /* para. length */
                    ParaAddr = &Handle.buff[strlen(CLI_Cmd[i].pCmd)]; /* para. address */

                    if(NULL != CLI_Cmd[i].pFun) {
                        printf("call junfeng function\n");
                        /* call the func. */
                        if(CLI_Cmd[i].pFun(ParaAddr, ParaLen)) {
                            PRINTF("\r\n-> OK\r\n");

#if CLI_HISTORY
                            cli_history_add((char *)Handle.buff);
#endif  /* CLI_HISTORY */

                            /* ECHO */
                            if(ENABLE == cli_echo_flag) {
                                Handle.buff[Handle.len] = '\0';
                                //PRINTF("[echo]: %s\r\n", (const char*)Handle.buff);
                                printf("001\n");
                            }
                        } else {
                            PRINTF("\r\n-> PARA. ERR\r\n");
                            /* parameter wrong */
                            PRINTF(CLI_Cmd[i].pHelp);
                        }
                    } else {
                        /* func. is void */
                        PRINTF("\r\n-> FUNC. ERR\r\n");
                    }
                }
            }

            if(FALSE == cmd_match) {
                /* no matching command */
                PRINTF("\r\n-> CMD ERR, try: help\r\n\r\n");
            }

            Handle.len = 0;

        }

    }


    if(Handle.len >= HANDLE_LEN) {
        /* full, so restart the count */
        Handle.len = 0;
    }
}

/**
  * @brief  tx handle
  * @param  null
  * @retval null
  */
static void cli_tx_handle(void)
{
    /* not used for now */
}


/**
  * @brief  command line task, schedule by sys. every 50ms
  * @param  null
  * @retval null
  */
void cli_run(void)
{
    cli_rx_handle(&cli_rx_buff);
    cli_tx_handle();
    // cli_ota(&cli_rx_buff);
}


/**
  * @brief  handle commands from the terminal
  * @param  commands
  * @retval null
  */
// void cli_ota(RX_BUFF_TYPE *rx_buff)
// {
//     static HANDLE_TYPE_S Handle = {.len = 0};
//     uint8_t i = Handle.len;
//     uint8_t len;
//     uint32_t crc;
//     int rs;
//     int64_t data;
//     int k;
//     /*  ---------------------------------------
//         Step1: save chars from the terminal
//         ---------------------------------------
//      */
//     // if(cli_rx==0)
//     //     return;

//     while (1)
//     {

//         if (Handle.len < HANDLE_LEN)
//         { /* check the buffer */

//             /* new char coming from the terminal, copy it to Handle.buff */
//             if (TRUE == QUEUE_OUT((*rx_buff), Handle.buff[Handle.len]))
//             {
//                 Handle.len++;
//             }
//             else
//             {
//                 /* all chars copied to Handle.buff */

//                 /* display char in terminal */
//                 for (; i < Handle.len; i++)
//                 {
//                     //USART_SendData(DEBUG_USARTx, Handle.buff[i]);
//                     //Tx_Byte(Handle.buff[i]);
//                     // Tx_Byte_To_Cat(Handle.buff[i]);
//                 }

//                 break;
//             }
//         }
//         else
//         {
//             /* buffer full */
//             break;
//         }
//     }

//     /*  ---------------------------------------
//         Step2: handle the commands
//         ---------------------------------------
//      */
//     if ((1 == Handle.len) && (KEY_ENTER == Handle.buff[Handle.len - 1]))
//     {
//         /* KEY_ENTER -->ENTER key from terminal */
//         Handle.len = 0;
//     }
//     else if (1 < Handle.len)
//     {
//         /* receive packet 10ms before */
//         if (uUserTick >= 10)
//         {
//             printf("receive packaet 10ms before\n");
//             Handle.buff[Handle.len] = '\0';
//             PRINTF("[echo]: %s\r\n", (const char *)Handle.buff);
//             len = Handle.len;

//             // receive ota request
//             if ((len == 3) && strncmp("ota", Handle.buff, 3) == 0)
//             {
//                 // if (ota_stas == OTA_RECEIVE_REQUEST)
//                 // {
//                 //     if (uUserTick1 >= 100)
//                 //     {
//                 //         ota_stas = OTA_STOP;
//                 //     }
//                 // }

//                 if (ota_stas == OTA_STOP)
//                 {
//                     ota_stas = OTA_RECEIVE_REQUEST;
//                     PRINTF("receive ota request\n");
//                     uUserTick1 = 0;

//                     //erase the flash
//                     Bootloader_Init();
//                     rs = Bootloader_Erase();
//                     if (rs != BL_OK)
//                     {
//                         PRINTF("erase flash failure\n");
//                     }
//                     else
//                     {
//                         PRINTF("erase flash successfully\n");
//                     }
//                 }

//                 Handle.len = 0;
//                 return;
//             }

//             // receive header
//             if (ota_stas == OTA_RECEIVE_REQUEST && len == 16)
//             {
//                 memcpy(m_BinHeader.temp, Handle.buff, 16);
//                 crc = CalcCRC32(m_BinHeader.temp, 0, 12, 0xffffffff);
//                 if (crc == m_BinHeader.header.header_crc)
//                 {
//                     PRINTF("receive header\n");
//                     ota_stas = OTA_RECEIVE_HEADER;
//                     binSize = m_BinHeader.header.fw_size;
//                     binCRC = m_BinHeader.header.fw_crc;
//                     binReceived = 0;

//                     //read to write flash
//                     Bootloader_FlashBegin();
//                 }

//                 Handle.len = 0;
//                 return;
//             }

//             // receive bin
//             if (ota_stas == OTA_RECEIVE_HEADER)
//             {
//                 //save bin to flash
//                 binReceived += len;

//                 // write block to flash
//                 for (k = 0; k < len; k += 8)
//                 {
//                     memset((void *)&data, 0xff, 8);

//                     if ((k + 8) > len)
//                         memcpy((void *)&data, Handle.buff + k, len - k);
//                     else
//                         memcpy((void *)&data, Handle.buff + k, 8);

//                     rs = Bootloader_FlashNext(data);
//                     // rs = BL_OK;
//                     if (rs != BL_OK)
//                     {
//                         printf("write flash block %d error\n", i / 8);
//                     }
//                 }

//                 printf("receive bin is %d\n", binReceived);
//                 if (binReceived == binSize)
//                 {
//                     printf("receive all bin file\n");
//                     ota_stas = OTA_STOP;

//                     //write flash finished
//                     Bootloader_FlashEnd();
//                 }

//                 Handle.len = 0;
//                 return;
//             }

//             Handle.len = 0;
//         }
//     }

//     if (Handle.len >= HANDLE_LEN)
//     {
//         /* full, so restart the count */
//         Handle.len = 0;
//     }

//     cli_rx = 0;
// }

/**
 * @brief the CRC32
 * 
 * @param ptr 
 * @param startIndex 
 * @param len 
 * @param seed 
 * @return uint32_t 
 */
uint32_t CalcCRC32(uint8_t *ptr, uint32_t startIndex, uint32_t len,int32_t seed)
{
    uint32_t  xbit;
    uint32_t  data;
    uint32_t  crc ;//= 0xFFFFFFFF; // init
    int i;
    int bits;
    crc=seed;
    for (i = startIndex; i < startIndex + len; i += 4)
    {
        xbit = 1u << 31;

        data = (ptr[i + 3]) | (ptr[i + 2] << 8) | (ptr[i + 1] << 16) | (ptr[i] << 24);
        for (bits = 0; bits < 32; bits++)
        {
            if ((crc & 0x80000000) != 0)
            {
                crc <<= 1;
                crc ^= dwPolynomial;
            }
            else
                crc <<= 1;

            if ((data & xbit) != 0)
                crc ^= dwPolynomial;

            xbit >>= 1;
        }
    }
    return crc;
}




