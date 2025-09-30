//extern.h
typedef unsigned char	byte;
typedef unsigned short	word;
typedef unsigned long	dword;

extern byte adc_cnt,Power_5V_OK;
extern unsigned long int ADC_Value_0;

extern void LCD_Tx(byte lcd_tx_num);

extern byte Rx1_buf[10];
extern word CheckSum_data;
extern word Fire_Detector_Data_crc, P_Type_Reciever_Data_crc;
extern byte CheckSum_ing;
extern byte Rx1_step,Rx1_index;
extern byte Rx1_CRC_H, Rx1_CRC_L,CheckSum_EN;
extern word Rcv1_ok;
extern byte rx1_data;
extern byte Tx_buffer[7];
extern byte Tx_Fg_Num;

extern byte Fire_Fg,Equipment_Num;

extern word CRC16(byte *buf, word size);

extern void P_Type_Reciever_Tx(byte p_reciever__tx_num);	//UART1