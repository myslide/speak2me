/*!
 * @file DFPlayer.c
 * @brief DFPlayer - An Mini MP3 Player driver for PSoC4 from mysli
 * @n Header file for DFPlayer
 *
 * @copyright	[DFRobot]( http://www.dfrobot.com ), 2016
 * @copyright   [mysli](http://www.mysli.de),2023
 * @copyright	GNU Lesser General Public License
 *
 * @author [Angelo](Angelo.qiao@dfrobot.com), mysli
 * @version  V0.0.1
 * @date  2023-02-15
 */

#include "DFPlayer.h"

  uint8_t _received[DFPLAYER_RECEIVED_LENGTH];
  uint8_t _sending[DFPLAYER_SEND_LENGTH] = {0x7EU, 0xFFU, 0x06U, 00U, 00U, 00U, 00U, 00U, 00U, 0xEFU};
  uint8_t device = DFPLAYER_DEVICE_SD;
  CySCB_Type *_scb_Port;
  /*******************************************************************************
   * Global Variables
   *******************************************************************************/
  cy_stc_scb_uart_context_t CYBSP_UART_context;

//private:
void _transmitStack(){
	Cy_SCB_UART_PutArrayBlocking(_scb_Port, _sending, DFPLAYER_SEND_LENGTH);
	/* Blocking wait for transfer completion */
	while (!Cy_SCB_UART_IsTxComplete(_scb_Port)) {
	}
  delay(5);
}

void _uint16ToArray(uint16_t value, uint8_t *array){
  *array = (uint8_t)(value>>8);
  *(array+1) = (uint8_t)(value);
}

uint16_t _calculateCheckSum(uint8_t *buffer){
  uint16_t sum = 0;
  for (int i=Stack_Version; i<Stack_CheckSum; i++) {
    sum += buffer[i];
  }
  return -sum;
}

void _sendStack(uint8_t command, uint16_t argument){
  _sending[Stack_Command] = command;
  _uint16ToArray(argument, _sending+Stack_Parameter);
  _uint16ToArray(_calculateCheckSum(_sending), _sending+Stack_CheckSum);
  _transmitStack();
  delay(100);
}

//public

cy_en_scb_uart_status_t dfp_init(CySCB_Type *scb_Port,cy_stc_scb_uart_config_t *uart_config){
	cy_en_scb_uart_status_t initstatus;
	initstatus=Cy_SCB_UART_Init(scb_Port, uart_config,
				&CYBSP_UART_context);
    Cy_SCB_UART_Enable(scb_Port);
    _scb_Port=scb_Port;
    return initstatus;
}

void delay(uint32_t milliseconds){
  Cy_SysLib_Delay(milliseconds);
}

void dfp_play(uint8_t fileNumber){
  _sendStack(0x03, fileNumber);
}

void dfp_stop(){
  _sendStack(0x16,0);
}

void dfp_volume(uint8_t volume){
  _sendStack(0x06, volume);
}
