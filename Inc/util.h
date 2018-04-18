/* util.h */
#ifndef UTIL_H
#define UTIL_H

#include "stm32f4xx_hal.h"

void to_hex(char byte, char *buf);

void Serial_print(char cadena [], UART_HandleTypeDef *huart4);
void Serial_println(char cadena [], UART_HandleTypeDef *huart4);

void Serial_print_N(int number, UART_HandleTypeDef *huart4);
void Serial_println_N(int number, UART_HandleTypeDef *huart4);

void Serial_println_lN(unsigned long int number, UART_HandleTypeDef *huart4);

void Serial_print_f(float number, UART_HandleTypeDef *huart4);
void Serial_println_f(float number, UART_HandleTypeDef *huart4);
	
int decimales_l (unsigned long number);
int decimales (unsigned int number);

#endif /* UTIL_H */
