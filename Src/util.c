#include "util.h"
#include "main.h"		
#include <stdio.h>

void to_hex(char byte, char *buf)
{
	int i;
	char nibbles[2];

	nibbles[0] = (byte & 0xF0) >> 4;
	nibbles[1] = (byte & 0x0F);

	for (i = 0; i < 2; i++) {
		if (nibbles[i] < 10) {
			buf[i] = '0' + nibbles[i];
		} else {
			buf[i] = 'A' + nibbles[i] - 10;
		}
	}
	buf[2] = '\0';
}

void Serial_println(char cadena [], UART_HandleTypeDef *huart4)
{
	int i = 0;
	uint8_t CR = '\n';
	
	while(cadena[i]!='\n')
	{
		uint8_t temp = cadena[i];
		HAL_UART_Transmit(huart4, &temp, sizeof(cadena[i]), 100);
		i++;
	}
	HAL_UART_Transmit(huart4, &CR, sizeof(CR), 100);
	
}
		
void Serial_print(char cadena [], UART_HandleTypeDef *huart4)
{
	int i = 0;
	
	while(cadena[i]!='\n')
	{
		uint8_t temp = cadena[i];
		HAL_UART_Transmit(huart4, &temp, sizeof(cadena[i]), 100);
		i++;
	}			
	
}

void Serial_println_N(int number, UART_HandleTypeDef *huart4)
{
	int size = number/10;
	char str[size+1];
	sprintf(str, "%d\n", number);
	Serial_println(str, huart4);
	
}

void Serial_print_N(int number, UART_HandleTypeDef *huart4)
{
	int size = decimales(number);
	char str[size+1];
	sprintf(str, "%d\n", number);
	Serial_print(str, huart4);
	
}	

void Serial_println_lN(unsigned long number, UART_HandleTypeDef *huart4)
{
	int j = decimales_l(number);
	char str[j+1];
	sprintf(str, "%lu\n", number);
	Serial_println(str, huart4);
	
}

void Serial_print_f(float number, UART_HandleTypeDef *huart4)
{
	char str[7];
	sprintf(str, "%0.10f\n", number);
	Serial_print(str, huart4);
	
}	

void Serial_println_f(float number, UART_HandleTypeDef *huart4)
{
	char str[7];
	sprintf(str, "%0.10f\n", number);
	Serial_println(str, huart4);
	
}


int decimales_l (unsigned long number)
{
	unsigned long number_2 = number;
		int j = 0;
	for (int i = 0; number_2/10 != 0; i++)
	{
		number_2=number_2/10;
		j=i;
	}
	return j;
}

int decimales (unsigned int number)
{
	unsigned int number_2 = number;
		int j = 0;
	for (int i = 0; number_2/10 != 0; i++)
	{
		number_2=number_2/10;
		j=i;
	}
	return j;
}

