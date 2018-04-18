/**
  ******************************************************************************
  * File Name          : ADS1299.c
  * Description        : Funtion to interact with the ADS1299
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "ADS1299.h"
#include "main.h"		
#include "util.h"
#include "stdbool.h"
#include <stdlib.h>

void Blinky(SPI_HandleTypeDef *hspi2)
{
	
		uint8_t opcode_1 = WREG | GPIO; //0x54  (WREG + GPIO) Registro encargado de escribir la configuración de los GPIO
		uint8_t opcode_2 = 0x00;		//Registros a leer (N-1)
		uint8_t reg_leido = 0x00;
		uint8_t reg_LED_ON 	= 0x00;
		uint8_t reg_LED_OFF 	= 0xF0;
		
	
		HAL_GPIO_WritePin(A_CS0_N_GPIO_Port, A_CS0_N_Pin, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(hspi2, &opcode_1, &reg_leido, 1, HAL_MAX_DELAY);
		HAL_SPI_TransmitReceive(hspi2, &opcode_2, &reg_leido, 1, HAL_MAX_DELAY);					
		HAL_SPI_TransmitReceive(hspi2, &reg_LED_ON, &reg_leido, 1, HAL_MAX_DELAY); //LED ON
		HAL_GPIO_WritePin(A_CS0_N_GPIO_Port, A_CS0_N_Pin, GPIO_PIN_SET);
	
		HAL_Delay(500);
	
		HAL_GPIO_WritePin(A_CS0_N_GPIO_Port, A_CS0_N_Pin, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(hspi2, &opcode_1, &reg_leido, 1, HAL_MAX_DELAY);
		HAL_SPI_TransmitReceive(hspi2, &opcode_2, &reg_leido, 1, HAL_MAX_DELAY);					
		HAL_SPI_TransmitReceive(hspi2, &reg_LED_OFF, &reg_leido, 1, HAL_MAX_DELAY); //LED OFF
		HAL_GPIO_WritePin(A_CS0_N_GPIO_Port, A_CS0_N_Pin, GPIO_PIN_SET);
		
		HAL_Delay(500);
}

void adc_send_command(uint8_t cmd, SPI_HandleTypeDef *SPI)
{
	uint8_t comando_temp = cmd;
	uint8_t zero = 0x00;
	//IPIN_MASTER_CS:
	HAL_GPIO_WritePin(A_CS0_N_GPIO_Port, A_CS0_N_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(SPI, &comando_temp, &zero, 1, 100); //SPI.transfer(cmd);
	HAL_Delay(1);
	HAL_GPIO_WritePin(A_CS0_N_GPIO_Port, A_CS0_N_Pin, GPIO_PIN_SET);
}

uint8_t adc_rreg(uint8_t reg, SPI_HandleTypeDef *SPI)
{
	uint8_t val = 0x00;
	uint8_t zero_t = 0x00;
	uint8_t zero_r = 0x00;
	uint8_t temp = RREG | reg;
	
	HAL_GPIO_WritePin(A_CS0_N_GPIO_Port, A_CS0_N_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(SPI, &temp, &zero_r, 1, 100);
	HAL_SPI_TransmitReceive(SPI, &zero_t, &zero_r, 1, 100);	// number of registers to be read/written
	HAL_SPI_TransmitReceive(SPI, &zero_t, &val, 1, 100);

	HAL_Delay(1);
	HAL_GPIO_WritePin(A_CS0_N_GPIO_Port, A_CS0_N_Pin, GPIO_PIN_SET);

	return val;
}

void adc_wreg(uint8_t reg, uint8_t val, SPI_HandleTypeDef *SPI)
{
	uint8_t zero_t = 0x00;
	uint8_t zero_r = 0x00;
	uint8_t reg_temp = WREG | reg;
	uint8_t val_temp = val;
	//uint8_t buffer [3] = {WREG | reg, 0x00, val};
	
	// IPIN_MASTER_CS
	HAL_GPIO_WritePin(A_CS0_N_GPIO_Port, A_CS0_N_Pin, GPIO_PIN_RESET);
	// ADS1298::WREG
	//HAL_SPI_Transmit(SPI, buffer, 3, 100);
	HAL_SPI_TransmitReceive(SPI, &reg_temp, &zero_r, 1, 100);
	HAL_SPI_TransmitReceive(SPI, &zero_t, &zero_r, 1, 100);	// number of registers to be read/written
	HAL_SPI_TransmitReceive(SPI, &val_temp, &zero_r, 1, 100);	

	HAL_Delay(1);
	HAL_GPIO_WritePin(A_CS0_N_GPIO_Port, A_CS0_N_Pin, GPIO_PIN_SET);
}

void read_data_frame(uint8_t data [], SPI_HandleTypeDef *SPI)
{
	uint8_t zero = 0x00;
	// IPIN_MASTER_CS	
//		HAL_GPIO_WritePin(A_CS0_N_GPIO_Port, A_CS0_N_Pin, GPIO_PIN_RESET);
//		HAL_SPI_Receive(SPI, data, 27, 100);
			for(int i = 0; i<27; i++)
			{
				HAL_SPI_TransmitReceive(SPI, &zero, &data[i], 1, 100);
			}
		//HAL_Delay(1);	// is this needed? Yes, it requires a 4Tclk = 0.2ms wait.
	
//		HAL_GPIO_WritePin(A_CS0_N_GPIO_Port, A_CS0_N_Pin, GPIO_PIN_SET);
//		while (HAL_GPIO_ReadPin(A_DRDY_N_GPIO_Port, A_DRDY_N_Pin) == GPIO_PIN_RESET) {}
}


/*
#if OPENHARDWAREEXG_HARDWARE_VERSION == 1                                       //OJO VER PARA QUE SIRVE
void update_leadoff_led_data(const ADS1298::Data_frame &frame)
{
	for (int channel = 0; channel < LIVE_CHANNELS_NUM; ++channel) {
		bool leadoff_p = frame.loff_statp(channel);
		lead_leds.set_green_positive(channel, !leadoff_p);
		lead_leds.set_yellow_positive(channel, leadoff_p);

		// if the negative electrodes are shared, use only the first LED.
		if (channel == 0 || !shared_negative_electrode) {
			bool leadoff_n = frame.loff_statn(channel);
			lead_leds.set_green_negative(channel, !leadoff_n);
			lead_leds.set_yellow_negative(channel, leadoff_n);
		} else {
			lead_leds.set_green_negative(channel, false);
			lead_leds.set_yellow_negative(channel, false);
		}
	}
}
#endif

*/

void update_bias_ref(uint8_t data[], SPI_HandleTypeDef *SPI)                        //OJO VER PARA QUE SIRVE
{

	static uint8_t last_loff_statp = 0xFF;
	static uint8_t last_loff_statn = 0xFF;
	static unsigned samples_since_last_bias_change = 0;
	const unsigned min_samples_between_bias_changes = 100;

	uint8_t loff_statp = frame_loff_statp(data);
	uint8_t leads_on_p = ~loff_statp;
	uint8_t loff_statn = frame_loff_statn(data);
	uint8_t leads_on_n = ~loff_statn;

		bool shared_negative_electrode = true;
	
	if (shared_negative_electrode) {
		loff_statn |= 0x01;	// count only the single shared electrode
	}
	// if the lead-off status has changed...
	if (samples_since_last_bias_change >= min_samples_between_bias_changes
	    && (last_loff_statp != loff_statp
		|| last_loff_statn != loff_statn)) {

		// Send SDATAC Command (Stop Read Data Continuously mode)
		// TODO: starting and stopping the data collection like this will
		// create a glitch in all channels of the recording whenever the
		// leadoff status of any channel changes.  This could be fixed by
		// capturing all data in single-shot mode, triggered by an interrupt.
		adc_send_command(SDATAC, SPI);

		// Use only the leads that are connected to drive the bias electrode.
		adc_wreg(RLD_SENSP, leads_on_p, SPI);
		adc_wreg(RLD_SENSN, leads_on_n, SPI);

		// Put the Device Back in Read DATA Continuous Mode
		adc_send_command(RDATAC, SPI);

		last_loff_statp = loff_statp;
		last_loff_statn = loff_statn;
		samples_since_last_bias_change = 0;
	} else {
		++samples_since_last_bias_change;
	}
}
/*
long extrae_un_canal (uint8_t data[], uint8_t canal)
{

  uint8_t icanal;
  long valor;

  valor=0;

  if (canal==0){
    //Serial.println("************ Canal solicitado erroneo ********************");
  }

  icanal=canal*3;

  valor= data[icanal];
  //Serial.print("valor= ");
  //Serial.print(valor,HEX);
  valor= valor << 8;
//  Serial.print("valor << 8 ");
//  Serial.println(valor);
  valor |= data[icanal+1];
  //Serial.print(valor);
  valor= valor << 8;
//  Serial.print("valor << 16 ");
//  Serial.println(valor);
  valor |= data[icanal+2];
//  Serial.print("valor total ");
  //Serial.println(valor,HEX);
  return valor;

}
*/
/*
// if this becomes more flexible, we may need to pass in
// the byte_buf size, but for now we are safe to skip it
void format_data_frame(uint8_t data [], char *byte_buf)
{
	uint8_t in_byte;
	unsigned int pos = 0;

	byte_buf[pos++] = '[';
	byte_buf[pos++] = 'g';
	byte_buf[pos++] = 'o';
	byte_buf[pos++] = ']';
  
//  Serial.print("size : "); //mias
//  Serial.println(frame.size);
	for (int i = 0; i < size; ++i) {
		in_byte = data[i];
//    Serial.print("in_byte : "); //mias
//    Serial.println(in_byte);
		to_hex(in_byte, byte_buf + pos);
		pos += 2;
	}

	byte_buf[pos++] = '[';
	byte_buf[pos++] = 'o';
	byte_buf[pos++] = 'n';
	byte_buf[pos++] = ']';
	byte_buf[pos++] = '\n';
	byte_buf[pos++] = 0;
}
*/
/*
void print_chip_id(SPI_HandleTypeDef *SPI, UART_HandleTypeDef *huart4)
{
	uint8_t version;
	int i = 0;
	char msg[40];

	msg[i++] = '[';
	msg[i++] = 'i';
	msg[i++] = 't';
	msg[i++] = ']';

	version = adc_rreg(ID, SPI);
	msg[i++] = 'c';
	msg[i++] = 'h';
	msg[i++] = 'i';
	msg[i++] = 'p';
	msg[i++] = ' ';
	msg[i++] = 'i';
	msg[i++] = 'd';
	msg[i++] = ':';
	msg[i++] = ' ';
	msg[i++] = '0';
	msg[i++] = 'x';
	to_hex(version, msg + i);
	i += 2;

	msg[i++] = '[';
	msg[i++] = 'i';
	msg[i++] = 's';
	msg[i++] = ']';
	msg[i++] = '\n';
	msg[i++] = '\0';
	Serial_print(msg, huart4);
  //Serial_print("A capón   ", huart4);
  Serial_println_N(version, huart4);
}
*/

	//#ifdef __cplusplus
			uint8_t frame_loff_statp(uint8_t data[])
			{
				return ((data[0] << 4) | (data[1] >> 4));
			};
			uint8_t frame_loff_statn(uint8_t data[])
			{
				return ((data[1] << 4) | (data[2] >> 4));
			};
			uint8_t frame_loff_statp_i(uint8_t data[], int i)
				{
				return ((frame_loff_statp(data) >> i) & 1);
			};
			uint8_t frame_loff_statn_i(uint8_t data[], int i)
			{
				return ((frame_loff_statn(data) >> i) & 1);
			};
	//#endif
		//};

			
float byte2float (uint8_t data_23_16, uint8_t data_15_8, uint8_t data_7_0)
	{
		float value = 0;
		
		union miDato{
		struct
		{
			uint8_t  b[4];     // Array de bytes de tamaño igual al tamaño de la primera variable: int = 2 bytes, float = 4 bytes
		}split;
			long dato;
	 } long_data; 
		
	 long_data.dato = 0;
	 
		if (data_23_16>0x80)
		{
			long_data.split.b[2] = ~data_23_16;
			long_data.split.b[1] = ~data_15_8;
			long_data.split.b[0] = ~data_7_0;
			value = long_data.dato;
			value = -value;
		}
		else
		{
			long_data.split.b[2] = data_23_16;
			long_data.split.b[1] = data_15_8;
			long_data.split.b[0] = data_7_0;
			value = long_data.dato;
		}
		
		value = value*4.5f/201326568.0f;
		
		return value;
	}
	
	
	void configADS(uint8_t config[], uint8_t config_channel[], SPI_HandleTypeDef *SPI, UART_HandleTypeDef *huart4)
	{
		int i;
		bool continuo = false;

		HAL_GPIO_WritePin(A_RESET_N_GPIO_Port, A_RESET_N_Pin, GPIO_PIN_RESET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(A_RESET_N_GPIO_Port, A_RESET_N_Pin, GPIO_PIN_SET);
		
		for (i = 0; i <8; ++i) {    
			HAL_Delay(50);
		}

		// Send SDATAC Command (Stop Read Data Continuously mode)
		adc_send_command(SDATAC, SPI);

		// Power up the internal reference and wait for it to settle
			adc_wreg(CONFIG3, config[3-1], SPI);
		//adc_wreg(CONFIG3, PD_REFBUF | CONFIG3_reserved | BIASREF_INT | PD_BIAS, SPI); //Default mode
		//adc_wreg(CONFIG3, SPI); // 0xEC = 1110 1100	
		
		HAL_Delay(150);
		
		adc_wreg(CONFIG4, config[4-1], SPI);
		//adc_wreg(CONFIG4, PD_LOFF_COMP, SPI); // Default mode
		//adc_wreg(CONFIG4, 0x02, SPI); // 0x02 = 0000 0010
		
		adc_wreg(LOFF, COMP_TH_80 | ILEAD_OFF_12nA, SPI);
		adc_wreg(LOFF_SENSP, 0xFF, SPI);
		adc_wreg(LOFF_SENSN, 0x01, SPI);
		
		// Use lead-off sensing in all channels (but only drive one of the
		// negative leads if all of them are connected to one electrode)
		
		adc_wreg(CONFIG1, config[1-1], SPI);	// 250 SPS
		//adc_wreg(CONFIG1, CONFIG1_reserved | DR_250_SPS, SPI);	// 250 SPS - Default mode - 0x96
		
		adc_wreg(CONFIG2, config[2-1], SPI);
		//adc_wreg(CONFIG2, CONFIG2_reserved | INT_CAL | CAL_FREQ_SLOW, SPI); //| TEST_AMP | TEST_FREQ0);	// generate internal test signals - Default mode
		//adc_wreg(CONFIG2, 0xD0, SPI);  //D0 = 1101 0000
		
		// If we want to share a single negative electrode, tie the negative
		// inputs together using the BIAS_IN line.
		//uint8_t mux = RLD_DRN;

		// connect the negative channel to the (shared) BIAS_IN line
		// Set the first LIVE_CHANNELS_NUM channels to input signal
		for (i = 1; i <= LIVE_CHANNELS_NUM; ++i) {
			//adc_wreg(CHnSET + i, mux | GAIN_12X);
			//adc_wreg(CHnSET + i, ELECTRODE_INPUT | GAIN_12X, SPI); 
			adc_wreg(CHnSET + i, config_channel[i-1], SPI);
		}
		// Set all remaining channels to shorted inputs
		for (; i <= 8; ++i) {
			adc_wreg(CHnSET + i, SHORTED | PDn, SPI);
		}
		
		HAL_Delay(3 * 1000);
		
		HAL_GPIO_WritePin(A_START_GPIO_Port, A_START_Pin, GPIO_PIN_SET);
		
		//adc_send_command(START, SPI);
		if (continuo == true)
		{
			adc_send_command(RDATAC, SPI);
		}else
		{
			adc_send_command(SDATAC, SPI);
		}
		
	}
	
	void adquire_single_data (uint8_t data[], SPI_HandleTypeDef *SPI, UART_HandleTypeDef *huart4)
	{
	// IPIN_MASTER_CS	

			adc_send_command(RDATAC, SPI);
		
			while (HAL_GPIO_ReadPin(A_DRDY_N_GPIO_Port, A_DRDY_N_Pin) == GPIO_PIN_SET){}
		
			HAL_GPIO_WritePin(A_CS0_N_GPIO_Port, A_CS0_N_Pin, GPIO_PIN_RESET);
				
			read_data_frame(data, SPI);
			
			while (HAL_GPIO_ReadPin(A_DRDY_N_GPIO_Port, A_DRDY_N_Pin) == GPIO_PIN_RESET) {}
				
			HAL_GPIO_WritePin(A_CS0_N_GPIO_Port, A_CS0_N_Pin, GPIO_PIN_SET);

			update_bias_ref(data, SPI);
}
		
	
	void adquire_array_data (uint8_t data[], float channel_X[], uint8_t channel, SPI_HandleTypeDef *SPI, UART_HandleTypeDef *huart4)
	{
		int debug = 255;
		// read 250 samples
		
		//adc_wreg(GPIO, 0x2C, SPI);				// Led 1 on, led 2 off
		
		adc_send_command(RDATAC, SPI);
		
		while (HAL_GPIO_ReadPin(A_DRDY_N_GPIO_Port, A_DRDY_N_Pin) == GPIO_PIN_SET){}
		
		HAL_GPIO_WritePin(A_CS0_N_GPIO_Port, A_CS0_N_Pin, GPIO_PIN_RESET);
		
		read_data_frame(data, SPI);
			
		channel_X[0] = byte2float(data[channel*3], data[channel*3 +1], data[channel*3 + 2]);
			
		while (HAL_GPIO_ReadPin(A_DRDY_N_GPIO_Port, A_DRDY_N_Pin) == GPIO_PIN_RESET){}
			
		for (int i = 1; i<250; i++)
		{
			while (HAL_GPIO_ReadPin(A_DRDY_N_GPIO_Port, A_DRDY_N_Pin) == GPIO_PIN_SET){}
			
			read_data_frame(data, SPI);

			channel_X[i] = byte2float(data[channel*3], data[channel*3 +1], data[channel*3 + 2]);
				
			while (HAL_GPIO_ReadPin(A_DRDY_N_GPIO_Port, A_DRDY_N_Pin) == GPIO_PIN_RESET){}
		}
		
		HAL_GPIO_WritePin(A_CS0_N_GPIO_Port, A_CS0_N_Pin, GPIO_PIN_SET);
		
		//adc_wreg(GPIO, 0x1C,SPI);				// Led 1 off, led 2 on					
		//Serial_println_N(debug, huart4);
	}
	
		void one_shot (uint8_t data[], SPI_HandleTypeDef *SPI, UART_HandleTypeDef *huart4)
	{
		uint8_t zero = 0x00;
		uint8_t cmd = RDATA;
	// IPIN_MASTER_CS	
		
			//while (HAL_GPIO_ReadPin(A_DRDY_N_GPIO_Port, A_DRDY_N_Pin) == GPIO_PIN_SET){}
		
			HAL_GPIO_WritePin(A_CS0_N_GPIO_Port, A_CS0_N_Pin, GPIO_PIN_RESET);
			
			HAL_SPI_TransmitReceive(SPI, &cmd, &zero,  1, 100);
		
			read_data_frame(data, SPI);
			
			//while (HAL_GPIO_ReadPin(A_DRDY_N_GPIO_Port, A_DRDY_N_Pin) == GPIO_PIN_RESET) {}			
				
			HAL_GPIO_WritePin(A_CS0_N_GPIO_Port, A_CS0_N_Pin, GPIO_PIN_SET);

			update_bias_ref(data, SPI);
}
	
		void one_shot_array (uint8_t data[],float channel_X[], uint8_t channel, SPI_HandleTypeDef *SPI, UART_HandleTypeDef *huart4)
	{
		uint8_t zero = 0x00;
		uint8_t cmd = RDATA;
		float anterior = 0x00;
		float pre_anterior = 0x00;
		
	// IPIN_MASTER_CS	
		
			//while (HAL_GPIO_ReadPin(A_DRDY_N_GPIO_Port, A_DRDY_N_Pin) == GPIO_PIN_SET){}
			for(int i = 0; i<2; i++)
		{
				HAL_GPIO_WritePin(A_CS0_N_GPIO_Port, A_CS0_N_Pin, GPIO_PIN_RESET);
					
				HAL_SPI_TransmitReceive(SPI, &cmd, &zero,  1, 100);
				
				//while(HAL_GPIO_ReadPin(A_DRDY_N_GPIO_Port, A_DRDY_N_Pin) == GPIO_PIN_RESET){}
				
				read_data_frame(data, SPI);
			
				channel_X[i] = byte2float(data[channel*3], data[channel*3 +1], data[channel*3 + 2]);
				//while (HAL_GPIO_ReadPin(A_DRDY_N_GPIO_Port, A_DRDY_N_Pin) == GPIO_PIN_RESET) {}			
				if (i==0){
					pre_anterior = channel_X[i] = byte2float(data[channel*3], data[channel*3 +1], data[channel*3 + 2]);
				}
//				else if (i==1){
//					anterior = channel_X[i] = byte2float(data[channel*3], data[channel*3 +1], data[channel*3 + 2]);
//				}
				HAL_GPIO_WritePin(A_CS0_N_GPIO_Port, A_CS0_N_Pin, GPIO_PIN_SET);
				
				update_bias_ref(data, SPI);
			}
		
		for(int i = 2; i<250;)
		{
			if 	(HAL_GPIO_ReadPin(A_DRDY_N_GPIO_Port, A_DRDY_N_Pin) == GPIO_PIN_RESET)
			{
				HAL_GPIO_WritePin(A_CS0_N_GPIO_Port, A_CS0_N_Pin, GPIO_PIN_RESET);
					
				HAL_SPI_TransmitReceive(SPI, &cmd, &zero,  1, 100);
				
				//while(HAL_GPIO_ReadPin(A_DRDY_N_GPIO_Port, A_DRDY_N_Pin) == GPIO_PIN_RESET){}
				
				read_data_frame(data, SPI);
			
				channel_X[i] = byte2float(data[channel*3], data[channel*3 +1], data[channel*3 + 2]);
				//while (HAL_GPIO_ReadPin(A_DRDY_N_GPIO_Port, A_DRDY_N_Pin) == GPIO_PIN_RESET) {}			
					
				HAL_GPIO_WritePin(A_CS0_N_GPIO_Port, A_CS0_N_Pin, GPIO_PIN_SET);
				
				update_bias_ref(data, SPI);
				if (channel_X[i] != anterior){
//					if (channel_X[i-1] != pre_anterior){
//					pre_anterior = anterior;
					anterior = channel_X[i];
					i++;
//					}
				}
			}
		}
}
	
/*****************************END OF FILE*****************************/
