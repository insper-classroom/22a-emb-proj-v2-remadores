/************************************************************************
* 5 semestre - Eng. da Computao - Insper
*
* 2021 - Exemplo com HC05 com RTOS
*
*/

#include <asf.h>
#include "conf_board.h"
#include <string.h>

/************************************************************************/
/* defines                                                              */
/************************************************************************/

// LEDs
#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      8
#define LED_IDX_MASK (1 << LED_IDX)

//// Botão
//#define BUT_PIO      PIOA
//#define BUT_PIO_ID   ID_PIOA
//#define BUT_IDX      11
//#define BUT_IDX_MASK (1 << BUT_IDX)

#define BUT_PIO      PIOD
#define BUT_PIO_ID   ID_PIOD
#define BUT_IDX      30
#define BUT_IDX_MASK (1 << BUT_IDX)

#define BUT1_PIO      PIOA
#define BUT1_PIO_ID   ID_PIOA
#define BUT1_IDX      6
#define BUT1_IDX_MASK (1 << BUT1_IDX)

// mexer no X
#define AFEC_VRX AFEC1
#define AFEC_VRX_ID ID_AFEC1
#define AFEC_VRX_CHANNEL 5 // Canal do pino PC30

// mexer no Y
#define AFEC_VRY AFEC0
#define AFEC_VRY_ID ID_AFEC0
#define AFEC_VRY_CHANNEL 5 // Canal do pino PB2


// usart (bluetooth ou serial)
// Descomente para enviar dados
// pela serial debug

//#define DEBUG_SERIAL

#ifdef DEBUG_SERIAL
#define USART_COM USART1
#define USART_COM_ID ID_USART1
#else
#define USART_COM USART0
#define USART_COM_ID ID_USART0
#endif

/************************************************************************/
/* RTOS                                                                 */
/************************************************************************/

#define TASK_BLUETOOTH_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_BLUETOOTH_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_ADC_STACK_SIZE (1024*10 / sizeof(portSTACK_TYPE))
#define TASK_ADC_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_PROC_STACK_SIZE (1024*10 / sizeof(portSTACK_TYPE))
#define TASK_PROC_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_ADC2_STACK_SIZE (1024*10 / sizeof(portSTACK_TYPE))
#define TASK_ADC2_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_PROC2_STACK_SIZE (1024*10 / sizeof(portSTACK_TYPE))
#define TASK_PROC2_STACK_PRIORITY (tskIDLE_PRIORITY)


/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

/************************************************************************/
/* RTOS application HOOK                                                */
/************************************************************************/

typedef struct {
	uint value;
} adcData;

/** Queue for msg log send data */
QueueHandle_t xQueueADC;
QueueHandle_t xQueuexX1;
QueueHandle_t xQueueADC2;
QueueHandle_t xQueueY1;

void configure_pio_input(Pio *pio, const pio_type_t ul_type, const uint32_t ul_mask, const uint32_t ul_attribute, uint32_t ul_id){
	pmc_enable_periph_clk(ul_id);
	pio_configure(pio, ul_type, ul_mask, ul_attribute);
	pio_set_debounce_filter(pio, ul_mask, 60);
}

/* Called if stack overflow during execution */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	* identify which task has overflowed its stack.
	*/
	for (;;) {
	}
}

/* This function is called by FreeRTOS idle task */
extern void vApplicationIdleHook(void) {
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/* This function is called by FreeRTOS each tick */
extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void io_init(void) {

	// Ativa PIOs
	pmc_enable_periph_clk(LED_PIO_ID);
	pmc_enable_periph_clk(BUT_PIO_ID);
	
	pmc_enable_periph_clk(BUT1_PIO_ID);
	
	//configure_pio_input(VRX_PIO, PIO_INPUT, VRX_IDX_MASK, PIO_PULLUP|PIO_DEBOUNCE, VRX_PIO_ID);


	// Configura Pinos
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT | PIO_DEBOUNCE);
	pio_configure(BUT_PIO, PIO_INPUT, BUT_IDX_MASK, PIO_PULLUP);
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_IDX_MASK, PIO_PULLUP);
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
		#endif
		.paritytype = CONF_UART_PARITY,
		#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
		#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	#if defined(__GNUC__)
	setbuf(stdout, NULL);
	#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	* emits one character at a time.
	*/
	#endif
}

uint32_t usart_puts(uint8_t *pstring) {
	uint32_t i ;

	while(*(pstring + i))
	if(uart_is_tx_empty(USART_COM))
	usart_serial_putchar(USART_COM, *(pstring+i++));
}

void usart_put_string(Usart *usart, char str[]) {
	usart_serial_write_packet(usart, str, strlen(str));
}

int usart_get_string(Usart *usart, char buffer[], int bufferlen, uint timeout_ms) {
	uint timecounter = timeout_ms;
	uint32_t rx;
	uint32_t counter = 0;

	while( (timecounter > 0) && (counter < bufferlen - 1)) {
		if(usart_read(usart, &rx) == 0) {
			buffer[counter++] = rx;
		}
		else{
			timecounter--;
			vTaskDelay(1);
		}
	}
	buffer[counter] = 0x00;
	return counter;
}

void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen,
char buffer_tx[], int timeout) {
	usart_put_string(usart, buffer_tx);
	usart_get_string(usart, buffer_rx, bufferlen, timeout);
}

void config_usart0(void) {
	sysclk_enable_peripheral_clock(ID_USART0);
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART0, &config);
	usart_enable_tx(USART0);
	usart_enable_rx(USART0);

	// RX - PB0  TX - PB1
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);
}

int hc05_init(void) {
	char buffer_rx[128];
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT+NAMEGuerreiros", 1000);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT+PIN1234", 1000);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
}


static void AFEC_vrx_Callback(void) {
	adcData adc;
	adc.value = afec_channel_get_value(AFEC_VRX, AFEC_VRX_CHANNEL);
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xQueueSendFromISR(xQueuexX1, &adc, &xHigherPriorityTaskWoken);
}

static void AFEC_vry_Callback(void) {
	adcData adc;
	adc.value = afec_channel_get_value(AFEC_VRY, AFEC_VRY_CHANNEL);
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xQueueSendFromISR(xQueueY1, &adc, &xHigherPriorityTaskWoken);
}

void TC1_Handler(void) {
	volatile uint32_t ul_dummy;

	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/* Selecina canal e inicializa conversão */
	afec_channel_enable(AFEC_VRX, AFEC_VRX_CHANNEL);
	afec_start_software_conversion(AFEC_VRX);
	
	afec_channel_enable(AFEC_VRY, AFEC_VRY_CHANNEL);
	afec_start_software_conversion(AFEC_VRY);

}



static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel,
                            afec_callback_t callback) {
  /*************************************
   * Ativa e configura AFEC
   *************************************/
  /* Ativa AFEC - 0 */
  afec_enable(afec);

  /* struct de configuracao do AFEC */
  struct afec_config afec_cfg;

  /* Carrega parametros padrao */
  afec_get_config_defaults(&afec_cfg);

  /* Configura AFEC */
  afec_init(afec, &afec_cfg);

  /* Configura trigger por software */
  afec_set_trigger(afec, AFEC_TRIG_SW);

  /*** Configuracao específica do canal AFEC ***/
  struct afec_ch_config afec_ch_cfg;
  afec_ch_get_config_defaults(&afec_ch_cfg);
  afec_ch_cfg.gain = AFEC_GAINVALUE_0;
  afec_ch_set_config(afec, afec_channel, &afec_ch_cfg);

  /*
  * Calibracao:
  * Because the internal ADC offset is 0x200, it should cancel it and shift
  down to 0.
  */
  afec_channel_set_analog_offset(afec, afec_channel, 0x200);

  /***  Configura sensor de temperatura ***/
  struct afec_temp_sensor_config afec_temp_sensor_cfg;

  afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
  afec_temp_sensor_set_config(afec, &afec_temp_sensor_cfg);

  /* configura IRQ */
  afec_set_callback(afec, afec_channel, callback, 1);
  NVIC_SetPriority(afec_id, 4);
  NVIC_EnableIRQ(afec_id);
}



void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq) {
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	pmc_enable_periph_clk(ID_TC);

	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	NVIC_SetPriority((IRQn_Type)ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type)ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}



/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_proc(void *pvParameters){
	config_AFEC_pot(AFEC_VRX, AFEC_VRX_ID, AFEC_VRX_CHANNEL, AFEC_vrx_Callback);

	TC_init(TC0, ID_TC1, 1, 10);
	tc_start(TC0, 1);
	
	// corsi
	adcData adc;

	while (1) {
		if (xQueueReceive(xQueuexX1, &(adc), 1000)) {
			//printf("ADC X: %d \n", adc);
		} else {
			//printf("Nao chegou um novo dado em 1 segundo");
		}
	}

}

static void task_proc2(void *pvParameters){
	config_AFEC_pot(AFEC_VRY, AFEC_VRY_ID, AFEC_VRY_CHANNEL, AFEC_vry_Callback);

	TC_init(TC0, ID_TC1, 1, 10);
	tc_start(TC0, 1);
	
	adcData adc;

	
	while (1) {
		if (xQueueReceive(xQueueY1, &(adc), 1000)) {
			//printf("ADC Y: %d \n", adc);
			} else {
			//printf("Nao chegou um novo dado em 1 segundo");
		}
	}
}


void task_bluetooth(void) {
	printf("Task Bluetooth started \n");
	
	printf("Inicializando HC05 \n");
	config_usart0();
	hc05_init();

	// configura LEDs e Botões
	io_init();


	char eof = 'X';
	
	char button_high = '1';
	char button_low = '0';
	char button1 = '0';
	char button2 = '0';
	char vx_esquerda = '0';
	char vx_direita = '0';
	char vy_cima = '0';
	char vy_baixo = '0';

	char head1 =  'O';
	char head2 =  'Z';
	char head_vx_esquerda = 'Y';
	char head_vx_direita = 'W';
	char head_vy_cima = 'S';
	char head_vy_baixo = 'T';

	adcData adcX1;
	adcData adcY1;
	
	// Task não deve retornar.
	while(1) {

	if (xQueueReceive(xQueuexX1, &(adcX1), 1000)) {
		usart_write(USART_COM, 'h');
		while(!usart_is_tx_ready(USART_COM)) {vTaskDelay(10 / portTICK_PERIOD_MS);}
		
		usart_write(USART_COM,  (adcX1.value >> 8));
		while(!usart_is_tx_ready(USART_COM)) {vTaskDelay(10 / portTICK_PERIOD_MS);}
		
		usart_write(USART_COM,  (adcX1.value & 0x00ff));
		while(!usart_is_tx_ready(USART_COM)) {vTaskDelay(10 / portTICK_PERIOD_MS);}		
		
		usart_write(USART_COM, eof);
		while(!usart_is_tx_ready(USART_COM)) {vTaskDelay(10 / portTICK_PERIOD_MS);}

	}
	
	if (xQueueReceive(xQueueY1, &(adcY1), 1000)) {
		usart_write(USART_COM, 'y');
		while(!usart_is_tx_ready(USART_COM)) {vTaskDelay(10 / portTICK_PERIOD_MS);}
		
		usart_write(USART_COM,  (adcY1.value >> 8));
		while(!usart_is_tx_ready(USART_COM)) {vTaskDelay(10 / portTICK_PERIOD_MS);}
		
		usart_write(USART_COM,  (adcY1.value & 0x00ff));
		while(!usart_is_tx_ready(USART_COM)) {vTaskDelay(10 / portTICK_PERIOD_MS);}
		
		usart_write(USART_COM, eof);
		while(!usart_is_tx_ready(USART_COM)) {vTaskDelay(10 / portTICK_PERIOD_MS);}

	}


		// atualiza valor do botão
		if(pio_get(BUT_PIO, PIO_INPUT, BUT_IDX_MASK) == 0) {
			head1 = 'A';
			button1 = button_high;
		} else {
			head1 = 'A';
			button1 = button_low;
		}
		
		if(pio_get(BUT1_PIO, PIO_INPUT, BUT1_IDX_MASK) == 0){
			head2 = 'B';
			button2 = button_high;
		} else {
			head2 = 'B';
			button2 = button_low;
		}
		
		//if (adc_vx <= 50){
			//head_vx_esquerda = 'L';
			//vx_esquerda = '1';
		//} else {
			//head_vx_esquerda = 'L';
			//vx_esquerda = '0';
		//}
		//
		//if (adc_vx >= 3500){
			//head_vx_direita = 'R';
			//vx_direita = '1';
		//} else {
			//head_vx_direita = 'R';
			//vx_direita = '0';
		//}
		//
		//if (adc_vy <= 20){
			//head_vy_cima= 'U';
			//vy_cima = '1';
		//} else {
			//head_vy_cima= 'U';
			//vy_cima = '0';
		//}
		//
		//if (adc_vy >= 3500){
			//head_vy_baixo = 'D';
			//vy_baixo = '1';
		//} else {
			//head_vy_baixo = 'D';
			//vy_baixo = '0';
		//}
	//
		//char protocol_status[20];
		//sprintf(protocol_status, "%d%d", button1, button2);
		
// 		
// 		usart_write(USART_COM, head1);
// 
// 
// 		while(!usart_is_tx_ready(USART_COM)) {
// 			vTaskDelay(10 / portTICK_PERIOD_MS);
// 		}
// 
// 		usart_write(USART_COM, button1);
// 		
// 		// envia fim de pacote
// 		while(!usart_is_tx_ready(USART_COM)) {
// 			vTaskDelay(10 / portTICK_PERIOD_MS);
// 		}
// 		
// 		usart_write(USART_COM, head2);
// 		
// 		while(!usart_is_tx_ready(USART_COM)) {
// 			vTaskDelay(10 / portTICK_PERIOD_MS);
// 		}
// 
// 		usart_write(USART_COM, button2);
// 		
// 		while(!usart_is_tx_ready(USART_COM)) {
// 			vTaskDelay(10 / portTICK_PERIOD_MS);
// 		}
		
		
			
// 		usart_write(USART_COM, head_vx_direita);
// 		
// 		while(!usart_is_tx_ready(USART_COM)) {
// 			vTaskDelay(10 / portTICK_PERIOD_MS);
// 		}
// 		
// 		usart_write(USART_COM, vx_direita);
// 		
// 		while(!usart_is_tx_ready(USART_COM)) {
// 			vTaskDelay(10 / portTICK_PERIOD_MS);
// 		}
// 		
// 		usart_write(USART_COM, head_vx_esquerda);
// 		
// 		while(!usart_is_tx_ready(USART_COM)) {
// 			vTaskDelay(10 / portTICK_PERIOD_MS);
// 		}
// 		
// 		usart_write(USART_COM, vx_esquerda);
// 		
// 		while(!usart_is_tx_ready(USART_COM)) {
// 			vTaskDelay(10 / portTICK_PERIOD_MS);
// 		}
// 			
// 		usart_write(USART_COM, head_vy_cima);
// 			
// 		while(!usart_is_tx_ready(USART_COM)) {
// 			vTaskDelay(10 / portTICK_PERIOD_MS);
// 		}
// 			
// 		usart_write(USART_COM, vy_cima);
// 		
// 		while(!usart_is_tx_ready(USART_COM)) {
// 			vTaskDelay(10 / portTICK_PERIOD_MS);
// 		}
// 		
// 		usart_write(USART_COM, head_vy_baixo);
// 		
// 		while(!usart_is_tx_ready(USART_COM)) {
// 			vTaskDelay(10 / portTICK_PERIOD_MS);
// 		}
// 		
// 		usart_write(USART_COM, vy_baixo);
// 	
		// dorme por 500 ms
		//vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}



/************************************************************************/
/* main                                                                 */
/************************************************************************/

  
int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	configure_console();

	/* Create task to make led blink */
	xTaskCreate(task_bluetooth, "BLT", TASK_BLUETOOTH_STACK_SIZE, NULL,	TASK_BLUETOOTH_STACK_PRIORITY, NULL);
	
	xQueuexX1 = xQueueCreate(100, sizeof(adcData));
	if (xQueuexX1 == NULL)
		printf("falha em criar a queue xQueueADC \n");
		
	xQueueY1 = xQueueCreate(100, sizeof(adcData));
	if (xQueueY1 == NULL)
	printf("falha em criar a queue xQueueADC \n");


	if (xTaskCreate(task_proc, "proc", TASK_PROC_STACK_SIZE, NULL,
	TASK_PROC_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test ADC task\r\n");
	}
 	
 	if (xTaskCreate(task_proc2, "proc2", TASK_PROC2_STACK_SIZE, NULL,
 	TASK_PROC2_STACK_PRIORITY, NULL) != pdPASS) {
 		printf("Failed to create test ADC task\r\n");
 	}
	



	/* Start the scheduler. */
	vTaskStartScheduler();


	while(1){
	}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
