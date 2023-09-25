#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* Botao da placa */
#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_DISTANCIA_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_DISTANCIA_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TRIG_PIO	PIOC
#define TRIG_PIO_ID ID_PIOC
#define TRIG_PIO_PIN	13
#define TRIG_PIO_PIN_MASK (1 << TRIG_PIO_PIN)

#define ECHO_PIO	PIOA
#define ECHO_PIO_ID ID_PIOA
#define ECHO_PIO_PIN	21
#define ECHO_PIO_PIN_MASK (1 << ECHO_PIO_PIN)


extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback(void);
void echo_callback(void);
static void _init(void);
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
void pin_toggle(Pio *pio, uint32_t mask);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

QueueHandle_t xQueueEchoId;
SemaphoreHandle_t xSemaphoreTrigger;
char str[2];



extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphoreTrigger, &xHigherPriorityTaskWoken); //se foi liberado de uma ISR
	
}


void echo_callback(void) {
	double valor;
	if (pio_get(ECHO_PIO, PIO_INPUT, ECHO_PIO_PIN_MASK)){
		RTT_init(8621, 0,0); //16000-> quantos tics em um segundo;

	}
	else{
		valor = rtt_read_timer_value(RTT);
		xQueueSendFromISR(xQueueEchoId, &valor, 0);
		
	}

}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
  gfx_mono_ssd1306_init();
  _init();

	for (;;)  {

		if (xSemaphoreTake(xSemaphoreTrigger, 1000)){
			pin_toggle(TRIG_PIO, TRIG_PIO_PIN_MASK);
			
		}
		
	
	}
}
static void task_distancia(void *pvParameters){
	  gfx_mono_ssd1306_init();
	  gfx_mono_draw_string("Distância (cm)", 0, 0, &sysfont);

	  _init();

	  
	  double valor;
	  double distancia;
	  for (;;){
		  if (xQueueReceive( xQueueEchoId, &valor, ( TickType_t ) 5 )){
			  distancia = (double)(valor * 340.29)*100/(2*8621);
			  
			   sprintf(str, "%f",distancia );
			   gfx_mono_draw_string(str, 50,16, &sysfont);
			   
			  
	  }
	 
}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/
void pin_toggle(Pio *pio, uint32_t mask) {
	pio_set(pio, mask);
	delay_us(10);
	pio_clear(pio, mask);
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

static void _init(void) {
	
	pmc_enable_periph_clk(BUT_PIO_ID);
	/* conf botão como entrada */
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);

	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback);
	
		pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	/* configura prioridae */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);
	
	//trigger e echo
	pmc_enable_periph_clk(TRIG_PIO_ID);
	pio_set_output(TRIG_PIO, TRIG_PIO_PIN_MASK, 0, 0, 0);
	
	
	pmc_enable_periph_clk(ECHO_PIO_ID);
	pio_set_input(ECHO_PIO, ECHO_PIO_PIN_MASK, PIO_DEFAULT);
	
	pio_handler_set(ECHO_PIO, ECHO_PIO_ID, ECHO_PIO_PIN_MASK, PIO_IT_EDGE , echo_callback);
	pio_enable_interrupt(ECHO_PIO, ECHO_PIO_PIN_MASK); //só entrada tem interrupção
	
	NVIC_EnableIRQ(ECHO_PIO_ID);
	NVIC_SetPriority(ECHO_PIO_ID, 4);
}
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
	
}




/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	
	xQueueEchoId = xQueueCreate(32, sizeof(double));
	xSemaphoreTrigger = xSemaphoreCreateBinary();

	
	if (xQueueEchoId == NULL){
		printf("falha em criar a fila \n");
	}
	
	if (xSemaphoreTrigger == NULL){
		printf("falha em criar o semaforo \n");
	}

	/* Initialize the console uart */
	configure_console();

	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create oled task\r\n");
	}
	
	if (xTaskCreate(task_distancia, "distancia", TASK_DISTANCIA_STACK_SIZE, NULL, TASK_DISTANCIA_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create distancia task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
