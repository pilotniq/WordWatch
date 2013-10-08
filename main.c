// processor family; required by nrf.h
// #define NRF51

// #include <core.h>
#include "nrf.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
// #include "nrf_delay.h"

/*
 * constants
 */
#define GPIOTE_CHANNEL_NUMBER 0
#define PIN_VCOM 25
#define PIN_DISP 27
#define PIN_SCLK 29
#define PIN_MOSI 17
#define PIN_MISO 16 // not used
#define PIN_DISP_CS 19 // not soldered at the momoent!
/*
 * static function prototypes
 */
 
/** @brief Function starting the internal LFCLK XTAL oscillator.
 */
static void lfclk_config(void);
static void hfclk_config( void );
static void rtc_config(void);
static inline void incTime( void );
static void display_init( void );
static void updateDisplay( void );
static void __INLINE nrf_delay_us(uint32_t volatile number_of_us);
static void inline display_toggle_COM( void );
static void updateDisplay( void );

/*
 * Global variables
 */
unsigned char secondCounter = 0;
unsigned char minuteCounter = 14;
unsigned char hourCounter = 22;
unsigned char dayCounter = 25;
unsigned char monthCounter = 9;
unsigned int yearCounter = 2013;

unsigned int VCOM_inhibit = 0;

/* SPI buffers */
static uint8_t tx_data[TX_RX_MSG_LENGTH]; /**< SPI TX buffer. */
static uint8_t rx_data[TX_RX_MSG_LENGTH]; /**< SPI RX buffer. */

#define DELAY_MS               100        /**< SPI Timer Delay in milli-seconds. */

int main( int argc, char **argv )
{
	// TODO: Disable DC/DC converter to save power
	// enable low power mdoe
	
	// setup GPIO

	// setup task to toggle pin.
	// Configure GPIOTE_CHANNEL_NUMBER to toggle the GPIO pin state with input.
  // @note Only one GPIOTE task can be coupled to an output pin.
  // GPIOTE uses too much power. Asked in support forum why.
	// nrf_gpiote_task_config( GPIOTE_CHANNEL_NUMBER, outPin /* GPIO_OUTPUT_PIN_NUMBER */, \
  //                         NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);

	// NRF_GPIO->OUT[0] = 0; // todo
	// NRF_GPIO->CONFIG[0] = 
	// setup RTC to generate event on tick every second
	// Use crystal as Low frequency clock source
	hfclk_config();
	lfclk_config();
	// gpio_config();
	display_init();
	// NRF_GPIO->OUTSET = (1UL << PIN_VCOM);
	rtc_config();

	updateDisplay();
	// NRF_GPIO->OUTSET = (1UL << PIN_VCOM);

	// setup PPI to automatically toggle GPIO pin on tick event

	// Configure PPI channel 0 to stop Timer 0 counter at TIMER1 COMPARE[0] match, which is every even number of seconds.
  // NRF_PPI->CH[0].EEP = (uint32_t)(&NRF_RTC0->EVENTS_COMPARE[0]);
  // NRF_PPI->CH[0].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[GPIOTE_CHANNEL_NUMBER];

  // Enable PPI channel 0
  // NRF_PPI->CHEN = (PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos);

	// sleep
	while( 1 )
		__WFE();
}

/** @brief Function starting the internal LFCLK XTAL oscillator.
 */
static void lfclk_config(void)
{
    NRF_CLOCK->LFCLKSRC             = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED  = 0;
    NRF_CLOCK->TASKS_LFCLKSTART     = 1;
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
			// __WFE(); // why does WFE not work here?
        //Do nothing.
    }
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
}
static void hfclk_config( void )
{
	// I'm not sure how to handle this
	// NRF_CLOCK->TASKS_HFCLKSTART = 1; // Start the high frequency clock
}

/** @brief Function for configuring the RTC with TICK to 8Hz and COMPARE0 to 10 sec.
 *  Every second (every 8 ticks), toggle VCOM
 */
static void rtc_config(void)
{
	// RTC0 is used by the S110 Softdevice, so use RTC1.
    NVIC_EnableIRQ(RTC1_IRQn);                                      // Enable Interrupt for the RTC in the core.
    NRF_RTC1->PRESCALER     = 32767;                    // Set prescaler to a TICK of RTC_FREQUENCY.
    NRF_RTC1->CC[0]         = 8; // 8 ticks = 1 second COMPARE_COUNTERTIME * RTC_FREQUENCY;  // Compare0 after approx COMPARE_COUNTERTIME seconds.

    // Enable TICK event and TICK interrupt:
    // NRF_RTC0->EVTENSET      = RTC_EVTENSET_TICK_Msk;
    // NRF_RTC0->INTENSET      = RTC_INTENSET_TICK_Msk;

	  NRF_RTC1->INTENSET      = RTC_INTENSET_COMPARE0_Msk;

		NRF_RTC1->TASKS_START = 1;

    // Enable COMPARE0 event and COMPARE0 interrupt:
    // NRF_RTC0->EVTENSET      = RTC_EVTENSET_COMPARE0_Msk;
}

// Interrupt handler
/** @brief: Function for handling the RTC1 interrupts.
 * Triggered on TICK and COMPARE0 match.
 */
void RTC1_IRQHandler()
{
/*
    if ((NRF_RTC0->EVENTS_TICK != 0) && 
        ((NRF_RTC0->INTENSET & RTC_INTENSET_TICK_Msk) != 0))
    {
        NRF_RTC0->EVENTS_TICK = 0;
        nrf_gpio_pin_toggle(GPIO_TOGGLE_TICK_EVENT);
    }
*/   
    if ((NRF_RTC1->EVENTS_COMPARE[0] != 0) && 
        ((NRF_RTC1->INTENSET & RTC_INTENSET_COMPARE0_Msk) != 0))
    {
        NRF_RTC1->EVENTS_COMPARE[0] = 0;
				NRF_RTC1->CC[0] = NRF_RTC0->CC[0] + 8;
			
				display_toggle_COM();

				incTime();
			
				updateDisplay();
    }
}

static void inline display_toggle_COM()
{
	// check if VCOM Inversion is inhibited by display write
	if( !VCOM_inhibit )
  {
		// raise DISP_CS
		nrf_gpio_pin_set( PIN_DISP_CS );
		// raise EXTCOMIN
		nrf_gpio_pin_set( PIN_VCOM );
		// wait 2 us
		nrf_delay_us( 2 );
		// lower EXTCOMIN
		// lower CS
		nrf_gpio_pin_clear( PIN_VCOM );
		nrf_gpio_pin_clear( PIN_DISP_CS );
		
		nrf_gpio_pin_toggle( PIN_VCOM );
  }
}

static void display_init()
{
	// set up GPIO pins. All start as low outputs
	nrf_gpio_cfg_output(PIN_VCOM);
	nrf_gpio_cfg_output(PIN_DISP);
	nrf_gpio_cfg_output(PIN_SCLK);
	nrf_gpio_cfg_output( PIN_MOSI );
	nrf_gpio_cfg_output( PIN_DISP_CS );
	nrf_gpio_cfg_input( PIN_MISO, NRF_GPIO_PIN_NOPULL ); // not actually used

	// Initialize SPI
	uint32_t *spi_base_address = spi_master_init(0, SPI_MODE0, (bool)lsb_first);
  assert(spi_base_address != 0);

	// map pins to SPI master functionality
	NRF_SPI0->PSELSCK = PIN_SCLK;
	NRF_SPI0->PSELMOSI = PIN_MOSI;
	NRF_SPI0->PSELMISO = PIN_MISO;
	
	NRF_SPI0->CONFIG = 0x0001; // sample on leading edge of clock, clock active high, LSB shifted first
	NRF_SPI0->FREQUENCY = 0x02000000; // Max clock frequency is 1 MHz. use lowest possible for testing (0x02000000=125 kHz)
	
	// power on specification for LS013B4DN04 display
	// should set up initial pixel data here, we'll ignore it for now
	// turn on DISP
	nrf_gpio_pin_set( PIN_DISP );
}

static void updateDisplay()
{
	int i;
	
	// wait 30 us
	nrf_delay_us( 30 );
	
	// turn on chip select
	nrf_gpio_pin_set( PIN_DISP_CS );

	// wait 6 us (tsSCS)
	nrf_delay_us( 6 );
	
	// power up SPI peripheral
	NRF_SPI0->ENABLE = 1;
	
	// inhibit VCOM toggle?
	VCOM_inhibit = 1;
	
	// send data
	NRF_SPI0->TXD = 0x1; // enter dynamic mode
	NRF_SPI0->TXD = 0x1; // double buffering, can send next byte immediately. 1 = line 1

	for( i = 0; i < 14; i++ ) // 96 bits data + 16 bits don't care
  {	
		while( NRF_SPI0->EVENTS_READY == 0 )
			;
		NRF_SPI0->EVENTS_READY = 0;
		NRF_SPI0->TXD = 0xAA;
  }	
	// wait 2 us (thSCS)
	nrf_delay_us( 2 );

	// turn off chip select
	nrf_gpio_pin_clear( PIN_DISP_CS );

	// wait twSCSL = 2 us
	nrf_delay_us( 2 );
	
	// turn on chip select
	nrf_gpio_pin_set( PIN_DISP_CS );

	// wait 6 us (tsSCS)
	nrf_delay_us( 6 );

	// send data
	NRF_SPI0->TXD = 0x0; // enter static mode
	NRF_SPI0->TXD = 0x0; // double buffering, can send next byte immediately. 1 = line 1

	while( NRF_SPI0->EVENTS_READY == 0 )
		;
	NRF_SPI0->EVENTS_READY = 0;
	
	// wait twSCSL = 2 us
	nrf_delay_us( 2 );

	// turn off chip select
	nrf_gpio_pin_clear( PIN_DISP_CS );
	
	// enable VCOM inversion
	VCOM_inhibit = 0;

	// power down SPI peripheral
	NRF_SPI0->ENABLE = 0;
}


static inline void incTime()
{
	secondCounter++;
	if( secondCounter == 60 )
  {
    secondCounter = 0;
		minuteCounter++;
		if( minuteCounter == 60 )
	  {
			minuteCounter = 0;
			hourCounter++;

		  if( hourCounter == 24 )
	    {
			  hourCounter = 0;
			  dayCounter++;
      }			
    }
  }
}
static void __INLINE nrf_delay_us(uint32_t volatile number_of_us)
{
    do 
    {
    __ASM volatile (
        "NOP\n\t"
        "NOP\n\t"
        "NOP\n\t"
        "NOP\n\t"
        "NOP\n\t"
        "NOP\n\t"
        "NOP\n\t"
        "NOP\n\t"
        "NOP\n\t"
        "NOP\n\t"
        "NOP\n\t"
        "NOP\n\t"
        "NOP\n\t"
        "NOP\n\t"
    );
    } while (--number_of_us);
}


