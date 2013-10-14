/*
   spiMaster.c

	Implementation for nRF51822
*/

#include <nrf51.h>    // for NRF_SPI_Type
#include <stdbool.h>
#include <stddef.h>

#include "spiMaster.h"

#include "nrf.h"
#include "nrf_gpio.h"
#include <nrf_delay.h>

/*
 * Constants
 */
#define TIMEOUT_COUNTER          0x3000UL  /*!< timeout for getting rx bytes from slave */

/*
 * static function prototypes
 */
static inline void slave_select( const sSPIMasterSlave *slave );
static inline void slave_deselect( const sSPIMasterSlave *slave );

/*
 * start of code
 */
void spiMaster_init( sSPIMaster *master, int module_number, int sckPin, int mosiPin, 
                     int misoPin )
{
	// should assert that module_number is 0 or 1
	
	// set up pins.
	nrf_gpio_cfg_output( sckPin );
	nrf_gpio_cfg_output( mosiPin );
	nrf_gpio_cfg_input( misoPin, NRF_GPIO_PIN_NOPULL );

  NRF_SPI_Type *spi_base_address = (0 == module_number)? NRF_SPI0 : (NRF_SPI_Type *)NRF_SPI1;

  /* Configure pins, frequency and mode */
	spi_base_address->PSELSCK  = sckPin;
	spi_base_address->PSELMOSI = mosiPin;
	spi_base_address->PSELMISO = misoPin;

	master->baseAddress = spi_base_address;
	master->previousSlave = NULL;
	/*
	Do this later in slave code
	        nrf_gpio_pin_set(SPI_PSELSS1);         // disable Set slave select (inactive high) 


	*/
}

// assumes change to slave structs not made between calls
// code fom spi_master example
// returns true on success, false otherwise
bool spiMaster_txRx( const SPIMaster master, const sSPIMasterSlave *slave, uint16_t transfer_size, 
                     const uint8_t *tx_data, uint8_t *rx_data )
{
	uint16_t number_of_txd_bytes = 0;
  uint32_t counter = 0;
	NRF_SPI_Type *spi_base;
	
	spi_base = master->baseAddress;
	if( master->previousSlave != slave )
	{
		uint32_t config_mode;
			
    spi_base->FREQUENCY = slave->frequencyConstant;

    switch( slave->mode )
    {
        /*lint -e845 -save // A zero has been given as right argument to operator '!'" */
        case 0:
            config_mode = (SPI_CONFIG_CPHA_Leading << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos);
            break;
        case 1:
            config_mode = (SPI_CONFIG_CPHA_Trailing << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos);
            break;
        case 2:
            config_mode = (SPI_CONFIG_CPHA_Leading << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_CPOL_ActiveLow << SPI_CONFIG_CPOL_Pos);
            break;
        case 3:
            config_mode = (SPI_CONFIG_CPHA_Trailing << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_CPOL_ActiveLow << SPI_CONFIG_CPOL_Pos);
            break;
        default:
            config_mode = 0;
            break;
        /*lint -restore */
    }
		
    if (slave->lsb_first)
        spi_base->CONFIG = (config_mode | (SPI_CONFIG_ORDER_LsbFirst << SPI_CONFIG_ORDER_Pos));
    else
        /*lint -e{845} // A zero has been given as right argument to operator '|'" */
        spi_base->CONFIG = (config_mode | (SPI_CONFIG_ORDER_MsbFirst << SPI_CONFIG_ORDER_Pos));

    spi_base->EVENTS_READY = 0U;
		master->previousSlave = slave;
		
		nrf_gpio_cfg_output( slave->selectPin );
	}
	
  // Enable 
  spi_base->ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);

  /* enable slave (slave select active low) */
  slave_select( slave );

	while(number_of_txd_bytes < transfer_size)
	{
			spi_base->TXD = (uint32_t)(tx_data[number_of_txd_bytes]);

			/* Wait for the transaction complete or timeout (about 10ms - 20 ms) */
		  /* Improve this code with interrupts or similar! */
			while ((spi_base->EVENTS_READY == 0U) && (counter < TIMEOUT_COUNTER))
					counter++;

			if (counter == TIMEOUT_COUNTER)
			{
					/* timed out, disable slave (slave select active low) and return with error */
					slave_deselect( slave );
				
					// disable SPI
				  spi_base->ENABLE = (SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos);

					return false;
			}
			else
			{   /* clear the event to be ready to receive next messages */
					spi_base->EVENTS_READY = 0U;
			}

			rx_data[number_of_txd_bytes] = (uint8_t)spi_base->RXD;
			number_of_txd_bytes++;
	};

	/* disable slave (slave select active low) */
	slave_deselect( slave );

	// disable SPI disable for debugging
	// spi_base->ENABLE = (SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos);

	return true;

}

// also does post-select delay
static inline void slave_select( const sSPIMasterSlave *slave )
{
	if( slave->selectActiveHigh )
		  nrf_gpio_pin_set( slave->selectPin );         // disable Set slave select (inactive high) 
	else
		  nrf_gpio_pin_clear( slave->selectPin );         // disable Set slave select (inactive high) 

	nrf_delay_us( slave->selectDelayUs );
}

static inline void slave_deselect( const sSPIMasterSlave *slave )
{
	if( slave->selectActiveHigh )
		  nrf_gpio_pin_clear( slave->selectPin );         // disable Set slave select (inactive high) 
	else
		  nrf_gpio_pin_set( slave->selectPin );         // disable Set slave select (inactive high) 
}
