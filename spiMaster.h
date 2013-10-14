/*
 *  Erl's header for embedded systems SPI master functions
 */
#ifndef ERL_SPI_MASTER
#define ERL_SPI_MASTER

#include <stdbool.h>
#include <stdint.h>

#include <nrf51.h> // for NRF_SPI_Type

/**
 *  SPI mode
 */
typedef enum
{
    //------------------------Clock polarity 0, Clock starts with level 0-------------------------------------------
    SPI_MODE0 = 0,          /*!< Sample data at rising edge of clock and shift serial data at falling edge */
    SPI_MODE1,              /*!< sample data at falling edge of clock and shift serial data at rising edge */
    //------------------------Clock polarity 1, Clock starts with level 1-------------------------------------------
    SPI_MODE2,              /*!< sample data at falling edge of clock and shift serial data at rising edge */
    SPI_MODE3               /*!< Sample data at rising edge of clock and shift serial data at falling edge */
} SPIMode;

typedef struct sSPIMasterSlave
{
	int selectPin;
	int selectActiveHigh;
	int selectDelayUs; // microseconds of delay required after select active
	SPIMode mode;
	int lsb_first;
	uint32_t frequencyConstant;
} sSPIMasterSlave, *SPIMasterSlave;

typedef struct sSPIMaster
{
  NRF_SPI_Type *baseAddress;
	const sSPIMasterSlave *previousSlave;
} sSPIMaster;

typedef struct sSPIMaster *SPIMaster;

void spiMaster_init( sSPIMaster *master, int module_number, int sckPin, int mosiPin, int misoPin );

// returns true on success, false on failure (timeout)
bool spiMaster_txRx( const SPIMaster, const sSPIMasterSlave *slave, uint16_t transfer_size, 
                     const uint8_t *tx_data, uint8_t *rx_data );

#endif
