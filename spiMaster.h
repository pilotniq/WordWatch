/*
 *  Erl's header for embedded systems SPI master functions
 */
#ifndef ERL_SPI_MASTER
#define ERL_SPI_MASTER

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
	SPIMode mode;
	int lsb_first;
	int frequency_kHz;
} sSPIMasterSlave, *SPIMasterSlave;

typedef struct sSPIMaster *SPIMaster;

void spiMaster_init( int module_number, SPIMode mode, int sckPin, int mosiPin, int misoPin ):

void spiMaster_txRx( SPIMaster, uint16_t transfer_size, const uint8_t *tx_data, uint8_t *rx_data, int ssPin, 
                     bool ssActiveHigh );

#endif