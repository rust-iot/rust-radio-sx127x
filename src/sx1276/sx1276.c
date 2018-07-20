/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Generic SX1276 driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Wael Guibene
*/
#include <math.h>
#include <string.h>
//#include "board.h"
#include "radio.h"
#include "sx1276.h"
//#include "sx1276-board.h"

/*
 * Local types definition
 */

/*!
 * Radio registers definition
 */
typedef struct
{
    RadioModems_t Modem;
    uint8_t       Addr;
    uint8_t       Value;
}RadioRegisters_t;

/*!
 * FSK bandwidth definition
 */
typedef struct
{
    uint32_t bandwidth;
    uint8_t  RegValue;
}FskBandwidth_t;


/*
 * Private functions prototypes
 */

/*!
 * Performs the Rx chain calibration for LF and HF bands
 * \remark Must be called just after the reset so all registers are at their
 *         default values
 */
static void RxChainCalibration( SX1276_t* sx1276 );

/*!
 * \brief Resets the SX1276
 */
void SX1276Reset( SX1276_t* sx1276 );

/*!
 * \brief Sets the SX1276 in transmission mode for the given time
 * \param [IN] timeout Transmission timeout [ms] [0: continuous, others timeout]
 */
void SX1276SetTx( SX1276_t* sx1276, uint32_t timeout );

/*!
 * \brief Writes the buffer contents to the SX1276 FIFO
 *
 * \param [IN] buffer Buffer containing data to be put on the FIFO.
 * \param [IN] size Number of bytes to be written to the FIFO
 */
void SX1276WriteFifo( SX1276_t* sx1276, uint8_t *buffer, uint8_t size );

/*!
 * \brief Reads the contents of the SX1276 FIFO
 *
 * \param [OUT] buffer Buffer where to copy the FIFO read data.
 * \param [IN] size Number of bytes to be read from the FIFO
 */
void SX1276ReadFifo( SX1276_t* sx1276, uint8_t *buffer, uint8_t size );

/*!
 * \brief Sets the SX1276 operating mode
 *
 * \param [IN] opMode New operating mode
 */
void SX1276SetOpMode( SX1276_t* sx1276, uint8_t opMode );

/*
 * SX1276 DIO IRQ callback functions prototype
 */

/*!
 * \brief DIO 0 IRQ callback
 */
void SX1276OnDio0Irq( SX1276_t* sx1276 );

/*!
 * \brief DIO 1 IRQ callback
 */
void SX1276OnDio1Irq( SX1276_t* sx1276 );

/*!
 * \brief DIO 2 IRQ callback
 */
void SX1276OnDio2Irq( SX1276_t* sx1276 );

/*!
 * \brief DIO 3 IRQ callback
 */
void SX1276OnDio3Irq( SX1276_t* sx1276 );

/*!
 * \brief DIO 4 IRQ callback
 */
void SX1276OnDio4Irq( SX1276_t* sx1276 );

/*!
 * \brief DIO 5 IRQ callback
 */
void SX1276OnDio5Irq( SX1276_t* sx1276 );

/*!
 * \brief Tx & Rx timeout timer callback
 */
void SX1276OnTimeoutIrq( SX1276_t* sx1276 );

/*
 * Private global constants
 */

/*!
 * Radio hardware registers initialization
 *
 * \remark RADIO_INIT_REGISTERS_VALUE is defined in sx1276-board.h file
 */
const RadioRegisters_t RadioRegsInit[] = RADIO_INIT_REGISTERS_VALUE;

/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET_LF                              -164
#define RSSI_OFFSET_HF                              -157

/*!
 * Precomputed FSK bandwidth registers values
 */
const FskBandwidth_t FskBandwidths[] =
{
    { 2600  , 0x17 },
    { 3100  , 0x0F },
    { 3900  , 0x07 },
    { 5200  , 0x16 },
    { 6300  , 0x0E },
    { 7800  , 0x06 },
    { 10400 , 0x15 },
    { 12500 , 0x0D },
    { 15600 , 0x05 },
    { 20800 , 0x14 },
    { 25000 , 0x0C },
    { 31300 , 0x04 },
    { 41700 , 0x13 },
    { 50000 , 0x0B },
    { 62500 , 0x03 },
    { 83333 , 0x12 },
    { 100000, 0x0A },
    { 125000, 0x02 },
    { 166700, 0x11 },
    { 200000, 0x09 },
    { 250000, 0x01 },
    { 300000, 0x00 }, // Invalid Badwidth
};

/*
 * Private global variables
 */

/*!
 * Radio callbacks variable
 */
static RadioEvents_t *RadioEvents;

/*!
 * Reception buffer
 */
static uint8_t RxTxBuffer[RX_BUFFER_SIZE];

/*
 * Public global variables
 */


/*!
 * Hardware DIO IRQ callback initialization
 */
DioIrqHandler *DioIrq[] = { SX1276OnDio0Irq, SX1276OnDio1Irq,
                            SX1276OnDio2Irq, SX1276OnDio3Irq,
                            SX1276OnDio4Irq, NULL };

/*!
 * Tx and Rx timers
 */
TimerEvent_t TxTimeoutTimer;
TimerEvent_t RxTimeoutTimer;
TimerEvent_t RxTimeoutSyncWord;

/*
 * Radio driver functions implementation
 */

void SX1276Init( SX1276_t* sx1276, RadioEvents_t *events )
{
    uint8_t i;

    RadioEvents = events;

    // Initialize driver timeout timers
    TimerInit( &TxTimeoutTimer, SX1276OnTimeoutIrq );
    TimerInit( &RxTimeoutTimer, SX1276OnTimeoutIrq );
    TimerInit( &RxTimeoutSyncWord, SX1276OnTimeoutIrq );

    sx1276->reset( sx1276->ctx );

    RxChainCalibration( sx1276 );

    SX1276SetOpMode( sx1276, RF_OPMODE_SLEEP );

    SX1276IoIrqInit( sx1276, DioIrq );

    for( i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
    {
        SX1276SetModem( sx1276, RadioRegsInit[i].Modem );
        SX1276Write( sx1276, RadioRegsInit[i].Addr, RadioRegsInit[i].Value );
    }

    SX1276SetModem( sx1276, MODEM_FSK );

    sx1276->settings.State = RF_IDLE;
}

RadioState_t SX1276GetStatus( SX1276_t* sx1276 )
{
    return sx1276->settings.State;
}

void SX1276SetChannel( SX1276_t* sx1276, uint32_t freq )
{
    sx1276->settings.Channel = freq;
    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    SX1276Write( sx1276, REG_FRFMSB, ( uint8_t )( ( freq >> 16 ) & 0xFF ) );
    SX1276Write( sx1276, REG_FRFMID, ( uint8_t )( ( freq >> 8 ) & 0xFF ) );
    SX1276Write( sx1276, REG_FRFLSB, ( uint8_t )( freq & 0xFF ) );
}

bool SX1276IsChannelFree( SX1276_t* sx1276, RadioModems_t modem, uint32_t freq, int16_t rssiThresh )
{
    int16_t rssi = 0;

    SX1276SetModem( sx1276, modem );

    SX1276SetChannel( sx1276, freq );

    SX1276SetOpMode( sx1276, RF_OPMODE_RECEIVER );

    sx1276->delay_ms( sx1276->ctx,   1 );

    rssi = SX1276ReadRssi( sx1276, modem );

    SX1276SetSleep( sx1276 );

    if( rssi > rssiThresh )
    {
        return false;
    }
    return true;
}

uint32_t SX1276Random( SX1276_t* sx1276 )
{
    uint8_t i;
    uint32_t rnd = 0;

    /*
     * Radio setup for random number generation
     */
    // Set LoRa modem ON
    SX1276SetModem( sx1276, MODEM_LORA );

    // Disable LoRa modem interrupts
     SX1276Write( sx1276, REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                  RFLR_IRQFLAGS_RXDONE |
                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                  RFLR_IRQFLAGS_VALIDHEADER |
                  RFLR_IRQFLAGS_TXDONE |
                  RFLR_IRQFLAGS_CADDONE |
                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                  RFLR_IRQFLAGS_CADDETECTED );

    // Set radio in continuous reception
    SX1276SetOpMode( sx1276, RF_OPMODE_RECEIVER );

    for( i = 0; i < 32; i++ )
    {
        sx1276->delay_ms( sx1276->ctx,   1 );
        // Unfiltered RSSI value reading. Only takes the LSB value
        rnd |= ( ( uint32_t ) SX1276Read( sx1276, REG_LR_RSSIWIDEBAND ) & 0x01 ) << i;
    }

    SX1276SetSleep( sx1276 );

    return rnd;
}

/*!
 * Performs the Rx chain calibration for LF and HF bands
 * \remark Must be called just after the reset so all registers are at their
 *         default values
 */
static void RxChainCalibration( SX1276_t* sx1276 )
{
    uint8_t regPaConfigInitVal;
    uint32_t initialFreq;

    // Save context
    regPaConfigInitVal = SX1276Read( sx1276, REG_PACONFIG );
    initialFreq = ( double )( ( ( uint32_t )SX1276Read( sx1276, REG_FRFMSB ) << 16 ) |
                              ( ( uint32_t )SX1276Read( sx1276, REG_FRFMID ) << 8 ) |
                              ( ( uint32_t )SX1276Read( sx1276, REG_FRFLSB ) ) ) * ( double )FREQ_STEP;

    // Cut the PA just in case, RFO output, power = -1 dBm
    SX1276Write( sx1276, REG_PACONFIG, 0x00 );

    // Launch Rx chain calibration for LF band
    SX1276Write( sx1276, REG_IMAGECAL, ( SX1276Read( sx1276, REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( SX1276Read( sx1276, REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }

    // Sets a Frequency in HF band
    SX1276SetChannel( sx1276, 868000000 );

    // Launch Rx chain calibration for HF band
    SX1276Write( sx1276, REG_IMAGECAL, ( SX1276Read( sx1276, REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_MASK ) | RF_IMAGECAL_IMAGECAL_START );
    while( ( SX1276Read( sx1276, REG_IMAGECAL ) & RF_IMAGECAL_IMAGECAL_RUNNING ) == RF_IMAGECAL_IMAGECAL_RUNNING )
    {
    }

    // Restore context
    SX1276Write( sx1276, REG_PACONFIG, regPaConfigInitVal );
    SX1276SetChannel( sx1276, initialFreq );
}

/*!
 * Returns the known FSK bandwidth registers value
 *
 * \param [IN] bandwidth Bandwidth value in Hz
 * \retval regValue Bandwidth register value.
 */
static uint8_t GetFskBandwidthRegValue( uint32_t bandwidth )
{
    uint8_t i;

    for( i = 0; i < ( sizeof( FskBandwidths ) / sizeof( FskBandwidth_t ) ) - 1; i++ )
    {
        if( ( bandwidth >= FskBandwidths[i].bandwidth ) && ( bandwidth < FskBandwidths[i + 1].bandwidth ) )
        {
            return FskBandwidths[i].RegValue;
        }
    }
    // ERROR: Value not found
    while( 1 );
}

void SX1276SetRxConfig( SX1276_t *sx1276, RadioModems_t modem, uint32_t bandwidth,
                         uint32_t datarate, uint8_t coderate,
                         uint32_t bandwidthAfc, uint16_t preambleLen,
                         uint16_t symbTimeout, bool fixLen,
                         uint8_t payloadLen,
                         bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                         bool iqInverted, bool rxContinuous )
{
    SX1276SetModem( sx1276, modem );

    switch( modem )
    {
    case MODEM_FSK:
        {
            sx1276->settings.Fsk.Bandwidth = bandwidth;
            sx1276->settings.Fsk.Datarate = datarate;
            sx1276->settings.Fsk.BandwidthAfc = bandwidthAfc;
            sx1276->settings.Fsk.FixLen = fixLen;
            sx1276->settings.Fsk.PayloadLen = payloadLen;
            sx1276->settings.Fsk.CrcOn = crcOn;
            sx1276->settings.Fsk.IqInverted = iqInverted;
            sx1276->settings.Fsk.RxContinuous = rxContinuous;
            sx1276->settings.Fsk.PreambleLen = preambleLen;

            datarate = ( uint16_t )( ( double )XTAL_FREQ / ( double )datarate );
            SX1276Write( sx1276, REG_BITRATEMSB, ( uint8_t )( datarate >> 8 ) );
            SX1276Write( sx1276, REG_BITRATELSB, ( uint8_t )( datarate & 0xFF ) );

            SX1276Write( sx1276, REG_RXBW, GetFskBandwidthRegValue( bandwidth ) );
            SX1276Write( sx1276, REG_AFCBW, GetFskBandwidthRegValue( bandwidthAfc ) );

            SX1276Write( sx1276, REG_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
            SX1276Write( sx1276, REG_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );

            if( fixLen == 1 )
            {
                SX1276Write( sx1276, REG_PAYLOADLENGTH, payloadLen );
            }
            else
            {
                SX1276Write( sx1276, REG_PAYLOADLENGTH, 0xFF ); // Set payload length to the maximum
            }

            SX1276Write( sx1276, REG_PACKETCONFIG1,
                         ( SX1276Read( sx1276, REG_PACKETCONFIG1 ) &
                           RF_PACKETCONFIG1_CRC_MASK &
                           RF_PACKETCONFIG1_PACKETFORMAT_MASK ) |
                           ( ( fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
                           ( crcOn << 4 ) );
        }
        break;
    case MODEM_LORA:
        {
            if( bandwidth > 2 )
            {
                // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
                while( 1 );
            }
            bandwidth += 7;
            sx1276->settings.LoRa.Bandwidth = bandwidth;
            sx1276->settings.LoRa.Datarate = datarate;
            sx1276->settings.LoRa.Coderate = coderate;
            sx1276->settings.LoRa.PreambleLen = preambleLen;
            sx1276->settings.LoRa.FixLen = fixLen;
            sx1276->settings.LoRa.PayloadLen = payloadLen;
            sx1276->settings.LoRa.CrcOn = crcOn;
            sx1276->settings.LoRa.FreqHopOn = freqHopOn;
            sx1276->settings.LoRa.HopPeriod = hopPeriod;
            sx1276->settings.LoRa.IqInverted = iqInverted;
            sx1276->settings.LoRa.RxContinuous = rxContinuous;

            if( datarate > 12 )
            {
                datarate = 12;
            }
            else if( datarate < 6 )
            {
                datarate = 6;
            }

            if( ( ( bandwidth == 7 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
                ( ( bandwidth == 8 ) && ( datarate == 12 ) ) )
            {
                sx1276->settings.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                sx1276->settings.LoRa.LowDatarateOptimize = 0x00;
            }

            SX1276Write( sx1276, REG_LR_MODEMCONFIG1,
                         ( SX1276Read( sx1276, REG_LR_MODEMCONFIG1 ) &
                           RFLR_MODEMCONFIG1_BW_MASK &
                           RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                           ( bandwidth << 4 ) | ( coderate << 1 ) |
                           fixLen );

            SX1276Write( sx1276, REG_LR_MODEMCONFIG2,
                         ( SX1276Read( sx1276, REG_LR_MODEMCONFIG2 ) &
                           RFLR_MODEMCONFIG2_SF_MASK &
                           RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK &
                           RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) |
                           ( datarate << 4 ) | ( crcOn << 2 ) |
                           ( ( symbTimeout >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) );

            SX1276Write( sx1276, REG_LR_MODEMCONFIG3,
                         ( SX1276Read( sx1276, REG_LR_MODEMCONFIG3 ) &
                           RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
                           ( sx1276->settings.LoRa.LowDatarateOptimize << 3 ) );

            SX1276Write( sx1276, REG_LR_SYMBTIMEOUTLSB, ( uint8_t )( symbTimeout & 0xFF ) );

            SX1276Write( sx1276, REG_LR_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );
            SX1276Write( sx1276, REG_LR_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );

            if( fixLen == 1 )
            {
                SX1276Write( sx1276, REG_LR_PAYLOADLENGTH, payloadLen );
            }

            if( sx1276->settings.LoRa.FreqHopOn == true )
            {
                SX1276Write( sx1276, REG_LR_PLLHOP, (  SX1276Read( sx1276, REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
                SX1276Write( sx1276, REG_LR_HOPPERIOD, sx1276->settings.LoRa.HopPeriod );
            }

            if( ( bandwidth == 9 ) && ( sx1276->settings.Channel > RF_MID_BAND_THRESH ) )
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                SX1276Write( sx1276, REG_LR_TEST36, 0x02 );
                SX1276Write( sx1276, REG_LR_TEST3A, 0x64 );
            }
            else if( bandwidth == 9 )
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                SX1276Write( sx1276, REG_LR_TEST36, 0x02 );
                SX1276Write( sx1276, REG_LR_TEST3A, 0x7F );
            }
            else
            {
                // ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
                SX1276Write( sx1276, REG_LR_TEST36, 0x03 );
            }

            if( datarate == 6 )
            {
                SX1276Write( sx1276, REG_LR_DETECTOPTIMIZE,
                             ( SX1276Read( sx1276, REG_LR_DETECTOPTIMIZE ) &
                               RFLR_DETECTIONOPTIMIZE_MASK ) |
                               RFLR_DETECTIONOPTIMIZE_SF6 );
                SX1276Write( sx1276, REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF6 );
            }
            else
            {
                SX1276Write( sx1276, REG_LR_DETECTOPTIMIZE,
                             ( SX1276Read( sx1276, REG_LR_DETECTOPTIMIZE ) &
                             RFLR_DETECTIONOPTIMIZE_MASK ) |
                             RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
                SX1276Write( sx1276, REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
            }
        }
        break;
    }
}

void SX1276SetTxConfig( SX1276_t *sx1276, RadioModems_t modem, int8_t power, uint32_t fdev,
                        uint32_t bandwidth, uint32_t datarate,
                        uint8_t coderate, uint16_t preambleLen,
                        bool fixLen, bool crcOn, bool freqHopOn,
                        uint8_t hopPeriod, bool iqInverted, uint32_t timeout )
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    SX1276SetModem( sx1276, modem );

    paConfig = SX1276Read( sx1276, REG_PACONFIG );
    paDac = SX1276Read( sx1276, REG_PADAC );

    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | SX1276GetPaSelect( sx1276, sx1276->settings.Channel );
    paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;

    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( power > 17 )
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    SX1276Write( sx1276, REG_PACONFIG, paConfig );
    SX1276Write( sx1276, REG_PADAC, paDac );

    switch( modem )
    {
    case MODEM_FSK:
        {
            sx1276->settings.Fsk.Power = power;
            sx1276->settings.Fsk.Fdev = fdev;
            sx1276->settings.Fsk.Bandwidth = bandwidth;
            sx1276->settings.Fsk.Datarate = datarate;
            sx1276->settings.Fsk.PreambleLen = preambleLen;
            sx1276->settings.Fsk.FixLen = fixLen;
            sx1276->settings.Fsk.CrcOn = crcOn;
            sx1276->settings.Fsk.IqInverted = iqInverted;
            sx1276->settings.Fsk.TxTimeout = timeout;

            fdev = ( uint16_t )( ( double )fdev / ( double )FREQ_STEP );
            SX1276Write( sx1276, REG_FDEVMSB, ( uint8_t )( fdev >> 8 ) );
            SX1276Write( sx1276, REG_FDEVLSB, ( uint8_t )( fdev & 0xFF ) );

            datarate = ( uint16_t )( ( double )XTAL_FREQ / ( double )datarate );
            SX1276Write( sx1276, REG_BITRATEMSB, ( uint8_t )( datarate >> 8 ) );
            SX1276Write( sx1276, REG_BITRATELSB, ( uint8_t )( datarate & 0xFF ) );

            SX1276Write( sx1276, REG_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
            SX1276Write( sx1276, REG_PREAMBLELSB, preambleLen & 0xFF );

            SX1276Write( sx1276, REG_PACKETCONFIG1,
                         ( SX1276Read( sx1276, REG_PACKETCONFIG1 ) &
                           RF_PACKETCONFIG1_CRC_MASK &
                           RF_PACKETCONFIG1_PACKETFORMAT_MASK ) |
                           ( ( fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |
                           ( crcOn << 4 ) );

        }
        break;
    case MODEM_LORA:
        {
            sx1276->settings.LoRa.Power = power;
            if( bandwidth > 2 )
            {
                // Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
                while( 1 );
            }
            bandwidth += 7;
            sx1276->settings.LoRa.Bandwidth = bandwidth;
            sx1276->settings.LoRa.Datarate = datarate;
            sx1276->settings.LoRa.Coderate = coderate;
            sx1276->settings.LoRa.PreambleLen = preambleLen;
            sx1276->settings.LoRa.FixLen = fixLen;
            sx1276->settings.LoRa.FreqHopOn = freqHopOn;
            sx1276->settings.LoRa.HopPeriod = hopPeriod;
            sx1276->settings.LoRa.CrcOn = crcOn;
            sx1276->settings.LoRa.IqInverted = iqInverted;
            sx1276->settings.LoRa.TxTimeout = timeout;

            if( datarate > 12 )
            {
                datarate = 12;
            }
            else if( datarate < 6 )
            {
                datarate = 6;
            }
            if( ( ( bandwidth == 7 ) && ( ( datarate == 11 ) || ( datarate == 12 ) ) ) ||
                ( ( bandwidth == 8 ) && ( datarate == 12 ) ) )
            {
                sx1276->settings.LoRa.LowDatarateOptimize = 0x01;
            }
            else
            {
                sx1276->settings.LoRa.LowDatarateOptimize = 0x00;
            }

            if( sx1276->settings.LoRa.FreqHopOn == true )
            {
                SX1276Write( sx1276, REG_LR_PLLHOP, (  SX1276Read( sx1276, REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
                SX1276Write( sx1276, REG_LR_HOPPERIOD, sx1276->settings.LoRa.HopPeriod );
            }

            SX1276Write( sx1276, REG_LR_MODEMCONFIG1,
                         ( SX1276Read( sx1276, REG_LR_MODEMCONFIG1 ) &
                           RFLR_MODEMCONFIG1_BW_MASK &
                           RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                           ( bandwidth << 4 ) | ( coderate << 1 ) |
                           fixLen );

            SX1276Write( sx1276, REG_LR_MODEMCONFIG2,
                         ( SX1276Read( sx1276, REG_LR_MODEMCONFIG2 ) &
                           RFLR_MODEMCONFIG2_SF_MASK &
                           RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) |
                           ( datarate << 4 ) | ( crcOn << 2 ) );

            SX1276Write( sx1276, REG_LR_MODEMCONFIG3,
                         ( SX1276Read( sx1276, REG_LR_MODEMCONFIG3 ) &
                           RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
                           ( sx1276->settings.LoRa.LowDatarateOptimize << 3 ) );

            SX1276Write( sx1276, REG_LR_PREAMBLEMSB, ( preambleLen >> 8 ) & 0x00FF );
            SX1276Write( sx1276, REG_LR_PREAMBLELSB, preambleLen & 0xFF );

            if( datarate == 6 )
            {
                SX1276Write( sx1276, REG_LR_DETECTOPTIMIZE,
                             ( SX1276Read( sx1276, REG_LR_DETECTOPTIMIZE ) &
                               RFLR_DETECTIONOPTIMIZE_MASK ) |
                               RFLR_DETECTIONOPTIMIZE_SF6 );
                SX1276Write( sx1276, REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF6 );
            }
            else
            {
                SX1276Write( sx1276, REG_LR_DETECTOPTIMIZE,
                             ( SX1276Read( sx1276, REG_LR_DETECTOPTIMIZE ) &
                             RFLR_DETECTIONOPTIMIZE_MASK ) |
                             RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
                SX1276Write( sx1276, REG_LR_DETECTIONTHRESHOLD,
                             RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
            }
        }
        break;
    }
}

uint32_t SX1276GetTimeOnAir( SX1276_t* sx1276, RadioModems_t modem, uint8_t pktLen )
{
    uint32_t airTime = 0;

    switch( modem )
    {
    case MODEM_FSK:
        {
            airTime = round( ( 8 * ( sx1276->settings.Fsk.PreambleLen +
                                     ( ( SX1276Read( sx1276, REG_SYNCCONFIG ) & ~RF_SYNCCONFIG_SYNCSIZE_MASK ) + 1 ) +
                                     ( ( sx1276->settings.Fsk.FixLen == 0x01 ) ? 0.0 : 1.0 ) +
                                     ( ( ( SX1276Read( sx1276, REG_PACKETCONFIG1 ) & ~RF_PACKETCONFIG1_ADDRSFILTERING_MASK ) != 0x00 ) ? 1.0 : 0 ) +
                                     pktLen +
                                     ( ( sx1276->settings.Fsk.CrcOn == 0x01 ) ? 2.0 : 0 ) ) /
                                     sx1276->settings.Fsk.Datarate ) * 1e3 );
        }
        break;
    case MODEM_LORA:
        {
            double bw = 0.0;
            // REMARK: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported
            switch( sx1276->settings.LoRa.Bandwidth )
            {
            //case 0: // 7.8 kHz
            //    bw = 78e2;
            //    break;
            //case 1: // 10.4 kHz
            //    bw = 104e2;
            //    break;
            //case 2: // 15.6 kHz
            //    bw = 156e2;
            //    break;
            //case 3: // 20.8 kHz
            //    bw = 208e2;
            //    break;
            //case 4: // 31.2 kHz
            //    bw = 312e2;
            //    break;
            //case 5: // 41.4 kHz
            //    bw = 414e2;
            //    break;
            //case 6: // 62.5 kHz
            //    bw = 625e2;
            //    break;
            case 7: // 125 kHz
                bw = 125e3;
                break;
            case 8: // 250 kHz
                bw = 250e3;
                break;
            case 9: // 500 kHz
                bw = 500e3;
                break;
            }

            // Symbol rate : time for one symbol (secs)
            double rs = bw / ( 1 << sx1276->settings.LoRa.Datarate );
            double ts = 1 / rs;
            // time of preamble
            double tPreamble = ( sx1276->settings.LoRa.PreambleLen + 4.25 ) * ts;
            // Symbol length of payload and time
            double tmp = ceil( ( 8 * pktLen - 4 * sx1276->settings.LoRa.Datarate +
                                 28 + 16 * sx1276->settings.LoRa.CrcOn -
                                 ( sx1276->settings.LoRa.FixLen ? 20 : 0 ) ) /
                                 ( double )( 4 * sx1276->settings.LoRa.Datarate -
                                 ( ( sx1276->settings.LoRa.LowDatarateOptimize > 0 ) ? 2 : 0 ) ) ) *
                                 ( sx1276->settings.LoRa.Coderate + 4 );
            double nPayload = 8 + ( ( tmp > 0 ) ? tmp : 0 );
            double tPayload = nPayload * ts;
            // Time on air
            double tOnAir = tPreamble + tPayload;
            // return us secs
            airTime = floor( tOnAir * 1e3 + 0.999 );
        }
        break;
    }
    return airTime;
}

void SX1276Send( SX1276_t* sx1276, uint8_t *buffer, uint8_t size )
{
    uint32_t txTimeout = 0;

    switch( sx1276->settings.Modem )
    {
    case MODEM_FSK:
        {
            sx1276->settings.FskPacketHandler.NbBytes = 0;
            sx1276->settings.FskPacketHandler.Size = size;

            if( sx1276->settings.Fsk.FixLen == false )
            {
                SX1276WriteFifo( sx1276, ( uint8_t* )&size, 1 );
            }
            else
            {
                SX1276Write( sx1276, REG_PAYLOADLENGTH, size );
            }

            if( ( size > 0 ) && ( size <= 64 ) )
            {
                sx1276->settings.FskPacketHandler.ChunkSize = size;
            }
            else
            {
                memcpy1( RxTxBuffer, buffer, size );
                sx1276->settings.FskPacketHandler.ChunkSize = 32;
            }

            // Write payload buffer
            SX1276WriteFifo( sx1276, buffer, sx1276->settings.FskPacketHandler.ChunkSize );
            sx1276->settings.FskPacketHandler.NbBytes += sx1276->settings.FskPacketHandler.ChunkSize;
            txTimeout = sx1276->settings.Fsk.TxTimeout;
        }
        break;
    case MODEM_LORA:
        {
            if( sx1276->settings.LoRa.IqInverted == true )
            {
                SX1276Write( sx1276, REG_LR_INVERTIQ, ( ( SX1276Read( sx1276, REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON ) );
                SX1276Write( sx1276, REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
            }
            else
            {
                SX1276Write( sx1276, REG_LR_INVERTIQ, ( ( SX1276Read( sx1276, REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
                SX1276Write( sx1276, REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
            }

            sx1276->settings.LoRaPacketHandler.Size = size;

            // Initializes the payload size
            SX1276Write( sx1276, REG_LR_PAYLOADLENGTH, size );

            // Full buffer used for Tx
            SX1276Write( sx1276, REG_LR_FIFOTXBASEADDR, 0 );
            SX1276Write( sx1276, REG_LR_FIFOADDRPTR, 0 );

            // FIFO operations can not take place in Sleep mode
            if( ( SX1276Read( sx1276, REG_OPMODE ) & ~RF_OPMODE_MASK ) == RF_OPMODE_SLEEP )
            {
                SX1276SetStby( sx1276 );
                sx1276->delay_ms( sx1276->ctx,   1 );
            }
            // Write payload buffer
            SX1276WriteFifo( sx1276, buffer, size );
            txTimeout = sx1276->settings.LoRa.TxTimeout;
        }
        break;
    }

    SX1276SetTx( sx1276, txTimeout );
}

void SX1276SetSleep( SX1276_t* sx1276 )
{
    TimerStop( &RxTimeoutTimer );
    TimerStop( &TxTimeoutTimer );

    SX1276SetOpMode( sx1276, RF_OPMODE_SLEEP );
    sx1276->settings.State = RF_IDLE;
}

void SX1276SetStby( SX1276_t* sx1276 )
{
    TimerStop( &RxTimeoutTimer );
    TimerStop( &TxTimeoutTimer );

    SX1276SetOpMode( sx1276, RF_OPMODE_STANDBY );
    sx1276->settings.State = RF_IDLE;
}

void SX1276SetRx( SX1276_t* sx1276, uint32_t timeout )
{
    bool rxContinuous = false;

    switch( sx1276->settings.Modem )
    {
    case MODEM_FSK:
        {
            rxContinuous = sx1276->settings.Fsk.RxContinuous;

            // DIO0=PayloadReady
            // DIO1=FifoLevel
            // DIO2=SyncAddr
            // DIO3=FifoEmpty
            // DIO4=Preamble
            // DIO5=ModeReady
            SX1276Write( sx1276, REG_DIOMAPPING1, ( SX1276Read( sx1276, REG_DIOMAPPING1 ) & RF_DIOMAPPING1_DIO0_MASK &
                                                                            RF_DIOMAPPING1_DIO1_MASK &
                                                                            RF_DIOMAPPING1_DIO2_MASK ) |
                                                                            RF_DIOMAPPING1_DIO0_00 |
                                                                            RF_DIOMAPPING1_DIO1_00 |
                                                                            RF_DIOMAPPING1_DIO2_11 );

            SX1276Write( sx1276, REG_DIOMAPPING2, ( SX1276Read( sx1276, REG_DIOMAPPING2 ) & RF_DIOMAPPING2_DIO4_MASK &
                                                                            RF_DIOMAPPING2_MAP_MASK ) |
                                                                            RF_DIOMAPPING2_DIO4_11 |
                                                                            RF_DIOMAPPING2_MAP_PREAMBLEDETECT );

            sx1276->settings.FskPacketHandler.FifoThresh = SX1276Read( sx1276, REG_FIFOTHRESH ) & 0x3F;

            SX1276Write( sx1276, REG_RXCONFIG, RF_RXCONFIG_AFCAUTO_ON | RF_RXCONFIG_AGCAUTO_ON | RF_RXCONFIG_RXTRIGER_PREAMBLEDETECT );

            sx1276->settings.FskPacketHandler.PreambleDetected = false;
            sx1276->settings.FskPacketHandler.SyncWordDetected = false;
            sx1276->settings.FskPacketHandler.NbBytes = 0;
            sx1276->settings.FskPacketHandler.Size = 0;
        }
        break;
    case MODEM_LORA:
        {
            if( sx1276->settings.LoRa.IqInverted == true )
            {
                SX1276Write( sx1276, REG_LR_INVERTIQ, ( ( SX1276Read( sx1276, REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF ) );
                SX1276Write( sx1276, REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
            }
            else
            {
                SX1276Write( sx1276, REG_LR_INVERTIQ, ( ( SX1276Read( sx1276, REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
                SX1276Write( sx1276, REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
            }

            // ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal
            if( sx1276->settings.LoRa.Bandwidth < 9 )
            {
                SX1276Write( sx1276, REG_LR_DETECTOPTIMIZE,  SX1276Read( sx1276, REG_LR_DETECTOPTIMIZE ) & 0x7F );
                SX1276Write( sx1276, REG_LR_TEST30, 0x00 );
                switch( sx1276->settings.LoRa.Bandwidth )
                {
                case 0: // 7.8 kHz
                    SX1276Write( sx1276, REG_LR_TEST2F, 0x48 );
                    SX1276SetChannel( sx1276, sx1276->settings.Channel + 7.81e3 );
                    break;
                case 1: // 10.4 kHz
                    SX1276Write( sx1276, REG_LR_TEST2F, 0x44 );
                    SX1276SetChannel( sx1276, sx1276->settings.Channel + 10.42e3 );
                    break;
                case 2: // 15.6 kHz
                    SX1276Write( sx1276, REG_LR_TEST2F, 0x44 );
                    SX1276SetChannel( sx1276, sx1276->settings.Channel + 15.62e3 );
                    break;
                case 3: // 20.8 kHz
                    SX1276Write( sx1276, REG_LR_TEST2F, 0x44 );
                    SX1276SetChannel( sx1276, sx1276->settings.Channel + 20.83e3 );
                    break;
                case 4: // 31.2 kHz
                    SX1276Write( sx1276, REG_LR_TEST2F, 0x44 );
                    SX1276SetChannel( sx1276, sx1276->settings.Channel + 31.25e3 );
                    break;
                case 5: // 41.4 kHz
                    SX1276Write( sx1276, REG_LR_TEST2F, 0x44 );
                    SX1276SetChannel( sx1276, sx1276->settings.Channel + 41.67e3 );
                    break;
                case 6: // 62.5 kHz
                    SX1276Write( sx1276, REG_LR_TEST2F, 0x40 );
                    break;
                case 7: // 125 kHz
                    SX1276Write( sx1276, REG_LR_TEST2F, 0x40 );
                    break;
                case 8: // 250 kHz
                    SX1276Write( sx1276, REG_LR_TEST2F, 0x40 );
                    break;
                }
            }
            else
            {
                SX1276Write( sx1276, REG_LR_DETECTOPTIMIZE,  SX1276Read( sx1276, REG_LR_DETECTOPTIMIZE ) | 0x80 );
            }

            rxContinuous = sx1276->settings.LoRa.RxContinuous;

            if( sx1276->settings.LoRa.FreqHopOn == true )
            {
                SX1276Write( sx1276, REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                                  //RFLR_IRQFLAGS_RXDONE |
                                                  //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=RxDone, DIO2=FhssChangeChannel
                SX1276Write( sx1276, REG_DIOMAPPING1, ( SX1276Read( sx1276, REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK  ) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO2_00 );
            }
            else
            {
                SX1276Write( sx1276, REG_LR_IRQFLAGSMASK, //RFLR_IRQFLAGS_RXTIMEOUT |
                                                  //RFLR_IRQFLAGS_RXDONE |
                                                  //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=RxDone
                SX1276Write( sx1276, REG_DIOMAPPING1, ( SX1276Read( sx1276, REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00 );
            }
            SX1276Write( sx1276, REG_LR_FIFORXBASEADDR, 0 );
            SX1276Write( sx1276, REG_LR_FIFOADDRPTR, 0 );
        }
        break;
    }

    memset( RxTxBuffer, 0, ( size_t )RX_BUFFER_SIZE );

    sx1276->settings.State = RF_RX_RUNNING;
    if( timeout != 0 )
    {
        TimerSetValue( &RxTimeoutTimer, timeout );
        TimerStart( &RxTimeoutTimer );
    }

    if( sx1276->settings.Modem == MODEM_FSK )
    {
        SX1276SetOpMode( sx1276, RF_OPMODE_RECEIVER );

        if( rxContinuous == false )
        {
            TimerSetValue( &RxTimeoutSyncWord, ceil( ( 8.0 * ( sx1276->settings.Fsk.PreambleLen +
                                                             ( ( SX1276Read( sx1276, REG_SYNCCONFIG ) &
                                                                ~RF_SYNCCONFIG_SYNCSIZE_MASK ) +
                                                                1.0 ) + 10.0 ) /
                                                             ( double )sx1276->settings.Fsk.Datarate ) * 1e3 ) + 4 );
            TimerStart( &RxTimeoutSyncWord );
        }
    }
    else
    {
        if( rxContinuous == true )
        {
            SX1276SetOpMode( sx1276, RFLR_OPMODE_RECEIVER );
        }
        else
        {
            SX1276SetOpMode( sx1276, RFLR_OPMODE_RECEIVER_SINGLE );
        }
    }
}

void SX1276SetTx( SX1276_t* sx1276, uint32_t timeout )
{
    TimerSetValue( &TxTimeoutTimer, timeout );

    switch( sx1276->settings.Modem )
    {
    case MODEM_FSK:
        {
            // DIO0=PacketSent
            // DIO1=FifoEmpty
            // DIO2=FifoFull
            // DIO3=FifoEmpty
            // DIO4=LowBat
            // DIO5=ModeReady
            SX1276Write( sx1276, REG_DIOMAPPING1, ( SX1276Read( sx1276, REG_DIOMAPPING1 ) & RF_DIOMAPPING1_DIO0_MASK &
                                                                            RF_DIOMAPPING1_DIO1_MASK &
                                                                            RF_DIOMAPPING1_DIO2_MASK ) |
                                                                            RF_DIOMAPPING1_DIO1_01 );

            SX1276Write( sx1276, REG_DIOMAPPING2, ( SX1276Read( sx1276, REG_DIOMAPPING2 ) & RF_DIOMAPPING2_DIO4_MASK &
                                                                            RF_DIOMAPPING2_MAP_MASK ) );
            sx1276->settings.FskPacketHandler.FifoThresh = SX1276Read( sx1276, REG_FIFOTHRESH ) & 0x3F;
        }
        break;
    case MODEM_LORA:
        {
            if( sx1276->settings.LoRa.FreqHopOn == true )
            {
                SX1276Write( sx1276, REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=TxDone, DIO2=FhssChangeChannel
                SX1276Write( sx1276, REG_DIOMAPPING1, (  SX1276Read( sx1276, REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK & RFLR_DIOMAPPING1_DIO2_MASK ) | RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO2_00 );
            }
            else
            {
                 SX1276Write( sx1276, REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                                  RFLR_IRQFLAGS_RXDONE |
                                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                                  RFLR_IRQFLAGS_VALIDHEADER |
                                                  //RFLR_IRQFLAGS_TXDONE |
                                                  RFLR_IRQFLAGS_CADDONE |
                                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                                  RFLR_IRQFLAGS_CADDETECTED );

                // DIO0=TxDone
                SX1276Write( sx1276, REG_DIOMAPPING1, (  SX1276Read( sx1276, REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_01 );
            }
        }
        break;
    }

    sx1276->settings.State = RF_TX_RUNNING;
    TimerStart( &TxTimeoutTimer );
    SX1276SetOpMode( sx1276, RF_OPMODE_TRANSMITTER );
}

void SX1276StartCad( SX1276_t* sx1276 )
{
    switch( sx1276->settings.Modem )
    {
    case MODEM_FSK:
        {

        }
        break;
    case MODEM_LORA:
        {
             SX1276Write( sx1276, REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                        RFLR_IRQFLAGS_RXDONE |
                                        RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                        RFLR_IRQFLAGS_VALIDHEADER |
                                        RFLR_IRQFLAGS_TXDONE |
                                        //RFLR_IRQFLAGS_CADDONE |
                                        RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL // |
                                        //RFLR_IRQFLAGS_CADDETECTED
                                        );

            // DIO3=CADDone
            SX1276Write( sx1276, REG_DIOMAPPING1, (  SX1276Read( sx1276, REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00 );

            sx1276->settings.State = RF_CAD;
            SX1276SetOpMode( sx1276, RFLR_OPMODE_CAD );
        }
        break;
    default:
        break;
    }
}

int16_t SX1276ReadRssi( SX1276_t* sx1276, RadioModems_t modem )
{
    int16_t rssi = 0;

    switch( modem )
    {
    case MODEM_FSK:
        rssi = -( SX1276Read( sx1276, REG_RSSIVALUE ) >> 1 );
        break;
    case MODEM_LORA:
        if( sx1276->settings.Channel > RF_MID_BAND_THRESH )
        {
            rssi = RSSI_OFFSET_HF + SX1276Read( sx1276, REG_LR_RSSIVALUE );
        }
        else
        {
            rssi = RSSI_OFFSET_LF + SX1276Read( sx1276, REG_LR_RSSIVALUE );
        }
        break;
    default:
        rssi = -1;
        break;
    }
    return rssi;
}

void SX1276SetOpMode( SX1276_t* sx1276, uint8_t opMode )
{
    if( opMode == RF_OPMODE_SLEEP )
    {
        SX1276SetAntSwLowPower( sx1276, true );
    }
    else
    {
        SX1276SetAntSwLowPower( sx1276, false );
        if( opMode == RF_OPMODE_TRANSMITTER )
        {
            SX1276SetAntSw( sx1276, 1 );
        }
        else
        {
            SX1276SetAntSw( sx1276, 0 );
        }
    }
    SX1276Write( sx1276, REG_OPMODE, (  SX1276Read( sx1276, REG_OPMODE ) & RF_OPMODE_MASK ) | opMode );
}

void SX1276SetModem( SX1276_t* sx1276, RadioModems_t modem )
{
    // assert_param( ( sx1276->Spi.Spi.Instance != NULL ) );

    if( sx1276->settings.Modem == modem )
    {
        return;
    }

    sx1276->settings.Modem = modem;
    switch( sx1276->settings.Modem )
    {
    default:
    case MODEM_FSK:
        SX1276SetOpMode( sx1276, RF_OPMODE_SLEEP );
        SX1276Write( sx1276, REG_OPMODE, (  SX1276Read( sx1276, REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF );

        SX1276Write( sx1276, REG_DIOMAPPING1, 0x00 );
        SX1276Write( sx1276, REG_DIOMAPPING2, 0x30 ); // DIO5=ModeReady
        break;
    case MODEM_LORA:
        SX1276SetOpMode( sx1276, RF_OPMODE_SLEEP );
        SX1276Write( sx1276, REG_OPMODE, (  SX1276Read( sx1276, REG_OPMODE ) & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON );

        SX1276Write( sx1276, REG_DIOMAPPING1, 0x00 );
        SX1276Write( sx1276, REG_DIOMAPPING2, 0x00 );
        break;
    }
}

void SX1276Write( SX1276_t* sx1276, uint8_t addr, uint8_t data )
{
    sx1276->write_buffer( sx1276->ctx,  addr, &data, 1 );
}

uint8_t SX1276Read( SX1276_t* sx1276, uint8_t addr )
{
    uint8_t data;
    sx1276->read_buffer( sx1276->ctx,  addr, &data, 1 );
    return data;
}

void SX1276WriteFifo( SX1276_t* sx1276, uint8_t *buffer, uint8_t size )
{
    sx1276->write_buffer( sx1276->ctx,  0, buffer, size );
}

void SX1276ReadFifo( SX1276_t* sx1276, uint8_t *buffer, uint8_t size )
{
    sx1276->read_buffer( sx1276->ctx,  0, buffer, size );
}

void SX1276SetMaxPayloadLength( SX1276_t* sx1276, RadioModems_t modem, uint8_t max )
{
    SX1276SetModem( sx1276, modem );

    switch( modem )
    {
    case MODEM_FSK:
        if( sx1276->settings.Fsk.FixLen == false )
        {
            SX1276Write( sx1276, REG_PAYLOADLENGTH, max );
        }
        break;
    case MODEM_LORA:
        SX1276Write( sx1276, REG_LR_PAYLOADMAXLENGTH, max );
        break;
    }
}

void SX1276OnTimeoutIrq( SX1276_t* sx1276 )
{
    switch( sx1276->settings.State )
    {
    case RF_RX_RUNNING:
        if( sx1276->settings.Modem == MODEM_FSK )
        {
            sx1276->settings.FskPacketHandler.PreambleDetected = false;
            sx1276->settings.FskPacketHandler.SyncWordDetected = false;
            sx1276->settings.FskPacketHandler.NbBytes = 0;
            sx1276->settings.FskPacketHandler.Size = 0;

            // Clear Irqs
            SX1276Write( sx1276, REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI |
                                        RF_IRQFLAGS1_PREAMBLEDETECT |
                                        RF_IRQFLAGS1_SYNCADDRESSMATCH );
            SX1276Write( sx1276, REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN );

            if( sx1276->settings.Fsk.RxContinuous == true )
            {
                // Continuous mode restart Rx chain
                SX1276Write( sx1276, REG_RXCONFIG,  SX1276Read( sx1276, REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
                TimerStart( &RxTimeoutSyncWord );
            }
            else
            {
                sx1276->settings.State = RF_IDLE;
                TimerStop( &RxTimeoutSyncWord );
            }
        }
        if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
        {
            RadioEvents->RxTimeout( );
        }
        break;
    case RF_TX_RUNNING:
        sx1276->settings.State = RF_IDLE;
        if( ( RadioEvents != NULL ) && ( RadioEvents->TxTimeout != NULL ) )
        {
            RadioEvents->TxTimeout( );
        }
        break;
    default:
        break;
    }
}

void SX1276OnDio0Irq( SX1276_t* sx1276 )
{
    volatile uint8_t irqFlags = 0;

    switch( sx1276->settings.State )
    {
        case RF_RX_RUNNING:
            //TimerStop( &RxTimeoutTimer );
            // RxDone interrupt
            switch( sx1276->settings.Modem )
            {
            case MODEM_FSK:
                if( sx1276->settings.Fsk.CrcOn == true )
                {
                    irqFlags = SX1276Read( sx1276, REG_IRQFLAGS2 );
                    if( ( irqFlags & RF_IRQFLAGS2_CRCOK ) != RF_IRQFLAGS2_CRCOK )
                    {
                        // Clear Irqs
                         SX1276Write( sx1276, REG_IRQFLAGS1, RF_IRQFLAGS1_RSSI |
                                                    RF_IRQFLAGS1_PREAMBLEDETECT |
                                                    RF_IRQFLAGS1_SYNCADDRESSMATCH );
                        SX1276Write( sx1276, REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN );

                        TimerStop( &RxTimeoutTimer );

                        if( sx1276->settings.Fsk.RxContinuous == false )
                        {
                            TimerStop( &RxTimeoutSyncWord );
                            sx1276->settings.State = RF_IDLE;
                        }
                        else
                        {
                            // Continuous mode restart Rx chain
                            SX1276Write( sx1276, REG_RXCONFIG,  SX1276Read( sx1276, REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
                            TimerStart( &RxTimeoutSyncWord );
                        }

                        if( ( RadioEvents != NULL ) && ( RadioEvents->RxError != NULL ) )
                        {
                            RadioEvents->RxError( );
                        }
                        sx1276->settings.FskPacketHandler.PreambleDetected = false;
                        sx1276->settings.FskPacketHandler.SyncWordDetected = false;
                        sx1276->settings.FskPacketHandler.NbBytes = 0;
                        sx1276->settings.FskPacketHandler.Size = 0;
                        break;
                    }
                }

                // Read received packet size
                if( ( sx1276->settings.FskPacketHandler.Size == 0 ) && ( sx1276->settings.FskPacketHandler.NbBytes == 0 ) )
                {
                    if( sx1276->settings.Fsk.FixLen == false )
                    {
                        SX1276ReadFifo( sx1276, ( uint8_t* )sx1276->settings.FskPacketHandler.Size, 1 );
                    }
                    else
                    {
                        sx1276->settings.FskPacketHandler.Size = SX1276Read( sx1276, REG_PAYLOADLENGTH );
                    }
                    SX1276ReadFifo( sx1276, RxTxBuffer + sx1276->settings.FskPacketHandler.NbBytes, sx1276->settings.FskPacketHandler.Size - sx1276->settings.FskPacketHandler.NbBytes );
                    sx1276->settings.FskPacketHandler.NbBytes += ( sx1276->settings.FskPacketHandler.Size - sx1276->settings.FskPacketHandler.NbBytes );
                }
                else
                {
                    SX1276ReadFifo( sx1276, RxTxBuffer + sx1276->settings.FskPacketHandler.NbBytes, sx1276->settings.FskPacketHandler.Size - sx1276->settings.FskPacketHandler.NbBytes );
                    sx1276->settings.FskPacketHandler.NbBytes += ( sx1276->settings.FskPacketHandler.Size - sx1276->settings.FskPacketHandler.NbBytes );
                }

                if( sx1276->settings.Fsk.RxContinuous == false )
                {
                    sx1276->settings.State = RF_IDLE;
                    TimerStart( &RxTimeoutSyncWord );
                }
                else
                {
                    // Continuous mode restart Rx chain
                    SX1276Write( sx1276, REG_RXCONFIG,  SX1276Read( sx1276, REG_RXCONFIG ) | RF_RXCONFIG_RESTARTRXWITHOUTPLLLOCK );
                }
                TimerStop( &RxTimeoutTimer );

                if( ( RadioEvents != NULL ) && ( RadioEvents->RxDone != NULL ) )
                {
                    RadioEvents->RxDone( RxTxBuffer, sx1276->settings.FskPacketHandler.Size, sx1276->settings.FskPacketHandler.RssiValue, 0 );
                }
                sx1276->settings.FskPacketHandler.PreambleDetected = false;
                sx1276->settings.FskPacketHandler.SyncWordDetected = false;
                sx1276->settings.FskPacketHandler.NbBytes = 0;
                sx1276->settings.FskPacketHandler.Size = 0;
                break;
            case MODEM_LORA:
                {
                    int8_t snr = 0;

                    // Clear Irq
                    SX1276Write( sx1276, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE );

                    irqFlags = SX1276Read( sx1276, REG_LR_IRQFLAGS );
                    if( ( irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
                    {
                        // Clear Irq
                        SX1276Write( sx1276, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR );

                        if( sx1276->settings.LoRa.RxContinuous == false )
                        {
                            sx1276->settings.State = RF_IDLE;
                        }
                        TimerStop( &RxTimeoutTimer );

                        if( ( RadioEvents != NULL ) && ( RadioEvents->RxError != NULL ) )
                        {
                            RadioEvents->RxError( );
                        }
                        break;
                    }

                    sx1276->settings.LoRaPacketHandler.SnrValue = SX1276Read( sx1276, REG_LR_PKTSNRVALUE );
                    if( sx1276->settings.LoRaPacketHandler.SnrValue & 0x80 ) // The SNR sign bit is 1
                    {
                        // Invert and divide by 4
                        snr = ( ( ~sx1276->settings.LoRaPacketHandler.SnrValue + 1 ) & 0xFF ) >> 2;
                        snr = -snr;
                    }
                    else
                    {
                        // Divide by 4
                        snr = ( sx1276->settings.LoRaPacketHandler.SnrValue & 0xFF ) >> 2;
                    }

                    int16_t rssi = SX1276Read( sx1276, REG_LR_PKTRSSIVALUE );
                    if( snr < 0 )
                    {
                        if( sx1276->settings.Channel > RF_MID_BAND_THRESH )
                        {
                            sx1276->settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 ) +
                                                                          snr;
                        }
                        else
                        {
                            sx1276->settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 ) +
                                                                          snr;
                        }
                    }
                    else
                    {
                        if( sx1276->settings.Channel > RF_MID_BAND_THRESH )
                        {
                            sx1276->settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_HF + rssi + ( rssi >> 4 );
                        }
                        else
                        {
                            sx1276->settings.LoRaPacketHandler.RssiValue = RSSI_OFFSET_LF + rssi + ( rssi >> 4 );
                        }
                    }

                    sx1276->settings.LoRaPacketHandler.Size = SX1276Read( sx1276, REG_LR_RXNBBYTES );
                    SX1276ReadFifo( sx1276, RxTxBuffer, sx1276->settings.LoRaPacketHandler.Size );

                    if( sx1276->settings.LoRa.RxContinuous == false )
                    {
                        sx1276->settings.State = RF_IDLE;
                    }
                    TimerStop( &RxTimeoutTimer );

                    if( ( RadioEvents != NULL ) && ( RadioEvents->RxDone != NULL ) )
                    {
                        RadioEvents->RxDone( RxTxBuffer, sx1276->settings.LoRaPacketHandler.Size, sx1276->settings.LoRaPacketHandler.RssiValue, sx1276->settings.LoRaPacketHandler.SnrValue );
                    }
                }
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            TimerStop( &TxTimeoutTimer );
            // TxDone interrupt
            switch( sx1276->settings.Modem )
            {
            case MODEM_LORA:
                // Clear Irq
                SX1276Write( sx1276, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE );
                // Intentional fall through
            case MODEM_FSK:
            default:
                sx1276->settings.State = RF_IDLE;
                if( ( RadioEvents != NULL ) && ( RadioEvents->TxDone != NULL ) )
                {
                    RadioEvents->TxDone( );
                }
                break;
            }
            break;
        default:
            break;
    }
}

void SX1276OnDio1Irq( SX1276_t* sx1276 )
{
    switch( sx1276->settings.State )
    {
        case RF_RX_RUNNING:
            switch( sx1276->settings.Modem )
            {
            case MODEM_FSK:
                // FifoLevel interrupt
                // Read received packet size
                if( ( sx1276->settings.FskPacketHandler.Size == 0 ) && ( sx1276->settings.FskPacketHandler.NbBytes == 0 ) )
                {
                    if( sx1276->settings.Fsk.FixLen == false )
                    {
                        SX1276ReadFifo( sx1276, ( uint8_t* )sx1276->settings.FskPacketHandler.Size, 1 );
                    }
                    else
                    {
                        sx1276->settings.FskPacketHandler.Size = SX1276Read( sx1276, REG_PAYLOADLENGTH );
                    }
                }

                if( ( sx1276->settings.FskPacketHandler.Size - sx1276->settings.FskPacketHandler.NbBytes ) > sx1276->settings.FskPacketHandler.FifoThresh )
                {
                    SX1276ReadFifo( sx1276, ( RxTxBuffer + sx1276->settings.FskPacketHandler.NbBytes ), sx1276->settings.FskPacketHandler.FifoThresh );
                    sx1276->settings.FskPacketHandler.NbBytes += sx1276->settings.FskPacketHandler.FifoThresh;
                }
                else
                {
                    SX1276ReadFifo( sx1276, ( RxTxBuffer + sx1276->settings.FskPacketHandler.NbBytes ), sx1276->settings.FskPacketHandler.Size - sx1276->settings.FskPacketHandler.NbBytes );
                    sx1276->settings.FskPacketHandler.NbBytes += ( sx1276->settings.FskPacketHandler.Size - sx1276->settings.FskPacketHandler.NbBytes );
                }
                break;
            case MODEM_LORA:
                // Sync time out
                TimerStop( &RxTimeoutTimer );
                // Clear Irq
                SX1276Write( sx1276, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXTIMEOUT );

                sx1276->settings.State = RF_IDLE;
                if( ( RadioEvents != NULL ) && ( RadioEvents->RxTimeout != NULL ) )
                {
                    RadioEvents->RxTimeout( );
                }
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            switch( sx1276->settings.Modem )
            {
            case MODEM_FSK:
                // FifoEmpty interrupt
                if( ( sx1276->settings.FskPacketHandler.Size - sx1276->settings.FskPacketHandler.NbBytes ) > sx1276->settings.FskPacketHandler.ChunkSize )
                {
                    SX1276WriteFifo( sx1276, ( RxTxBuffer + sx1276->settings.FskPacketHandler.NbBytes ), sx1276->settings.FskPacketHandler.ChunkSize );
                    sx1276->settings.FskPacketHandler.NbBytes += sx1276->settings.FskPacketHandler.ChunkSize;
                }
                else
                {
                    // Write the last chunk of data
                    SX1276WriteFifo( sx1276, RxTxBuffer + sx1276->settings.FskPacketHandler.NbBytes, sx1276->settings.FskPacketHandler.Size - sx1276->settings.FskPacketHandler.NbBytes );
                    sx1276->settings.FskPacketHandler.NbBytes += sx1276->settings.FskPacketHandler.Size - sx1276->settings.FskPacketHandler.NbBytes;
                }
                break;
            case MODEM_LORA:
                break;
            default:
                break;
            }
            break;
        default:
            break;
    }
}

void SX1276OnDio2Irq( SX1276_t* sx1276 )
{
    switch( sx1276->settings.State )
    {
        case RF_RX_RUNNING:
            switch( sx1276->settings.Modem )
            {
            case MODEM_FSK:
                if( ( sx1276->settings.FskPacketHandler.PreambleDetected == true ) && ( sx1276->settings.FskPacketHandler.SyncWordDetected == false ) )
                {
                    TimerStop( &RxTimeoutSyncWord );

                    sx1276->settings.FskPacketHandler.SyncWordDetected = true;

                    sx1276->settings.FskPacketHandler.RssiValue = -( SX1276Read( sx1276, REG_RSSIVALUE ) >> 1 );

                    sx1276->settings.FskPacketHandler.AfcValue = ( int32_t )( double )( ( ( uint16_t ) SX1276Read( sx1276, REG_AFCMSB ) << 8 ) |
                                                                           ( uint16_t ) SX1276Read( sx1276, REG_AFCLSB ) ) *
                                                                           ( double )FREQ_STEP;
                    sx1276->settings.FskPacketHandler.RxGain = (  SX1276Read( sx1276, REG_LNA ) >> 5 ) & 0x07;
                }
                break;
            case MODEM_LORA:
                if( sx1276->settings.LoRa.FreqHopOn == true )
                {
                    // Clear Irq
                    SX1276Write( sx1276, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );

                    if( ( RadioEvents != NULL ) && ( RadioEvents->FhssChangeChannel != NULL ) )
                    {
                        RadioEvents->FhssChangeChannel( ( SX1276Read( sx1276, REG_LR_HOPCHANNEL ) & RFLR_HOPCHANNEL_CHANNEL_MASK ) );
                    }
                }
                break;
            default:
                break;
            }
            break;
        case RF_TX_RUNNING:
            switch( sx1276->settings.Modem )
            {
            case MODEM_FSK:
                break;
            case MODEM_LORA:
                if( sx1276->settings.LoRa.FreqHopOn == true )
                {
                    // Clear Irq
                    SX1276Write( sx1276, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL );

                    if( ( RadioEvents != NULL ) && ( RadioEvents->FhssChangeChannel != NULL ) )
                    {
                        RadioEvents->FhssChangeChannel( ( SX1276Read( sx1276, REG_LR_HOPCHANNEL ) & RFLR_HOPCHANNEL_CHANNEL_MASK ) );
                    }
                }
                break;
            default:
                break;
            }
            break;
        default:
            break;
    }
}

void SX1276OnDio3Irq( SX1276_t* sx1276 )
{
    switch( sx1276->settings.Modem )
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        if( ( SX1276Read( sx1276, REG_LR_IRQFLAGS ) & RFLR_IRQFLAGS_CADDETECTED ) == RFLR_IRQFLAGS_CADDETECTED )
        {
            // Clear Irq
            SX1276Write( sx1276, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED | RFLR_IRQFLAGS_CADDONE );
            if( ( RadioEvents != NULL ) && ( RadioEvents->CadDone != NULL ) )
            {
                RadioEvents->CadDone( true );
            }
        }
        else
        {
            // Clear Irq
            SX1276Write( sx1276, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE );
            if( ( RadioEvents != NULL ) && ( RadioEvents->CadDone != NULL ) )
            {
                RadioEvents->CadDone( false );
            }
        }
        break;
    default:
        break;
    }
}

void SX1276OnDio4Irq( SX1276_t* sx1276 )
{
    switch( sx1276->settings.Modem )
    {
    case MODEM_FSK:
        {
            if( sx1276->settings.FskPacketHandler.PreambleDetected == false )
            {
                sx1276->settings.FskPacketHandler.PreambleDetected = true;
            }
        }
        break;
    case MODEM_LORA:
        break;
    default:
        break;
    }
}

void SX1276OnDio5Irq( SX1276_t* sx1276 )
{
    switch( sx1276->settings.Modem )
    {
    case MODEM_FSK:
        break;
    case MODEM_LORA:
        break;
    default:
        break;
    }
}
