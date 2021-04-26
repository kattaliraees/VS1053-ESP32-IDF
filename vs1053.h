#include "defines.h"

/*
 * VS1053.h
 *
 *  Created on: Oct 5, 2020
 *      Author: Raees
 */

//------------------------------------------------------------------------------
/** \name SCI_Register_Group
 *  VS10xx Serial Control Interface (aka SCI) Registers
 *  /@{
 */

/**
 * \brief A macro of the SCI MODE register's address (R/W)
 *
 * SCI_MODE is a Read/Write register used to control the operation of VS1053b and defaults to 0x0800
 * (SM_SDINEW set).
 *
 * \see SCI_MODE_Bit_Definitions_Group
 *
 */
#define SCI_MODE              0x00

/**
 * \brief A macro of the SCI STATUS register's address (R/W)
 *
 * SCI_STATUS is a Read/Write register containing information on the current status of VS1053b. It also controls some
 * low-level things that the user does not usually have to care about.
 */
#define SCI_STATUS            0x01

/**
 * \brief A macro of the SCI BASS register's address (R/W)
 *
 * SCI_BASS is a Read/Write register used to control VSBE.
 * The Bass Enhancer VSBE is a powerful bass boosting DSP algorithm, which tries to take the
 * most out of the users earphones without causing clipping.
 */
#define SCI_BASS              0x02

/**
 * \brief A macro of the SCI CLOCKF register's address (R/W)
 *
 * SCI_CLOCKF is a Read/Write register used to control Clock Multipler.
 */
#define SCI_CLOCKF            0x03

/**
 * \brief A macro of the SCI Decode Time register's address (R/W)
 *
 * SCI_DECODE_TIME is a Read/Write register containing information when decoding correct data,
 * current decoded time is shown in this register in full seconds.
 * The user may change the value of this register. In that case the new value should be written
 * twice to make absolutely certain that the change is not overwritten by the firmware. A write to
 * SCI_DECODE_TIME also resets the byteRate calculation.
 * SCI_DECODE_TIME is reset at every hardware and software reset. It is no longer cleared
 * when decoding of a file ends to allow the decode time to proceed automatically with looped
 * files and with seamless playback of multiple files.
 */
#define SCI_DECODE_TIME       0x04

/**
 * \brief A macro of the SCI AUDATA register's address (R/W)
 *
 * SCI_AUDATA is a Read/Write register containing information when decoding correct data,
 * the current samplerate and number of channels can be found
 * in bits 15:1 and 0 of SCI_AUDATA, respectively. Bits 15:1 contain the samplerate divided by
 * two, and bit 0 is 0 for mono data and 1 for stereo. Writing to SCI_AUDATA will change the
 * samplerate directly.
 */
#define SCI_AUDATA            0x05

/**
 * \brief A macro of the SCI WRAM register's address (R/W)
 *
 * SCI_AUDATA is a Read/Write register used to upload application programs and data to instruction and data RAMs.
 * The start address must be initialized by writing to SCI_WRAMADDR prior to the first write/read
 * of SCI_WRAM. As 16 bits of data can be transferred with one SCI_WRAM write/read, and the
 * instruction word is 32 bits long, two consecutive writes/reads are needed for each instruction
 * word. The byte order is big-endian (i.e. most significant words first). After each full-word
 * write/read, the internal pointer is autoincremented.
 */
#define SCI_WRAM              0x06

/**
 * \brief A macro of the SCI WRAMADDR register (W)
 *
 * SCI_WRAMADDR is a Write only register used to set the program address for following SCI_WRAM writes/reads.
 * Use an address offset from the following table to access X, Y, I or peripheral memory.
 */
#define SCI_WRAMADDR          0x07

/**
 * \brief A macro of the SCI HDAT0 register's address (R/W)
 * SCI_HDAT0 register is a Read/Write register which contains header information that is extracted from MP3 stream
 * currently being decoded. After reset both registers are cleared, indicating no data has been found yet.
 *
 * The register is used in conjuction with HDAT1 to provide various information about
 * the current operating mode of the VSdsp. Where its value may represent different information based on
 * the value of HDAT1. Typically, HDAT0's value indicates something about the various speed or rate.
 * \see <A HREF = "http://www.vlsi.fi/en/products/vs1053.html"> VS1053 Datasheet</A>.
 */
#define SCI_HDAT0             0x08
/**
 * \brief A macro of the SCI HDAT1 register's address (R/W)
 *
 * SCI_HDAT1 register is a Read/Write register which contains header information that is extracted from MP3 stream
 * currently being decoded. After reset both registers are cleared, indicating no data has been found yet.
 *
 * The register is used in conjuction with HDAT0 to provide various information about
 * the current operating mode of the VSdsp. Where its value typically indicates the encoding format of the
 * current stream of playback.
 * \see <A HREF = "http://www.vlsi.fi/en/products/vs1053.html"> VS1053 Datasheet</A>.
 */
#define SCI_HDAT1             0x09

/**
 * \brief A macro of the SCI AIADDR register's address (R/W)
 *
 * SCI_AIADDR is a Read/Write register indicates the start address of the application code written
 * earlier with SCI_WRAMADDR and SCI_WRAM registers. If no application code is used, this register
 * should not be initialized,or it should be initialized to zero. For more details,
 * \see Application Notes for VS10XX.
 *
 * \note Reading AIADDR is not recommended. It can cause samplerate to be set to a very low value.
 */
#define SCI_AIADDR            0x0A

/**
 * \brief A macro of the SCI VOL register's address (R/W)
 *
 * SCI_VOL is a Read/Write register to control master volume for the player hardware.
 * The most significant byte of the volume register controls the left channel volume,
 * the low part controls the right channel volume. The channel volume sets the attenuation from the
 * maximum volume level in 0.5 dB steps. Thus, maximum volume is 0x0000 and total silence is 0xFEFE.
 *
 * \note After hardware reset the volume is set to full volume. Resetting the software does not reset the volume setting.
 *
 * Setting SCI_VOL to 0xFFFF will activate analog powerdown mode.
 */
#define SCI_VOL               0x0B

/**
 * \brief A macro of the SCI AICTRL[x] register's address (R/W)
 *
 * SCI_AICTRL[x] registers ( x=[0 .. 3] ) are Read/Write that can be used to access the user's application program.
 * The AICTRL registers are also used with PCM/ADPCM encoding mode.
 */
#define SCI_AICTRL0           0x0C

/**
 * \brief A macro of the SCI AICTRL[x] register's address (R/W)
 *
 * SCI_AICTRL[x] registers ( x=[0 .. 3] ) are Read/Write that can be used to access the user's application program.
 * The AICTRL registers are also used with PCM/ADPCM encoding mode.
 */
#define SCI_AICTRL1           0x0D

/**
 * \brief A macro of the SCI AICTRL[x] register's address (R/W)
 *
 * SCI_AICTRL[x] registers ( x=[0 .. 3] ) are Read/Write that can be used to access the user's application program.
 * The AICTRL registers are also used with PCM/ADPCM encoding mode.
 */
#define SCI_AICTRL2           0x0E

/**
 * \brief A macro of the SCI AICTRL[x] register (R/W)
 *
 * SCI_AICTRL[x] registers ( x=[0 .. 3] ) are Read/Write that can be used to access the user's application program.
 * The AICTRL registers are also used with PCM/ADPCM encoding mode.
 */
#define SCI_AICTRL3           0x0F

/** End SCI_Register_Group
 *  /@}
 */

//------------------------------------------------------------------------------
/** \name SCI_MODE_Bit_Definitions_Group
 *  VS10xx SCI_MODE bitmasks
 *  /@{
 */

/**
 * \brief A macro of the SM_DIFF bit mask of the SCI_MODE register
 *
 * When SM_DIFF is set, the player inverts the left channel output. For a stereo input this creates
 * virtual surround, and for a mono input this creates a differential left/right signal.
 */
#define SM_DIFF             0x0001

/**
 * \brief A macro of the SM_LAYER12 bit mask of the SCI_MODE register
 *
 * SM_LAYER12 enables MPEG 1.0 and 2.0 layer I and II decoding in addition to layer III. If you
 * enable Layer I and Layer II decoding, you are liable for any patent issues that may arise.
 * Joint licensing of MPEG 1.0 / 2.0 Layer III does not cover all patents pertaining to layers I and II.
 */
#define SM_LAYER12          0x0002

/**
 * \brief A macro of the SM_DIFF bit mask of the SM_RESET register
 *
 * Software reset is initiated by setting SM_RESET to 1. This bit is cleared automatically.
 */
#define SM_RESET            0x0004

/**
 * \brief A macro of the SM_DIFF bit mask of the SCI_MODE register
 *
 * If you want to stop decoding a in the middle, set SM_CANCEL, and continue sending data
 * honouring DREQ. When SM_CANCEL is detected by a codec, it will stop decoding and return
 * to the main loop. The stream buffer content is discarded and the SM_CANCEL bit cleared.
 * SCI_HDAT1 will also be cleared. See Chapter 9.5.2 for details.
 */
#define SM_CANCEL           0x0008

/**
 * \brief A macro of the SM_EARSPEAKER_LO bit mask of the SCI_MODE register
 *
 * Bits SM_EARSPEAKER_LO and SM_EARSPEAKER_HI control the EarSpeaker spatial processing.
 * If both are 0, the processing is not active. Other combinations activate the processing
 * and select 3 different effect levels: LO = 1, HI = 0 selects minimal, LO = 0, HI = 1 selects normal,
 * and LO = 1, HI = 1 selects extreme. EarSpeaker takes approximately 12 MIPS at 44.1 kHz
 * samplerate.
 */
#define SM_EARSPEAKER_LO    0x0010

/**
 * \brief A macro of the SM_TESTS bit mask of the SCI_MODE register
 *
 * Bits SM_EARSPEAKER_LO and SM_EARSPEAKER_HI control the EarSpeaker spatial processing.
 * If both are 0, the processing is not active. Other combinations activate the processing
 * and select 3 different effect levels: LO = 1, HI = 0 selects minimal, LO = 0, HI = 1 selects normal,
 * and LO = 1, HI = 1 selects extreme. EarSpeaker takes approximately 12 MIPS at 44.1 kHz
 * samplerate.
 */
#define SM_TESTS            0x0020

/**
 * \brief A macro of the SM_STREAM bit mask of the SCI_MODE register
 *
 * If SM_TESTS is set, SDI tests are allowed. For more details on SDI tests, look at Chapter 9.12.
 */
#define SM_STREAM           0x0040

/**
 * \brief A macro of the SM_EARSPEAKER_HI bit mask of the SCI_MODE register
 *
 * SM_STREAM activates VS1053b's stream mode. In this mode, data should be sent with as
 * even intervals as possible and preferable in blocks of less than 512 bytes, and VS1053b makes
 * every attempt to keep its input buffer half full by changing its playback speed upto 5%. For best
 * quality sound, the average speed error should be within 0.5%, the bitrate should not exceed
 * 160 kbit/s and VBR should not be used. For details, see Application Notes for VS10XX. This
 * mode only works with MP3 and WAV files.
 */
#define SM_EARSPEAKER_HI    0x0080

/**
 * \brief A macro of the SM_DACT bit mask of the SCI_MODE register
 *
 * SM_DACT defines the active edge of data clock for SDI. When '0', data is read at the rising
 * edge, when '1', data is read at the falling edge.
 */
#define SM_DACT             0x0100

/**
 * \brief A macro of the SM_SDIORD bit mask of the SCI_MODE register
 *
 * When SM_SDIORD is clear, bytes on SDI are sent MSb first. By setting SM_SDIORD, the user
 * may reverse the bit order for SDI, i.e. bit 0 is received first and bit 7 last. Bytes are, however,
 * still sent in the default order. This register bit has no effect on the SCI bus.
*
 * \note This value is set correctly and should not be changed for use with AVR's Arduiino ATmega SPI operation.r.
 */
#define SM_SDIORD           0x0200

/**
 * \brief A macro of the SM_SDISHARE bit mask of the SCI_MODE register
 *
 * Setting SM_SDISHARE makes SCI and SDI share the same chip select, as explained in Chapter
 * 7.2, if also SM_SDINEW is set.
 *
 * \note This value is set correctly and should not be changed for use with AVR's Arduiino ATmega SPI operation.r.
 */
#define SM_SDISHARE         0x0400

/**
 * \brief A macro of the SM_SDINEW bit mask of the SCI_MODE register
 *
 * Setting SM_SDINEW will activate VS1002 native serial modes as described in Chapters 7.2.1 and 7.4.2.
 * This bit is set as a default when VS1053b is started up.
 *
 * \note This value is set correctly and should not be changed for use with AVR's Arduiino ATmega SPI operation.r.
 */
#define SM_SDINEW           0x0800

/**
 * \brief A macro of the SM_ADPCM bit mask of the SCI_MODE register
 *
 * By activating SM_ADPCM and SM_RESET at the same time, the user will activate IMA ADPCM
 * recording mode (see section 9.8).
 */
#define SM_ADPCM            0x1000

/**
 * \brief A macro of the SM_PAUSE bit mask of the SCI_MODE register
 *
 * Play pause is traditionally implemented by stopping sending data to VS10xx. With lowbitrate
 * songs, especially MIDI, decoding will continue for a long time because stream
 * buffer is filled with data. This patch implements pause by stopping audio generation if
 * bit 13 in SCI_MODE is set.
 *
 * \note This is only available with patch loaded. Typically done with SFEMP3Shield::vs_init()
 */
#define SM_PAUSE            0x2000  // note: Only availble with patch. This only quickly pauses the VS's internal buffer, when canceling quickly. It won't unpause.

/**
 * \brief A macro of the SM_LINE1 bit mask of the SCI_MODE register
 *
 * SM_LINE_IN is used to select the left-channel input for ADPCM recording. If '0', differential
 * microphone input pins MICP and MICN are used; if '1', line-level MICP/LINEIN1 pin is used.
 *
 * \see VS_LINE1_MODE
 */
#define SM_LINE1            0x4000

/**
 * \brief A macro of the SM_CLK_RANGE bit mask of the SCI_MODE register
 *
 * SM_CLK_RANGE activates a clock divider in the XTAL input. When SM_CLK_RANGE is set,
 * the clock is divided by 2 at the input. From the chip's point of view e.g. 24 MHz becomes
 * 12 MHz. SM_CLK_RANGE should be set as soon as possible after a chip reset.
 */
#define SM_CLK_RANGE        0x8000

/**
 * \brief A macro of the VS_LINE1_MODE to configure either Line level or
 * microphone input.
 *
 * Used by SFEMP3Shield::ADMixerLoad to determine if the SM_LINE1 of SCI_MODE being set as to
 * use both MICP and LINEIN1 pins as \b stereo input, at line levels.
 * Commenting it out, will result MICP and MICN used as differential input with
 * resulting \b mono signal on the left channel.
 *
 * \see SM_LINE1
 */
// configure Line1 as single ended, otherwise as differential 10x gain for microphones.
#define VS_LINE1_MODE



/** End SCI_MODE_Group
 *  /@}
 */

//------------------------------------------------------------------------------
/** \name SCI_STATUS_Bit_Definitions_Group
 *  VS10xx SCI_STATUS bitmasks
 *  /@{
 */

/**
 * \brief A macro of the SS_VU_ENABLE bit mask of the SCI_STATUS register
 *
 * When SS_VU_ENABLE is set and patch 1.2 or greater is loaded the player the
 * VU meter is enabled.
 * See data patches data sheet VU meter for details.
 * \see setVUmeter and getVUlevel
 */
#define SS_VU_ENABLE        0x0200

/** End SCI_STATUS_Group
 *  /@}
 */


void vs1053_init();
void vs1053_write_sci(uint8_t addr, uint16_t data);
void vs1053_write_sdi(uint8_t *data, uint8_t bytes);
uint16_t vs1053_read_sci(uint8_t addr);
void vs1053_set_volume(uint8_t vol);
