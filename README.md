## British GPO 746 bluetooth conversion (HFP)

Firmware and hardware related resources to add hands-free bluetooth support to a 746 british phone - WIP.

## Hardware requirements

* esp32 devboard (v1)
* Raspberry PI Pico
* external DAC (TLC5615)
* 746 telephone

## Software dependencies

* esp-idf (v5.1)
* pico-sdk ( >= v1.5.0)
* opencd (optional)
* picoprobe (optiona)

## Audio input/output

In a nutshell, the ESP32 is configured with `SCO data path = HCI`, this enables the improved mSBC codec on some AGs; the audio in/out is received/sent from/to a Raspberry Pi Pico I2C slave, acting as a recording/playback device.  
The microphone is sampled using the Pico's 12bit ADC, while the earpiece speaker is driven by a buffered 10bit external DAC which interfaces with the Pico using SPI.

This is more complicated than it should mostly due to my lack of experience with the ESP32 and lack of a propper external ADC.  
My initial plan was to use the ESP32's ADC & DAC simultaneous, but the I've run into the following problems:
1. the ADC & DAC cannot be used at the same time in DMA Mode, since they both share the same DMA fifo of the I2S0 peripheral: [ADC continuous mode Hardware Limitations](https://docs.espressif.com/projects/esp-idf/en/v5.1/esp32/api-reference/peripherals/adc_continuous.html?highlight=adc_continuous_read#hardware-limitations) - I couldn't find any proper way of making it work
2. even if both ADC & DAC could work in continuous DMA mode, the ADC configuration doesn't allow sampling frequencies below 20khz (for mSBC & CVSD codecs a lower sampling rate is required: 16khz, 8khz respectively)
3. the ESP32 doesn't have an analog ground which makes both the ADC & DAC really noisy

In a previous iteration of the project, both the Pico and the external DAC were connected to the ESP32 via SPI, but it struggled to keep a constant rate of sending/receiving PCM samples during an active call because each PCM sample required its own SPI transaction (both the Pico and the TLC5615 support only 2 byte transactions) - this cause horrible audio on both ends.

## Audio transmission
The Pico is configured as a slave I2C device with an additional two GPIO input pins configured as an ENABLE pin controlled by the ESP32 and a DATA READY pin which is toggled HIGH by the Pico when samples are avalable from the ADC.
Basic data flow between ESP32 and PICO:

    1. (master) set ENABLE pin HIGH
    2. (master) write sample rate via I2C
    3. (slave) read sample rate and initialize ADC & DAC
    4. (slave) start ADC capturing & stream data through I2C
    5. (master) buffer the PCM samples on master
    6. (master) send any available PCM samples to slave

## Building

### esp32

    cd src/esp32
    source /path/to/esp-idf/export.sh
    idf.py build flash

### pico
If using picoprobe and opencd:

    # open pico-install.sh.template and change the path to opencd binary
    # rename pico-install.sh.template to pico-install.sh
    # make pico-install.sh executable
    # copy pico-install.sh to a folder which is in your $PATH (I usually place this little scripts in ~/bin)
    cd src/pico
    make install

If not using picoprobe & opencd just build and flash using the manual steps provided in the getting started pico-sdk guide
