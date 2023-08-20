## British GPO 746 bluetooth conversion (HFP)

Firmware and hardware related resources to add hands-free bluetooth support to a 746 british phone - WIP.

## Hardware requirements

* esp32 devboard
* Raspberry PI Pico
* 746 telephone

## Software dependencies

* esp-idf (v5.1)
* pico-sdk ( >= v1.5.0)
* opencd (optional)
* picoprobe (optiona)

## Building

### esp32

    cd esp32
    source /path/to/esp-idf/export.sh
    idf.py build flash
    
### pico
If using picoprobe and opencd:

    # open pico-install.sh.template and change the path to opencd binary
    # rename pico-install.sh.template to pico-install.sh
    # make pico-install.sh executable
    # copy pico-install.sh to a folder which is in your $PATH (I usually place this little scripts in ~/bin)
    make install

If not using picoprobe & opencd just build and flash using the manual steps provided in the getting started pico-sdk guide