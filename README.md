Getting started
---------------

#cleaning the previous compilation
make clean
# compiling
make -j4
# switch st-link with a controller, reading the flash contents (recorded data or previous firmware)
st-flash read stm32_was_before_20231003_082226_190450.bin 0x08000000 10000000000
# Clearing the entire flash memory
st-flash erase
# flashing the firmware to the controller:
st-flash write build/stm32_voltage_reader.bin 0x08000000
