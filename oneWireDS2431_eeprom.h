#ifndef ONEWIRE_DS2431_EEPROM_H
#define ONEWIRE_DS2431_EEPROM_H

#define DS2431_NR_OF_PAGES 4
#define DS2431_PAGE_SIZE 32
#define DS2431_CHUNK_SIZE 8
#define DS2431_SERIAL_SIZE 6

bool oneWireDS2431ReadSerial(uint8_t index, uint8_t serial[6]);
bool oneWireDS2431Read(uint8_t index, uint8_t address, uint8_t* data, uint8_t count);
bool oneWireDS2431WritePage(uint8_t index, uint8_t address, const uint8_t data[8]);

#endif//ONEWIRE_DS2431_EEPROM_H
