#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include "Configuration.h"

void Config_ResetDefault();

#ifdef EEPROM_CHITCHAT
void Config_PrintSettings();
#else
FORCE_INLINE void Config_PrintSettings() {}
#endif

#ifdef EEPROM_SETTINGS
void Config_StoreSettings();
void Config_RetrieveSettings();
#else
FORCE_INLINE void Config_StoreSettings() {}
FORCE_INLINE void Config_RetrieveSettings() { Config_ResetDefault(); Config_PrintSettings(); }
#endif

uint8_t EE24C02_Write_Byte(uint16_t addr,uint8_t byte);

uint8_t EE24C02_Read_Byte(uint8_t addr);

uint8_t EE24C02_Write_Bytes(uint16_t addr,uint8_t *pbyte,uint16_t length);

uint8_t EE24C02_Read_Bytes(uint16_t addr,uint8_t *pdata,uint8_t length);

uint8_t EE24C02_Write_Bytes1(uint16_t addr,uint8_t *pbyte,uint16_t length);

uint8_t EE24C02_Read_Bytes1(uint16_t addr,uint8_t *pdata,uint8_t length);

uint8_t EE24C02_Write_Page(uint16_t addr,uint8_t *pbyte,uint16_t length);


void _EEPROM_writeData(int &pos, uint8_t* value, uint8_t size);
void _EEPROM_readData(int &pos, uint8_t* value, uint8_t size);

#endif//CONFIG_STORE_H

