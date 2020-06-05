# RC522
RC522 library and example 
Pinout RC522 <-> STM32F0 
PA7 - MoSi, 
Pa6 - MiSo, 
PA5 - SCK, 
PA4 - CS, 
PA3 - Reset RC522, 
PA0 - Irq (Not used in Example), 

LCD SSD1306 (i2C) connected to PB7, PB6) 

One time per second read:  Serial number, Chip Id, Mem Size and Mem field  #8 with A-Key FF FF FF FF......
