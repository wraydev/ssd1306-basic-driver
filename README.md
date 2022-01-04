# ssd1306-basic-driver
A basic implementation for interacting with the SSD1306 in 8bit SPI mode - target is to be written such that it is easy to both port and accelerate.

## Assembly Acceleration
The source is written such that it can be configured to accelerate certain functions (Including but not limited to: Set_pixel and Clear_pixel) through the use of custom assembly.
If the macro ``` CONFIG_ASM_ACC ``` is set to 1 - the project should use the accelerated assembly file to implement crtaion functions, otherwise the c source implementaitons will be used. 
