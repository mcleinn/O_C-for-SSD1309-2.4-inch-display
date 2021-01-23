Experimental branch!!!

EastRising display ER-OLEDM-024-2 does not seem to support more than 10MHz on the serial bus
(according to datasheet & my own tests). If run with the original 30MHz, it will remain black.

With dynamical changing of SPI bus frequency, the maximum ISR=DAC frequency currently is at 6.3kHz (instead of 16.67kHz)
I am pretty sure this can be mitigated with a few architectural improvements. 
