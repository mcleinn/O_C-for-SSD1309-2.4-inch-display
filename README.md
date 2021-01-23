![experimental-setup](D:\Synth\[Firmware]\O_C-for-SSD1309-2.4-inch-display\experimental-setup.jpg)

**Experimental branch!!!**

[EastRising's ER-OLEDM-024-2](https://www.buydisplay.com/catalogsearch/advanced/result/?resolution=150&diagonal_size[]=301) does not seem to support more than 10MHz on the serial bus
(according to [datasheet](https://www.buydisplay.com/download/manual/ER-OLED024-2_Series_Datasheet.pdf) & my own tests). If run with the original 30MHz, it will remain black.

With dynamical changing of SPI bus frequency, the maximum ISR=DAC frequency currently is at 6.3kHz (instead of 16.67kHz). I am pretty sure this can be mitigated with a few architectural improvements. This is only the first test release. Please join the effort!

New Haven's 2.7" display (as used in Monome Teletype), which uses SSD1322, won't make much difference, as it would bloat the SPI bus with unnecessary grey scale data.

*Transfer of one OLED GDDRAM page (see [SSD1309 datasheet](https://datasheetspdf.com/pdf/1017173/SolomonSystech/SSD1309/1), page 24)*

![DSView-one-page](D:\Synth\[Firmware]\O_C-for-SSD1309-2.4-inch-display\DSView-one-page.png)

*All eight pages:*

![DSView-eight-pages](D:\Synth\[Firmware]\O_C-for-SSD1309-2.4-inch-display\DSView-eight-pages.png)