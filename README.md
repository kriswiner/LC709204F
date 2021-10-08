# LC709204F
OnSemi low-power LiPo battery fuel gauge

Arduino test sketch for the [BMA400](https://www.bosch-sensortec.com/media/boschsensortec/downloads/product_flyer/bst-bma400-fl000.pdf) acclerometer and [LC709204F](https://www.mouser.com/pdfDocs/LC709204F-D.pdf) LiPo battery fuel gauge demonstrating wake-on-motion, sleep-on-no-motion, alarm-on-low-battery-voltage, and alarm-on-high-temperature behavior. These are low power devices (800 nA BMA400 in low power mode running at 25 Hz, 2 uA for LC709204F in operate mode) perfect for ultra-low-power, battery-operated wearables and IoT applications.

Added the CRC8 check on register read and write as well as added word read and write functions for the LC709204F. Now the cell voltage and time to empty updates properly.

Sketch designed to run on an STM32L432 ([Ladybug](https://www.tindie.com/products/tleracorp/ladybug-stm32l432-development-board/)) development board.
