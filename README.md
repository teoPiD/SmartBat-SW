# SmartBat

This repository contains the code for the custom made PCBs. 
The code can be improved with the next PCB iteration (REV1.1).

## Program

To upload this program to ATTiny3216 (the one currently being used), you should install [megaTinyCore](https://github.com/SpenceKonde/megaTinyCore). Follow the [installation instructions](https://github.com/SpenceKonde/megaTinyCore/blob/master/Installation.md).

### PCBs

There are 3 custom made PCBs in the drone. In the motor's arms you can find equal PCBs and in the middle of the drone there is a different one. The front one is numbered 1 (see bottom of PCB), the back one is numbered 2 and the middle one is numbered 3. To program PCB 1 and 2, change ```#define BOARD``` to ```NOT_CUSTOM_INA``` and their addresses (```#define Address```) to ```0x0B``` and ```0x0D```, respectively. To program PCB 3, change ```#define BOARD``` to ```CUSTOM_INA``` and its address (```#define Address```) to ```0x0F```.
