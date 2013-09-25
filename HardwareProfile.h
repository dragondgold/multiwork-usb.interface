#ifndef HARDWAREPROFILE_H
#define HARDWAREPROFILE_H

//#define USE_SELF_POWER_SENSE_IO
#define tris_self_power     TRISAbits.TRISA2    	//Input
#if defined(USE_SELF_POWER_SENSE_IO)
	#define self_power          PORTAbits.RA2
#else
	#define self_power          1
#endif

//#define USE_USB_BUS_SENSE_IO
#define tris_usb_bus_sense  TRISAbits.TRISA1    	//Input
#if defined(USE_USB_BUS_SENSE_IO)
	#define USB_BUS_SENSE       PORTAbits.RA1
#else
	#define USB_BUS_SENSE       1
#endif

//Uncomment the following line to make the output HEX of this  
//project work with the MCHPUSB Bootloader    
//#define PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER
	
//Uncomment the following line to make the output HEX of this 
//project work with the HID Bootloader
//#define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER		

#define CLOCK_FREQ 48000000

#endif
