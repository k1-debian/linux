===============================================================================
			U S E R  M A N U A L

  Copyright (C) BouffaloLab 2017-2021

1) FOR DRIVER BUILD

	Goto source code directory 
	make [clean]	
	The driver bl_fdrv.ko can be found in fullmac directory.
	The driver code supports Linux kernel from 3.10 to 5.5.19.

2) FOR DRIVER INSTALL

	a) Copy firmware wholeimg_if.bin and bl_caldata.bin to /lib/firmware/
	   or 
	   convert the fw binary and bl_caldata binary to array and put into driver folder
	   1. xxd -i wholeimg_if.bin > bl_fwbin.c
	      xxd -i bl_caldata.bin  > bl_caldata.c
	   2. copy the bl_fwbin.c and bl_caldata.c to driver source code
	   3. modify the Makefile 
	      vim fullmac/Makefile, turn off the fw bin download
	      CONFIG_BL_DNLD_FWBIN ?=n  
	b) Install WLAN driver
	   insmod bl_fdrv.ko <opmode=0> <wifi_mac=xx:xx:xx:xx:xx:xx>
		opmode: is optional param to assign the operation mode of wireless card:
			0:STA/AP only mode (default)
			1:Resved
			2:AP+STA concurrent mode
			3:AP+STA repeater mode
		wifi_mac: is optional param to override WiFi MAC address, 6 Byte Hex value.
			1st priority use insmod module param MAC addr,
			2nd priority use eFuse MAC addr,
			3rd priority use configfile(/lib/firmware/bl_settings.ini) MAC addr.

	c) uninstall driver
	   ifconfig wlan0 down
	   rmmod bl_fdrv

3) SPECIAL OPERATION MODE
	a) repeater operation mode
	   1. insmod bl_fdrv.ko opmode=3
	   2. start SoftAP in ap0 interface by hostapd
			hostapd hostapd.conf	(need add "bridge=br0" in hostadp.conf, to add a bridge named "br0" associated to this hostapd interface)
	   3. start STA in wlan0 interface by wpa_supplicant
			wpa_supplicant -iwlan0 -Dnl80211 -c wpa.conf -b br0 -d & 	(add the "-b br0" to make a relationship between STA and bridge)
	   4. manually add STA intreface to bridge
			brctl addif br0 wlan0
	   5. start bridge interface
			ifconfig br0 up

4) Process cal_data file instead read power offset and cap code from efuse. The cal data was not programmed to efuse to save efuse memory
     insmod bl_fdrv.ko cal_data_cfg=path of rf_para.conf
     rf_para.conf
     * capcode and power offset value in decimal 
     rf_para={
        capcode=32
        power_offset=0,3,0,0,0,0,-6,0,1,0,1,0,3,0  //14 channel power offset value
        mac=18:4E:34:23:05:96
     }
     
