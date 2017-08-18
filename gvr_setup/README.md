# Network Configuration Instructions for the Raspberry Pi (RPi)

This set of instructions explains how to connect the Raspberry Pi to different networks.  For instance, depending upon the circumstances, you may wish to connect your Raspberry Pi to the Internet or another computer via the wireless EECSDS3, the wired or wireless EECSnet, or the AT&T Mi-Fi.  In some cases, it may be necessary to establish a peer-to-peer connection between the Raspberry Pi and a laptop using an Ethernet cable.  These instructions are intended to assist you with changing between these network options.  


### Template /etc/network/interfaces file for your Raspberry Pi.
1.  On some installations of Ubuntu-Mate, we have noticed that the GUI network manager is not loading properly and is unavailable.  As a result, it may be best to configure different connections using the "interfaces" file located in the /etc/network directory of Raspberry Pi.     
    + Inside this Github directory, there is a template "interfaces" file that you can copy over to your Raspberry Pi.  The file contains comments on which lines should be uncommented in order to connect to certain networks.
    
### Establishing an Ethernet connection directly between the RPi and a laptop. 
1.  At times, it may be necessary to establish a direct Ethernet connection between your laptop and RPi using a CAT5 cable.  This is especially useful when you encounter wireless trouble and can't connect the RPi to a montior for troubleshooting.
2.  Here are the steps for establishing such a connection:
    + Program your RPi to automatically configure its Ethernet port at every startup so that it is ready for connecting to your laptop.
    + You'll need to find out the name of your ethernet port on the RPi in order update your "interfaces" file.
        + On your RPi, type `ipconfig -a` to show all interfaces, including those that are currently down.
        + Note the interface name of your ethernet port; it should begin with 'enx' and follow with the MAC address of the interface.
        + See the template 'interfaces file'; uncomment the portion for configuring a static IP address to the Ethernet port.
        + Be sure to change the interface name, as well as the IP address you would like to give your RPi ethernet port.
        + For these changes to take effect, type `sudo ifup enx_(your_Ethernet_MAC_address)` in the terminal.
    + Next, configure your laptop Ethernet port to be on the same network.
        + Use the Ubuntu Network Manager GUI to configure this Ethernet connection.
            + Click on the wireless icon in the upper-left corner of the screen.
            + Click 'Edit Connections'
            + Highlight 'Ethernet', then click 'Add'
            + Ensure the dialog box drop-down has 'Ethernet' selected
            + On the 'General' tab, unselect 'Automatically connect...'
            + On the 'Ethernet' tab, select the appropriate Ethernet port
            + On the 'IPv4 Settings' tab, select 'Manual' in the 'Method' drop-down; then click 'Add' to insert the IP address & subnet mask.
            + Choose an IP address on the same network as the RPi (i.e., IP = 192.168.0.xxx , Subnet = 255.255.255.0 where xxx = some number from 1-254 but is different than the IP on the RPi)
            + Click 'Save'
    + Once both computers have been configured, you should be able to ping the RPi from the laptop.
         + You may need to reboot the RPi for the settings in the 'interfaces' file to take effect.
         + Ensure your Ethernet port on your laptop is 'up' by typing the following command in the terminal:  `sudo ifconfig yourLaptopEthernetPortName up` or by selecting the interface in Network Manager GUI.
         + Once the interface is up, try pinging the RPi by typing `ping 192.168.0.xxx` where xxx is the last octet address of your RPi.
         + If you can successfully ping the RPi, then you'll be able to ssh into the device. 
            
    
              


