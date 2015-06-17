# To get pps workin may need to do the following
sudo modprobe 8250   		# Load the serial drivers
ldattach PPS /dev/ttyS0		# Attach line on serial device to PPS line discipline(ldisc)
sudo ./pps_config		# Run config script as sudo to enable both rising and falling edge triggers

# Alternatively the first two lines can be replaced with more permanent solutions as follows

# Load the serial driver on startup
sudo nano /etc/modules
# add line "8250"

# Modify udev rules file 
sudo nano /etc/udev/rules.d/50-pps.rules
add this to the file:
SUBSYSTEM=="pps", MODE="0666", GROUP="dialout"           # Add pps devices to dialout group with 666 permissions 
KERNEL=="ttyS0", RUN+="/usr/sbin/ldattach pps /dev/%k"   # When serial device becomes active, run ldattach

