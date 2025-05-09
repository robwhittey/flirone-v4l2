### Flir One for Linux v4l2

Radxa FLIR ONE PRO setup
1.	Sudo apt update
2.	sudo apt install build-essential git dkms linux-headers-$(uname -r)
3.	git clone https://github.com/robwhittey/v4l2loopback.git
4.	git clone https://github.com/robwhittey/flirone-v4l2.git 
5.	cd v4l2loopback
6.	make KERNELDIR=/usr/src/linux-headers-5.10.110-1-rockchip
7.	sudo make install
8.	sudo mkdir -p /lib/modules/$(uname -r)/kernel/drivers/media/video/
9.	sudo cp ~/v4l2loopback/v4l2loopback.ko /lib/modules/$(uname -r)/kernel/drivers/media/video/
10.	create udev rules file: sudo nano /etc/udev/rules.d/99-flir.rules
  a.	In file, paste: SUBSYSTEM=="usb", ATTR{idVendor}=="09cb", MODE="0666" 
  b.	Save and close: Ctrl+o, enter, ctrl+x
  c.	sudo udevadm control --reload-rules
  d.	sudo udevadm trigger
11.	sudo depmod -a
12.	sudo modprobe v4l2loopback devices=2 video_nr=2,3 card_label=”FLIR_visible”,”FLIR_thermal” exclusive_caps=1
13.	(from root, if you have a py file to execute) python3 thermal.py

Note: On fresh startup, you will need to run from line 12 (unless you have used subprocess.run() in your .py file).



#### Note from owner

This is a cleaned up version of code posted here:
http://www.eevblog.com/forum/thermal-imaging/question-about-flir-one-for-android/

All credit goes to tomas123, cynfab etc from that forum who did the awesome research and work to make this support.
