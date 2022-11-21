#!/bin/bash

alsa_file="/tmp/alsa_driver.loaded"

if [ -e  $alsa_file ]
then
    echo "audio device driver alread loaded (including uac2 device)!"
else
    echo "install alsa driver, es7210, es8156 and usb audio etc..."

    # install actual micphone & speaker audio card driver.
    mount -o,remount rw /
    echo 8 >/proc/sys/kernel/printk
    echo 112 >/sys/class/gpio/export
    echo out >/sys/class/gpio/gpio112/direction
    echo 1 >/sys/class/gpio/gpio112/value
    echo 118 >/sys/class/gpio/export
    echo out >/sys/class/gpio/gpio118/direction
    echo 1 >/sys/class/gpio/gpio118/value
    
    modprobe -r es7210
    modprobe -r es8156
    modprobe -r hobot-i2s-dma
    modprobe -r hobot-cpudai
    modprobe -r hobot-snd-7210 snd_card=5
    
    modprobe es7210
    modprobe es8156
    modprobe hobot-i2s-dma
    modprobe hobot-cpudai
    modprobe hobot-snd-7210 snd_card=5
    
    # install usb audio driver
    systemctl stop adbd
    /etc/init.d/usb-gadget.sh stop
    /etc/init.d/usb-gadget.sh start uac2
    
    # create alsa laoded file
    touch /tmp/alsa_driver.loaded
fi

# alsa devices list
## root@ubuntu:/userdata# aplay -l
## **** List of PLAYBACK Hardware Devices ****
## card 0: hobotsnd5 [hobotsnd5], device 1: (null) es8156.0-0008-1 []
##   Subdevices: 1/1
##   Subdevice #0: subdevice #0
## card 1: UAC2Gadget [UAC2_Gadget], device 0: UAC2 PCM [UAC2 PCM]
##   Subdevices: 1/1
##   Subdevice #0: subdevice #0
## root@ubuntu:/userdata#
## root@ubuntu:/userdata#
## root@ubuntu:/userdata# arecord -l
## **** List of CAPTURE Hardware Devices ****
## card 0: hobotsnd5 [hobotsnd5], device 0: (null) ES7210 4CH ADC 0-0 []
##   Subdevices: 1/1
##   Subdevice #0: subdevice #0
## card 1: UAC2Gadget [UAC2_Gadget], device 0: UAC2 PCM [UAC2 PCM]
##   Subdevices: 1/1
##   Subdevice #0: subdevice #0
## root@ubuntu:/userdata#
## root@ubuntu:/userdata#

# audio pipeline
## received audio data from PC and playback in speaker
#  arecord -Dhw:1,0 -r 16000 -f S16_LE -c 1 -d 500000 | aplay -Dhw:0,1 &

## record mic data and send to PC
# arecord -Dhw:0,0 -r 16000 -f S16_LE -c 1 -d 500000 | aplay -Dhw:1,0 &
