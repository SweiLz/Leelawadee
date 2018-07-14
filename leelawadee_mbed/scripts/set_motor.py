#!/usr/bin/env python
import serial
import time

# ser = serial.Serial('/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',baudrate=115200)
ser = serial.Serial('/dev/ttyUSB0',baudrate=115200)


def sendCmd(cmd):
    global ser
    print(cmd)
    ser.write(cmd)
    time.sleep(1)


# 1:9600, 2:19200, 3:38400, 4:57600, 5:115200, 6:128000, 7:256000, 8:1000000
def main():


    # cmd = "#255P2000T1000\r\n"
    # for baud in [9600,19200,38400,57600,115200,128000,256000,1000000]:
    #     ser = serial.Serial('/dev/ttyUSB0',baudrate=baud)
    #     print cmd,baud
    #     ser.write(cmd)
    #     time.sleep(3)
    #     ser.close()
    # ser = serial.Serial('/dev/ttyUSB0',baudrate=115200)
    # while True:
        # print cmd
        # ser.write(cmd)
        # time.sleep(1)
    # sendCmd("#255PCLE0\r\n"
    # sendCmd("#255PMOD7\r\n")
    while True:
        sendCmd("#255P2000T1\r\n")

if __name__ == '__main__':
    main()
