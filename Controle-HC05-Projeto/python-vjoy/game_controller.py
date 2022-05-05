import serial
import argparse
import time
import logging
import pyvjoy # Windows apenas

class MyControllerMap:
    def __init__(self):
        self.button = {'A': 1, 'B': 2, 'C': 3, 'D': 4}

class SerialControllerInterface:

    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate=baudrate)
        self.mapping = MyControllerMap()
        self.j = pyvjoy.VJoyDevice(1)
        self.incoming = ''

    def Convert(lst):
        res_dct = {lst[i]: lst[i + 1] for i in range(0, len(lst), 2)}
        return res_dct

    def update(self):
        ## Sync protocol

        while self.incoming != b'X':
            self.incoming = self.ser.read()

            logging.debug("Received INCOMING: {}".format(self.incoming))

        dataHead = self.ser.read()
        dataMSB = self.ser.read()
        dataLSB = self.ser.read()

        dataOriginal = int.from_bytes(dataMSB + dataLSB, "big")

        # print ("BUT: least", dataLSB)
        # print ("BUT: most", dataMSB)
        # print ("BUT: head", dataHead)
        print(dataOriginal)

        if dataHead == b's':
            status = int.from_bytes(dataLSB, "big")
            if status == 1:
                self.ser.write(b'1') 
                print("Comunicação estabelecida")
        
        if dataHead == b'h':
            self.j.set_axis(pyvjoy.HID_USAGE_X, dataOriginal*8)
            # print(int.from_bytes(dataMSB + dataLSB, "big"))

        elif dataHead == b'y':
            self.j.set_axis(pyvjoy.HID_USAGE_Y, dataOriginal*8)
            # print(int.from_bytes(dataMSB + dataLSB, "big"))
        
        elif dataHead == b'i':
            self.j.set_axis(pyvjoy.HID_USAGE_RX, dataOriginal*8)

        elif dataHead == b'z':
            self.j.set_axis(pyvjoy.HID_USAGE_RY, dataOriginal*8)

        else:
            if (dataHead != b's'):
                head = dataHead.decode("utf-8") 
                button = int.from_bytes(dataLSB, "big")
                print(head)
                print(button)
                self.j.set_button(self.mapping.button[head], button)

        self.incoming = self.ser.read()

        return True

if __name__ == '__main__':
    interfaces = 'serial'
    argparse = argparse.ArgumentParser()
    argparse.add_argument('serial_port', type=str)
    argparse.add_argument('-b', '--baudrate', type=int, default=9600)
    argparse.add_argument('-c', '--controller_interface', type=str, default='serial', choices=interfaces)
    argparse.add_argument('-d', '--debug', default=False, action='store_true')
    args = argparse.parse_args()
    if args.debug:
        logging.basicConfig(level=logging.DEBUG)

    print("Connection to {} using {} interface ({})".format(args.serial_port, args.controller_interface, args.baudrate))

    controller = SerialControllerInterface(port=args.serial_port, baudrate=args.baudrate)

    while True:
        controller.update()
