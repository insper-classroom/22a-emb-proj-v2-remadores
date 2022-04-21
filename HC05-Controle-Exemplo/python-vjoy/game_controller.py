import serial
import argparse
import time
import logging
import pyvjoy # Windows apenas

class MyControllerMap:
    def __init__(self):
        self.button = {'A': 1, 'B': 2}



class SerialControllerInterface:

    # Protocolo
    # byte 1 -> Botão 1 (estado - Apertado 1 ou não 0)
    # byte 2 -> EOP - End of Packet -> valor reservado 'X'

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

        data_lista = []

        while self.incoming != b'X':
            self.incoming = self.ser.read()
            logging.debug("Received INCOMING: {}".format(self.incoming))

        dataHead = self.ser.read()
        dataMSB = self.ser.read()
        dataLSB = self.ser.read()

        dataOriginal = int.from_bytes(dataMSB + dataLSB, "big")

        # print ("BUT: least", dataLSB)
        # print ("BUT: most", dataMSB)
        print ("BUT: head", dataHead)
        print(dataOriginal)
        
        if dataHead == b'h':
            self.j.set_axis(pyvjoy.HID_USAGE_X, dataOriginal*8)
            # print(int.from_bytes(dataMSB + dataLSB, "big"))

        elif dataHead == b'y':
            self.j.set_axis(pyvjoy.HID_USAGE_Y, dataOriginal*8)
            # print(int.from_bytes(dataMSB + dataLSB, "big"))
        
        else:
            head = dataHead.decode("utf-8") 
            button = int.from_bytes(dataLSB, "big")
            print(head)
            print(button)
            self.j.set_button(self.mapping.button[head], button)

        # if dataHead == b'A':
        #     head = dataHead.decode("utf-8") 
        #     button = int.from_bytes(dataLSB, "big")
        #     print(head)
        #     print(button)
        #     self.j.set_button(self.mapping.button[head], button)


        self.incoming = self.ser.read()

        return True

        # data = self.ser.read()
        dict = SerialControllerInterface.Convert(data_lista)
        # print(dict)
        if dict[b'A'] == b'1':
            logging.info("Sending press")
            self.j.set_button(self.mapping.button['A'], 1)
        elif dict[b'A'] == b'0':
            self.j.set_button(self.mapping.button['A'], 0)
        if dict[b'B'] == b'1':
            logging.info("Sending press")
            self.j.set_button(self.mapping.button2['B'], 1)
        elif dict[b'B'] == b'0':
            self.j.set_button(self.mapping.button2['B'], 0)
        if dict[b'R'] == b'1':
            logging.info("Sending press")
            self.j.set_button(self.mapping.button3['R'], 1)
        elif dict[b'R'] == b'0':
            self.j.set_button(self.mapping.button3['R'], 0)
        if dict[b'L'] == b'1':
            logging.info("Sending prCCCess")
            self.j.set_button(self.mapping.button4['L'], 1)
        elif dict[b'L'] == b'0':
            self.j.set_button(self.mapping.button4['L'], 0)
        
        if dict[b'U'] == b'1':
            logging.info("Sending press")
            self.j.set_button(self.mapping.button5['U'], 1)
        elif dict[b'U'] == b'0':
            self.j.set_button(self.mapping.button5['U'], 0)
        if dict[b'D'] == b'1':
            logging.info("Sending press")
            self.j.set_button(self.mapping.button6['D'], 1)
        elif dict[b'D'] == b'0':
            self.j.set_button(self.mapping.button6['D'], 0)


        self.incoming = self.ser.read()


class DummyControllerInterface:
    def __init__(self):
        self.mapping = MyControllerMap()
        self.j = pyvjoy.VJoyDevice(1)

    def update(self):
        self.j.set_button(self.mapping.button['A'], 1)
        time.sleep(0.1)
        self.j.set_button(self.mapping.button['A'], 0)
        logging.info("[Dummy] Pressed A button")
        time.sleep(1)


if __name__ == '__main__':
    interfaces = ['dummy', 'serial']
    argparse = argparse.ArgumentParser()
    argparse.add_argument('serial_port', type=str)
    argparse.add_argument('-b', '--baudrate', type=int, default=9600)
    argparse.add_argument('-c', '--controller_interface', type=str, default='serial', choices=interfaces)
    argparse.add_argument('-d', '--debug', default=False, action='store_true')
    args = argparse.parse_args()
    if args.debug:
        logging.basicConfig(level=logging.DEBUG)

    print("Connection to {} using {} interface ({})".format(args.serial_port, args.controller_interface, args.baudrate))
    if args.controller_interface == 'dummy':
        controller = DummyControllerInterface()
    else:
        controller = SerialControllerInterface(port=args.serial_port, baudrate=args.baudrate)

    while True:
        controller.update()
