#!/usr/bin/env python3

import MotorControlArchitecture_v03 as mc
import serial 
import threading



# convert ultrasound to distance
def ultrasound_microseconds_to_meters(microseconds):
    speed_of_sound_meters_per_second = 343 
    meters = (microseconds * speed_of_sound_meters_per_second / 1e6)/2  # define a conversion here
    return meters



class sensor_suite (threading.Thread):
    def __init__(self, saveFile=0):
        threading.Thread.__init__(self)
        self.saveFile = saveFile        
        
        self.encoder_counts_0 = 0
        self.encoder_counts_1 = 0
        self.ultrasound_microseconds_0 = 0
        self.ultrasound_microseconds_1 = 0
        self.ultrasound_microseconds_2 = 0
        self.analog_0 = 0
        self.analog_1 = 0
        self.analog_2 = 0
        self.analog_3 = 0
        self.analog_4 = 0
        self.analog_5 = 0
        
        self.ultrasound_meters_0 = 0
        self.ultrasound_meters_1 = 0
        self.ultrasound_meters_2 = 0
        
        counts_per_encoder_revolution = 12
        encoder_to_output_shaft_multiplier = (float(13)/34 * float(12)/34 * float(13)/35 * float(10)/38)  # roughly 75.81
        shaft_radians_to_output_units_ratio = 0.030 # 1 radian makes this many linear meters
        
        #Encoder object instantiation:  quadrature_encoder(serial_string_identifier, counts_per_encoder_revolution, output_gear_reduction_ratio, shaft_radians_to_output_units_ratio, forward_encoder_rotation_sign)
        self.encoder_0 = mc.quadrature_encoder("E0",counts_per_encoder_revolution, encoder_to_output_shaft_multiplier, shaft_radians_to_output_units_ratio, -1)
        self.encoder_1 = mc.quadrature_encoder("E1",counts_per_encoder_revolution, encoder_to_output_shaft_multiplier, shaft_radians_to_output_units_ratio, 1)
        



    #--------------main-----------------
    def run(self):
        self.ser = serial.Serial('/dev/ttyS0')  #serial port to alamode
        self.ser.baudrate = 115200 
        self.ser.bytesize = 8
        self.ser.parity = 'N'
        self.ser.stopbits = 1
        self.ser.timeout = 1 # one second time out. 
        
        if self.saveFile:
            filename = "SensorData_{0}.dat".format(time.strftime("%Y%m%d-%H_%M_%S"))
            with open(filename,"w") as self.fid :
                self._listenloop()
        else:
            self._listenloop()
            
        
    def _listenloop(self):
        #ser.reset_input_buffer() 
        self.ser.flushInput()
        self.ser.readline()
        while True:
            # Here we read the serial port for a string that looks like "e0:123456", which is an Encoder reading. 
            line = self.ser.readline().decode().strip() #blocking function, will wait until read entire line
            if self.saveFile:
                self.fid.write(line+"\n")
            line = line.split(":")	
            name = line[0]
            try:
                val = int(line[1])
            except: 
                continue 
            
            if name=='E0':
                self.encoder_counts_0 = val
                self.encoder_0.update(self.encoder_counts_0)
            elif name=='E1':	
                self.encoder_counts_1 = val
                self.encoder_1.update(self.encoder_counts_1)
            elif name=='U0':
                self.ultrasound_microseconds_0 = val
                self.ultrasound_meters_0 = ultrasound_microseconds_to_meters(self.ultrasound_microseconds_0)
            elif name=='U1':
                self.ultrasound_microseconds_1 = val
                self.ultrasound_meters_1 = ultrasound_microseconds_to_meters(self.ultrasound_microseconds_1)
            elif name=='U2':
                self.ultrasound_microseconds_2 = val
                self.ultrasound_meters_2 = ultrasound_microseconds_to_meters(self.ultrasound_microseconds_2)
            elif name=='A0':
                self.analog_0 = val
            elif name=='A1':
                self.analog_1 = val
            elif name=='A2':
                self.analog_2 = val
            elif name=='A3':
                self.analog_3 = val
            elif name=='A4':
                self.analog_4 = val
            elif name=='A5':
                self.analog_5 = val
                
            
            
