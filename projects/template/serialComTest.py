from influxdb import InfluxDBClient
from datetime import datetime
import influxdb
import logging
import struct
import serial
import sys
import re

try:
    serialPort = serial.Serial(port=f'/dev/ttyUSB1', baudrate=115200)
except serial.SerialException as e:
    logging.error(e)
    print("Errore rilevato...")
    print(e)
    exit()


def storeData(serialMessage: bytes):
    
    #print(serialMessage)
    reader_string = str(serialMessage, 'latin1')

    reader_string = re.split(r'\t+', reader_string.rstrip('\t'))

    print(reader_string)
    
    
    #print(serialMessage.decode('ascii'))
    #temp_str = serialMessage.decode('utf-8')
    #print(temp_str)

    #print(re.split(r'\t+', temp_str.rstrip('\t')))
    
    id          = reader_string[1]
    temp        = reader_string[3]
    humidity    = reader_string[5]
    pressure    = reader_string[7]

    data = [
        {   
            "measurement": "transmissionData",
            "tags": {
                "openmoteID": id,
                #"location": sys.argv[2]
            },"fields": {
                "temp": temp,
                "humidity": humidity,
                "pressure": pressure
            }

        }
    ]    
    print(data)

    done = clientTest.write_points(data, protocol="json")

clientTest = InfluxDBClient(
    host='127.0.0.1', port=8086, database='openmoteTest')

message = bytes(37)
buffer = bytes(1)

i = 0
while True:
    
    try:
        
        buffer += serialPort.read(1)
        #print(buffer.decode('utf-8'))

        #if buffer[-1] == 126:
        if buffer[-1] == 126:
            message = buffer
            buffer = bytes(1)
            if len(message) > 2:
                i = i+1
                print("Lettura pacchetto: ", i," terminata...")
                storeData(message)
    except Exception as e:
        logging.error(e)
        print(e) 

""" import serial

baud_rate = 115200 #whatever baudrate you are listening to
com_port1 = '/dev/tty1' #replace with your first com port path

listener = serial.Serial(com_port1, baud_rate)

while 1:
    serial_out = listener.read(size=1)
    print(serial_out) #or write it to a file 
     """