import random
import time
from influxdb import InfluxDBClient
from datetime import datetime
import influxdb
import logging
import struct
import serial
import sys
import re


clientTest = InfluxDBClient(
    host='192.168.178.134', port=8086, database='iotlab')


#for i in range(0,100):
while(True):
    # generate random number
    id = random.randint(1,5);
    temp = float(random.randint(0,40)+random.randint(0,id));
    humidity = float(random.randint(0,100));
    pressure = float(random.randint(0,2000));
    light = float(random.randint(0,10000));

    data = [
                {   
                    "measurement": "transmissionData",
                    "tags": {
                        "openmoteID": id,                    
                    },"fields": {
                        "temp": temp,
                        "humidity": humidity,
                        "pressure": pressure,
                        "light": light
                    }

                }
            ]    
    print("JSON: ", data)


    done = clientTest.write_points(data, protocol="json")

    

    print('Execution done:', done)

    #system pause
    time.sleep(0.01)
