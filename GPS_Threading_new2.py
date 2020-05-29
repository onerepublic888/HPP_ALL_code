from __future__ import print_function
from pymavlink import mavutil
import threading

class GPS_Rx(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)               
        self.GPS_array = []
        self.daemon = True
        self._stop = False
        
    def run(self):
        while True:
            master = mavutil.mavlink_connection('COM44',baud=57600)                 #數傳的com port
            try:        
                for _ in range(10):
                    msg = master.recv_match(type = 'AHRS3', blocking = True)   #AHRS = Attitude and heading reference system
                                                                                #AHRS3:m8n   AHRS2:px4
                    d = msg.to_dict()
                    lat = d['lat'] / 1e7
                    lon = d['lng'] / 1e7
                    alt = d['altitude']     
                    if lon > 1000 or lon < 100:
                        raise ValueError('lon is %d' % lon)
#                    print('GPS Value correct')
                    
#                    print('lat,lon,alt: ',lat,lon,alt)
#                    print('lat,lon,alt length', len(lat), len(lon), len(alt))
                    if lat != '' and lon != '' and alt !='':
                        self.GPS_array = [lat, lon, alt]  
#                    print(self.GPS_array)
                
            except ValueError as err:
#                print('In except ValueError')
                print(err)
                
            finally:  
                master.close()
#                print('finally port close')
            
    def stop(self):
        self._stop = True
