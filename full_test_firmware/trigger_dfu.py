Import("env")

import serial
import time
def before_upload(source, target, env):
    print("Attempting to trigger DFU mode on connected device...")
    if 'UPLOAD_PORT' in env and isinstance(env['UPLOAD_PORT'], str) and env['UPLOAD_PORT']:
        try:
            ser = serial.Serial(env['UPLOAD_PORT'], 1200)
            ser.close()
        except Exception as e:
            print(e)
        time.sleep(1) # Allow time for device to enumerate
    else:
        print("Warning: unable to determine upload port, cannot trigger DFU mode.")

env.AddPreAction("upload", before_upload)