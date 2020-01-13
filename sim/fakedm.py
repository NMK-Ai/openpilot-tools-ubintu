#!/usr/bin/env python3
import cereal.messaging as messaging
import time

if __name__ == "__main__":
  pm = messaging.PubMaster(['driverMonitoring'])
  while 1:
    dat = messaging.new_message()
    dat.init('driverMonitoring')
    dat.driverMonitoring.faceProb = 1.0
    pm.send('driverMonitoring', dat)
    time.sleep(0.1)

