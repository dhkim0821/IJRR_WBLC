from gopro import GoPro
import time

camera = GoPro.GoPro()
print 'Locating Go Pro. It should beep for 3 seconds'
camera.locate('on')
time.sleep(3.0)
camera.locate('off')
print 'Beep Off'