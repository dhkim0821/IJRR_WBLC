from gopro import GoPro

import requests
import humanize
import time
import re
from bs4 import BeautifulSoup

# For detecting a control-c
import signal
import sys

SAVE_DIRECTORY='/Users/donghyunkim/Repository/dynacore/experiment_data/'

def signal_handler(signal, frame):
        print '---- Ctrl-C was pressed ---'
        stop_video_record()
        print 'Saving the latest video'
        save_latest_video()
        print 'Exiting program...'
        sys.exit()

def save_latest_video():
    global camera_ip, camera_media_url
    # HTTP request
    r=requests.get(camera_media_url,allow_redirects=True)    

    if r.status_code == requests.codes.ok:
        print 'HTTP Request to GoPro successful'    
        soup = BeautifulSoup(r.content,'html.parser')
        links = soup.findAll('a')                               # Get all the links
        video_links = ['http://'+camera_ip+link['href'] for link in links if link['href'].endswith('MP4')] #extract the MP4 Link
        media_url = video_links[-1]                             # get the last video of the list

        # Prints all the media url
        #for item in video_links: 
        #    print item

        print 'URL of the Latest media: ', media_url  #print the url
        video = requests.get(media_url,stream=True)

        if video.status_code == requests.codes.ok:
            print 'Successfully accessed the video'
            save_video(video)
        else:
            print 'ERROR. Could not access the video file'

    else:
        print 'ERROR. Cannot connect to the GoPro camera server'

def save_video(video):
    global SAVE_DIRECTORY
    timestr=time.strftime("%Y%m%d_%H_%M_%S")              # get the current time and use it as a filename
    filename='record_'+timestr+'.mp4'                     # set the filename
    print 'Save Directory:', SAVE_DIRECTORY
    print "Saving Video..."
    fs = open(SAVE_DIRECTORY+filename,'wb').write(video.content)              # save video file
    print "Video Successfully Saved!"
    camera.delete_last()

def start_video_record():
    global camera
    camera.mode('video') # Set Camera mode to video    
    print 'Start Video Recording '
    camera.video()

def stop_video_record():
    global camera
    print 'End Video Recording'    
    camera.stop()    

def beep_once():
    global camera
    camera.locate('on')
    time.sleep(1.0)
    camera.locate('off')

if __name__ == "__main__":
    # Register the CTRL-C Command
    signal.signal(signal.SIGINT, signal_handler)

    # Initialize camera object
    camera = GoPro.GoPro()
    print 'GoPro Status:[', camera.status['ok'], ']'

    camera_info = camera.status
    camera_ip = camera.status['ip']
    camera_media_url = 'http://'+ camera_ip + '/videos/DCIM/100GOPRO/' 
    print 'Camera IP:', camera_ip
    print 'Camera Media URL:', camera_media_url
    print 'Camera Storage:', camera.status['storage']
    start_video_record()
    print 'Press Ctrl-C to end the video recording'
    signal.pause()     # Wait Until User presses CTRL-C
    print 'Program ends'
