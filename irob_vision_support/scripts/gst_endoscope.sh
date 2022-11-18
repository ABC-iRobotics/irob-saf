#! /bin/bash

gst-launch-1.0 decklinkvideosrc mode=pal device-number=0 ! videorate ! "video/x-raw,framerate=30/1" ! xvimagesink &
gst-launch-1.0 decklinkvideosrc mode=pal device-number=1 ! videorate ! "video/x-raw,framerate=30/1" ! xvimagesink &
