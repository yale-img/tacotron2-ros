#!/usr/bin/env python3
#
# Text to speech engine via Tacotron2
#
# Created 01/16/2020 by Nathan Tsoi

import rospy
from std_msgs.msg import String
from tacotron2 import Tacotron2

class Tacotron2Node:
    def __init__(self):
        rospy.init_node("tacotron2")
        self.tacotron2 = Tacotron2()
        sub = rospy.Subscriber("/tacotron2/tts", String, self.callback)
        rospy.spin()

    def callback(self, data):
        rospy.loginfo("Tacotron2 TTS: {}".format(data.data))
        wav_obj = self.tacotron2.to_wav(data.data)
        wav_obj.play().wait_done()

def main():
    try:
        Tacotron2Node()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

