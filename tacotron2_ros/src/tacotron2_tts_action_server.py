#! /usr/bin/env python

import rospy
import actionlib
from tacotron2 import Tacotron2
import tacotron2_ros.msg

class Tacotron2TTSActionServer(object):
    # create messages that are used to publish feedback/result

    def __init__(self, name):
        self._action_name = name
        self.tacotron = Tacotron2()
        self.feedback = tacotron2_ros.msg.TTSFeedback()
        self.result = tacotron2_ros.msg.TTSResult()
        self.server = actionlib.SimpleActionServer(self._action_name, tacotron2_ros.msg.TTSAction, execute_cb=self.execute_cb, auto_start=False)
        self.server.start()

    def execute_cb(self, goal):
        wav_obj = self.tacotron2.to_wav(goal.Message)
        wav_obj.play().wait_done()
        self.result.Message = "done"
        self.server.set_succeeded(self.result)

def main():
    rospy.init_node('tacotron2_tts')
    server = Tacotron2TTSActionServer(rospy.get_name())
    rospy.spin()

if __name__ == '__main__':
    main()

