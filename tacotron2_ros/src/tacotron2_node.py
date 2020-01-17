#!/usr/bin/env python3
#
# Text to speech engine via Tacotron2
#
# Created 01/16/2020 by Nathan Tsoi
#
# Based on: https://github.com/NVIDIA/tacotron2/blob/master/inference.ipynb

import os
DIR = os.path.dirname(os.path.abspath(__file__))
MODULE_DIR = os.path.join(DIR, '..', 'modules')
MODEL_DIR = os.path.join(DIR, '..', 'models')
TACOTRON_CHECKPOINT_FILE = os.path.join(MODEL_DIR, "tacotron2_statedict.pt")
WAVEGLOW_CHECKPOINT_FILE = os.path.join(MODEL_DIR, "waveglow_256channels_ljs_v2.pt")

import sys
sys.path.append(os.path.join(MODULE_DIR, 'tacotron2'))
sys.path.append(os.path.join(MODULE_DIR, 'tacotron2', 'waveglow'))

# TODO: find the denoiser class and parameterize
DENOISER_STRENGTH = 0.01

import io
import rospy
from std_msgs.msg import String

import matplotlib
import matplotlib.pylab as plt

import numpy as np
import torch

from hparams import create_hparams
from model import Tacotron2
from layers import TacotronSTFT, STFT
from audio_processing import griffin_lim
from train import load_model
from text import text_to_sequence

from denoiser import Denoiser

import soundfile
import simpleaudio as sa

class Tacotron2:
    def __init__(self):
        rospy.init_node("tacotron2")
        self.tacotron2_init()
        sub = rospy.Subscriber("/tacotron2/tts", String, self.callback)
        rospy.spin()

    def tacotron2_init(self):
        self.plot_wav_data = False
        # set parameters
        self.hparams = create_hparams()
        self.hparams.sampling_rate = 22050
        # load tacotron2
        self.model = load_model(self.hparams)
        self.model.load_state_dict(torch.load(TACOTRON_CHECKPOINT_FILE)['state_dict'])
        _ = self.model.cuda().eval().half()
        # load waveglow
        self.waveglow = torch.load(WAVEGLOW_CHECKPOINT_FILE)['model']
        self.waveglow.cuda().eval().half()
        for k in self.waveglow.convinv:
            k.float()
        self.denoiser = Denoiser(self.waveglow)

    def text_to_sequence(self, text):
        ''' convert text message into sequence '''
        sequence = np.array(text_to_sequence(text, ['english_cleaners']))[None, :]
        sequence = torch.autograd.Variable(torch.from_numpy(sequence)).cuda().long()
        return sequence

    def plot_data(self, data, figsize=(16, 4), filename="/tmp/plot_data.png"):
        fig, axes = plt.subplots(1, len(data), figsize=figsize)
        for i in range(len(data)):
            axes[i].imshow(data[i], aspect='auto', origin='bottom',
                           interpolation='none')
        plt.savefig(filename)


    def infer(self, sequence):
        ''' decode text to spectrogram via tacotron and convert to audio via waveglow '''
        mel_outputs, mel_outputs_postnet, _, alignments = self.model.inference(sequence)
        if self.plot_wav_data:
            self.plot_data((mel_outputs.float().data.cpu().numpy()[0],
                       mel_outputs_postnet.float().data.cpu().numpy()[0],
                       alignments.float().data.cpu().numpy()[0].T))
        with torch.no_grad():
            audio = self.waveglow.infer(mel_outputs_postnet, sigma=0.666)
            if DENOISER_STRENGTH > 0:
                audio = self.denoiser(audio, DENOISER_STRENGTH)[:, 0]
        wav = audio[0].data.cpu().numpy()
        mem_file = io.BytesIO()
        soundfile.write(mem_file, wav, self.hparams.sampling_rate, format="WAV")
        mem_file.seek(0)
        return sa.WaveObject.from_wave_file(mem_file)

    def callback(self, data):
        rospy.loginfo("Tacotron2 TTS: {}".format(data.data))
        wav_obj = self.infer(self.text_to_sequence(data.data))
        # TODO: handle concurrency (via lock)?
        wav_obj.play().wait_done()

def main():
    try:
        Tacotron2()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

