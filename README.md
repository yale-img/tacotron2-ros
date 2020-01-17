# TacoTron2 ROS

A ros wrapper for https://github.com/NVIDIA/tacotron2

## Setup

- Update submodules

```
pushd tacotron2_ros/modules/tacotron2
git submodule init; git submodule update
popd
```

- Install cuda 10.1

```
pushd
wget https://developer.nvidia.com/compute/cuda/10.1/Prod/local_installers/cuda_10.1.105_418.39_linux.run
chmod +x cuda_10.1.105_418.39_linux.run
sudo ./cuda_10.1.105_418.39_linux.run
# install the toolkit only
popd
```

- Make a virtual env named `.venv` after you `cd` into the project directory and activate the virtual environment. Install python requirements:

```
virtualenv -p $(which python3) .venv
source .venv/bin/activate
pip install torch torchvision \
            numpy scipy matplotlib \
            librosa==0.6.0 \
            tensorflow==1.15 tensorboardX \
            inflect==0.2.5 Unidecode==1.0.22 pyyaml \
            rospkg simpleaudio
pushd tacotron2_ros/modules/apex
export PATH=/usr/local/cuda-10.1/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-10.1/lib64:$LD_LIBRARY_PATH
pip install -v --no-cache-dir --global-option="--cpp_ext" --global-option="--cuda_ext" ./
popd
```

- Download pretrained models

  - Download our published Tacotron 2 model: [tacotron2_statedict.pt](https://drive.google.com/file/d/1c5ZTuT7J08wLUoVZ2KkUs_VdZuJ86ZqA/view)

  - Download the WaveGlow published model: [waveglow_256channels_ljs_v2.pt](https://drive.google.com/file/d/1WsibBTsuRg_SF2Z6L6NFRTT-NjEy1oTx/view)

  - Copy both into `tacotron2_ros/models/

## Usage

```
rosrun tacotron2_ros tacotron2_node.py
```

Then publish the text of the message to the topic: `/tacotron2/tts`

```
rostopic pub /tacotron2/tts std_msgs/String "Hello human. I am a robot."
```

Note: strings that are too short generate sound files with long echos and reverberations

