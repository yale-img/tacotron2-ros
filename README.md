# TacoTron2 ROS

A ros wrapper for https://github.com/NVIDIA/tacotron2

## Setup

- Update submodules

```
pushd tacotron2_ros/modules/tacotron2
git submodule init; git submodule update
popd
```

- Install cuda 11.1

```
installation instructions for CUDA 11.1
```

- Tensorflow v1.15 requires Python 3.6 or lower, which can be installed using [pyenv](https://github.com/pyenv/pyenv). After following the installation instructions for pyenv, run:

```
pyenv install 3.6
```

- Pipenv provides an easy way to manage a virtual environment with a different version of Python than system. Note that Pipenv does not provide an automatic way to find links from an HTML file, so we use `pip -f` instead.

```
pip install --user pipenv
export PIPENV_VENV_IN_PROJECT="true"
pipenv --python 3.6
```

- Install python requirements:

```
source .venv/bin/activate
pip install torch==1.9.0+cu111 torchvision==0.10.0+cu111 -f https://download.pytorch.org/whl/torch_stable.html
pip install soundfile \
            numpy scipy matplotlib \
            numba==0.48.0 \
            librosa==0.6.0 \
            tensorflow==1.15 tensorboardX \
            inflect==0.2.5 Unidecode==1.0.22 pyyaml \
            rospkg simpleaudio
pushd tacotron2_ros/modules/apex
export PATH=/usr/local/cuda-11.1/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11.1/lib64:$LD_LIBRARY_PATH
pip install -v --no-cache-dir --global-option="--cpp_ext" --global-option="--cuda_ext" ./
popd
```

- Download pretrained models

  - Download our published Tacotron 2 model: [tacotron2_statedict.pt](https://drive.google.com/file/d/1c5ZTuT7J08wLUoVZ2KkUs_VdZuJ86ZqA/view)

  - Download the WaveGlow published model: [waveglow_256channels_ljs_v2.pt](https://drive.google.com/file/d/1WsibBTsuRg_SF2Z6L6NFRTT-NjEy1oTx/view)

  - Copy both into `tacotron2_ros/models/

## Usage

### Topic mode (async)

```
rosrun --prefix "$(rospack find tacotron2_ros)/../.venv/bin/python" tacotron2_ros tacotron2_node.py
```

Then publish the text of the message to the topic: `/tacotron2/tts`

```
rostopic pub /tacotron2/tts std_msgs/String "Hello human. I am a robot."
```

Note: strings that are too short generate sound files with long echos and reverberations

### Action mode (sync)

```
rosrun --prefix "$(rospack find tacotron2_ros)/../.venv/bin/python" tacotron2_ros tacotron2_tts_action_server.py
```

Then publish the text of the message to the topic: `/tacotron2/tts/goal`

```
rostopic pub /tacotron2_tts/goal tacotron2_ros/TTSActionGoal "header: ..."
```

Note that constructing the entire TTSActionGoal message by hand is tedious. To simplify, rely on tab completion, by pressing TAB after typing ```tacotron2_ros/TTSActionGoal``` and just substituting the desired string in the field ```Message```.

Note: strings that are too short generate sound files with long echos and reverberations

