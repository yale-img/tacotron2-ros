# TacoTron2 ROS

A ros wrapper for https://github.com/NVIDIA/tacotron2

## Setup

- Update submodules

  ```console
  # cd tacotron2-ros  # this repository
  $ pushd tacotron2_ros/modules/tacotron2
  $ git submodule init; git submodule update
  $ popd
  ```

- Install cuda 11.1

  ```
  installation instructions for CUDA 11.1
  ```

- Tensorflow v1.15 requires Python 3.6 or lower, which can be installed using [pyenv](https://github.com/pyenv/pyenv).
  When installing **pyenv**, follow the instructions for [Automatic installer](https://github.com/pyenv/pyenv#automatic-installer), then follow the **bash** instructions for [shell environment setup](https://github.com/pyenv/pyenv#set-up-your-shell-environment-for-pyenv).
  After following the installation instructions for pyenv, run:

  ```console
  $ pyenv install 3.6.15
  ```

  Tacotron2 is known to work with Python 3.6.15, but the latest available release should also work and should be installed.

- [Pipenv](https://pipenv.pypa.io) provides an easy way to manage a virtual environment with a different version of Python than system.
  Note that pipenv does not provide an automatic way to find links from an HTML file, so we use `pip -f` instead.

  ```console
  $ pip install --user pipenv
  $ export PIPENV_VENV_IN_PROJECT="true"
  $ pipenv --python 3.6

  # if the above command FAILS, see the next list item for a workaround
  ```

- As of release `2022.4.8`, pipenv no longer supports creating virtual environments with Python `3.6`.
  See [GitHub Issue](https://github.com/pypa/pipenv/issues/5406) for more discussion.

  To workaround this issue, create the virtual environment manually:

  ```console
  $ PYENV_VERSION=3.6.15 python -m venv .venv
  ```

- Install python requirements:

```console
source .venv/bin/activate
pip install --upgrade pip
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
deactivate
```

- Download pretrained models

  - Download our published Tacotron 2 model: [tacotron2_statedict.pt](https://drive.google.com/file/d/1c5ZTuT7J08wLUoVZ2KkUs_VdZuJ86ZqA/view)

  - Download the WaveGlow published model: [waveglow_256channels_ljs_v2.pt](https://drive.google.com/file/d/1WsibBTsuRg_SF2Z6L6NFRTT-NjEy1oTx/view)

  - Copy both into `tacotron2_ros/models/`


## Usage

### Topic mode (async)

```console
$ rosrun --prefix "$(rospack find tacotron2_ros)/../.venv/bin/python" tacotron2_ros tacotron2_node.py
```

Then publish the text of the message to the topic: `/tacotron2/tts`

```console
$ rostopic pub -1 /tacotron2/tts std_msgs/String "Hello human. I am a robot."
```

Note: strings that are too short generate sound files with long echos and reverberations


### Action mode (sync)

```console
$ rosrun --prefix "$(rospack find tacotron2_ros)/../.venv/bin/python" tacotron2_ros tacotron2_tts_action_server.py
```

Then publish the text of the message to the topic: `/tacotron2_tts/goal`

```console
$ rostopic pub -1 /tacotron2_tts/goal tacotron2_ros/TTSActionGoal "goal: {Message: 'Hello human. I am a robot.'}"
```

Note: strings that are too short generate sound files with long echos and reverberations.