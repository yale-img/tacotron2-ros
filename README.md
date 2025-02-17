# TacoTron2 ROS

A ros wrapper for https://github.com/NVIDIA/tacotron2

## Setup

- Update submodules

  ```console
  # cd tacotron2-ros  # this repository
  $ git submodule update --init --recursive
  ```

- If you are training or finetuning a TacoTron2 model, install cuda 11.1

  ```
  installation instructions for CUDA 11.1
  ```

- [gdown](https://github.com/wkentaro/gdown) is a Python package and command-line tool for easily fetching files from Google Drive.
  It is strongly recommended to install gdown with [pipx](https://pipx.pypa.io/latest/):

  ```console
  $ pip install --user pipx
  $ pipx ensurepath
  $ pipx install gdown
  ```

- Tensorflow v1.15 requires Python 3.6 or lower, which can be installed using [pyenv](https://github.com/pyenv/pyenv).
  When installing **pyenv**, follow the instructions for [Automatic installer](https://github.com/pyenv/pyenv#automatic-installer), then follow the **bash** instructions for [shell environment setup](https://github.com/pyenv/pyenv#set-up-your-shell-environment-for-pyenv).
  After following the installation instructions for pyenv, run:

  ```console
  $ pyenv install 3.6.15
  ```

- As of release `2022.4.8`, pipenv no longer supports creating virtual environments with Python `3.6`.
  See [GitHub Issue](https://github.com/pypa/pipenv/issues/5406) for more discussion.

  To workaround this issue, create the virtual environment manually:

  ```console
  $ PYENV_VERSION=3.6.15 python -m venv .venv
  ```

- Install python requirements:

  ```console
  $ source .venv/bin/activate
  $ pip install --upgrade pip
  $ pip install -r torch-requirements.txt
  $ pip install -r requirements.txt
  $ deactivate
  ```

- If you are training or finetuning a TacoTron2 model, also compile Apex:

  ```console
  $ source .venv/bin/activate
  $ pushd tacotron2_ros/modules/apex
  $ export PATH=/usr/local/cuda-11.1/bin:$PATH
  $ export LD_LIBRARY_PATH=/usr/local/cuda-11.1/lib64:$LD_LIBRARY_PATH
  $ pip install -v --no-cache-dir --global-option="--cpp_ext" --global-option="--cuda_ext" ./
  $ popd
  $ deactivate
  ```

- Download pretrained models

  ```console
  $ gdown -O tacotron2_ros/models/ 1y5Nd-e5SGyHTJTvr2CNwSH481yFJ8QGZ
  $ gdown -O tacotron2_ros/models/ 1DnbjPf5yJCTSEdXDT_n1_RyK69V-qol_
  ```


## Usage

### Topic mode (async)

```console
$ roslaunch tacotron2_ros tacotron2.launch action_server:=false
```

Then publish the text of the message to the topic: `/tacotron2/tts`

```console
$ rostopic pub -1 /tacotron2/tts std_msgs/String "Hello human. I am a robot."
```

Note: strings that are too short generate sound files with long echos and reverberations


### Action mode (sync)

```console
$ roslaunch tacotron2_ros tacotron2.launch action_server:=true
```

Then publish the text of the message to the topic: `/tacotron2_tts/goal`

```console
$ rostopic pub -1 /tacotron2_tts/goal tacotron2_ros/TTSActionGoal "goal: {Message: 'Hello human. I am a robot.'}"
```

Note: strings that are too short generate sound files with long echos and reverberations.

## Remote Usage

An experimental procedure for playing Tacotron2 speech over SSH is documented in [stream_remote.md](./stream_remote.md)
