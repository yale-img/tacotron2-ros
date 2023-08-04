# Streaming Tacotron2 Audio

This document describes how to stream the output from Tacotron2 on a Mac client over an SSH connection.

Based on [blogpost: Play remote audio over an SSH connection with a Mac Client](https://medium.com/@cristianduguet/play-remote-audio-over-an-ssh-connection-with-a-mac-client-9b7135dfe129).


## Prerequisites

This document assumes that you have installed `tacotron2_ros` as described in the [readme](README.md).

Additionally, it is assumed that you have a client running macOS and are connected to the same network as the ROS machine (potentially through campus VPN).
The macOS client should have [homebrew](https://brew.sh/) installed.
To check homebrew:

```bash
$ brew --version
Homebrew 4.1.3  # expected output
```


## Client Setup (macOS)

First, install `pulseaudio`:

```bash
$ brew install pulseaudio
```

Importantly, do NOT start the pulseaudio at login:

```bash
# Do NOT run the following command after installation!!
$ brew services start pulseaudio
```

Next, start a listening daemon:

```bash
$ pulseaudio --load=module-native-protocol-tcp --exit-idle-time=-1 --daemon
I: [] main.c: Daemon startup successful.  # expected output
```

Lookup the port for the pulseaudio listening daemon:

```bash
$ lsof -i -P | grep -i "listen" | grep -i "pulse"
# the desired line should include:
pulseaudi ... TCP *:4713 (LISTEN)
```

In this case, the listening port is `4713`.
If your output gives a different TCP port, substitute that port value in the commands below.

Then, copy your pulseaudio cookie to the remote machine:

```bash
$ scp ~/.config/pulse/cookie username@host:~/.config/pulse/cookie
```

For more options, consult `man scp` on your local machine.

Finally, connect to the Tacotron2 host with SSH.
In particular, use a Reverse Port Tunnel to forward audio from an arbitrary port (in this case, `14713`) to the listening port discovered above:

```bash
$ ssh -R 14713:localhost:4713 username@host
```


## Remote Setup (Ubuntu)

Check that pulseaudio is installed:

```bash
$ apt policy pulseaudio
pulseaudio:
  Installed: 1:13.99.1-1ubuntu3.13  # or similar; will have (none) if not installed
  ...
```

You can install with the package manager if necessary (requires sudo access):

```bash
$ sudo apt install pulseaudio
```

Next, configure pulseaudio to broadcast over the chosen port, `14713`:

```bash
$ export PULSE_SERVER="tcp:localhost:14713"
$ pactl info
Server String: tcp:localhost:14713
...
Host Name: <your client hostname here>
```

You can verify that the audio plays on your client machine:

```bash
$ speaker-test -P 2
```

Finally, start Tacotron2 and give it messages.
It is convenient to use a terminal multiplexer (e.g., `tmux`) to easily manage multiple terminal sessions over a single SSH connection.

```bash
$ roslaunch tacotron2_ros tacotron2.launch

# in a separate terminal
$ rostopic pub -1 /tacotron2_tts/goal tacotron2_ros/TTSActionGoal "goal: {Message: 'Hello human. I am a robot.'}"
```


## Caveats

+ It is recommended to setup Tacotron2 first.
+ This procedure has only been tested with a macOS client.
+ The quality and responsiveness of audio depends on the quality of your Internet connection.
  Poor bandwidth and connectivity will result in degraded audio playback.