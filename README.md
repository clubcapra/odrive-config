# odrive-config

## Setup

```sh
python -m venv venv
source ./venv/bin/activate
python -m pip install pip --upgrade
pip install -r requirements.txt
```

## Usage

In any case, you will always need to source the venv:
```sh
source venv/bin/activate
```

### Bluetooth Control

Make an alias to easily start the program (edit .bashrc and add this line):
```sh
alias odriverun="sudo ip link set can0 down; sudo ip link set can0 up type can bitrate 500000 txqueuelen 1000; python run.py"
```

Also to run the script detached from the connection, you can use `screen`.
You can exit the screen with `CTRL + A` and then `D`.
