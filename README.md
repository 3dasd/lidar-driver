
# Hacked `avrdude`:
```sh
# compile
arduino-cli compile -v -b arduino:avr:nano ~/MyArduinoSketch

# upload with hacked acrdude
cd ~/.arduino15/packages/arduino/tools/avrdude/6.3.0-arduino17/bin
arduino-cli upload --port /dev/ttyS0 -b arduino:avr:nano -v ~/MyArduinoSketch/
```

# Platformio:

0. Install Rust

> This is only necessary until recent versions of `cryptography` are available in Piwheels.
(see https://github.com/piwheels/packages/issues/188) Until then we need to be able to build
`cryptography` ourselves...

```sh
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
source $HOME/.cargo/env
```

1. install platformio
```sh
sudo apt install python3-pip libffi-dev libssl-dev
python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio/master/scripts/get-platformio.py)"
```

2. install remote agent

Takes ages (especially while 0. isn't fixed and it compiles cryptography as well)...

```sh
pio remote agent start
```

This gave:

```s
ImportError: cannot import name '_get_backend' from 'cryptography.hazmat.backends' (/home/pi/.platformio/packages/contrib-pysite/cryptography/hazmat/backends/__init__.py)
```

3. Install cryptography manually

>(see point 0)

Picked an earlier version that was still present in piwheels:

```sh
/home/pi/.platformio/penv/bin/python -m pip install --no-cache-dir --no-compile -t /home/pi/.platformio/packages/contrib-pysite  "cryptography=3.3.2"
```


4. Create systemd service:

To always start remote agent:

```bash

# Get token with `pio account token` on host machine
PLATFORMIO_AUTH_TOKEN="<TOKEN GOES HERE>"

sudo tee /etc/systemd/system/pio-remote.service <<EOF
[Unit]
Description=pio remote agent
Requires=network-online.target
After=network-online.target

[Service]
Type=simple
User=pi
Group=pi
Environment="PLATFORMIO_AUTH_TOKEN=$PLATFORMIO_AUTH_TOKEN"
WorkingDirectory=/home/pi
ExecStart=/home/pi/.platformio/penv/bin/pio remote agent start
Restart=no

[Install]
WantedBy=multi-user.target
EOF
```

Enable and start with:
`systemctl enable pio-remote && systemctl start pio-remote`





# Pio upload from RPi

```
pio run --target upload --upload-port /dev/ttyS0
```

# Changes required for platformio.ini:

Add to `platformio.ini`:
```
extra_scripts = post:shared/extra_script.py
```

`shared/extra_script.py`:
```python
import subprocess
import time

Import("env")

print("[DEBUG] extra_script.py is running")

def reset_pin_before_upload(source, target, env):
    print("[DEBUG] driving RST pin")
    subprocess.call(["gpio", "-g", "mode", "18", "out"])
    subprocess.call(["gpio", "-g", "write", "18", "0"])
    time.sleep(1)
    subprocess.call(["gpio", "-g", "write", "18", "1"])

env.AddPreAction("upload", reset_pin_before_upload)
```