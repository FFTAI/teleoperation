# silverscreen

## Environment Setup

1. Clone the official repository

```bash
    git clone https://gitee.com/FourierIntelligence/silverscreen.git
```

2. Create a virtual environment

```bash
    conda create -n silverscreen python==3.11
    conda activate silverscreen
```

3. Install the required packages
The environment setup could be done simply doen by using `pdm` package manager. Thus, please make sure to install `pdm` first.

```bash
    pip install pdm 
    pdm install -d -Gfourier -Gdepthai -v
```

4. (Optional) Install ZED SDK

    The ZED setup composes with two parts:

    - Install the ZED SDK:
        ZED SDK coould be installed from the [official website](https://www.stereolabs.com/en-sg/developers/release). Please select the version that matches your operating system.

    - Install the ZED Python API

        The ZED python API could be installed with following command:

        ```bash
        # Activate the your virtual environment
        conda activate silverscreen

        # install Python (x64 version) and the pip package manager. Then install the dependencies via pip in a terminal.
        python -m pip install cython numpy opencv-python pyopengl

        # Install ZED python API
        cd /usr/local/zed/ 
        python get_python_api.py
        ```

## Setup VisionPro

The VisionPro setup is the same as the original [OpenTeleVision](https://github.com/OpenTeleVision/TeleVision/blob/main/README.md).

### Setup for connecting VisionPro to the local machine

Apple restricts WebXR access on non-HTTPS connections. To test the application locally, you need to set up a self-signed certificate and install it on the client device. Here's what you'll need:

1. An Ubuntu machine.
2. A router.
3. VisionPro connected to the same network with the Ubuntu machine.

> Note: Make sure to connect both the VisionPro and the Ubuntu machine to the router, ensuring they are on the same network, and proceed with the self-signed certificate setup.

### Self-signed certificate setup

We'll be using `mkcert` to create a self-signed certificate. and `mkcert` is a simple tool for making locally-trusted development certificates. It requires no configuration. Here's how to set it up:

1. Please follow the instructions on the [official website](https://github.com/FiloSottile/mkcert) to install `mkcert`.

2. check the internet IP information with

```bash
    ifconfig | grep inet
```

3. Creating the certificate with `mkcert`, make sure to put the IP address of your computer in the command

```bash
    mkcert -install && mkcert -cert-file cert.pem -key-file key.pem {Your IP address} localhost 127.0.0.1
```

  **example usage:**

```bash
    mkcert -install && mkcert -cert-file cert.pem -key-file key.pem 192.168.1.100 localhost 127.0.0.1
 ```

 > Attention: `192.168.1.100` is a fake IP address just for example, please replace it with your actual IP address

4. Turn on firewall setup

```bash
    sudo iptables -A INPUT -p tcp --dport 8012 -j ACCEPT
    sudo iptables-save
    sudo iptables -L
```

  or setup firewall with `ufw`

```bash
    sudo ufw allow 8012
```

5. install ca-certificates on VisionPro

```bash
    mkcert -CAROOT
```

  Copy the `rootCA.pem` file to the VisionPro device through the Airdrop.
  
  Settings > General > About > Certificate Trust Settings. Under "Enable full trust for root certificates", turn on trust for the certificate.

  Settings > Apps > Safari > Advanced > Feature Flags > Enable WebXR Related Features

  > Note: For some general setting up questions on visionpro could be found in the [VisionPro Setting FAQ](./visionpro_setting_FAQ.md).
  
6. open the browser on Safari on VisionPro and go to <https://192.168.1.100:8012?ws=wss://192.168.1.100:8012>

> Attention: `192.168.1.100` is a fake IP address just for example, please replace it with your actual IP address

  You will see the message "Your connection is not secure" because we are using a self-signed certificate. Click "Advanced" and then "proceed to website". You will be prompted to accept the certificate.

  Also, since the python script is not running, the browser will show a message "Safari cannot open the page because the server could not be found.". This is expected behavior. **Refresh the page after running the python script** and you will see the VR session.

7. Run the python script on the Ubuntu machine. Please see the [Usage](#usage) section for more details.

## Usage

### Setup robot
>
> Note: If you are using the simulation mode, you can skip this step.

Since we are connecting to the Fourier GR1T2 humanoids, please follow the instructions about how to use the GR1T2 humanoids before running the script, and make sure the `fourier_grx` server is running if you want to control it. You can setup and control GR1T2 humanoids with the official [Fourier GRX Documentation](https://github.com/FFTAI/fourier-grx-client).

### Python script

To start teleoperation with the default settings, simply run the following command in the terminal:

```bash
    python -m silverscreen.main tests
```

For data collection, you can use the following command:

```bash
    python -m silverscreen.main tests --record
```

> Caution: If you are using the real robot with Fourier GRX, please make sure to leave a empty space between the robot and the table to avoid the robot arm collide with the table. The robot arm will lite it arm up directly after you running the script.

It will start the teleoperation in the simulation mode without control the waist and head of the robot. The `--record` flag will turn on the recording mode, which will save the recorded data in the `tests` directory.

### Enter VR

After running the python command, you can open the browser on the VisionPro device and go to `https://192.168.1.100:8012?ws=wss://192.168.1.100:8012`. Or if you already in this website, you can refresh the page and click until see the camera image in the VR session.

Finallly, Click `Enter VR` and `Allow` to start the VR session.

### Flag explanation

- `--record`: Default to `False`. Turn on recording mode. The recorded data will be saved in the `tests` directory.
- `--waist`: Default to `False`. Turn on or off the retargeting of the robot's waist.
- `--head`: Default to `False`. Turn on or off the retargeting of the robot's head.
- `--sim`: Default to `False`. Start the teleoperation with simulation mode, which means the robot will not move, only show in the meshcat.
- `--wait_time`: Default to `1`. Set the wait time for user to initialize their hand pose to align with the robot's hand.
- `session_name`: Name of the session.
