# ZED SDK Installation

The ZED setup composes with two parts:

- Install the ZED SDK:
    ZED SDK coould be installed from the [official website](https://www.stereolabs.com/en-sg/developers/release). Please select the version that matches your operating system.

- Install the ZED Python API

    The ZED python API could be installed with following command:

    ```bash
    # Activate the your virtual environment
    conda activate teleop

    # install Python (x64 version) and the pip package manager. Then install the dependencies via pip in a terminal.
    python -m pip install cython numpy opencv-python pyopengl

    # Install ZED python API
    cd /usr/local/zed/
    python get_python_api.py
    ```
