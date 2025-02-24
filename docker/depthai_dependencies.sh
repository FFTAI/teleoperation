#!/bin/bash

trap 'RET=$? ; echo -e >&2 "\n\x1b[31mFailed installing dependencies. Could be a bug in the installer or unsupported platform. Open a bug report over at https://github.com/luxonis/depthai - exited with status $RET at line $LINENO \x1b[0m\n" ; exit $RET' ERR

readonly linux_pkgs=(
    python3
    python3-pip
    udev
    cmake
    git
    python3-numpy
)

readonly ubuntu_pkgs=(
    ${linux_pkgs[@]}
    # https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html
    build-essential
    libgtk2.0-dev
    pkg-config
    libavcodec-dev
    libavformat-dev
    libswscale-dev
    python3-dev
    libtbb2
    libtbb-dev
    libjpeg-dev
    libpng-dev
    libtiff-dev
    libdc1394-dev
    # https://stackoverflow.com/questions/55313610
    ffmpeg
    libsm6
    libxext6
    libgl1-mesa-glx
    python3-pyqt5
    python3-pyqt5.qtquick
    qml-module-qtquick-controls2
    qml-module-qt-labs-platform
    qtdeclarative5-dev
    qml-module-qtquick2
    qtbase5-dev
    qtchooser
    qt5-qmake
    qtbase5-dev-tools
    qml-module-qtquick-layouts
    qml-module-qtquick-window2
)

readonly ubuntu_arm_pkgs=(
    "${ubuntu_pkgs[@]}"
    # https://stackoverflow.com/a/53402396/5494277
    libhdf5-dev
    libhdf5-dev
    libatlas-base-dev
    # libjasper-dev
    # https://github.com/EdjeElectronics/TensorFlow-Object-Detection-on-the-Raspberry-Pi/issues/18#issuecomment-433953426
    libilmbase-dev
    libopenexr-dev
    libgstreamer1.0-dev
)

readonly fedora_pkgs=(
    ${linux_pkgs[@]}
    gtk2-devel
    # Fedora uses pkgconf instead of pkg-config
    tbb-devel
    libjpeg-turbo-devel
    libpng-devel
    libtiff-devel
    libdc1394-devel
    # TODO(PM): ffmpeg requires enabling rpmfusion-free-updates
    # TODO(PM): libavcodec-dev libavformat-dev libswscale-dev python-dev libtbb2
    #           libsm6 libxext6 libgl1-mesa-glx
)

print_action () {
    green="\e[0;32m"
    reset="\e[0;0m"
    printf "\n$green >>$reset $*\n"
}
print_and_exec () {
    print_action $*
    $*
}

if [[ $(uname) == "Darwin" ]]; then
    echo "During Homebrew install, certain commands need 'sudo'. Requesting access..."
    true
    arch_cmd=
    if [[ $(uname -m) == "arm64" ]]; then
        arch_cmd="arch -x86_64"
        echo "Running in native arm64 mode, will prefix commands with: $arch_cmd"
        # Check if able to run with x86_64 emulation
        retcode=0
        $arch_cmd true || retcode=$?
        if [[ $retcode -ne 0 ]]; then
            print_action "=== Installing Rosetta 2 - Apple binary translator"
            # Prompts the user to agree to license: <A> <Enter>
            # Could be automated by adding: --agree-to-license
            print_and_exec softwareupdate --install-rosetta
        fi
    fi
    homebrew_install_url="https://raw.githubusercontent.com/Homebrew/install/master/install.sh"
    print_action "Installing Homebrew from $homebrew_install_url"
    # CI=1 will skip some interactive prompts
    CI=1 $arch_cmd /bin/bash -c "$(curl -fsSL $homebrew_install_url)"
    print_and_exec $arch_cmd brew install python3 git
    print_and_exec python3 -m pip install -U pip
    echo
    echo "=== Installed successfully!  IMPORTANT: For changes to take effect,"
    echo "please close and reopen the terminal window, or run:  exec \$SHELL"
elif [ -f /etc/os-release ]; then
    # shellcheck source=/etc/os-release
    source /etc/os-release

    if [[ "$ID" == "ubuntu" || "$ID" == "debian" || "$ID_LIKE" == "ubuntu" || "$ID_LIKE" == "debian" || "$ID_LIKE" == "ubuntu debian" ]]; then
        if [[ ! $(uname -m) =~ ^arm* ]]; then
            apt-get update
            apt-get install -y "${ubuntu_pkgs[@]}"
            python3 -m pip install --upgrade pip
        elif [[ $(uname -m) =~ ^arm* ]]; then
            apt-get update
            apt-get install -y "${ubuntu_arm_pkgs[@]}"
            python3 -m pip install --upgrade pip
        fi

        # As set -e is set, retrieve the return value without exiting
        RET=0
        dpkg -s uvcdynctrl > /dev/null 2>&1 || RET=$? || true
        # is uvcdynctrl installed
        if [[ "$RET" == "0" ]]; then
          echo -e "\033[33mWe detected \"uvcdynctrl\" installed on your system. \033[0m"
          echo -e "\033[33mWe recommend removing this package, as it creates a huge log files if a camera is used in UVC mode (webcam)\033[0m"
          echo -e "\033[33mYou can do so by running the following commands:\033[0m"
          echo -e "\033[33m$ apt remove uvcdynctrl uvcdynctrl-data\033[0m"
          echo -e "\033[33m$ rm -f /var/log/uvcdynctrl-udev.log\033[0m"
          echo ""
        fi

    elif [[ "$ID" == "fedora" ]]; then
        dnf update -y
        dnf install -y "${fedora_pkgs[@]}"
        dnf groupinstall -y "Development Tools" "Development Libraries"
        python3 -m pip install --upgrade pip
    else
        echo "ERROR: Distribution not supported"
        exit 99
    fi

    # Allow all users to read and write to Myriad X devices
    echo "Installing udev rules..."
    mkdir -p /etc/udev/rules.d/
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | tee /etc/udev/rules.d/80-movidius.rules > /dev/null
    # udevadm control --reload-rules && udevadm trigger
else
    echo "ERROR: Host not supported"
    exit 99
fi
