# echo 'alias grx-cli="docker run --rm -it --name grx --net=host -v ${PWD}/:/app yuxianggao/grx:1.0.0a15"' >> ~/.bashrc
docker run --rm -it --name grx --net=host -v ${PWD}/:/app yuxianggao/grx:1.0.0a16 run ./gr1t2.yaml --namespace gr/daq


# python -m pip install fourier-grx==1.0.0a15
