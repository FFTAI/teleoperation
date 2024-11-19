docker run --rm -it --name grx-server --net=host -v ./server_config:/app 192.168.3.15:9595/grx/server:1.0.0a20 grx run ./gr1t2.yaml --namespace gr/daq
