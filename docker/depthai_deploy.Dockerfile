# FROM 192.168.3.15:9595/farts/depthai:latest
FROM yuxianggao/depthai:latest 

# # Install the ca-certificate package
# RUN apt-get update && apt-get install -y ca-certificates && apt-get clean && rm -rf /var/lib/apt/lists/*
# # Copy the CA certificate from the context to the build container
# COPY /usr/local/share/ca-certificates/mkcert_development_CA_83999876043114983078982074664490221239.crt  /usr/local/share/ca-certificates/
# # Update the CA certificates in the container
# RUN update-ca-certificates

COPY src/ /app/src/
COPY assets/ /app/assets/
# COPY configs/ /app/configs/
COPY pyproject.toml /app/
COPY pdm.lock /app/
COPY README.md /app/

WORKDIR /app

RUN pip install --upgrade --no-cache-dir pip && \
    pip install --no-cache-dir -e ".[fourier,depthai]" -i https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple --extra-index-url https://download.pytorch.org/whl/cpu && \
    pip uninstall -y typing
