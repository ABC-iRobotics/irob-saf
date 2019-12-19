FROM nvidia/cuda:10.1-base-ubuntu18.04
LABEL Name=docker Version=1.0.0
RUN apt -y update \
    && apt install -y python3 python3-pip libsm6 libxext6 libxrender-dev
RUN pip3 install numpy opencv-python
RUN pip3 install torch torchvision