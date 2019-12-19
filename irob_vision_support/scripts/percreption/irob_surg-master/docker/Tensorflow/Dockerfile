FROM nvidia/cuda:10.0-base-ubuntu18.04
LABEL Name=docker Version=1.0.0
RUN apt -y update \
    && apt install -y python3 python3-pip libsm6 libxext6 libxrender-dev
RUN pip3 install numpy opencv-python scipy
RUN pip3 install tensorflow-gpu