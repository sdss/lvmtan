FROM ubuntu:20.04

## Get some karma ##
MAINTAINER Florian Briegel, briegel@mpia.de

# podman  build --tag ubuntu_lvm_tan --rm lvm_tan

# podman run -ti --rm --name lvm_tan -e DISPLAY --network=host --ipc=host --device /dev/dri -v ~/.Xauthority:/root/.Xauthority:Z -v .:/root/lvmt:Z -P localhost/ubuntu_lvm_tan


RUN apt-get update
#RUN apt-get install -y locales locales-all
#ENV LC_ALL en_US.UTF-8
#ENV LANG en_US.UTF-8
#ENV LANGUAGE en_US.UTF-8

RUN apt-get install -y locales
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV DEBIAN_FRONTEND noninteractive

RUN apt update -y && \
    apt install -y wget && \
    DEBIAN_FRONTEND=noninteractive apt install -y software-properties-common keyboard-configuration && \ 
    wget -qO - https://zeroc.com/download/GPG-KEY-zeroc-release-B6391CB2CFBA643D | apt-key add - && \
    apt-add-repository "deb http://zeroc.com/download/ice/3.7/ubuntu20.04 stable main"

RUN apt install -y sudo subversion git lsof

RUN apt update -y
RUN apt install -y python3-setuptools python3-pip python3-toml \
        git tightvncserver fluxbox wmctrl novnc

RUN apt install -y packagekit-gtk3-module python3-opencv

RUN apt update -y && \
    wget -qO - https://svn.mpia.de/repo/linux/ubuntu/archive.key | apt-key add - && \
    echo "deb [ allow-insecure=yes ] https://svn.mpia.de/repo/linux/ubuntu focal main" > /etc/apt/sources.list.d/mpia.list

RUN apt update -y

RUN apt-get install -y --allow-unauthenticated tan-basda-mocca

#RUN cd /root && \
#    git clone https://github.com/sdss/lvmtan.git && \
#    (cd lvmtan && python3 setup.py install || echo "fix me") && \
#    cp lvmtan/container/run-basdard.sh . && \
#    rm -rf lvmtan

RUN pip3 install numpy astropy sdss-clu sdss-lvmtan

EXPOSE 40000/tcp

WORKDIR /root

COPY ./container/env-basdard.sh .
COPY ./container/run-basdard.sh .
COPY ./container/novnc_server .

CMD ["/root/run-basdard.sh"]
