FROM ubuntu:20.04

## Get some karma ##
MAINTAINER Florian Briegel, briegel@mpia.de

# podman build --tag lvmtan --rm  -f ./container/Dockerfile .

# podman run -ti --rm --name lvmtan -p 40000:40000 -p 5905:5900 -p 8095:6080 -e BASDARD_PORT=40000 -e BASDARD_CONFIG=lvm/lvm.moe-sim.conf -e BASDARD_UI=lvm/lvm.all.ui -e VNC_GEOM=860x900 -e LVM_DEBUG=yes -e LVM_ROOT=/root/lvm -v ~/lnsw/lvmtan:/root/lvm localhost/lvmtan



RUN apt-get update
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

RUN apt update -y && \
    wget -qO - https://svn.mpia.de/repo/linux/ubuntu/archive.key | apt-key add - && \
    echo "deb [ allow-insecure=yes ] https://svn.mpia.de/repo/linux/ubuntu focal main" > /etc/apt/sources.list.d/mpia.list

RUN apt update -y

RUN apt-get install -y --allow-unauthenticated tan-basda-mocca
RUN dpkg --purge --force-all python3-astropy

COPY . lvmtan
RUN cd lvmtan && pip3 install .

EXPOSE 40000/tcp

WORKDIR /root

COPY ./container/env-basdard.sh .
COPY ./container/run-basdard.sh .
COPY ./container/novnc_server .

CMD ["/root/run-basdard.sh"]
