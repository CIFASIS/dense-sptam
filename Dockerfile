FROM osrf/ros:kinetic-desktop-full-xenial
MAINTAINER Ariel

ENV DEBIAN_FRONTEND noninteractive

# Basic tools
RUN apt-get update && \
	apt-get install -y locales apt-utils wget git openssh-client && \
	rm -rf /var/lib/apt/lists/*

# Locale magic
RUN echo "en_US.UTF-8 UTF-8" > /etc/locale.gen && \
    locale-gen en_US.UTF-8 && \
    dpkg-reconfigure locales && \
    /usr/sbin/update-locale LANG=en_US.UTF-8

# ROS robot-localization lib - needed by dense node
RUN apt-get update && \
	apt-get install -y ros-kinetic-robot-localization && \
	rm -rf /var/lib/apt/lists/*

# Install libsuitesparse lib - needed by sptam node
RUN apt-get update && \
	apt-get install -y libsuitesparse-dev && \
	rm -rf /var/lib/apt/lists/*

# Install g2o lib - needed by sptam node
RUN git clone https://github.com/RainerKuemmerle/g2o.git && cd g2o/ && \
	git checkout -q 4b9c2f5b68d14ad479457b18c5a2a0bce1541a90 && \
	mkdir build && cd build && cmake ../ && make && make install && \
	cd ../../ && rm -rf g2o/

# Install catkin tool
RUN apt-get update && \
	apt-get install -y python-catkin-tools && \
	rm -rf /var/lib/apt/lists/*

# Set ROS master URI as localhost is not found in /etc/hosts
ENV ROS_MASTER_URI "http://127.0.0.1:11311"

WORKDIR /usr/src/dense_sptam

CMD ["/bin/bash"]
