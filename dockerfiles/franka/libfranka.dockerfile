# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

#Install libfranka: https://support.franka.de/docs/installation_linux.html#build-libfranka
RUN apt install -y build-essential
RUN apt install -y cmake
RUN apt install -y git
RUN apt install -y libpoco-dev
RUN apt install -y libeigen3-dev

WORKDIR /home/$UNAME
RUN git clone --recursive https://github.com/frankaemika/libfranka
WORKDIR /home/$UNAME/libfranka
RUN git checkout 0.13.3
RUN git submodule update
RUN mkdir build
WORKDIR /home/$UNAME/libfranka/build
RUN cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
RUN cmake --build .
RUN cpack -G DEB
RUN dpkg -i libfranka*.deb