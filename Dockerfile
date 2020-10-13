FROM ubuntu:20.04
RUN apt update && apt install -y sdcc git make binutils
COPY . /code
WORKDIR /code
RUN git submodule update --init
WORKDIR /code/src
RUN make
