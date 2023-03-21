FROM ubuntu:jammy-20230308

ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y python3 python3-dev python3-pip
RUN apt-get update && \
    apt-get install -y nano tmux curl htop net-tools iproute2 iputils-ping gdb sudo git

RUN useradd -ms /bin/bash tj2

RUN chown -R tj2 /home/tj2 && \
  adduser tj2 sudo && \
  echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER tj2
WORKDIR /home/tj2

RUN sudo ln -s /usr/bin/python3 /usr/bin/python
ENV PATH=/home/tj2/.local/bin${PATH:+:${PATH}}

RUN python -m pip install robotpy
RUN robotpy-installer download-python

COPY --chown=1001:1001 ./install /home/tj2/install

RUN python -m pip install -r install/requirements.txt
RUN robotpy-installer download -r install/requirements.txt
