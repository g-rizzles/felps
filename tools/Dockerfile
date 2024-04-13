FROM debian:bullseye-20230208-slim

ENV ARDUINO_VERSION="1.8.19"
ENV DEBIAN_FRONTEND=noninteractive

RUN apt -y -qq update && \
  apt -y -qq --no-install-recommends --allow-change-held-packages install \
  software-properties-common \
  wget \
  zip \
  git \
  make \
  srecord \
  bc \
  xz-utils \
  gcc \
  curl \
  xvfb \
  python3 \
  python3-dev \
  python3-pip \
  build-essential \
  libncurses-dev \
  flex \
  bison \
  gperf \
  libxrender1 \
  libxtst6 \
  libxi6 \
  jq \
  cmake \
  && apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Install arduino-cli
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
ENV ARDUINO_PATH=/root/.arduino15
COPY arduino-cli.yaml ${ARDUINO_PATH}/

# Update cores from config file
RUN arduino-cli core update-index
RUN arduino-cli core install STMicroelectronics:stm32

# Update libs from config file
# TODO as needed

# Setup working directory
RUN mkdir /workspace

# Copy in the command scripts
COPY cmd.sh /opt
#CMD [ "/opt/cmd.sh help" ]