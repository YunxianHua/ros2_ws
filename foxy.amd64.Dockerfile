FROM --platform=linux/amd64 ros:foxy

RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA
# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends nlohmann-json3-dev  \
    libasio-dev libboost-all-dev  \
    libssl-dev libwebsocketpp-dev \
    zip unzip
RUN rm -rf /var/lib/apt/lists/*