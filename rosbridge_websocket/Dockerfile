FROM ros:humble-ros-core-jammy

# rosbridge needs the system loader workaround because the base image protects ldconfig.
RUN [ -f /sbin/ldconfig ] && mv /sbin/ldconfig /sbin/ldconfig.real && ln -s /bin/true /sbin/ldconfig || true

RUN apt-get update && apt-get install -y \
    ros-humble-rosbridge-suite \
    && rm -rf /var/lib/apt/lists/*

RUN [ -f /sbin/ldconfig.real ] && rm /sbin/ldconfig && mv /sbin/ldconfig.real /sbin/ldconfig && ldconfig || true


EXPOSE 8000

CMD ["ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml", "port:=8000"]