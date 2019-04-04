FROM nvidia/cuda:8.0-cudnn7-devel-ubuntu14.04

ENV DEBIAN_FRONTEND=noninteractive

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Run installers. (root level installs)
COPY installers /tmp/installers

RUN bash /tmp/installers/install_all.sh
RUN bash /tmp/installers/install_user.sh

## Opengl fix
COPY --from=nvidia/opengl:1.0-glvnd-runtime-ubuntu14.04 \
  /usr/local/lib/x86_64-linux-gnu \
  /usr/local/lib/x86_64-linux-gnu

COPY --from=nvidia/opengl:1.0-glvnd-runtime-ubuntu14.04 \
  /usr/local/share/glvnd/egl_vendor.d/10_nvidia.json \
  /usr/local/share/glvnd/egl_vendor.d/10_nvidia.json

RUN echo '/usr/local/lib/x86_64-linux-gnu' >> /etc/ld.so.conf.d/glvnd.conf && \
    ldconfig && \
    echo '/usr/local/$LIB/libGL.so.1' >> /etc/ld.so.preload && \
    echo '/usr/local/$LIB/libEGL.so.1' >> /etc/ld.so.preload 

# Define environment variables
ENV USERNAME yx

# Change user
USER $USERNAME
WORKDIR /home/$USERNAME

# User level commands
RUN bash /tmp/installers/post_install.sh
