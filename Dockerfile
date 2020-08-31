FROM coderbyheart/fw-nrfconnect-nrf-docker:latest
RUN rm -rf /workdir/ncs
COPY . /workdir/ncs/tcr
RUN \
    # Zephyr requirements of tcr
    cd /workdir/ncs/tcr; west init -l && \
    cd /workdir/ncs; west update && \
    pip3 install -r zephyr/scripts/requirements.txt && \
    pip3 install -r nrf/scripts/requirements.txt && \
    pip3 install -r bootloader/mcuboot/scripts/requirements.txt && \
    rm -rf /workdir/ncs/tcr
