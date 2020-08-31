# Temperature controlled relay using the nRF9160 DK

This implements a temperature controlled relay using the nRF9160 DK:

- temperature is measured via a DHT sensor (AM2302)
- a relay is switched on, if the temperature is below a threshold
- the threshold can be increased/decreased by pressing button 2/1
- (TODO) publish temperature to cloud
- (TODO) read threshold from cloud

## Building

    docker build -t tcr-docker .
    # Build the firmware
    docker run --rm -v ${PWD}:/workdir/ncs/tcr tcr-docker \
        /bin/bash -c 'cd /workdir/ncs/tcr && west build -p always -b nrf9160dk_nrf9160ns'
    # Flash the firmware
    nrfjprog -f nrf91 --program build/zephyr/merged.hex --sectoranduicrerase -r --log
