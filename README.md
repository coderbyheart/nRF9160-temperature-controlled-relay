# Temperature controlled relay using the nRF9160 DK

This implements a temperature controlled relay using the nRF9160 DK:

- temperature is measured via a DHT sensor (AM2302)
- a relay is switched on, if the temperature is below a threshold
- the threshold can be increased/decreased by pressing button 2/1
- it publishes the temperature to the cloud
- read threshold from cloud
- (TODO) override cloud threshold on button press

## Building

### Prepare the Docker image with all build dependencies

    docker build -t sdk-nrf-tcr .

### Build the firmware

    docker run --rm -v ${PWD}:/workdir/ncs/tcr sdk-nrf-tcr \
        /bin/bash -c 'cd /workdir/ncs/tcr && west build -p always -b nrf9160dk_nrf9160ns'

### Flash the firmware

    nrfjprog -f nrf91 --program build/zephyr/merged.hex --sectoranduicrerase -r --log
