# Supervisor

The supervisor is designed to:

- monitor connection to the StreetDrone
- check that nodes remain alive
- monitor cpu/gpu/ram/disk space
- monitor any error states from our software
- check that sensors are correctly returning data

As long as no problems are found, it sends the required heartbeat to the StreetDrone.