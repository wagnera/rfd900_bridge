# rfd900_bridge
This is a ros packge intended to bridge important topics necessary for teleoperation between a base station (GCS) from a remote rover. Currently it sends important tf information (map->base_link) and cost maps from the rover. It sends cmd_vel messages from the GCS to the rover.

## RFD900 Radios
The RFD900+ is a high performance 900MHz, ISM band radio modem covering the 902 – 928 MHz frequency band. It is designed for long range serial communications applications requiring best in class radio link performance.
### Key Features:
- Long range >40km depending on antennas and GCS setup
- 1 Watt (+30dBm) transmit power.
<img src="http://rfdesign.com.au/wp-content/uploads/rfd900.png" alt="alt text" width="300" height="whatever">
http://rfdesign.com.au/products/rfd900-modem/

## Currently being developed:
- Parameterize certain varaibles such as rate at which information is sent (throttle)
- Add messages for move_base status and goals
- Add messages for GPS

## Stretch goals:
- Add point clouds (currently take too much bandwidth)
