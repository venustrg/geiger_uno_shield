# geiger_uno_shield
geiger counter built on Arduino Uno + display shield.

schematic and original fw by toxcat - https://github.com/project37cat/gca01

described and discussed here - https://cxem.net/dozimetr/3-10.php

![Screenshot](Schematic_geiger_uno_shield_2020-10-07_23-34-40.png)

+ remastered and optimized sketch;
+ schematic modified to avoid interference;
+ added zener diodes to protect the tube from overvoltage (although they are not required);
+ added BME280 baro/temp/humidity sensor support. connected to A4 (SDA), A5 (SCL);

baro/temp/humidity sensor is optional.
