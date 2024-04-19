# IntegratedStepper
Integrated Stepper Motor Node


## Relevant Links
- [Annin AR3](https://robodk.com/robot/Annin-Robotics/AR3)
- [SimpleFOC](https://github.com/simplefoc/Arduino-FOC)
- [SimpleFOC CANCommander spec](https://docs.google.com/spreadsheets/d/1VWHshxOOnxZyKf5IjVzg1VQMbh7cPu5BULljX7UZJ8g)
- [CAN 2.0B spec](https://affon.narod.ru/CAN/CAN20B.pdf)
- [Waveshare USB CAN adapter](https://www.waveshare.com/usb-can-a.htm)
- [python-can docs](https://python-can.readthedocs.io/en/stable/)
- [Lemon Pepper Stepper](https://github.com/VIPQualityPost/lemon-pepper-stepper)
- [Simple-FOC-CAN](https://github.com/jkirsons/Simple-FOC-CAN.git)
- [SimpleFOC forums CAN megathread](https://community.simplefoc.com/t/can-bus-interface-simplefoc-standard-identifiers/577/32)
- [SimpleFOC Commander interface](https://docs.simplefoc.com/commander_interface)


## Gotchas
- 7 MSBs of Extended Format Base ID (ID-28 - ID-22) cannot all be recessive (CAN 2.0B spec, p. 13)

## Notes

CAN node to motor = one-to-many relationship -> bus id request has 0 motor id
