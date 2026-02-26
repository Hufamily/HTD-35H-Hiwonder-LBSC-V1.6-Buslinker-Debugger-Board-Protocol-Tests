# HTD-35H-Hiwonder-LBSC-V1.6-Buslinker-Debugger-Board-Protocol-Tests

## Instructions
### Hardware

#### Wiring
  Arduino TX1 ───────────────── Servo Controller TX
  Arduino RX1 ───────────────── Servo Controller RX
  Arduino GND ───────────────── Servo Controller GND
  Arduino 5V  ───────────────── Servo Controller 5V
  12V Power Source───────────── Servo Controller Battery Port

#### Other Specifications
Check port: Typically COM5
If it's beeping, voltage might be too low (Don't go above 12.3V, needs to be above 6V)

### Software

Using the Arduino IDE, upload `relay.ino` onto your Arduino

Create and activate a venv
```
python -m venv venv
source venv/bin/activate
# .venv/Scripts/Activate.ps1
pip install -r requirements.txt
```

Run the script you want
```
python test.py
```
## Devices

[Arduino Micro](https://store-usa.arduino.cc/products/arduino-micro?utm_source=google&utm_medium=cpc&utm_campaign=US-Pmax&gad_source=4&gad_campaignid=21317508903&gbraid=0AAAAACbEa84ezBf2GeRW1_Pw_74qXgXAs&gclid=CjwKCAiA2PrMBhA4EiwAwpHyCzAHIVqp-YAa69eI18p6cOKIGeaISQbqLujXqceblS3VTSWAMG4nmRoCcc0QAvD_BwE)

[Hiwonder Bus Servo Controller](https://www.amazon.com/Hiwonder-Servo-Controller-Serial-Tester/dp/B09ZL6ZBBN/ref=sr_1_1?crid=1KN6G68ITKMGZ&dib=eyJ2IjoiMSJ9.Fv1Ijh342JLYB8lAtR64dRXj5emVxY0rWD7DD3p2jsGvWUlJ0pbk48pQ-gXCXj4lQ_zuzn1Kke9vgXjOTvQh3CaQ-DNjiKRsPkTOHUrFa5CPxzDnIZ5i06ugbN8Ba0UrY1QVumiyKIjvzcKKz5J8aiVM_KWd6ev78kIyCQ_dNthbpxs1PXoLqrqBFiqMc-agIZxyhXr3vGJkFJrhdaIrm4wWS93FEjl7eKCBQk1gILB7ZbSWiWbZ7OBu793_iFUZ_FK7foimL9WO3TPt66SUq7dN9mPqkhJPfBh_VFxUHRU.IGoaK4Kp-7Ax1_mNlA1H55tE5GKN2yElhUfdvfYA06Q&dib_tag=se&keywords=Hiwonder+Servo+Controller&qid=1772076467&sprefix=hiwonder+servo+con%2Caps%2C294&sr=8-1) (LBSC - V1.6)

[HTD-35H](https://www.amazon.com/Hiwonder-Channels-Temperature-Position-Feedback/dp/B0C9CZXWXR?ref_=ast_sto_dp&th=1)

## Useful Links

[Hiwonder Wiki](https://wiki.hiwonder.com/en/latest/)

Drive with Servo Controller: [https://drive.google.com/drive/folders/1B7h0Q0HydRYi348VFJnfbAmWay4W_1Sd](https://drive.google.com/drive/folders/1B7h0Q0HydRYi348VFJnfbAmWay4W_1Sd)

Drive with Motor: [https://drive.google.com/drive/folders/1yaZ8iRYgWncdHPopioo7OYuHoGhrMxCi](https://drive.google.com/drive/folders/1yaZ8iRYgWncdHPopioo7OYuHoGhrMxCi)