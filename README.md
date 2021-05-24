# solderIt
Simple soldering controller for hakko 936 type soldering iron. Based on attiny85.

Controlled by encoder.

To save pins of attiny85 one pin is used for encoder input (3 contacts) and to output sound to buzzer.

Encoder inputs connected to form resistor ladder.

The input to ADC looks like this:
![Coded input to ADC](https://github.com/cppBaddy/solderIt/blob/main/ADC_coded_rotary.png?raw=true)

And decoded like this:
![Coded input and decoded signal](https://github.com/cppBaddy/solderIt/blob/main/ADC_decoded.png?raw=true)

Display based on ssd1306 controller driven by i2c bus


Schematics already modified to use T12 handles:

T12 Soldering Controller based on Attiny85, 0.98" OLED display, encoder and buzzer.

Features:

    Temperature PID controller
    One shared pin to read encoder and output buzzer signal
    Graphical OLED display (set point, current temperature and analog power bar)
    Input for handle temperature sensor

![solderIt schematics](https://github.com/CppBaddy/solderIt/blob/main/schematics.png?raw=true)

Schematic and PCB design can be found [here](https://easyeda.com/Yulay/Simple_Soldering_Controller-6bac2e98d4c44ddf9c34261973521047) 
