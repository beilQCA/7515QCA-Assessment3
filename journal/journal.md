# 7515QCA Final project journal: *Brendan Eilola*

## Related projects ##

### *POV Globe 24bit True Color and Simple HW* ###

![RelatedProject1](RelatedProject1.jpg)

[POV Globe 24bit True Color and Simple HW (https://www.instructables.com/id/POV-Globe-24bit-True-Color-and-Simple-HW/)

This project provided the inspiration for the 3D printed rotor design as noted in the previous assessment item... the overal form is pretty much the same with some modification for my needs. Also, I used Solidworks to realise the STL file. Looking at this project now, I will spend some time to review how the code drivers were implemented - possibly providing further hints at how I might proceed with my project as I intend.

### *Build a Persistence-of-Vision LED Globe* ###

![RelatedProject2](RelatedProject2.jpg)

[Build a Persistence-of-Vision LED Globe (https://makezine.com/projects/persistence-vision-led-globe/)

This project was one that I decided was the antithesis of how I would approach development. While it employs an Arduino Nano, it is overly complex in circuitry employing multiple shift-registers and what seems to an example of over-engineering particularly for running single-colour LEDs. The POV Calculator tool that the author produced to create pixel-maps is interesting but again, over-engineered for its purpose... why go to the effort of developing software to import images for tracing when it could be done in software directly?!?

### *How to Build Custom Android App for your Arduino Project using MIT App Inventor* ###

![RelatedProject3](RelatedProject3.jpg)

[How to Build Custom Android App for your Arduino Project using MIT App Inventor (https://howtomechatronics.com/tutorials/arduino/how-to-build-custom-android-app-for-your-arduino-project-using-mit-app-inventor/)

This project provided some insight which led to my further investigating and subsequently employing MIT App Inventor to provide the rapid prototyping tool for developing the phone App that would interface with the project's microcontrollers.

### *Wifi Enable POV with 1 meter 144 APA102 LED Strip* ###

![RelatedProject4](RelatedProject4.jpg)

[Wifi Enable POV with 1 meter 144 APA102 LED Strip (https://hackaday.io/project/18290-wifi-enable-pov-with-1-meter-144-apa102-led-strip/details)

This was another 'out there' version which while impressive in its operation, lacked the finesse in its construction. This uses a strip in a 'propellor' fashion mounted on a very ugly stand... none of the physical construction appeals, but the software engineering is detailed and may provide some useful insight for future development.

### *Make Your Own POV LED Globe* ###

![RelatedProject5](RelatedProject5.jpg)

[Make Your Own POV LED Globe] (https://www.instructables.com/id/Make-Your-Own-POV-LED-Globe/)

The accompanying YouTube video for this project provided significant insight I was looking for in the physical construction of the device. I adopted a number of ideas including the use of heavy-gauge steel plate and mounting the DC motor to it. Also, the accompanying code example was used as a starting point for creating the character set idea. While this project's code is somewhat clumsy and lacking elegant succinct solutions, it was enough to spark an idea which I took further to create a full alphabet utilising a much more efficient lookup mapping of characters in a single loop construct.


## Other research ##

Some useful resources used in the research for this project... including tangental ideas. This is not an exhaustive list but merely representative of the effort expended.

https://github.com/hzeller/rpi-rgb-led-matrix/issues/322 - initial research looking at displaying of video on RGB LED displays.
http://www.digital-led-strips.com/category/addressable-led-strip - sourcing the highest density addressable RGB LED strips available.
https://lastminuteengineers.com/a4988-stepper-motor-driver-arduino-tutorial/ - investigating the use of stepper motor drivers.
https://learn.adafruit.com/adafruit-dotstar-leds/python-circuitpython - reference material pertaining to APA102 RGB LED programming.
https://itnext.io/using-a-raspberry-pi-to-control-leds-part-iii-react-native-app-29ee3f4afb8c - considering Bluetooth/MCU options.
https://github.com/Polidea/react-native-ble-plx#configuration--installation - resource relating to Bluetooth LE and React Native.
https://medium.com/@andrewlr/raspberry-pi-zero-w-setup-ab16f89d8120 - considering using Raspberry Pi Zero W as MCU.
https://techtutorialsx.com/2017/09/30/esp32-arduino-external-interrupts/ - researching low-level hardware interrupts on ESP32 MCU's.
https://learn.sparkfun.com/tutorials/apa102-addressable-led-hookup-guide/arduino-example - another APA102 programming resource.
http://blog.pagefault-limited.co.uk/wemos-lolin32-pinout-vs-wemos-lolin32-lite-pinout - pinout schematic for WEMOS LOLIN32 MCU.
https://github.com/ahettlin/PiLED - a branch of the FastLED library targeted for use on Raspberry Pi MCUs.
https://www.pjrc.com/teensy/interrupts.html - investigating potential use of Teensy as MCU - looking at hardware interrupts.
https://github.com/samguyer/FastLED - THE definitive FastLED branch that works with ESP32 et.al. MCU's. 
https://github.com/espressif/arduino-esp32/blob/master/libraries/BluetoothSerial/examples/SerialToSerialBT/SerialToSerialBT.ino - example code addressing serial comms via Bluetooth using Arduino/C.
https://github.com/espressif/arduino-esp32/issues/2055 - investigating issues pertaining to ESP32/Arduino and multiple simultaneous Bluetooth connections.
https://community.appinventor.mit.edu/t/multiple-bluetooth-connection-in-one-screen/2058/6 - investigating multiple Bluetooth connections in MIT App Inventor.
https://github.com/nkolban/esp32-snippets/blob/master/Documentation/BLE%20C%2B%2B%20Guide.pdf - guide to BluetoothLE and ESP32 MCU's
https://www.element14.com/community/groups/exploringarduino/blog/2017/07/16/bluetooth-data-file-image-transfer-and-control-using-arduino - investigating the application of image transfer using Bluetooth/Arduino.
https://www.arduino.cc/reference/en/ - definitive Arduino programming reference.
https://community.appinventor.mit.edu/t/how-to-convert-image-to-string-base64-and-send-to-bluetooth-arduino/1906/6 - investigating the use of base64 image compression for potential use in file transfer between phone app and Arduino.
https://www.deviceplus.com/arduino/jpeg-decoding-on-arduino-tutorial/ - research pertaining to JPEG decoding with Arduino.
https://www.youtube.com/watch?v=mwXidmTDAfs - investigating potential ideas for simple real-time drawing interface.
https://randomnerdtutorials.com/esp32-bluetooth-low-energy-ble-arduino-ide/ - researching BLE comms with ESP32 and Arduino IDE.

## Conceptual development ##

### Design intent ###
<!--- Include your design intent here. It should be about a 10 word phrase/sentence. --->

### Design ideation ###
<!--- Document your ideation process. This will include the design concepts presented for assessment 2. You can copy and paste that information here. --->

### Final design concept ###
<!--- This should be a description of your concept including its context, motivation, or other relevant information you used to decide on this concept. --->

### Interaction flowchart ###
<!--- Include an interaction flowchart of the interaction process in your project. Make sure you think about all the stages of interaction step-by-step. Also make sure that you consider actions a user might take that aren't what you intend in an ideal use case. Insert an image of it below. It might just be a photo of a hand-drawn sketch, not a carefully drawn digital diagram. It just needs to be legible. --->

![Image](missingimage.png)

## Process documentation ##
<!--- In this section, include text and images (and potentially links to video) that represent the development of your project including sources you've found (URLs and written references), choices you've made, sketches you've done, iterations completed, materials you've investigated, and code samples. Use the markdown reference for help in formatting the material.

This should have quite a lot of information! It will likely include most of the process documentation from assessment 2 which can be copied and pasted here.

Use subheadings to structure this code. See https://guides.github.com/features/mastering-markdown/ for details of how to insert subheadings.

There will likely by a dozen or so images of the project under construction. The images should help explain why you've made the choices you've made as well as what you have done. --->

## Final code ##

<!--- Include here screenshots of the final code you used in the project if it is done with block coding. If you have used javascript, micropython, C, or other code, include it as text formatted as code using a series of three backticks ` before and after the code block. See https://guides.github.com/features/mastering-markdown/ for more information about that formatting. --->


## Design process discussion ##
<!--- Discuss your process used in this project, particularly with reference to aspects of the Double Diamond design methodology or other relevant design process. --->


## Reflection ##

<!--- Describe the parts of your project you felt were most successful and the parts that could have done with improvement, whether in terms of outcome, process, or understanding.

What techniques, approaches, skills, or information did you find useful from other sources (such as the related projects you identified earlier)?

What parts of your project do you feel are novel. This is IMPORTANT to help justify a key component of the assessment rubric.

What might be an interesting extension of this project? In what other contexts might this project be used? --->
