# 3dCane


## Introduction
In 2017, we, Jacek Martyniak and Łukasz Nowarkiewicz from the Electrical and Electronic School in Szczecin, under the guidance of our mentor with a PhD, Dariusz Mostowski, took on a project for the XVII Innovation and Invention Olympics. Our goal was simple: to design an intelligent electronic cane for the visually impaired and blind. This cane was intended to make navigating both outdoor city environments and indoor spaces easier for them. We focused on ensuring the cane was easy and intuitive to use, with a modular design that could be produced and maintained at a reasonable cost.

## Project assumptions

Our project, 3D.CANE, is built on the foundation of enhancing mobility for the visually impaired through a three-dimensional environmental analysis using a multidirectional ultrasonic scanner. This electronic cane system delivers clear information about potential obstacles and features dual-mode communication (acoustic messages and tactile feedback via vibration) to ensure user safety and convenience. It includes features like fall and assault alarms, time and location updates, and modular design for easy maintenance and upgrades, all while focusing on cost-effective production and user-friendly design.

## Hardware - electronics

The core of the 3D.CANE system is built around two 8-bit AVR ATmega microcontrollers by ATMEL, with the electronic cane module based on an ATmega64 for its advanced capabilities in handling two-way communication with the wrist-worn control module and controlling the ultrasonic scanner module. Essential to the project's functionality, we incorporated various dedicated circuits, including a digital accelerometer (ADXL325), a radio communication module (NRF24L01), vibration motor controllers (BC847 transistors), and distance measuring modules (US-015), all chosen for their reliability and cost-effectiveness. The software was developed in the C language within the Eclipse Indigo environment, emphasizing modular and easily upgradable design.


## Hardware - electronics - 3d.cane 

1. **Power Supply Block:** Utilizes a 3.7V lithium-ion battery type 18650, paired with a compact TP4056 integrated circuit charger, enabling charging via an external adapter through a micro USB port on the handle casing.

2. **Central Processing Unit (CPU):** Heart of the cane, featuring an 8-bit AVR microcontroller by Atmel based on a Reduced Instruction Set Computing (RISC) architecture. This unit, equipped with dual UART interfaces, facilitates programming and communication with the ultrasonic sensors, and possesses picoPower technology for energy efficiency.

3. **Ultrasonic Sensor Block:** Employs three US-015 ultrasonic modules for a three-axis environmental scanner, which communicate via trig and echo lines to measure distances using 40 KHz sound waves, and calculates distance based on the time duration of the high state signal.

4. **Logic Level Conversion Block:** Incorporates a 74LVC125 buffer/converter IC to accommodate devices requiring lower voltage levels than the microcontroller's 5V, including the microSD card, radio communication module, and accelerometer.

5. **Radio Communication Block:** Facilitates wireless interaction between the cane and a wrist-worn device using an nRF24L01 module operating at 2.4GHz, enabling remote activation of the cane's auditory signals for easy location.

6. **Memory Card Slot:** Utilizes a microSD card for storing voice recordings, easily transferable from a computer, providing information such as battery status and charging state.

7. **Accelerometer:** The ADXL345, a 3-axis, high-resolution 13-bit accelerometer, detects falls and improper use by monitoring deviations from vertical alignment, contributing to the user's learning process for correct cane usage.

8. **Audio Block:** Contains a speaker powered by an LM4871 amplifier, providing clear audio communications about battery charge and operational status, with recordings decoded from the microSD card via the microcontroller's PWM signal.

9. **Vibration Motors Block:** Equipped with three vibration motors positioned to indicate obstacle directionality (left, right, or front) to the user, operating in pulsating or continuous modes based on obstacle proximity, powered directly without needing a voltage regulator due to calculated current requirements.

This system ensures a robust, interactive aid for visually impaired users, blending complex electronics and intuitive feedback mechanisms to enhance navigation and safety.

## Hardware - electronics - watch

The wristwatch module is an integral part of our design, functioning as a remote interface for the electronic cane, allowing users to trigger an alarm and access time through voice commands. This module, mirroring the electronic cane's hardware framework, is streamlined for wearable use and comprises seven main functional blocks:

1. **Power Supply Block:** Unlike the cane, the watch uses a smaller, rechargeable YX-S1 battery tailored for smartwatches, featuring a 380mAh capacity and 3.7V nominal voltage with built-in short-circuit and under-voltage protection, ensuring safety and reliability.

2. **Accelerometer:** Employed for dual purposes—detecting falls to initiate an alarm and enabling gesture control to activate the watch's voice functions, such as announcing the time upon a specific arm movement.

3. **Memory Card Slot:** Functions similarly to the cane's, storing voice recordings that include battery status and time announcements, ensuring vital information is readily available to the user.

4. **RTC (Real-Time Clock):** Utilizes the DS3231 integrated circuit for accurate timekeeping, adjusting for temperature variations to maintain precise time. This feature is essential for providing current time updates through voice output.

5. **Central Processing Unit:** Built around the ATmega328P microcontroller, this unit is designed for efficient operation within the compact dimensions of a wristwatch, handling input from buttons, time management, audio playback, and wireless communication with the cane.

6. **Audio Block:** Similar to the cane's audio system but with the amplifier directly powered by the battery, ensuring clear voice playback without the need for voltage level conversion, simplifying the design and maximizing battery efficiency.

7. **Radio Communication Block:** Enables wireless communication with the electronic cane without the need for logic level translation, as both the microcontroller and the nRF24L01 communication module operate at 3.3V, facilitating seamless interaction between the cane and the watch.

This design ethos ensures that the wristwatch module is not only a functional extension of the electronic cane system but also a standalone device capable of providing users with critical information and controls, enhancing the overall utility and accessibility of the system for visually impaired individuals.


## Hardware - mechanics

The 3D.CANE's mechanical structure, designed in Autodesk Inventor and realized through a combination of 3D printing and aluminum tubing, emphasizes practicality and user adaptability:

1. **Cane Shaft:** Made from aluminum tubes for lightweight strength.
2. **Magnetic Tip Attachment:** Allows for easy swapping of interchangeable tips to match different terrains.
3. **Interchangeable Tips:** Varied designs cater to specific environmental needs.
4. **Ultrasonic Sensor Housing:** Positioned at the cane's lower end for environmental scanning.
5. **Wristwatch Module Casing:** Compact and ergonomic, houses the essential electronic components.

This design strategy ensures the cane is both functional and adaptable, meeting the diverse needs of its users.


## Prototyping and testing

The development and assembly of the 3D.CANE system involved a meticulous process of prototyping, circuit assembly, case fabrication, and thorough testing of components. 

### 1. System Concept Prototyping
Initial prototyping focused on validating accelerometer functionality and audio playback from a memory card using the Atmel ATB rev. 1.04 development board. This versatile setup minimized the need for a breadboard and extensive wiring, streamlining early testing.

### 2. Printed Circuit Board Assembly
Given the fine traces and small component sizes, PCBs were produced using an LPKF machine, which mechanically mills away excess copper and drills mounting holes. This process was followed by deburring, flux application to prevent corrosion, and manual creation of vias using wire or conductive paste. Both through-hole (THT) and surface-mount (SMT) components were manually soldered using a WEP 937D soldering station and WEP 858D HotAir station.

### 3. Module Casing Fabrication
Module casings for both the cane's handle and the wristwatch were prototyped using 3D printing technology, utilizing ABS and PET plastics for durability and lightweight design. This approach allowed for ergonomic and user-friendly designs to be tested and refined quickly.

### 4. Programming and Module Testing
The USBASP programmer was used for flashing the AVR microcontrollers, accommodating the different operating voltages (5V for the cane and 3.3V for the watch) without risking damage to the components. Each device's functionality, including RTC, wireless communication, and other subsystems, was individually verified before integrating them into the complete system.

### Assembly Challenges
Assembling the wristwatch involved attaching charging connectors and magnets, soldering the battery and wiring, fitting the components into the casing, and securing the cover. Assembling the cane was more complex, requiring the aluminum structure assembly, sensor mounting, wiring, and integration of vibratory motors, speaker, and electronic components into the handle.

### Testing and Finalization
Using the mkAVRcalculator software, the microcontrollers in both the cane and watch were detected and programmed. After verifying the operation of individual modules like the RTC and wireless communication unit, the subsystems were integrated into their respective devices, culminating in the final assembly of fully functional prototypes.

## Summary, future work

The conclusion of the 3D.CANE system development underscores both the achievements and the avenues for future improvements:

### Achievements
- The system successfully integrates an electronic cane and a wristwatch module, enhancing navigational capabilities for visually impaired users.
- The calibration and configuration process is straightforward, allowing users to easily activate and set up both the cane and the watch.
- The prototype demonstrates the practical application of ultrasonic sensors for environment scanning, along with audible and tactile feedback mechanisms.

### Areas for Improvement and Future Work
- **Cane Illumination:** The project did not accomplish the implementation of cane illumination due to the depletion of transparent filament. Adding illumination could enhance visibility and safety during low-light conditions.
- **Sensor Data Processing Speed:** Limited development time impacted the optimization of sensor data processing. Future iterations could focus on enhancing the speed at which sensor data is analyzed and acted upon, improving real-time feedback to users.
- **Communication Range Between the Cane and Watch:** The potential to extend the communication range was not fully explored. Future developments could focus on maximizing the capabilities of the radio module to increase the operational distance between the cane and the wristwatch, enhancing usability and flexibility.

### Next Steps
- **Optimization of Code and Hardware:** Refining the software to process sensor inputs more efficiently and exploring more advanced hardware solutions could significantly improve the system's responsiveness and user experience.
- **Extended Functionality:** Introducing new features, such as GPS navigation or integration with smartphone apps, could provide additional value, making the system even more versatile and user-friendly.
- **User Testing and Feedback:** Conducting extensive user testing with visually impaired individuals will be crucial to identify practical challenges and user needs, guiding further refinements and ensuring the system meets its intended goals effectively.

The 3D.CANE project, while successful in its current form, presents several opportunities for enhancements that could make it an even more valuable aid for the visually impaired community. Continuous development, guided by user feedback and technological advancements, will be key to realizing its full potential.
