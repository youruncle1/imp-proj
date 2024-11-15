# IMP Project: Digital Clock with Alarm on FITkit 3

**Course:** Microprocessor and Embedded Systems (IMP)  
**Platform:** FITkit 3 with Kinetis K60 (ARM Cortex-M4)  
**Language:** C  

## Project Description
The project involved creating an embedded application for a digital clock with alarm functionality using the Real Time Clock (RTC) module of the Kinetis K60 microcontroller.

### Features
1. **Clock and Alarm:**
   - Set current time and alarm time.
   - Turn the alarm ON/OFF.

2. **Alarm Notifications:**
   - Audible and visual notifications using:
     - Three built-in sound melodies.
     - Three built-in light signaling patterns.

3. **Snooze Functionality:**
   - Adjustable number of retries.
   - Configurable delay between retries(sleep).

4. **User Interaction:**
   - Terminal-based interface with colored text using ANSI escape sequences.
   - Display of current time, alarm time, alarm status, and additional information such as melody and retry IDs, setup menu.

### Implementation
- Developed in **C** using **Kinetis Design Studio (KDS)** and MCUXpresso.
- Utilized the **Real Time Clock (RTC)** module for timekeeping and alarm functionality.
- Configured and controlled peripherals directly through platform registers and APIs.

**Score:** 14/14  