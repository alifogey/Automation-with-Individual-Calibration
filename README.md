# Automation for one arduino
Folder with UI, Arduino, and python code to successfully control 5 pumps using the UI

## To view UI
Open the UI (Try5 project) **or** run the Python app to look at the user interface

## Layout

UI+ Automation/
 ├─ arduino/
 │    └─ RAMPS_PumpControl.ino     # Full updated Arduino firmware
 ├─ UI/
 │    ├─ Main.qml
 │    ├─ SetupPageForm.ui.qml
 │    ├─ RunPageForm.ui.qml
 │    ├─ RunPumpCardForm.ui.qml
 │    ├─ AutomationPageForm.ui.qml
 │    ├─ AutomationPumpCardForm.ui.qml
 │    └─ PumpCardForm.ui.qml
 ├─ backend.py                     # Python → Arduino serial link
 ├─ main.py                        # Python launcher
 └─ calibration.ini                # Saved per-pump calibration

## Run
Arduino
1. Plug in arduino, make sure motor drivers and pumps are connect
2. Open Arduino IDE and select the correct board (Arduino Mega 2560) and port
3. Click File Open and find UNO_loopback_test.ino in UNO_loopback_test folder.
4. Make sure baud is set to 115200
5. Click upload (light on arduino should flash)
6. Plug in external power source
7. To confirm that everythings working, open the Serial Monitor in arduino and test this command, -- If pump is wired correctly, it should move briefly 
```
{"prime": 1}
```
7. Close Serial Monitor (if not closed, won't run)
9. Then open powershell
10. Change the file path to where you downloaded the zip and open UI+ Automation (where main.py lives), example below
cd "C:\Users\aliso\Documents\Microfluidics\UI+automation" 
12. Once inside the right file path, copy this whole block of code
```
pip install PySide6 pyserial
$env:PUMP_SERIAL_PORT="COM4"   # use your real COM port
python main.py
```
