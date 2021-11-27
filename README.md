# RPM
Remote Patient Moniroring System
# Project Idea
Health care industry new comers and legacy players alike are vying to contribute to the growing market of Remote Patient monitoring Systems(RPM) especially after the pandemic. RPM services leverage technology to offer healthcare delivery by gathering patient’s data outside of traditional healthcare settings; a technology that proves extremely useful in line with social distancing requirements that limit people’s physical visits to their healthcare providers. 
We aim to provide a low-cost alternative to existing RPM services that caters for two basic needs a) The need to detect patient’s vital signals and medication doses timely and b) The need to notify caregivers in due time in case of life-threatening situations.

# System Features
## Our system is comprised of four main features:


**1. Monitoring vital signals**
Report heart rate, oxygen level and blood pressure in real-time to the caregiver and alert in case data exceeds pre-defined thresholds

**2. 24-hour location tracking**
Use GPS and GSM module in phone to track exact location of the patient and in case of emergencies( person moving out of a pre-set zone), alert the caregiver by sending their location. 

**3. Scheduled Voice-Notifications medication reminders.**
Remind patients of pre-set medication times with a specified pre-configured voice message for each medication

**4. SOS buttons for emergencies**
Special SOS button is to be used by the patient in case of discomfort or when facing any adverse situation to send their location to their caregiver to be able to intervene immediately

# System Design 
![](https://github.com/Dinamarei/RPM/blob/main/diagram.png?raw=true)

## Hardware Components 
	•	BLE33 microcontroller
	•	Holter ECG 
	•	Heart-rate sensor
	•	Blood-Oxygen level sensor (SpO2)
	•	Push buttons
	•	Speaker/ MP3 module
	•	LED indicators


Sample Use Cases 
![](https://github.com/Dinamarei/RPM/blob/main/Screen%20Shot%202021-11-18%20at%209.46.11%20PM.png)

Patient A is diagnosed with dementia and can make an attempt to leave their homes and wander away into life-threatening situations. With the help of our system, a caregiver can be more assured knowing that the patient is continually monitored and in safe hands. If the GPS tracker detects that a person is detected outside of the pre-defined zone, the exact location is sent as an SMS to their caregiver.

Patient B is diagnosed with a cardiovascular disease and for patients with high blood pressure the treatment is typically ongoing and involves one or more medications. Hence, our RPS system can be used to monitor the vitals of the patient consistently and schedule medication timings that can be altered to accommodate to adjustments and alert the caretaker. 



