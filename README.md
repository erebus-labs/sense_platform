sense-platform
==============
#Sensor Platform 
+ *Title:* A low-cost,  low-power sensor platform with modular sensor addition and base station reprogramming/configuration
+ *Project Short Name:* Open Sensor Platform
+ *Sponsors:* 
  + [Dr. Mike Borowczak](https://www.linkedin.com/in/mborowczak) @ [Erebus Labs](http://www.erebuslabs.com)
  + [Dr. Andrea Burrows] (http://uwyo.academia.edu/AndreaBurrows) @ [University of Wyoming](http://www.uwyo.edu/seced/faculty-staff/andrea-burrows.html)
+ *Senior Design Team:* 
    + Colten Nye
    + Steve Pierce
    + Golriz Sedaghat
+ *PSU Academic Advisor:* [Dr. Lisa Zurk](http://web.cecs.pdx.edu/~zurkl/) 
+ *Targeted Users:* K-20 students (and families/communities) who want to know about their environment 

## Problem/Task
Current sensor platforms are either completed closed source and expensive or partially open, non-customizable, and trivial. The task at hand is to build a fully contained sensor platform / collection device for $25-$50 USD for at volume runs (development versions would be higher with short PCB runs).  The sensor platform must include the ability to add both sensors (physical components) as well as custom code in an intuitive (GUI?) programming environment.

##Background
Current hardware sensor solutions either require programming/electronics experience beyond the reach of many K12 (K20) students, parents, teachers and faculty – or they so turn-key that there is little to no “STEM” required to get a solution up and running. Furthermore, there are currently no “build it yourself” approaches to sensor design. The closest existing solutions revolve around the Arduino platform – though the short coming here is again the abstraction of many of the details as well as the barrier of entry for most K12 students, parents, faculty – especially in mid-large scale deployments in a classroom or a school (>30-1000 units).

In 2013/2014 a first revision Proof of Concept was developed and implemented – this can be used as a starting point once preliminary design investigation has concluded.

##Skill Requirements (Order of importance):
+ embedded System Design {Architecture, Programming}
+ Platform Agnostic Program Development (e.g. crossplatform or web UI)
+ PCB Layout and Design
+	Communication (BT, Serial, wifi?) 

##Other Details: 
+	IP Ownership: Joint/”none”: The goal is to release an open hardware device and software; joint ownership of the original design, publicly available for reuse under similar attribution
+	Extension after End of Project: 50-100 parts run to distribute to high-needs schools as a mechanism to enhance/drive STEM as a college degree option.
+	Publication Opportunities: Conference and Journal Publication both in K20 Education, IEEE Spectrum, and/or others (joint ownership: ECE team and Sponsors) [resume/CV builder!]
+	No NDA required. 

##Requirements (Wish List): **THIS SECTION SHOULD BE REPLACED BY PSU TEAM BEFORE DESIGN START**

###Must Have
+ [ ]	Low hardware cost <$50 per unit
+ [ ]	Open Source Hardware Design & Board (can use “closed source” components: ASICs, uC, etc.)
  + [ ]	Must Have a Multi-Chip solution – e.g. no single SoC; split data store
+ [ ]	Open Software Repository (github)
+ [ ]	Built-in basic sensor array (Oxygen, VoC, Accelerometer) + Ability to add at least one sensor
+ [ ]	Power:
  + [ ]	Low power operation (3 month duty-cycle; minimum 12 measurements per hour +sync)
  + [ ] Built in power source / power pack
+ [ ]	External Device Tethering/Sync upon physical request
+ [ ] Parameterized data collection mechanism
  + [ ] Set sensor Upper and Lower Thresholds
  + [ ] Polling frequency
  + [ ] Measurement to Physical Value Translation (they’re not all linear)
+ [ ] Set of units for each team member and two sponsors

###Like to Have
+ [ ]	Hardware cost <$30 per unit
+ [ ]	High Humidity Operation
+ [ ]	Swappable, Muxable Sensor array +software support
+ [ ]	Power
  + [ ] Ultra-low power operation (6 month duty-cycle; minimum 12 measurements per hour +sync)
  + [ ] Built in rechargeable power source / power pack
+ [ ] Device Tethering/Sync through Bluetooth/wifi in-range
+ [ ] High Level API wrapper to control device functionality for novice programmers
+ [ ] Partnership building with a local K-12 school; Flexible options include teacher conversations, lesson development, and/or helping with a lesson stemming from this research
+ [ ] Set of units for each team member and two sponsors + 5 extra units

###Nice to Have
+ [ ] Hardware cost <$15 per unit
+ [ ] GUI based overlay for API suitable for use by K-8 students
+ [ ] Power
  + [ ] Extremely-low power operation (12 month duty-cycle; minimum 12 measurements per hour +sync)
  + [ ] Built in power source w/solar recharge
+ [ ] Set of units for each team member and two sponsors + 10 extra units (5 to sponsors; 5 to PSU)

##Milestones:

###Hardware Evaluation to satisfy given constraints 
+ [ ] Main System: Modification of existing Open Hardware Design to create part or creation of custom PCB?
+ [ ] Power System: Battery capacity + recharge + cycled usage + communication cost?
+ [ ] Communication mechanisms: Effective low cost communication and sync (without taxing power source)

###Implementation
+ [ ] Development of full hardware solution (can you detect VoCs > ~0.5 PPM)
+ [ ] Develop a [G]UI to [re]program and sync data to/from remote sensing board 
+ [ ] Development of power system
+ [ ] Development of communication system
+ [ ] Develop the Android/iOS/PC software for receiving sync data, database/cloud upload, visualization
