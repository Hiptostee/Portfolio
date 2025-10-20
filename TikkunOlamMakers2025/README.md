# Motor-Controlled Assistive Phone Stand (ONGOING) 📱♿

This project implements a precise, joystick-controlled positioning system for an assistive phone mount, allowing a user in a wheelchair to independently position their mobile phone. It is a custom piece of assistive technology designed to help the user better access their phone for activities like streaming, gaming, and podcasting.

The system currently uses an **open-loop control system** with custom software limits and a user-defined "zero" position to ensure safety and repeatability. However, we will be transitioning to a **closed-loop control system** using a through bore encoder.

---

## 🧠 Project Goal and Design Philosophy

The primary objective, derived from direct communication with the end-user ("need-knower"), was to replace a previous, heavy pulley system with an electrical system to achieve a reliable range of motion. The design goal was to position the phone exactly where the user wants it, prioritizing long battery life, safety features, and reliability. The current design as well as feasibility calculations (torque physics, max current for motor) can be found in the [Plans Folder](https://github.com/Hiptostee/Portfolio/blob/main/TikkunOlamMakers2025/plans/plans_and_calculations.pdf).

![Original Design (previous year)](/images/originalphonestand.jpeg)

---

## 💻 Key Software Features (C++ / Arduino)

The custom firmware provides two distinct operational modes, toggled by the joystick's push-button:

### **RUN MODE**

- The motor's movement is software-restricted between a defined lower and upper boundary (0 and 15,000 steps).
- This ensures the phone mount stays within the user’s **safe range of motion**.

### **ADJUST MODE (Calibration)**

- Allows free, unrestricted movement of the motor to manually position the mount to a starting point.
- This mode is indicated by a **periodic buzzer tone**.
- Exiting this mode sets the current physical position as the new logical zero (`_currentPosition = 0`).

### **Control Mapping**

- Maps the joystick’s **X-axis** position to the step pulse delay, providing **variable speed** and **smooth acceleration/deceleration**.

---

## ⚙️ Hardware Overview

| Component           | Role in the System               | Key Specification / Setting                                                               |
| ------------------- | -------------------------------- | ----------------------------------------------------------------------------------------- |
| **Microcontroller** | Arduino Nano                     | Runs the control logic and pulse generation.                                              |
| **Motor**           | NEMA 23 Stepper Motor            | Provides high torque for positioning the load.                                            |
| **Gearbox**         | Planetary Gearbox                | 50:1 ratio; Max Permissible Torque: 20 Nm.                                                |
| **Driver**          | TB6600 Stepper Motor Driver      | Converts Arduino signals to high-current pulses. Current Limit: 0.5 A to protect gearbox. |
| **Input**           | Joystick Module                  | A0 for X-axis movement; Pin 7 for mode toggling.                                          |
| **Power**           | LiFePO4 Battery / Buck Converter | Rechargeable power source with integrated voltage regulation.                             |
| **Structure**       | 3D Printed Adapters              | Custom parts for mounting the system to the wheelchair.                                   |

---

## 🧩 Technical Rigor & Safety

### **Gearbox Safety (Electrical vs. Mechanical)**

The NEMA 23 motor has a rated current of **2.9 A**, which, when multiplied by the **50:1 gear ratio**, gives a theoretical max torque of approximately **94.5 Nm**.  
Since the gearbox is rated for a **Max Permissible Torque of 20 Nm**, the current limit on the TB6600 driver was strictly set to **0.5 A**.  
This electronically limits the motor’s output torque to approximately **14.65 Nm**, ensuring the **mechanical integrity and long-term reliability** of the system.

---

## 🚀 Future Plans

Our next step is to integrate a **through-bore encoder** to enable **closed-loop feedback control**, providing greater precision and stability during operation.  
We will also be **mounting the full system onto the need-knower's wheelchair** to finalize its usability in his daily setup.  
A **meeting with the need-knower is planned in the coming weeks** to test and refine the new design, ensuring it fully meets his accessibility needs.

---

## 👥 Team

**Joe Marra**, **Jamie Pittman**, **Kristen Donahue**, **Eli Zawatsky**, **Saachi Bhatia**, **Tony Fang**, **Katie Wang**
