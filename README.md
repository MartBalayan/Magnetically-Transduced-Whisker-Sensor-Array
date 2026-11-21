# Magnetically Transduced Whisker Sensor Array

### Bio-Inspired Tactile Sensing | Embedded Systems | PCB Design | MuJoCo Simulation

A multidisciplinary engineering project involving the design and development of a bio-inspired whisker sensing system. The system uses magnetic transduction and Hall-effect sensors to detect mechanical interactions through changes in the relative position of magnets and sensors.

The project combines mechanical sensing, custom electronics, STM32 embedded firmware and MuJoCo simulation, resulting in both a functional hardware prototype and a simulation environment.

---

## 1. Project Overview

Whiskers are highly effective biological sensing structures that allow animals to perceive their surroundings through physical contact and mechanical feedback.

Inspired by this biological mechanism, this project investigates the development of an artificial whisker sensor array capable of detecting mechanical interactions using magnetic sensing technology.

The sensing principle relies on the relative displacement between a magnet and a Hall-effect sensor. When a whisker experiences mechanical deflection, the relative position of the magnet changes, producing a variation in the magnetic field measured by the sensor.

These variations are acquired and processed by an STM32 microcontroller, providing the electronic foundation for a tactile sensing system.

The project covers the development of both the physical sensing hardware and a MuJoCo simulation model for investigating whisker movement and mechanical interactions.

## 2. System Architecture

The system consists of three main engineering components:

### Magnetic Whisker Sensing

The sensing mechanism uses magnets and Hall-effect sensors to translate mechanical whisker displacement into measurable electrical signals.

The relative movement between the magnetic elements and sensors produces changes in the measured magnetic field, allowing mechanical interactions to be detected electronically.

### Embedded Processing — STM32

An STM32 microcontroller serves as the embedded processing platform.

The firmware provides the interface between the magnetic sensing hardware and the digital processing system.

### Mechanical Simulation — MuJoCo

A simulation environment developed using MuJoCo models whisker movement and mechanical interactions.

This provides a virtual environment for investigating the mechanical behavior of the whisker sensing structure.

## 3. Hardware Development

The physical prototype integrates the sensing elements, electronic circuitry and embedded processing hardware.

**Main hardware elements:**

- STM32 microcontroller
- Hall-effect magnetic sensors
- Permanent magnets
- Whisker sensing structures
- Custom-designed PCB
- Electronic interfacing and signal acquisition circuitry

The PCB design provides the electronic foundation for integrating the magnetic sensors with the embedded processing platform.

## 4. Embedded Firmware

The embedded software is developed for an STM32 microcontroller and is responsible for interfacing with the sensing electronics.

The firmware is organized within the STM32 project files included in this repository.

**Platform:** STM32

**Programming language:** Embedded C/C++

**Main focus:** Microcontroller-based sensor interfacing and data acquisition.

## 5. MuJoCo Simulation

The project includes a MuJoCo simulation environment developed to investigate the mechanical behavior of the whisker structures.

The simulation focuses on:

- Whisker movement and deflection.
- Mechanical interaction with surrounding objects.
- Behavior of the whisker structures under external interaction.
- Simulation-based investigation of the sensing mechanism.

The simulation complements the physical prototype by providing a virtual environment for studying the mechanical aspects of the sensing system.

## 6. Project Results

The development resulted in two complementary outcomes:

**Physical hardware prototype**
- Implementation of the magnetically transduced whisker sensing concept.
- Integration of Hall-effect sensors and magnetic elements.
- Custom PCB and STM32-based embedded electronics.

**Simulation environment**
- MuJoCo-based whisker model.
- Simulation of whisker movement.
- Investigation of mechanical interactions.

Together, these components demonstrate the implementation of a bio-inspired tactile sensing concept through both physical prototyping and simulation.

## 7. Repository Structure

```text
Magnetically-Transduced-Whisker-Sensor-Array/
│
├── Mujoco Simulation/
│   └── MuJoCo simulation files
│
├── PCB Design/
│   └── PCB design files
│
├── STM32 Core Files/
│   └── STM32 firmware and project files
│
├── A Magnetically Transdiced Whisker
│   Sensor Array - Masters Thesis.pdf
│
└── README.md
```

## 8. Technologies and Engineering Skills

| Category | Technology |
|---|---|
| Embedded Systems | STM32 |
| Programming | Embedded C/C++ |
| Magnetic Sensing | Hall-effect sensors |
| Electronics | PCB Design |
| Simulation | MuJoCo |
| Mechanical Systems | Whisker mechanics and deflection |
| System Development | Hardware-software integration |

## 9. Research Documentation

This project was developed as part of a Master's thesis.

The complete thesis contains the research background, engineering methodology, system development and detailed technical documentation.

[**Read the Master's Thesis (PDF)**](./A%20Magnetically%20Transdiced%20Whisker%20Sensor%20Array%20-%20Masters%20Thesis.pdf)

## 10. Project Information

| Attribute | Description |
|---|---|
| Project | Magnetically Transduced Whisker Sensor Array |
| Project Type | Master's Thesis / Engineering Research |
| Application Area | Bio-Inspired Tactile Sensing |
| Embedded Platform | STM32 |
| Magnetic Sensing | Hall-effect sensors |
| Simulation Environment | MuJoCo |
| Outcome | Working hardware prototype and simulation |

---

**Explore the implementation:** Browse the PCB design files, STM32 firmware and MuJoCo simulation included in this repository.
