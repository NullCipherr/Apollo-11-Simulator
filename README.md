# # Apollo-11-Simulator

A C-based simulation of the Apollo 11 mission control systems, featuring real-time monitoring and control of flight systems, propulsion, and power management.

## Overview

This simulator recreates the core systems of the Apollo 11 mission, including:

- Flight control module
- Propulsion control module
- Power management system
- Life support systems
- Mission state management
- Real-time physics simulation

## Features

- Multi-threaded system architecture
- Real-time physics calculations
- Mission state progression
- Power and fuel management
- Environmental control systems
- Emergency protocols
- Interactive user interface
- Adjustable simulation speed

## Requirements

- GCC compiler
- POSIX-compliant system (Linux/Unix/WSL)
- pthread library
- math library

## Building

Use the provided Makefile to build the project:

```bash
make
```

## Usage

To run the simulator:

```bash
make run
```

### Controls

- `A` - Accelerate simulation (2x, 4x, 8x...)
- `D` - Decelerate simulation
- `P` - Advance to next mission state
- `E` - Trigger emergency protocol
- `S` - Exit simulator

## Mission States

1. PREPARATION
2. LAUNCH
3. EARTH ORBIT
4. LUNAR TRANSIT
5. LUNAR ORBIT
6. LUNAR LANDING
7. LUNAR SURFACE
8. EARTH RETURN
9. REENTRY
10. SPLASHDOWN
11. COMPLETION
12. EMERGENCY

## Technical Details

- Written in C
- Uses POSIX threads for parallel processing
- Real-time physics calculations
- Simulated systems:
  - Navigation
  - Propulsion
  - Power management
  - Life support
  - Environmental controls
  - Emergency protocols

## Building from Source

1. Clone the repository:

```bash
git clone https://github.com/yourusername/Apollo-11-Simulator.git
```

2. Navigate to the project directory:

```bash
cd Apollo-11-Simulator
```

3. Build the project:

```bash
make
```

## Clean Build

To clean build files:

```bash
make clean
```

## License

[Add your chosen license here]

## Author

Andrei Costa

## Contributing

Feel free to submit issues and pull requests.
Apollo-11-Simulator
