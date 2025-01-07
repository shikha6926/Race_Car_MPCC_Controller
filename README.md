# A Modified MPCC Controller with Splines for Autonomous Racing Application

This repository implements a **Modified Model Predictive Contouring Control (MPCC)** framework with spline-based track modeling for autonomous racing applications. The project demonstrates advanced control strategies and spline generation techniques, tailored for high-speed autonomous vehicles.

---

## Directory Structure

```plaintext
Python_Spline_Generation/
├── .idea/                           # IDE-specific configuration files
├── FREIBURG_FULL_TRACK.yaml         # YAML representation of the Freiburg full track
├── custom_length_track_spline.png   # Visualization of a custom-length track spline
├── fulltrack_spline.png             # Visualization of the full track spline
├── fulltrack_spline.yaml            # YAML file for the full track spline
├── track_splines.py                 # Script for spline generation
├── src/
    ├── crs/
        ├── commons/                 # Common utilities and helper functions
        ├── controls/
            ├── common/              # Common control logic
            ├── mpc_controller/      # Core MPCC implementation
            ├── mpc_solvers/         # Solvers for MPC optimization


## Features

- **Spline-based Track Representation**  
  Smooth spline curves are used to model race tracks, ensuring efficient and precise navigation for autonomous vehicles.

- **Modified MPCC Algorithm**  
  An enhanced Model Predictive Contouring Control (MPCC) framework is implemented for better trajectory tracking and time-optimized racing.

- **Visualization Tools**  
  Scripts are provided to generate and visualize spline-based tracks, aiding in analysis and debugging.

- **Flexible and Modular Design**  
  The codebase is structured in a modular way, allowing easy customization and expansion for future research and development.

- **Track Data Representation**  
  YAML files are used to store track information, enabling reusable and scalable input data handling.

- **Simulation and Analysis Outputs**  
  Provides detailed logs, trajectory visualizations, and performance metrics for simulation analysis.

## Getting Started

### Prerequisites

To run this project, ensure you have the following:

- **Python**: Version 3.8 or higher
- **Required Python Libraries**: 
  - `numpy`
  - `matplotlib`
  - `scipy`
  - `pyyaml`

Install the required Python packages using:

```bash
pip install -r requirements.txt

## Project Report and Presentation

This repository includes:

- **[Project Report](report.pdf)**: A detailed explanation of the methodology, implementation, and results for the project, titled *A Modified MPCC Controller with Splines for Autonomous Racing Application*.
- **[Presentation Slides](presentation.pdf)**: A concise overview of the project, highlighting key techniques, results, and conclusions.

---

## Contributors

- **Ankita Pawar** 
- **Shikha Tiwari** 

---

## Acknowledgments

- **Freiburg Racing Dataset**: Provided essential track data for spline generation and evaluation.
- **Open-source Libraries**: Libraries like `numpy`, `matplotlib`, and `scipy` supported the development of this project.

---

## Code Sharing Notice

Since this project involves internal code developed as part of university research, **not all code could be shared publicly**. This repository contains only the portions of the code that were implemented and can be shared externally by our team. Some proprietary components remain internal to the university and are not included here.
