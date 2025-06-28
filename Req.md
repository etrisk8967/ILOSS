Integrated Launch and Orbit Simulation System (ILOSS)
Systems Requirements Specification (SRS)
Document Information

Version: 3.0

Date: June 25, 2025

Classification: Systems Requirements Specification (SRS)

System Name: Integrated Launch and Orbit Simulation System (ILOSS)

Table of Contents
System Overview

Functional Requirements

Performance Requirements

Interface Requirements

Design Constraints

Quality Attributes

Technical Architecture

Data Requirements

Verification and Validation

Future Extensibility

Appendices

1. System Overview
1.1 Purpose
The Integrated Launch and Orbit Simulation System (ILOSS) shall provide a high-fidelity, scientific-grade simulation environment for modeling rocket launches and subsequent orbital mechanics. The system shall serve as a digital twin for launch vehicle operations, accurately modeling all significant physical phenomena from the launch pad to orbital insertion and beyond.

1.2 Scope
The system shall encompass:

Complete launch trajectory simulation from any terrestrial location.

Seamless transition from atmospheric flight to orbital mechanics.

High-precision orbital propagation with a comprehensive and configurable suite of perturbation models.

Real-time 3D/2D visualization on a high-resolution, georeferenced globe.

Bidirectional mission planning (orbit-to-launch and launch-to-orbit).

1.3 System Context
ILOSS shall operate as a standalone desktop application on Windows and Linux platforms, providing professional-grade simulation capabilities suitable for an advanced engineering portfolio project.

1.4 Simulation Fidelity and Accuracy
1.4.1 Validation Standard: The system's propagators shall be validated against established, high-fidelity benchmarks like NASA's GMAT or AGI's STK.

1.4.2 Position Accuracy: The propagated state vector shall adhere to the following accuracy targets when compared to validation benchmarks under identical force model configurations:

Launch Phase: ±1 meter

Low Earth Orbit: ±10 meters over a 24-hour propagation period.

Geostationary Orbit: ±100 meters over a 24-hour propagation period.

1.4.3 Velocity Accuracy:

Launch Phase: ±0.1 m/s

Orbital Phase: ±0.01 m/s

1.5 Supported Mission Profiles
Launch Trajectories: From any location on Earth to any achievable direct-injection orbit.

Low Earth Orbit (LEO): 160-2,000 km altitude.

Medium Earth Orbit (MEO): 2,000-35,786 km altitude.

Geosynchronous Earth Orbit (GEO): 35,786 km altitude.

Highly Elliptical Orbits (HEO): Perigee in LEO, apogee beyond GEO.

Escape Trajectories: Hyperbolic orbits with e>1.

2. Functional Requirements
2.1 Launch Simulation Module (LSM)
LSM-REQ-001: Pre-configured Launch Sites: The system shall provide pre-configured launch sites including Cape Canaveral, Kennedy Space Center, Vandenberg SFB, Baikonur Cosmodrome, and Guiana Space Centre.

LSM-REQ-002: Custom Launch Sites: The system shall allow users to define custom launch locations with geodetic coordinates (latitude, longitude), terrain elevation (from loaded DEM data), launch azimuth constraints, and range safety boundaries.

LSM-REQ-003: 6-DOF Vehicle Dynamics: The system shall implement 6-DOF rigid body dynamics including translational motion, rotational motion, mass variation due to propellant consumption, and center of gravity shift.

LSM-REQ-004: Rocket Propulsion Modeling: The system shall model rocket propulsion with user-definable thrust curves (as a function of time and altitude), specific impulse (Isp) variation, throttle capability, thrust vector control, and engine startup/shutdown transients.

LSM-REQ-005: Aerodynamic Modeling: The system shall compute aerodynamic forces using imported coefficient databases (Cd, Cl, Cm) provided as functions of Mach number and angle of attack. The system must support a standard import format (e.g., CSV table lookup).

LSM-REQ-006: Standard Atmospheric Model: The system shall implement the US Standard Atmosphere 1976 model.

LSM-REQ-007: Dynamic Atmospheric Model: The system shall support the import of atmospheric data profiles (e.g., from GRIB2 files) to model wind, turbulence, and temperature/density deviations from standard.

LSM-REQ-008: Guidance and Control: The system shall implement a configurable launch guidance system including gravity turn logic, pitch program scheduling, and closed-loop guidance targeting for orbital insertion parameters.

2.2 Orbit Propagation Module (OPM)
OPM-REQ-001: State Transition: The system shall seamlessly transition from the launch to the orbit phase when altitude exceeds a user-configurable threshold (default: 100 km, Kármán line).

OPM-REQ-002: Gravitational Models:

a) Two-Body Dynamics: The system shall implement the fundamental two-body gravitational dynamics.

b) Earth Gravity Field: The system shall implement a spherical harmonic gravity model. The degree and order of the model must be configurable by the user up to a maximum of (70,70), with (8,8) as a default for general use. The model will use coefficients from a standard gravity model like EGM2008.

c) Third-Body Perturbations: The system shall model gravitational effects from the Sun, Moon, and major planets (e.g., Jupiter, Venus, Mars) using SPICE ephemerides.

OPM-REQ-003: Non-Gravitational Forces: All non-gravitational forces shall be individually selectable for inclusion in the simulation.

a) Atmospheric Drag: The system shall implement drag modeling using user-defined coefficients and cross-sectional areas. It shall offer a choice of atmospheric density models, including a simple exponential model and the NRLMSISE-00 model for high-fidelity applications.

b) Solar Radiation Pressure (SRP): The system shall model SRP with a user-definable reflectivity coefficient and include a conical shadow model with penumbra.

c) Earth Radiation Pressure: The system shall optionally include Earth albedo and infrared radiation pressure.

d) Relativistic Effects: Relativistic perturbation corrections (e.g., Schwarzschild terms) shall be included as a selectable, high-precision option disabled by default.

OPM-REQ-004: Orbital Maneuvers:

a) Impulsive Burns: The system shall model instantaneous velocity changes (Δv) in user-defined directions (Prograde, Radial, Normal, etc.).

b) Finite Burns: The system shall model continuous thrust maneuvers with mass depletion calculated via the Tsiolkovsky rocket equation.

c) Standard Maneuvers: The system shall provide tools to plan common maneuvers such as Hohmann transfers, bi-elliptic transfers, and plane changes.

OPM-REQ-005: Numerical Integration: The system shall provide a selection of numerical integrators, including a fixed-step Runge-Kutta 4th order (RK4) and an adaptive-step integrator like Runge-Kutta-Fehlberg 7(8) with user-configurable error tolerances.

OPM-REQ-006: Coordinate Systems: The system shall support transformations between ECI (J2000), ECEF (WGS84), Topocentric (ENU), and Orbital (RTN, LVLH) reference frames.

OPM-REQ-007: Time Systems: The system shall handle UTC, TAI, GPS Time, and TDB, including correct leap second application.

OPM-REQ-008: Orbital Element Conversions: The system shall compute and convert between Cartesian state vectors and Classical Keplerian elements.

2.3 Mission Planning Module (MPM)
MPM-REQ-001: Forward Planning (Launch-to-Orbit): Given launch and vehicle parameters, the system shall compute the resulting orbit.

MPM-REQ-002: Inverse Planning (Orbit-to-Launch): Given a target orbit, the system shall allow the user to calculate optimal launch windows and required azimuths from a selected launch site.

MPM-REQ-003: Launch Window Analysis: The system shall provide tools to analyze and visualize launch windows based on orbital plane alignment and user-defined constraints.

2.4 Visualization Module (VIS)
VIS-REQ-001: Geospatial Engine: The system shall render a 3D digital twin of the Earth using a high-performance, native C++ geospatial software development kit (SDK). The engine must be capable of draping high-resolution satellite imagery and other vector data (coastlines, borders) over a global Digital Elevation Model (DEM) based on the WGS84 ellipsoid.

Implementation Note: osgEarth is the recommended SDK for this purpose due to its maturity, performance, native C++ API, and widespread use in the aerospace and defense simulation industry. It provides the necessary "plug-and-play" architecture for integration with a custom physics engine.

VIS-REQ-002: 3D Scene Rendering: The system shall render in 3D the real-time rocket position and orientation (using a 3D model), its trajectory path (as a ribbon or line), and an accurately lit Earth model that reflects the current simulation time.

VIS-REQ-003: Camera Systems: The system shall provide multiple camera modes, including ground-based (fixed to launch site), chase (following the vehicle), onboard (first-person point-of-view), and a free-look camera with 6-DOF control.

VIS-REQ-004: 2D Plotting: The system shall generate 2D plots of critical mission data, including altitude vs. downrange distance, velocity components, acceleration (G-forces), and the time evolution of classical orbital elements.

2.5 Data Management Module (DMM)
DMM-REQ-001: Configuration Management: The system shall save and load entire mission scenarios, including vehicle configurations, launch sites, environmental models, and propagation settings.

DMM-REQ-002: Output Generation: The system shall export simulation time history results in CSV format and generate high-level mission reports in PDF.

2.6 Physics Engine Implementation Details
PHYS-REQ-001: Computational Core: The physics engine shall be implemented in modern C++ (C++20 or newer).

PHYS-REQ-002: Force Model Aggregation: Total acceleration shall be computed as a sum of all user-selected force models. The system must allow individual models to be enabled or disabled for performance tuning and analysis.

PHYS-REQ-003: State Vector: The primary state vector shall consist of position, velocity, and mass.

PHYS-REQ-004: Event Detection: The system shall detect and log key orbital events, including apogee/perigee passages, node crossings, and eclipse entry/exit.

3. Performance Requirements
PERF-REQ-001: Real-Time Performance: The system shall maintain real-time simulation (1:1 time ratio) with a 0.1-second timestep on reference hardware (CPU: Intel i7-10700K or equivalent, RAM: 16 GB DDR4).

PERF-REQ-002: Time Acceleration: The system shall support time acceleration up to 1000x for orbital phase simulation.

PERF-REQ-003: Launch Phase Computation: Launch phase computation shall not exceed 50 milliseconds per timestep.

PERF-REQ-004: Capacity: The system shall support simulation durations up to 365 days and storage for 10 million trajectory points.

4. Interface Requirements
4.1 User Interface
UI-REQ-001: The system shall provide a Qt6-based graphical interface with a ribbon-style menu system and dockable panels.

4.2 Data Interfaces
DI-REQ-001: Data Import: The system shall import vehicle configurations (JSON/XML), TLE sets, weather profiles (GRIB2), and terrain data (GeoTIFF).

DI-REQ-002: Data Export: The system shall export trajectory data (CSV), visualization snapshots (PNG, JPEG), and mission reports (PDF).

DI-REQ-003: Data Acquisition: The system shall provide documentation guiding the user on where to acquire and how to install the required large-scale geospatial datasets (DEM, imagery).

5. Design Constraints
5.1 Platform Constraints
DC-REQ-001: The system shall operate on:

Windows 10/11 (64-bit)

Ubuntu 20.04 LTS or newer

5.2 Technology Constraints
DC-REQ-002: The system shall be implemented using:

Core Logic: C++20 standard

User Interface: Qt6 framework (6.5 LTS or newer)

3D Visualization: osgEarth SDK

Build System: CMake 3.20+

5.3 Regulatory Constraints
DC-REQ-003: The system must not contain any restricted cryptographic functions or export-controlled (e.g., ITAR) data.

DC-REQ-004: Licensing: All third-party libraries must have licenses compatible with a public, open-source portfolio project (e.g., MIT, BSD, LGPL, Apache 2.0).

6. Quality Attributes
QA-REQ-001: Reliability: Mean Time Between Failures (MTBF) > 1000 hours.

QA-REQ-002: Maintainability: Modular architecture with defined interfaces and comprehensive logging.

QA-REQ-003: Usability: Intuitive workflow for common tasks with context-sensitive documentation.

QA-REQ-004: Testability: Unit test coverage > 80%.

7. Technical Architecture
7.1 System Architecture
┌─────────────────────────────────────────────────────────────┐
│                    User Interface Layer (Qt6)                 │
├─────────────────────────────────────────────────────────────┤
│                    Application Logic Layer                    │
├──────────────┬────────────┬────────────┬───────────────────┤
│Launch Module │Orbit Module│Mission Plan│Visualization (osgE) │
├──────────────┴────────────┴────────────┴───────────────────┤
│                    Core Services Layer                        │
├──────────────┬────────────┬────────────┬───────────────────┤
│Math Libraries│GIS Services│Data Manager│Config Manager      │
├──────────────┴────────────┴────────────┴───────────────────┤
│                    Platform Abstraction Layer                 │
└─────────────────────────────────────────────────────────────┘

7.2 Key Components
7.2.1 Launch Physics Engine: 6-DOF solver, aerodynamic calculator, propulsion model, atmospheric interface.

7.2.2 Orbit Propagator: Integrators, force model aggregator, coordinate transformation utilities.

7.2.3 Visualization Engine: osgEarth integration layer, camera management, trajectory rendering pipeline.

7.2.4 Data Management: SQLite database for mission data, configuration parser, export generators.

7.3 External Dependencies
Eigen3: Linear algebra (MPL2 License)

Boost: Utility libraries (Boost License)

GDAL: Geospatial data abstraction (MIT-style License)

Qt6: GUI framework (LGPL/Commercial License)

osgEarth: Core 3D visualization engine (OSGPL License - LGPL Style)

SQLite: Embedded database (Public Domain)

SPICE: Ephemerides and time conversions (MIT-style License)

GeographicLib: Geodetic calculations (MIT License)

8. Data Requirements
8.1 Static Data: NASA SRTM 30m terrain data, Sentinel-2 imagery, Natural Earth vectors, US Standard Atmosphere tables.

8.2 Dynamic Data: State vectors, environmental conditions, event logs.

8.3 Data Formats: Inputs (JSON, XML, CSV, GRIB2, GeoTIFF), Outputs (CSV, KML, GPX).

9. Verification and Validation
9.1 Verification: Unit Testing (Google Test), Integration Testing, System Testing.

9.2 Validation: Comparison with GMAT/STK results, analytical test cases, conservation law verification.

9.3 Test Cases: ISS insertion, GTO insertion, polar/retrograde orbits, failure modes.

Benchmark Case: A Falcon 9 launch to a 51.6-degree inclined LEO orbit shall be compared against a publicly available mission trajectory.

10. Future Extensibility
(Features explicitly out of scope for the current version but planned for the future)

Multi-stage vehicle dynamics (separation, interstage physics)

Monte Carlo analysis and trajectory optimization

Rendezvous and re-entry simulation

REST API for automation and Python bindings for scripting

11. Appendices
Appendix A: Reference Vehicle Parameters (Falcon 9)
Parameter

Value

Units

Gross Liftoff Mass

549,054

kg

Payload Capacity (LEO)

22,800

kg

Number of Engines

9 × Merlin 1D



Sea Level Thrust

7,607

kN

Vacuum Thrust

8,227

kN

Vacuum ISP

311

s

Length

70

m

Diameter

3.7

m

Appendix B: Coordinate Systems
ECI (Earth-Centered Inertial): J2000 epoch

ECEF (Earth-Centered Earth-Fixed): WGS84

LVLH (Local Vertical Local Horizontal): Orbit-relative

Appendix C: Physical Constants and Models
Parameter

Value

Units

Gravitational Constant (G)

6.67430×10⁻¹¹

m³ kg⁻¹ s⁻²

Earth Mass (M⊕)

5.972168×10²⁴

kg

Earth Equatorial Radius (R⊕)

6,378,137.0

m

GM (μ)

3.986004418×10¹⁴

m³/s²

J₂ Coefficient

1.082626683×10⁻³



Appendix D: User-Defined Parameters
Parameter

Range

Default

Units

Description

Mass

500 - 3,000,000

1,000

kg

Total satellite mass

Drag Coeff (Cd)

2.0 - 2.5

2.2

-

Aerodynamic drag coefficient

Area

0.1 - 1000

10

m²

Effective area for drag

Altitude

160 - 35,786

500

km

Initial orbital altitude

Inclination

0 - 180


51.6

deg

Orbital inclination

Launch Azimuth

0 - 360

90

deg

Launch direction from north

Appendix E: Mathematical Formulations
Kepler's Equation: For elliptical orbits, M=E−e⋅sin(E)

Orbital Velocity: For an elliptical orbit, v= 
μ( 
r
2
​
 − 
a
1
​
 )
​
 

Plane Change Δv: Δv=2v⋅sin( 
2
Δi
​
 )

Document Control
Revision History
| Version | Date | Author | Description |
| :--- | :--- | :--- | :--- |
| 1.0 | [Date] | [Author] | Initial Release |
| 2.0 | [Date] | [Author] | Added detailed physics engine specifications |
| 3.0 | 2025-06-25 | [Author] | Finalized requirements, selected osgEarth |

Approval Signatures

System Engineer: _________________

Project Manager: _________________

Glossary of Terms

6-DOF: Six Degrees of Freedom

ECEF: Earth-Centered Earth-Fixed

ECI: Earth-Centered Inertial

GEO: Geostationary Earth Orbit

GIS: Geographic Information System

GMAT: General Mission Analysis Tool

ISP: Specific Impulse

LEO: Low Earth Orbit

osgEarth: OpenSceneGraph Earth

SDK: Software Development Kit

STK: Systems Tool Kit