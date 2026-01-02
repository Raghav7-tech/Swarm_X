# 🛸 SWARM_X: Autonomous Airspace Management Engine

![Status](https://img.shields.io/badge/Status-Stealth_Prototype-red?style=for-the-badge)
![Tech](https://img.shields.io/badge/Engine-Python_3.10_%7C_PyGame-green?style=for-the-badge)
![Physics](https://img.shields.io/badge/Core-Vector_Physics_%7C_NumPy-blue?style=for-the-badge)

> **"The Roads of the Future are in the Sky."**

## 📜 Executive Summary
**Swarm_X** is a high-fidelity **Unmanned Traffic Management (UTM)** simulation engine designed to solve the "Last Mile" logistics problem for autonomous drone swarms. 

Unlike traditional centralized controllers, Swarm_X utilizes **Decentralized Swarm Intelligence (Boids Algorithm)** and **Vector-Based Physics** to manage hundreds of agents in a shared 3D airspace without collisions. The system features a tactical "Dark Mode" radar interface and supports **Hardware-in-the-Loop (HITL)** integration with ESP32 telemetry units.

---

## 🧠 Core Architecture
The system is built on a modular "Engine-First" philosophy:

1.  **Physics Kernel (`core/physics.py`):**
    * Calculates Inertia, Drag, and Thrust vectors for every agent.
    * Implements **Euler Integration** for smooth, organic movement (Drift Model).
    
2.  **Swarm Logic (`core/boids.py`):**
    * **Separation:** Agents actively repel neighbors to prevent mid-air crashes.
    * **Alignment:** Squads fly in formation automatically.
    * **Cohesion:** Drones stay within signal range of the "Hive."

3.  **Tactical HUD (`vis/radar.py`):**
    * 60 FPS Rendering using PyGame.
    * Real-time telemetry display (Speed, Heading, ID).
    * Dynamic "Green Corridor" visualization for Emergency Medical Drones.

---

## 🕹️ Controls & Usage
* **Left Click:** Set Global Target (The Swarm will converge).
* **Right Click:** Deploy Reinforcements (Spawn new autonomous drone).
* **Spacebar:** Trigger "Emergency Stop" Protocol (Coming Soon).

---

## 🛠️ Installation (Dev Mode)

```bash
# 1. Clone the secure repository
git clone [https://github.com/YOUR_USERNAME/Swarm_X.git](https://github.com/YOUR_USERNAME/Swarm_X.git)

# 2. Initialize the environment
cd Swarm_X
python -m venv venv
source venv/bin/activate  # (Windows: venv\Scripts\activate)

# 3. Install High-Performance dependencies
pip install -r requirements.txt

# 4. Launch the Radar
python main.py