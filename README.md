# 🛸 SWARM_X: Autonomous Airspace Management Engine

> *"The Roads of the Future are in the Sky."*

![Python](https://img.shields.io/badge/Python-3.10%2B-blue?style=flat&logo=python)
![Pygame](https://img.shields.io/badge/Engine-Pygame-green?style=flat&logo=pygame)
![Status](https://img.shields.io/badge/Status-Prototype-orange?style=flat)

## 📜 Executive Summary
**Swarm_X** is a high-fidelity **Unmanned Traffic Management (UTM)** simulation engine designed to solve the "Last Mile" logistics problem for autonomous drone swarms.

Unlike traditional centralized controllers, Swarm_X utilizes **Decentralized Swarm Intelligence (Boids Algorithm)**, **A* Pathfinding**, and **Vector-Based Physics** to manage mixed-priority agents in a shared 3D airspace without collisions. The system features a tactical "Dark Mode" radar interface and creates a complex urban environment to stress-test autonomous decision-making.

---

## 🧠 Core Architecture
The system is built on a modular "Engine-First" philosophy, prioritizing mathematical accuracy over game mechanics.

### 1. Physics Kernel (`core/physics.py`)
* **Vector Dynamics:** Calculates Inertia, Drag, and Thrust vectors for every agent independently.
* **Drift Model:** Implements Euler Integration to simulate organic drift and momentum; drones do not stop instantly, they decelerate.

### 2. Swarm Logic (`core/boids.py`)
* **Separation:** Agents actively repel neighbors to maintain safe flight distance.
* **Alignment:** Squads synchronize velocity vectors to fly in formation.
* **Cohesion:** Drones tend to stay within signal range of their local cluster.

### 3. Priority & Routing (`core/pathfinding.py`)
The engine implements a dynamic **Impact Hierarchy** to resolve right-of-way conflicts:
* 🔴 **Medical Drones:** Highest Priority. They trigger a "Green Corridor," forcing other agents to yield.
* 🟡 **Military Drones:** High Velocity, aggressive pathing.
* ⚪ **Commercial Drones:** Fuel-efficient pathing, yields to all others.

---

## 🛠️ Built With

### Software Stack
* **Python 3.10+:** Core Simulation Logic.
* **Pygame:** Real-time rendering engine (60 FPS).
* **NumPy:** High-performance vector mathematics and matrix operations.

### Tools
* **Custom Map Editor:** Built-in tool to trace real-world building layouts and save them as collision maps.

---

## 🕹️ Controls & Usage

| Input | Action |
| :--- | :--- |
| **Left Click** | **Set Target:** The swarm will recalculate A* paths to converge on this location. |
| **Right Click** | **Deploy Agent:** Spawns a new autonomous drone into the airspace. |
| **Spacebar** | **Emergency Stop:** Freezes simulation state (Debug Mode). |
| **S Key** | **Save Map:** (In Editor Mode) Saves current obstacles to `city_map.pkl`. |

---

## 🚀 Installation

1.  **Clone the Repo:**
    ```bash
    git clone [https://github.com/YourUsername/Swarm_X.git](https://github.com/YourUsername/Swarm_X.git)
    cd Swarm_X
    ```

2.  **Install Dependencies:**
    ```bash
    pip install pygame numpy
    ```

3.  **Run the Engine:**
    ```bash
    python main.py
    ```

---

## 🔮 Future Roadmap (Upcoming V2.0)
We are currently developing a **Cyber-Physical System** update to bridge the gap between the simulation and the real world.

* **Hardware-in-the-Loop (HITL):** Integrating an **ESP32** microcontroller to receive real-time telemetry from the Python engine.
* **Physical Dashboard:** Using an **OLED Display** and **Status LEDs** to show mission status (Medical/Military/Commercial) physically on the user's desk.
* **Collision Alarm:** Active buzzer integration to sound alerts when a simulation crash occurs.
* **Energy Anxiety:** Implementing battery drain logic requiring drones to abandon missions for charging.

---

## 👥 Author
* **[Raghav Dhingra]** - *Lead Developer*

*"Code determines the behavior, but Physics determines the destiny."*

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
