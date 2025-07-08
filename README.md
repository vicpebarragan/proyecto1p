# Lap Follower Node – F1TENTH ROS 2 (Humble)

Este nodo implementa un **controlador reactivo basado en Follow the Gap** que permite al vehículo recorrer de forma autónoma una pista (como Brands Hatch), mientras **cuenta las vueltas completadas** y **registra el tiempo por vuelta**.

---

## 📌 Enfoque utilizado: Follow the Gap

El algoritmo Follow the Gap analiza los datos del sensor LiDAR para encontrar la región libre más amplia (gap) frente al vehículo y dirige la marcha hacia el centro de esa región.

Este nodo optimizado mejora el enfoque clásico mediante:
- Suavizado Gaussiano sobre los datos del LiDAR para evitar decisiones erráticas.
- Selección del **gap más largo continuo**, no solo el punto más lejano.
- Cálculo del **ángulo de dirección** a partir del centro del gap.
- Ajuste de la **velocidad en función del ángulo**: mayor velocidad en rectas, menor en curvas.

Además, se incluye una lógica basada en `/odom` que detecta el cruce de una línea de meta definida por el usuario, permitiendo contar las vueltas y medir el tiempo entre cada cruce.

---

## 🧠 Estructura del código

| Archivo                  | Descripción                                                                 |
|--------------------------|-----------------------------------------------------------------------------|
| `lap_follower_node.py`   | Nodo principal que controla el vehículo, cuenta vueltas y mide tiempos.    |
| `setup.py`               | Define el punto de entrada para ejecutar el nodo con `ros2 run`.           |
| `maps/BrandsHatch_map.*` | Imagen y configuración del mapa utilizado para la simulación.              |

**Temas utilizados:**
- `/scan` (input LiDAR - tipo `sensor_msgs/LaserScan`)
- `/ego_racecar/odom` (posición del auto - tipo `nav_msgs/Odometry`)
- `/drive` (comandos de movimiento - tipo `ackermann_msgs/AckermannDriveStamped`)

---

## 🚀 Instrucciones de ejecución

### 1. Preparar el entorno

```bash
cd ~/ros2_ws
colcon build --packages-select f1tenth_gym_ros
source install/setup.bash
```
### 2. Ejecutar el simulador

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
### 3. Ejecutar el nodo de control

```bash
ros2 run f1tenth_gym_ros lap_follower_node
```
