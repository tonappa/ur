# ur — Workspace ROS 2 per cella UR5e "ur_automata"

Workspace ROS 2 **Jazzy** per guidare un Universal Robots UR5e (la cella
`ur_automata`) con **MoveIt 2**, sia in simulazione (URSim / Gazebo) sia sul
robot reale. Lo sviluppo avviene dentro un container Docker: la root del repo
viene montata nel container come workspace colcon e compilata lì dentro.

L'interfaccia ROS esposta (topic `/joint_states`, action
`/scaled_joint_trajectory_controller/follow_joint_trajectory`, ecc.) è
**identica** tra simulazione e robot reale. Cambia solo *quale launch file*
avvii e l'IP del controller: il resto (controller, MoveIt, nodi applicativi)
resta invariato.

> Per il *perché* dietro l'architettura, i launch file e i controller, vedi il
> tutorial lungo `ur.md` (in italiano).

---

## Indice

1. [Requisiti host](#1-requisiti-host)
2. [Clone del repository](#2-clone-del-repository)
3. [Build dell'immagine Docker](#3-build-dellimmagine-docker)
4. [Avvio del container e build del workspace](#4-avvio-del-container-e-build-del-workspace)
5. [Struttura dei pacchetti](#5-struttura-dei-pacchetti)
6. [Configurazione centrale (`automata_config.yaml`)](#6-configurazione-centrale-automata_configyaml)
7. [Avvio in simulazione](#7-avvio-in-simulazione)
8. [Avvio sul robot reale](#8-avvio-sul-robot-reale)
9. [Eseguire una scansione](#9-eseguire-una-scansione)
10. [Riferimenti](#10-riferimenti)

---

## 1. Requisiti host

- **Linux** (testato su Ubuntu) con kernel recente.
- **Docker** + **Docker Compose v2** (`docker compose`, non `docker-compose`).
- **NVIDIA Container Toolkit** (`nvidia-container-toolkit`) per il passthrough
  GPU (RViz/Gazebo). Senza GPU NVIDIA va rimossa la sezione `deploy` da
  `docker-compose.yaml`.
- Utente host nei gruppi `audio` e `video` (lo script `run.sh` legge i loro GID).
- `xhost` / X11 per il forwarding delle GUI (RViz, Gazebo).

> **Nota kernel ≥ 6.15:** URSim fallisce all'avvio (`URControl ENOSYS on
> socket()`). Usa lo script `start_ursim_seccomp.sh` incluso (vedi
> [§7](#7-avvio-in-simulazione)), non lo `start_ursim.sh` ufficiale.

---

## 2. Clone del repository

Il workspace usa **git submodule** per i pacchetti upstream di Universal Robots
(pinnati al branch `jazzy`). Clona con `--recursive`:

```bash
git clone --recursive <URL_DEL_REPO> ur
cd ur
```

Se hai già clonato senza `--recursive`, inizializza i submodule a mano:

```bash
git submodule update --init --recursive
```

### Submodule inclusi (`src/utils/`)

| Submodule | Repo upstream | Branch |
|---|---|---|
| `Universal_Robots_ROS2_Description` | [UniversalRobots/Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) | `jazzy` |
| `Universal_Robots_ROS2_Driver` | [UniversalRobots/Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) | `jazzy` |
| `src/ur_description` | [UniversalRobots/Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) | `jazzy` |

> I submodule vanno trattati come **codice vendor read-only**: servono come
> riferimento / per eventuali patch. L'immagine Docker installa già via apt le
> versioni binarie (`ros-jazzy-ur`, `ros-jazzy-ur-simulation-gz`, ecc.), quindi
> i submodule **non sono strettamente necessari** per compilare il workspace.

---

## 3. Build dell'immagine Docker

Tutto è guidato da `run.sh`, che ricava automaticamente i nomi
`IMAGE_NAME`/`CONTAINER_NAME` dal nome della cartella corrente ed esporta
UID/GID host, GID audio/video, variabili X11/Pulse e i mount di
bash-history/VSCode prima di invocare `docker compose`.

```bash
./run.sh build      # build con cache (veloce)
./run.sh rebuild    # build --no-cache (da zero, lento)
```

L'immagine (`ros:jazzy` come base) installa tra l'altro:
`ros-jazzy-ur`, `ros-jazzy-ur-simulation-gz`, `ros-jazzy-moveit`,
`ros-jazzy-ros-gz`, `ros-jazzy-rviz2`, `ros-jazzy-plotjuggler-ros`,
`ros-jazzy-rqt-runtime-monitor`, `ros-jazzy-trac-ik-kinematics-plugin`, più i
pacchetti Python in `docker/requirements.txt` (`pyquaternion`, `paho-mqtt`).

Il container gira con `network_mode: host` (necessario per la discovery DDS di
ROS 2 e per il link TCP/IP verso il controller UR), `ROS_DOMAIN_ID=44`,
priorità real-time (`SYS_NICE`, `rtprio: 99`) e passthrough GPU NVIDIA.

---

## 4. Avvio del container e build del workspace

```bash
./run.sh run        # esegue `xhost +local:docker` poi `docker compose run`
```

Sei ora dentro il container, nella cartella di lavoro `~/ros/ur` (la root del
repo montata come volume). L'`entrypoint.sh` ha già sorgenti `setup.bash` di
ROS, lanciato `apt update/upgrade` e `rosdep install`.

**Compila il workspace** (questa è la riga esatta che l'entrypoint stampa se il
workspace non è ancora compilato):

```bash
colcon build --cmake-args -DCMAKE_CXX_FLAGS="-w" --executor sequential
```

Poi **ri-sorgi** l'overlay:

```bash
source install/setup.bash
```

Comandi utili:

```bash
# compila un solo pacchetto
colcon build --packages-select ur_automata_moveit_config

# compila un pacchetto e tutto ciò che ne dipende
colcon build --packages-up-to ur_automata_bringup
```

> **IMPORTANTE:** colcon e i launch ROS vanno eseguiti **solo dentro il
> container**, mai dall'host.
>
> I launch file leggono `automata_config.yaml` dal *package share installato*,
> non da `src/`. Dopo aver modificato il YAML, ricompila `ur_automata_bringup`
> (o tutto il workspace) e ri-sorgi `install/setup.bash`.

Per fermare/rimuovere il container:

```bash
./run.sh down
```

---

## 5. Struttura dei pacchetti

Tutti i pacchetti applicativi stanno in `src/automata_robot/`:

| Pacchetto | Ruolo |
|---|---|
| `ur_automata_description` | URDF/xacro della cella + mesh (TCP custom `ee_automata`) + config RViz. `urdf/ur_automata.urdf.xacro` parametrizza `ur_type` (default `ur5e`). `launch/display.launch.py` = solo visualizzazione (RSP + joint_state_publisher_gui + RViz). |
| `ur_automata_moveit_config` | Config MoveIt 2 (SRDF, kinematics, limiti giunti/cartesiani, `moveit_controllers.yaml`, `ros2_controllers.yaml`) generata col Setup Assistant. Target tipico di modifica. |
| `ur_automata_bringup` | Launch di alto livello: orchestra driver/control + MoveIt + scena. Contiene `config/automata_config.yaml` (vedi §6) e i controller. |
| `ur_automata_scene` | Pubblica gli oggetti della planning scene (piattaforma + pezzo da scansionare). Mesh in `meshes/` (`platform.stl`, `ceramic_model.obj`). |
| `ur_automata_scan` | Nodo C++ `scan_executor_node`: genera waypoint su una sfera attorno al pezzo, risolve IK e pianifica/esegue la scansione. |

`src/utils/` = submodule upstream (vedi §2).

---

## 6. Configurazione centrale (`automata_config.yaml`)

Quasi tutto si controlla da un solo file:
`src/automata_robot/ur_automata_bringup/config/automata_config.yaml`.

Parametri chiave:

```yaml
robot:
  ip: 192.168.1.97     # IP del controller. Reale: IP del UR. URSim: 192.168.56.101
  type: ur5            # ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e

planning:
  group: ur_manipulator
  global_frame: world
  end_effector_link: ee_automata_tcp   # deve coincidere col SRDF
  use_moveit: true
  use_scene:  true     # spawn della planning scene
  use_rviz:   true

scan:
  center: [0.133, 0.40, 0.504]   # centro del pezzo, in metri, nel global_frame
  platform_sim: false            # true=disco+gambe simulati | false=mesh STL platform.stl
  radius: 0.30                   # raggio della sfera di waypoint
  hemisphere: full               # upper | lower | full
  # ... molti altri parametri di scan, IK, occlusione, fallback e planner OMPL
```

Il file è **ampiamente commentato in italiano**: leggilo per i dettagli su
emisferi, esclusione equatoriale, ricerca del pitch, occlusion check, fallback
e scelta del planner (OMPL / Pilz).

> Dopo ogni modifica: ricompila `ur_automata_bringup` e ri-sorgi l'overlay
> (vedi §4).

---

## 7. Avvio in simulazione

Due strade: **URSim** (controller UR virtuale, identico al reale) oppure
**Gazebo** (simulazione fisica). URSim è quella allineata a `automata_config`.

### Opzione A — URSim (consigliata, stessa pipeline del robot reale)

URSim emula il controller PolyScope: il driver `ur_robot_driver` ci si collega
esattamente come al robot vero.

**1) Sull'host**, avvia URSim (gira in un suo container Docker su rete
`192.168.56.0/24`, IP `192.168.56.101`):

```bash
./start_ursim_seccomp.sh
```

Lo script scarica la URCap *External Control*, monta uno storage persistente e
avvia URSim con `seccomp=unconfined` (workaround kernel ≥ 6.15). Interfacce:
- VNC: `localhost:5900`
- Web (noVNC): <http://localhost:6080>

Sul teach pendant virtuale carica/avvia il programma con **External Control** e
premi **Play**.

**2) In `automata_config.yaml`** imposta `robot.ip: 192.168.56.101` (e
`robot.type` coerente con `ROBOT_MODEL` dello script, di default UR5). Ricompila
`ur_automata_bringup`.

**3) Dentro il container**, avvia driver + MoveIt + scena + RViz:

```bash
ros2 launch ur_automata_bringup ur_automata_bringup.launch.py
```

### Opzione B — Gazebo (GZ Sim)

Usa i launch upstream di `ur_simulation_gz` (l'hardware interface è il plugin
Gazebo invece del driver reale):

```bash
# solo Gazebo + controller + RViz
ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e

# Gazebo + MoveIt
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e
```

> In Gazebo il controller di traiettoria si chiama `joint_trajectory_controller`
> (sul reale `scaled_joint_trajectory_controller`) e va usato
> `use_sim_time:=true` per il move_group. Dettagli e test di pipeline in `ur.md`.

### Test rapido senza robot (mock hardware)

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e robot_ip:=0.0.0.0 use_mock_hardware:=true
```

---

## 8. Avvio sul robot reale

L'unica differenza rispetto a URSim è l'IP del controller e una calibrazione una
tantum. Tutto il resto (launch, controller, MoveIt, nodi) è identico.

### 8.1 Calibrazione (una tantum per robot)

Esegui `ur_calibration` contro il controller fisico e salva lo YAML risultante:

```bash
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=<IP_DEL_ROBOT> \
  target_filename:="${HOME}/my_robot_calibration.yaml"
```

Questo YAML va passato al driver tramite `kinematics_params_file` (default in
`ur_automata_control.launch.py` = cinematica nominale, OK solo per URSim).

### 8.2 Configurazione

In `automata_config.yaml` imposta:
- `robot.ip` = IP reale del controller UR
- `robot.type` = modello reale (es. `ur5e`)

Ricompila `ur_automata_bringup` e ri-sorgi l'overlay.

### 8.3 Bring-up

```bash
ros2 launch ur_automata_bringup ur_automata_bringup.launch.py \
  kinematics_params_file:=${HOME}/my_robot_calibration.yaml
```

> Per passare la calibrazione di default (senza scrivere il flag ogni volta)
> aggiorna il `default_value` di `kinematics_params_file` in
> `ur_automata_control.launch.py`.

### 8.4 External Control sul teach pendant

Sul pendant deve girare il programma con la URCap **External Control** che punta
all'IP del PC ROS.

> **IMPORTANTE:** ogni volta che riavvii il bring-up sul robot reale devi
> ri-premere **Play** sul teach pendant, altrimenti il driver resta in attesa
> della connessione e i comandi non vengono eseguiti.

---

## 9. Eseguire una scansione

Con il bring-up attivo (sim o reale), in un **secondo terminale dentro il
container** (ricorda `source install/setup.bash`):

```bash
# (opzionale) spawn della sola scena, se non avviata dal bringup
ros2 launch ur_automata_scene scene.launch.py

# genera i waypoint sulla sfera, risolve IK ed esegue la scansione
ros2 launch ur_automata_scan scan.launch.py
```

Il nodo `scan_executor_node` carica i parametri dalla sezione `scan` di
`automata_config.yaml` e la `kinematics.yaml` di `ur_automata_moveit_config`
(necessaria perché l'IK lato client funzioni — senza, `setFromIK` fallisce in
silenzio). Regola emisfero, raggio, numero di anelli/punti, occlusion check,
fallback e planner direttamente nel YAML.

---

## 10. Riferimenti

- **`ur.md`** — tutorial lungo (IT): architettura, bring-up, scrittura di un
  nodo C++, da zero al robot reale.
- **`CLAUDE.md`** — note operative sul workspace.
- Documentazione ufficiale UR ROS 2:
  <https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation>
- UR ROS 2 Driver: <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver>
- UR ROS 2 Description: <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description>
- UR Simulation (Gazebo): <https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation>
- External Control URCap: <https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap>
- MoveIt 2: <https://moveit.ai>
