# ur — workspace ROS 2 per la cella `ur_automata`

Workspace ROS 2 **Jazzy** per pilotare un braccio Universal Robots (cella
`ur_automata`) con **MoveIt 2**, in simulazione **URSim** o sul **robot reale**.
Tutto lo sviluppo avviene dentro un container Docker: la root del repo viene
montata nel container e compilata lì con `colcon`.

L'interfaccia ROS è la stessa nei due casi — stesso `/joint_states`, stessa
action `/scaled_joint_trajectory_controller/follow_joint_trajectory`, stessi
controller, stesso MoveIt. Cambia solo **l'IP del controller** e il **modello di
robot** dichiarato: sotto, il driver `ur_robot_driver` parla con un controller
UR vero o con quello simulato da URSim, e non se ne accorge nessuno più in alto.

Applicazione principale: una **scansione sferica** dell'oggetto posato sulla
piattaforma. Il TCP percorre una griglia di waypoint su una sfera centrata
sull'oggetto, puntandolo sempre; a ogni waypoint si potrà scattare una foto.

---

## 1. Prerequisiti sulla macchina host

- Docker Engine + plugin `docker compose`
- `nvidia-container-toolkit` (il compose chiede la GPU NVIDIA; senza, va rimossa
  la sezione `deploy.resources.reservations.devices` da `docker-compose.yaml`)
- X11 in esecuzione (per RViz), `xauth`, `xhost`
- utente host nei gruppi `audio` e `video` (`run.sh` legge i loro GID)

```bash
sudo usermod -aG audio,video $USER   # poi ri-login
```

## 2. Clone del repo

I pacchetti UR upstream sono submoduli git, servono per riferimento e per
eventuali patch (l'immagine Docker installa comunque i `.deb` di `ros-jazzy-ur`):

```bash
git clone <url-del-repo> ur
cd ur
git submodule update --init --recursive
```

## 3. Immagine e container Docker

`run.sh` fa da wrapper su `docker compose`: ricava i nomi dalla cartella
corrente, esporta UID/GID dell'utente host, le variabili X11/Pulse, i GID di
audio/video e i mount di bash-history e VSCode.

```bash
./run.sh build     # costruisce l'immagine (con cache)
./run.sh rebuild   # ricostruisce da zero (--no-cache, lento)
./run.sh run       # apre una shell interattiva nel container
./run.sh down      # ferma e rimuove il container
```

Con la cartella del repo chiamata `ur` si ottiene:

| | valore |
|---|---|
| immagine | `ur_ws:jazzy` |
| container | `ur_container` |
| workspace nel container | `/home/ros/ur` |
| rete | `host` (necessaria per il discovery DDS e per il TCP/IP verso il robot) |
| `ROS_DOMAIN_ID` | `44` |

**Seconda shell nello stesso container** (serve quasi sempre: bringup in un
terminale, scan nell'altro). `docker exec` non passa dall'entrypoint, quindi va
fatto il source a mano:

```bash
docker exec -it ur_container bash
source /opt/ros/jazzy/setup.bash
source /home/ros/ur/install/setup.bash
```

L'entrypoint della shell principale, invece, fa da solo: source di ROS, source
di `install/setup.bash` se esiste, `apt update/upgrade` e
`rosdep install --from-paths src --ignore-src -r -y --skip-keys "moveit_resources"`.

## 4. Compilare il workspace (dentro il container)

```bash
cd /home/ros/ur
colcon build --cmake-args -DCMAKE_CXX_FLAGS="-w" --executor sequential
source install/setup.bash
```

Un solo pacchetto:

```bash
colcon build --packages-select ur_automata_bringup
source install/setup.bash
```

Test (c'è una gtest sul planner di sequenza, gli altri pacchetti non hanno test):

```bash
colcon test --packages-select ur_automata_scan
colcon test-result --verbose
```

> **Attenzione — il file di configurazione va ricompilato.**
> Tutti i launch file leggono `automata_config.yaml` dallo **share installato**,
> non da `src/`. Dopo ogni modifica al YAML serve
> `colcon build --packages-select ur_automata_bringup` e un nuovo
> `source install/setup.bash`, altrimenti si continua a lanciare la vecchia
> configurazione.

## 5. Il file di configurazione unico

`src/automata_robot/ur_automata_bringup/config/automata_config.yaml` è il punto
da cui passano *tutti* i parametri: bringup, MoveIt, scena e scansione lo
leggono. Tre sezioni:

```yaml
robot:
  ip: 192.168.56.101      # controller UR (URSim o reale)
  type: ur5               # ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e

planning:
  group: ur_manipulator
  global_frame: world
  end_effector_link: ee_automata_tcp
  trajectory_scaling_factor: 0.4
  home_pose_name: home
  lower_home_pose_name: lower_scan_ready
  use_moveit: true
  use_scene: true
  use_rviz: true

scan:
  center: [0.133, 0.40, 0.504]
  radius: 0.30
  ...
```

Note su `planning`:

- `end_effector_link: ee_automata_tcp` è il link definito in fondo a
  `ur_automata.urdf.xacro` (offset `xyz="0 0.052 0.153"` da `tool0`) e deve
  coincidere con il tip link del gruppo nel SRDF.
- `home_pose_name` e `lower_home_pose_name` sono `group_state` definiti in
  `ur_automata_moveit_config/config/ur_automata.srdf` (`home`, `up`,
  `lower_scan_ready`). Se ne cambi il nome qui, deve esistere lì.
- `trajectory_scaling_factor` finisce in `setMaxVelocityScalingFactor` **e**
  `setMaxAccelerationScalingFactor` dei nodi di scansione.
- `use_moveit`, `use_rviz`, `use_scene` sono i default degli omonimi argomenti
  di launch e si possono sovrascrivere da riga di comando.

---

## 6. Bring-up in simulazione (URSim)

URSim è il simulatore ufficiale UR: gira in un suo container Docker ed espone
un controller PolyScope completo. Non è Gazebo — non c'è fisica dell'ambiente,
ma il comportamento del controller (programmi, External Control, safety) è
quello vero. È il modo più fedele per provare tutto prima del robot reale.

### 6.1 Avviare URSim (sull'host, **non** dentro il container)

```bash
./start_ursim_seccomp.sh
```

Lo script:

- scarica l'URCap *External Control* v1.0.5 (`.jar`) in `~/.ursim/e-series/urcaps`
- crea la rete Docker `ursim_net` (`192.168.56.0/24`, gateway `192.168.56.1`)
- avvia `universalrobots/ursim_e-series:5.25.1` con IP fisso **192.168.56.101**
- monta programmi e impostazioni PolyScope sotto `~/.ursim/e-series` (persistenti)
- usa `--security-opt seccomp=unconfined`: è il workaround per i kernel ≥ 6.15,
  dove `URControl` fallisce con ENOSYS sulla `socket()`. Con lo script ufficiale
  `start_ursim.sh` il simulatore non parte.

Interfaccia PolyScope: browser su **http://localhost:6080** (noVNC) oppure client
VNC su `localhost:5900`.

### 6.2 Preparare PolyScope

1. Accendi il robot: *Power on* → *Start* → *OK* (il braccio deve risultare
   `RUNNING`, non `IDLE`).
2. *Installation* → *URCaps* → **External Control**:
   - **Host IP**: `192.168.56.1` (il gateway della rete `ursim_net`, cioè
     l'host su cui gira il driver ROS)
   - **Custom port**: `50002`
3. *Program* → aggiungi il nodo **External Control** al programma e salvalo.
4. Premi **Play** *solo dopo* aver avviato il bringup ROS (punto 6.3): il
   programma va in errore se non trova il driver in ascolto.

### 6.3 Avviare il bringup

Nel container, con `robot.ip: 192.168.56.101` in `automata_config.yaml`:

```bash
ros2 launch ur_automata_bringup ur_automata_bringup.launch.py
```

Cosa parte:

- `ur_automata_control.launch.py` → `ur_control.launch.py` del driver upstream
  con la nostra URDF (`ur_automata.urdf.xacro`), i nostri controller
  (`ur_automata_controllers.yaml`) e `scaled_joint_trajectory_controller` attivo
- `ur_automata_moveit.launch.py` → `move_group` + RViz con il pannello
  MotionPlanning (se `use_moveit`/`use_rviz` sono true)
- `scene.launch.py` di `ur_automata_scene` → pubblica la planning scene
  (tavolo, piattaforma, oggetto) via `/apply_planning_scene` (se `use_scene`)

Override da riga di comando degli argomenti dichiarati al livello alto:

```bash
ros2 launch ur_automata_bringup ur_automata_bringup.launch.py \
  use_moveit:=true use_rviz:=false use_scene:=true
```

Ora premi **Play** su PolyScope. Nel log del driver deve comparire
`Robot connected to reverse interface. Ready to receive control commands.`

### 6.4 Verifica rapida

```bash
ros2 topic echo /joint_states --once
ros2 control list_controllers
ros2 action list | grep follow_joint_trajectory
```

In RViz: trascina il marker interattivo, *Plan*, poi *Execute* — il braccio si
muove anche in PolyScope.

---

## 7. Bring-up sul robot reale

### 7.1 Rete

Collega il PC al controller UR via Ethernet e mettili nella stessa sottorete.
Sul teach pendant: *Settings* → *System* → *Network*, IP statico (nella cella si
usa `192.168.1.97`). Dall'host:

```bash
ping 192.168.1.97
```

### 7.2 Calibrazione cinematica (una volta per robot)

Ogni UR esce di fabbrica con i suoi parametri DH misurati: senza di essi
l'errore in punta può arrivare a qualche millimetro. Va fatto **una volta** per
ogni robot fisico, con il robot acceso e raggiungibile:

```bash
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=192.168.1.97 \
  target_filename:="/home/ros/ur/src/automata_robot/ur_automata_bringup/config/ur5e_calibration.yaml"
```

Il YAML prodotto va poi passato al bringup (vedi sotto). Senza, si usano i
valori nominali di `ur_description` — accettabili per URSim, **non** per il
robot reale.

### 7.3 Configurazione

In `automata_config.yaml`:

```yaml
robot:
  ip: 192.168.1.97
  type: ur5e            # il modello vero della cella
```

e ricompila `ur_automata_bringup` (vedi §4).

Per il file di calibrazione, che non ha una voce nel YAML, lancia i due livelli
separatamente — è l'unico modo pulito per passare argomenti al livello control,
perché il launch di alto livello non li ridichiara:

```bash
# terminale 1 — driver + controller
ros2 launch ur_automata_bringup ur_automata_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.1.97 \
  kinematics_params_file:=/home/ros/ur/src/automata_robot/ur_automata_bringup/config/ur5e_calibration.yaml

# terminale 2 — MoveIt + RViz (+ scena)
ros2 launch ur_automata_bringup ur_automata_moveit.launch.py \
  ur_type:=ur5e use_rviz:=true use_scene:=true
```

`ur_type` **deve essere identico nei due comandi**: altrimenti il modello
cinematico di `move_group` non corrisponde al TF pubblicato dal driver e le
traiettorie escono sbagliate senza errori evidenti.

Argomenti utili di `ur_automata_control.launch.py`:

| argomento | default | note |
|---|---|---|
| `ur_type` | da YAML | modello UR |
| `robot_ip` | da YAML | IP del controller |
| `kinematics_params_file` | nominale di `ur_description` | YAML di `ur_calibration` |
| `headless_mode` | `false` | `true` = il driver invia lo URScript direttamente, senza il programma External Control sul pendant |
| `tf_prefix` | `""` | prefisso su joint e link |
| `initial_joint_controller` | `scaled_joint_trajectory_controller` | MoveIt si aspetta questo |

### 7.4 Sequenza di accensione

1. Accendi il controller, sblocca i freni (*Power on* → *Start*).
2. Carica il programma con il nodo **External Control** (Host IP = IP del PC su
   cui gira ROS, porta `50002`).
3. Avvia il bringup ROS.
4. Premi **Play** sul pendant.

> **Ogni volta che riavvii il bringup devi ripremere Play**: allo stop del
> driver il programma URScript sul controller termina e va rilanciato a mano.

### 7.5 Prima messa in moto

- Abbassa `trajectory_scaling_factor` a `0.05`–`0.1` per i primi movimenti.
- Riduci lo *speed slider* di PolyScope al 20–30%: lo
  `scaled_joint_trajectory_controller` lo rispetta e rallenta la traiettoria
  senza deformarla.
- Tieni il pulsante di emergenza a portata di mano; verifica che la scena
  MoveIt (tavolo, piattaforma, oggetto) corrisponda davvero all'ingombro reale
  prima di eseguire una scansione completa.

---

## 8. Simulazione vs robot reale — cosa cambiare

| | URSim | Robot reale |
|---|---|---|
| `robot.ip` | `192.168.56.101` | IP del controller (es. `192.168.1.97`) |
| `robot.type` | `ur5` (URSim parte con `ROBOT_MODEL=UR5`) | modello reale, es. `ur5e` |
| `kinematics_params_file` | default nominale, va bene | **obbligatorio** il YAML di `ur_calibration` |
| `trajectory_scaling_factor` | `0.4` tranquillamente | partire da `0.05`–`0.1` |
| `planning.use_rviz` | `true` | `true` per verificare, `false` a regime |
| `scan.platform_sim` | `true` se la piattaforma reale non c'è | `false`: mesh STL della piattaforma vera |
| External Control | Play su PolyScope in noVNC | Play sul teach pendant, dopo ogni bringup |
| avvio | `./start_ursim_seccomp.sh` + bringup | solo bringup |

Il resto — controller, MoveIt, scena, nodi di scansione — non cambia.

---

## 9. Eseguire la scansione

Con il bringup già attivo, in una seconda shell del container:

```bash
ros2 launch ur_automata_scan scan_sequence.launch.py
```

Il nodo:

1. genera i waypoint sulla sfera (`scan.center`, `scan.radius`, emisfero,
   direzione, esclusioni all'equatore);
2. **enumera offline** le soluzioni IK di ogni waypoint (TRAC-IK in modalità
   `Speed`, più pitch e più seed per ramo), scartando quelle in collisione con
   la scena o con il braccio che occlude la vista camera→centro;
3. sceglie la sequenza con una **DP a strati**, minimizzando lo spostamento nei
   giunti tra waypoint consecutivi;
4. esegue, waypoint per waypoint, provando la catena di planner `planners` e
   ripiegando su un punto vicino se il waypoint è irraggiungibile;
5. torna a `home` e stampa il riepilogo (raggiunti / falliti, cause, tempo,
   quante volte è stato usato ciascun planner).

**Lo scan parte sempre in pausa.** Per farlo partire:

```bash
ros2 service call /scan_sequence_node/start std_srvs/srv/Trigger {}
ros2 service call /scan_sequence_node/pause std_srvs/srv/Trigger {}
```

Se il nodo gira su un terminale interattivo (non via `ros2 launch`) funzionano
anche i tasti: **SPAZIO** = pausa/riprendi, **Q** = esci.

`scan_executor_node` (`scan.launch.py`) è la versione precedente, senza
enumerazione offline né DP: pianifica ed esegue waypoint per waypoint. È tenuta
come riferimento/backup e i suoi service sono `/scan_executor_node/start` e
`/scan_executor_node/pause`.

> I marker dei waypoint (sfere + frecce dell'orientamento, colorate per stato)
> sono pubblicati su `/scan_waypoints_markers`. La config `automata.rviz` del
> bringup ha già il display **Scan waypoints** su quel topic: non serve
> aggiungerlo a mano. Se lanci RViz con un'altra config, il display va creato.

### 9.1 Parametri di scansione (`scan:`)

**Geometria**

| parametro | effetto |
|---|---|
| `center` | centro della sfera nel frame `planning.global_frame`, in metri |
| `radius` | raggio della sfera. È il vincolo più duro: allargarlo porta subito i waypoint fuori portata |
| `hemisphere` | `upper` \| `lower` \| `full` (prima il superiore, poi l'inferiore) |
| `direction` | `latitudinal` (anello per anello) \| `longitudinal` (meridiano per meridiano) |
| `num_rings`, `points_per_ring` | densità in modalità latitudinal |
| `num_arc`, `points_per_arc` | densità in modalità longitudinal |
| `equator_exclusion_upper_deg`, `equator_exclusion_lower_deg` | banda vietata attorno all'equatore: il robot non può puntare in linea con il supporto. I due emisferi possono avere valori diversi |
| `stagger_rings` | anelli pari sfasati di mezzo passo → copertura più uniforme |
| `adaptive_rings` | i punti per anello scalano con `sin(theta)`: meno al polo, più all'equatore |

**Orientamento del TCP**

| parametro | effetto |
|---|---|
| `lock_pitch` | `true`: X del TCP orizzontale, nessuna rotazione attorno a Y. `false`: pitch libero, si cerca l'offset con IK valida |
| `pitch_search_range_deg`, `pitch_search_step_deg` | intervallo e passo della ricerca del pitch |
| `pitch_xparallel_bias` | `0.0` = qualsiasi pitch va bene; `0.05` = preferenza moderata per X parallelo; `>0.3` = X parallelo quasi sempre |
| `occlusion_check`, `occlusion_threshold_deg` | scarta le IK in cui un link del braccio entra nel cono di vista camera→centro (tipico 15–25°) |

**IK e planning**

| parametro | effetto |
|---|---|
| `ik_timeout` | timeout di una singola `setFromIK` in esecuzione. Con TRAC-IK bastano 5–50 ms |
| `enum_ik_timeout` | timeout della IK in fase di enumerazione (`scan_sequence_node`). Costo massimo della fase = waypoint × pitch × seed × timeout |
| `planners` | catena di `scan_sequence_node`: ogni tratto prova i planner in ordine e si ferma al primo che riesce. Default `[pilz_ptp, ompl]`; la catena completa `[pilz_circ, pilz_ptp, stomp, ompl]` è stata misurata più lenta (262 s contro 140 s) senza ridurre i tratti su OMPL |
| `planner` | planner di `scan_executor_node` (nodo di backup): `pilz_ptp`, `pilz_lin`, `ompl` |
| `ompl_algorithm` | usato quando si pianifica con OMPL (`RRTConnect`, `RRTstar`, `PRM`, …) |
| `planning_time`, `planning_attempts` | tempo e tentativi indipendenti per waypoint. In scena affollata alzare a 10–15 s aiuta i punti difficili |
| `fallback_search`, `fallback_radius_mm`, `fallback_planning_time`, `fallback_max_plan_attempts` | ricerca di un punto alternativo vicino a un waypoint fallito |

Le voci ammesse in `planners`:

| voce | cosa fa | quando fallisce |
|---|---|---|
| `pilz_circ` | arco sulla sfera con centro `scan.center`: il TCP resta sulla sfera e la camera inquadra l'oggetto per tutto il tratto | se non si parte da un punto della sfera (da `home` e dalle pose di recovery viene saltato), vicino alle singolarità, o se l'arco collide |
| `pilz_ptp` | retta in joint space, deterministica, ~10 ms | quando la retta attraversa un ostacolo |
| `stomp` | parte dalla stessa retta e la deforma finché esce dalla collisione: percorso liscio e ripetibile | quando l'ostacolo è troppo grande per una deformazione locale |
| `ompl` | RRTConnect: trova quasi sempre un percorso, ma diverso a ogni run | raramente; è l'ultima risorsa |

Il riepilogo finale stampa la riga `Planner usati:` con il conteggio per
planner: è la metrica con cui si confrontano due run. Molti tratti su `ompl`
significano movimenti ampi e imprevedibili.

> `pilz_*` **ignora** il path constraint usato quando `lock_pitch: false`.
> `pilz_circ` fa eccezione: usa un path constraint proprio (il centro
> dell'arco), che il nodo imposta e rimuove attorno al singolo tentativo.
> `stomp` ha bisogno di `stomp_planning.yaml` in `ur_automata_moveit_config/config`.

**Scena**

`platform_sim: true` costruisce la piattaforma come disco + 3 gambe cilindriche;
`false` carica la mesh reale `ur_automata_scene/meshes/disk.stl` (esportata in
mm, quindi scalata ×0.001), posizionata in modo che la faccia superiore del
disco coincida con `scan.center`: spostare la piattaforma vuol dire cambiare
solo `scan.center`. Con `false` nella scena **non c'è nessun sostegno** sotto il
disco: se quello reale ne ha uno, va aggiunto prima di scansionare l'emisfero
inferiore sul robot vero. L'oggetto da scansionare è `meshes/ceramic_model.obj`,
piazzato in `scan.center`.

`platform_margin` e `table_margin` (metri, `0` = disattivo) aggiungono due
keep-out gialli semitrasparenti: un cilindro attorno al disco (raggio +margine,
spessore +2·margine) e una lastra alta *margine* sul tavolo sotto la sfera, che
parte da y = 0.12 per non toccare la base. Il check di collisione di MoveIt è
binario, quindi il margine si ottiene così; vale per il filtro dei candidati IK
e per i planner, percorsi inclusi. Un oggetto spesso non si "buca" fra due
controlli come una piastra da 4 mm. Attenzione sotto il disco: fra tavolo e
piattaforma ci sono ~28 cm e il braccio entra di taglio, 2 cm di margine da
entrambi i lati bastano a perdere i waypoint più bassi.

`walls` (`back_y`, `left_x`, `right_x`, metri, `0` = nessun muro) aggiunge le
pareti della cella dietro e ai lati del robot, mai davanti. Limitano lo spazio
in cui i planner possono deviare, non accorciano i percorsi al suo interno.
Vincoli: `back_y` non oltre −0.42 (a `home` il gomito arriva a y −0.37),
`right_x` non sotto 0.75 (a `lower_scan_ready` l'end effector arriva a x 0.71).

Il punto della sfera più lontano dalla base dista `|scan.center| + radius`:
con questo end effector il limite pratico dell'UR5e è ~0.92 m, quindi con
`radius: 0.30` il centro deve stare entro ~0.60 m dalla base.

---

## 10. Struttura del repo

```
docker/                  Dockerfile, entrypoint, requirements Python
docker-compose.yaml      servizio ros_dev: GPU, X11, rete host, mount del workspace
run.sh                   wrapper build/rebuild/run/down
start_ursim_seccomp.sh   avvio URSim con workaround seccomp

src/automata_robot/
  ur_automata_bringup/         config unica + launch di alto livello (control, moveit, bringup) + RViz
  ur_automata_description/     URDF/xacro della cella, mesh dell'end effector, launch di sola visualizzazione
  ur_automata_moveit_config/   SRDF, kinematics (TRAC-IK), limiti, pipeline OMPL/Pilz/STOMP, controller MoveIt, launch generati
  ur_automata_scene/           planning scene: tavolo, piattaforma, oggetto; mesh STL/OBJ
  ur_automata_scan/            generazione waypoint sferici, planner di sequenza (DP), nodi esecutori

src/utils/                     submoduli UR upstream (driver e description), branch jazzy — read-only
```

Visualizzare solo il modello, senza driver né MoveIt:

```bash
ros2 launch ur_automata_description display.launch.py ur_type:=ur5e
```

Provare MoveIt senza robot e senza URSim (hardware fittizio):

```bash
ros2 launch ur_automata_moveit_config demo.launch.py
```

L'end effector è la mesh `ee_automata_V2.stl` agganciata a `tool0`; il TCP
`ee_automata_tcp` è a `xyz = (0, 0.052, 0.153)` da `tool0`. Cambiare versione di
end effector significa aggiornare mesh **e** offset del TCP in
`ur_automata.urdf.xacro`.

---

## 11. Problemi frequenti

| Sintomo | Causa / rimedio |
|---|---|
| Modifico `automata_config.yaml` e non cambia niente | i launch leggono lo share installato: `colcon build --packages-select ur_automata_bringup` + `source install/setup.bash` |
| Il driver parte ma il robot non si muove | manca il **Play** su External Control: va ripremuto dopo ogni riavvio del bringup. In alternativa `headless_mode:=true` |
| `Can't accept new action goals. Controller is not running` | il programma sul controller non è in esecuzione (stesso problema di sopra) |
| URSim non parte, `URControl` va in errore | kernel ≥ 6.15: usa `./start_ursim_seccomp.sh`, non lo script ufficiale |
| PolyScope non vede l'URCap | il `.jar` deve stare in `~/.ursim/e-series/urcaps`; i vecchi `.urcap` non sono più riconosciuti da PolyScope 5.25 |
| External Control non si connette da URSim | Host IP deve essere `192.168.56.1` (gateway di `ursim_net`), non `127.0.0.1` |
| `No kinematics solver instantiated for group ur_manipulator` | il nodo applicativo non ha caricato `kinematics.yaml`: i launch di `ur_automata_scan` lo passano sotto `robot_description_kinematics`, un nodo scritto a mano deve fare lo stesso |
| Traiettorie plausibili ma posizioni sbagliate | `ur_type` diverso tra control e moveit, oppure manca il file di calibrazione sul robot reale |
| Non vedo i marker dei waypoint in RViz | RViz deve girare con `automata.rviz` (lo fa il bringup) e il display **Scan waypoints** deve essere abilitato; il topic è `/scan_waypoints_markers` |
| Molti waypoint falliscono in IK | `radius` troppo grande o `center` troppo lontano dalla base: la sfera esce dalla portata del braccio |
| Molti fallimenti in plan | alza `planning_time` e `planning_attempts`, oppure aggiungi `ompl` in fondo a `planners` |
| Plan fallito in pochi ms con `INVALID_MOTION_PLAN`, in move_group `ValidateSolution: Computed path is not valid` | il planner ha trovato un percorso ma lo ha controllato a passo troppo largo e sfiora un ostacolo sottile: abbassa `longest_valid_segment_fraction` in `config/ompl_planning.yaml` (oggi 0.001 ≈ 1.5°) |
| move_group avvisa `Cannot find planning configuration ... kConfigDefault` | manca la voce in `planner_configs` di `ompl_planning.yaml`: OMPL ignora `scan.ompl_algorithm` e usa RRTConnect |
| RViz non si apre dal container | `xhost +local:docker` (lo fa già `./run.sh run`) e `DISPLAY` valorizzato sull'host |
