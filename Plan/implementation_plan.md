# Piano di implementazione — catena di planner per `scan_sequence_node`

Data: 2026-09-08
Stato: eseguito, vedi §8
Ambito: `ur_automata_scan` (scan_sequence_node), `ur_automata_moveit_config`, `ur_automata_bringup` (config), README

---

## 1. Contesto e problema

`scan_sequence_node` esegue la scansione in tre fasi: enumerazione offline dei
candidati IK per ogni waypoint, scelta della sequenza con programmazione
dinamica (DP) a livelli, esecuzione tratto per tratto. Ogni tratto è oggi
pianificato con Pilz PTP (retta in joint space) e, se la retta collide, con
OMPL RRTConnect.

Risultato di riferimento (dp7, URSim, centro (0.133, 0.35, 0.470), r 0.30,
4x15 full): **82/82 waypoint, 143 s, 6 tratti su OMPL**.

Il problema residuo è qualitativo: i 6-12 tratti che cadono su RRTConnect
producono deviazioni ampie e casuali ("movimenti brutti"), diverse a ogni
run. Ogni ostacolo aggiunto alla scena (margini, muri) aumenta il numero di
rette bloccate e quindi di deviazioni.

## 2. Requisiti

### Funzionali
- R1. Il movimento primario fra due waypoint deve seguire un arco sulla
  sfera di scansione con centro `scan.center`, così che la camera resti
  orientata verso l'oggetto lungo tutto il tratto.
- R2. Quando l'arco non è pianificabile, si deve ricadere su una retta in
  joint space (comportamento attuale).
- R3. Quando anche la retta collide, il ripiego deve essere un planner che
  *deforma* la retta quanto basta per uscire dalla collisione (STOMP),
  non un planner a campionamento casuale.
- R4. RRTConnect resta disponibile come ultima risorsa.
- R5. La catena di planner deve essere configurabile da
  `automata_config.yaml` senza ricompilare.
- R6. Il riepilogo di fine scansione deve riportare quante volte è stato
  usato ciascun planner: è la metrica con cui si valuta il risultato.

### Non funzionali
- N1. Nessun peggioramento rispetto a dp7: 82/82 e tempo ≤ 143 s.
- N2. Nessuna dipendenza aggiuntiva nell'immagine Docker se non
  strettamente necessaria.
- N3. `scan_executor_node` (nodo storico di backup) e la sua interfaccia di
  configurazione (`planner`, `fallback_planner`) restano intatti.
- N4. Nessuna assunzione sim/reale nel codice: la catena vale identica su
  URSim e sul robot fisico.

### Vincoli verificati sul sistema
- STOMP è già installato: `ros-jazzy-moveit` dipende da `moveit-planners`
  che dipende da `moveit-planners-stomp`. Il file di configurazione di
  riferimento è `/opt/ros/jazzy/share/moveit_configs_utils/default_configs/stomp_planning.yaml`.
- `MoveItConfigsBuilder` carica automaticamente ogni `config/*_planning.yaml`
  del package MoveIt come pipeline con id pari al prefisso del file. Non
  servono modifiche ai launch di bringup.
- Pilz CIRC (header `trajectory_generator_circ.hpp`, `path_circle_generator.hpp`):
  - accetta il goal in joint space (fa la FK per ottenere la posa finale);
  - richiede `path_constraints.name = "center"`, esattamente una
    `PositionConstraint` con `link_name` uguale al tip del solver
    (`ee_automata_tcp`) ed esattamente una `primitive_poses` con il centro;
  - legge il centro nel frame del modello, che qui è `world` (root URDF,
    nessun virtual joint nel SRDF) e coincide con `planning.global_frame`;
  - tolleranza sul raggio `MAX_RADIUS_DIFF = 1e-2` m: se start e goal non
    sono alla stessa distanza dal centro fallisce con
    `INVALID_MOTION_PLAN` (eccezione `CenterPointDifferentRadius`).

## 3. Logica architetturale

### 3.1 Principio: dal deterministico al stocastico

I planner sono ordinati per "prevedibilità" decrescente. Ogni tratto prova
la catena nell'ordine e si ferma al primo successo:

| Ordine | Planner   | Pipeline MoveIt                  | Natura                                   | Quando fallisce                                        |
|--------|-----------|----------------------------------|------------------------------------------|--------------------------------------------------------|
| 1      | pilz_circ | pilz_industrial_motion_planner   | arco cartesiano, deterministico          | start non sulla sfera, singolarità, collisione sull'arco |
| 2      | pilz_ptp  | pilz_industrial_motion_planner   | retta in joint space, deterministico     | la retta collide                                        |
| 3      | stomp     | stomp                            | ottimizzazione locale della retta        | ostacolo troppo grande per una deformazione locale      |
| 4      | ompl      | ompl                             | RRTConnect, campionamento casuale        | non trova un percorso nel tempo concesso                |

Atteso su dp10: CIRC per la maggioranza dei tratti, PTP nei cambi di anello
e nelle vicinanze di singolarità, STOMP in pochi casi, OMPL ~0.

### 3.2 Componenti coinvolti

```
automata_config.yaml (scan.planners)
        │
        ▼
scan_sequence.launch.py  ──►  scan_sequence_node
                                  │  per ogni tratto: catena di planner
                                  ▼
                              move_group  ──► pipeline ompl / pilz / stomp
                                  ▲
                                  │  config/*_planning.yaml (auto-discovery)
                     ur_automata_moveit_config
```

- **move_group**: guadagna la pipeline `stomp` tramite un nuovo file YAML.
  Le pipeline `ompl` e `pilz_industrial_motion_planner` esistono già.
- **scan_sequence_node**: unico nodo modificato. La struttura a tre fasi
  non cambia; cambia solo la funzione che pianifica un singolo tratto.
- **Configurazione**: nuova chiave `scan.planners` (lista ordinata). Le
  chiavi `scan.planner` e `scan.fallback_planner` restano per
  `scan_executor_node`.

### 3.3 Pilz CIRC nel flusso a goal joint

La DP sceglie per ogni waypoint una configurazione di giunti precisa
(`Candidate.joints`). Il nodo continua a usare `setJointValueTarget` con
quella configurazione; per CIRC aggiunge il vincolo di percorso "center".

Punti chiave:

1. **Vincolo senza regione.** Il `PositionConstraint` porta il centro solo
   in `constraint_region.primitive_poses`, senza alcun `primitives`. Pilz
   legge soltanto la posizione; il response adapter `ValidateSolution`
   verifica il percorso anche contro `req.path_constraints`, e un vincolo
   di posizione con una regione reale (il TCP "nel centro") boccerebbe
   ogni arco. Un vincolo senza regione è disabilitato lato MoveIt e
   quindi ignorato dalla validazione.
2. **Vincolo attivo solo per CIRC.** Il vincolo viene impostato subito
   prima del tentativo CIRC e rimosso subito dopo, riuscito o fallito.
   Gli altri planner devono partire senza path constraints.
3. **Precondizione "sulla sfera".** CIRC ha senso solo se lo stato di
   partenza è un waypoint già raggiunto. Da `home`, dalle pose di recovery
   e dopo un'esecuzione fallita il raggio non coincide e CIRC fallirebbe
   sempre: il nodo tiene un flag `on_sphere` e salta CIRC quando è falso.
   Questo evita anche errori rumorosi nel terminale del bringup.
4. **Coerenza del ramo IK.** Con goal joint Pilz calcola la posa finale
   con la FK, poi ricava i giunti lungo l'arco con IK, seme = campione
   precedente. Se il ramo IK di partenza non è lo stesso del target
   scelto dalla DP, la traiettoria termina in una configurazione diversa
   da quella attesa e il tratto successivo partirebbe da uno stato non
   previsto. Il nodo confronta l'ultimo punto della traiettoria con il
   target (differenza per giunto, con wrap a ±π, sotto una soglia fissa
   di 0.01 rad, mappando i giunti per nome): se non coincide, il piano
   CIRC viene scartato e si passa al planner successivo.
5. **Orientazione lungo l'arco.** Pilz interpola l'orientazione fra start
   e goal a asse singolo: la camera resta approssimativamente rivolta al
   centro; l'esattezza è garantita solo agli estremi, dove si scatta.

### 3.4 STOMP come ripiego della retta

STOMP inizializza la traiettoria con l'interpolazione lineare in joint
space fra start e goal (la stessa retta di PTP) e la deforma
iterativamente con rumore controllato minimizzando costo di collisione e
di accelerazione. Il risultato è una retta "piegata" quanto basta,
ripetibile e liscia, dove RRTConnect produrrebbe un percorso arbitrario.

- Configurazione: copia del file di default di `moveit_configs_utils`
  (60 timesteps, 40 iterazioni, 30 rollouts, arresto alla prima traiettoria
  valida, TOTG per la temporizzazione, `ValidateSolution` a valle).
- Nessun parametro personalizzato in questa fase: si tarano solo se dp10
  mostra fallimenti STOMP su tratti che OMPL invece risolve.

### 3.5 Interazione con la DP

La sequenza e le configurazioni scelte in fase 2 non cambiano. La catena
agisce solo in fase 3, sul *come* percorrere ciascun tratto. Grazie al
controllo del ramo IK (3.3 punto 4), ogni planner che riesce lascia il
robot esattamente nella configurazione prevista dalla DP, quindi l'ipotesi
della DP sul punto di partenza del tratto successivo resta valida.

### 3.6 Gestione errori e osservabilità

- Ogni tentativo fallito produce una riga WARN con planner, motivo
  (`plan_error_name`) e planner successivo, come oggi.
- La descrizione di `INVALID_MOTION_PLAN` viene estesa: con CIRC lo
  stesso codice indica anche raggio o piano dell'arco non validi, non solo
  il rifiuto di `ValidateSolution`.
- L'intestazione della tabella mostra la catena completa
  (`pilz/CIRC > pilz/PTP > stomp > ompl/RRTConnect`).
- Il riepilogo finale aggiunge una riga con il conteggio dei successi per
  planner. Include anche i movimenti verso `home`: è accettabile, il
  confronto fra run usa sempre la stessa convenzione.
- Nome sconosciuto nella lista `planners` → WARN e voce ignorata; lista
  vuota dopo il filtro → errore e arresto del nodo (nessun planner
  utilizzabile).

### 3.7 Invarianti da preservare

- Interfaccia ROS (topic, action, servizio `start`) invariata.
- `scan_executor_node`, `scan.launch.py` e le loro chiavi YAML invariati.
- Nessun riferimento a sim o reale nel codice del nodo.
- Le pose di recovery e il ritorno a `home` usano la catena senza CIRC.

## 4. Passi di implementazione

### Step 0 — Consolidare il lavoro pendente
Committare le modifiche già presenti nel working tree (keep-out
`platform_margin`/`table_margin`, muri `walls.*`, `SceneOptions`, README
§9.1) come commit separato, con i valori correnti (`platform_margin 0.01`,
`table_margin 0`, muri `-0.50 / -0.8 / 0.8`). Motivo: dp10 deve essere
confrontabile con dp7 e il diff del planner deve restare leggibile.

### Step 1 — Verifica dei pacchetti nel container
Dentro il container: `ros2 pkg list | grep -E 'stomp|pilz'`. Attesi
`moveit_planners_stomp`, `stomp`, `pilz_industrial_motion_planner`. Solo
se `moveit_planners_stomp` manca, aggiungere `<exec_depend>moveit_planners_stomp</exec_depend>`
a `ur_automata_moveit_config/package.xml` (il `rosdep install`
dell'entrypoint lo installerà al prossimo avvio).

### Step 2 — Pipeline STOMP in `ur_automata_moveit_config`
Creare `config/stomp_planning.yaml` copiando il file di default di
`moveit_configs_utils`. Nessun'altra modifica: il builder lo rileva dal
nome. Verifica dopo il riavvio del bringup:
`ros2 param get /move_group planning_pipelines` deve elencare `stomp`.

### Step 3 — Configurazione `automata_config.yaml`
Nella sezione `scan`:
- aggiungere `planners: [pilz_circ, pilz_ptp, stomp, ompl]` con un
  commento che spiega ruolo e ordine di ciascun planner;
- aggiornare il commento di `planner` e `fallback_planner` indicando che
  sono usati solo da `scan_executor_node`;
- aggiornare l'elenco delle pipeline nel commento esistente
  (`ompl`, `pilz_ptp`, `pilz_lin`, `pilz_circ`, `stomp`).

### Step 4 — Launch `scan_sequence.launch.py`
Sostituire il passaggio di `scan_planner` e `scan_fallback_planner` con
`scan_planners` (lista di stringhe letta da `s["planners"]`). Nessun altro
parametro cambia.

### Step 5 — `scan_sequence_node.cpp`

5.1 Parametri: rimuovere `scan_planner` e `scan_fallback_planner`;
dichiarare `scan_planners` come array di stringhe con default
`[pilz_ptp, ompl]` (comportamento equivalente all'attuale se il YAML non
è aggiornato).

5.2 `resolve_planner`: aggiungere `pilz_circ` (pipeline Pilz, planner id
`CIRC`) e `stomp` (pipeline `stomp`, planner id `stomp`).

5.3 `PlannerSetup`: sostituire `primary`/`fallback`/`has_fallback` con una
lista ordinata di `PlannerChoice` e con il messaggio di vincolo
`circ_center`, costruito una sola volta in `main` secondo 3.3 punto 1
(frame `global_frame`, link `end_effector_link`, centro `scan.center`,
peso 1, nessuna primitiva). Commento nel codice che spiega perché la
regione deve restare vuota.

5.4 `plan_with_fallback`: nuova firma con il flag `allow_circ` e la
mappa dei conteggi per planner. Ciclo sulla lista: salta CIRC se il flag
è falso; per CIRC imposta il vincolo prima e lo rimuove sempre dopo; al
successo di CIRC esegue il controllo del ramo IK (3.3 punto 4) e, se
fallisce, prosegue con il planner successivo; al primo successo incrementa
il conteggio e ritorna. Eliminare il ripristino del planner primario a
fine funzione: ogni tentativo imposta esplicitamente pipeline e planner.

5.5 Flag `on_sphere` in `main`: falso all'avvio, dopo ogni
`recover_to_safe_pose` e dopo un `execute` fallito; vero dopo l'esecuzione
riuscita di un waypoint. `go_home` chiama sempre con CIRC disabilitato.

5.6 Helper per il controllo del ramo IK: confronto fra l'ultimo punto
della traiettoria pianificata e il target joint, mappando per nome
giunto, con wrap angolare e soglia fissa 0.01 rad. La soglia è
un'euristica dichiarata nel codice con il suo limite noto.

5.7 Messaggi e riepilogo: estendere la descrizione di
`INVALID_MOTION_PLAN`; comporre `planner_label` come catena con ` > `;
stampare la riga `Planner usati: ...` dopo `Scan complete`.

5.8 Allineare i commenti in testa alle funzioni modificate (in inglese o
italiano coerenti con il file) e i commenti che citano ancora
"primario/riserva".

### Step 6 — README
- §9.1: riga per `planners` con la descrizione dell'ordine e la nota su
  CIRC (solo fra due punti sulla sfera; da `home` e recovery passa a
  PTP); `planner`/`fallback_planner` marcati "solo `scan_executor_node`".
- §10: la descrizione di `ur_automata_moveit_config` cita le pipeline
  OMPL, Pilz e STOMP.
- Procedura di lettura del riepilogo: aggiungere `Planner usati` alle
  stringhe da cercare nei log.

### Step 7 — Build e riavvio (nel container)
`colcon build --packages-select ur_automata_moveit_config ur_automata_bringup ur_automata_scan`,
`source install/setup.bash`, riavvio del bringup (nuova pipeline in
move_group), su URSim premere Play sul programma External Control.

### Step 8 — Run dp10
`ros2 launch ur_automata_scan scan_sequence.launch.py 2>&1 | tee ~/ur/log/scan_seq_dp10.log`,
avvio con il servizio `start`. Dal log (strip ANSI) estrarre `Sequenza:`,
`Scan complete`, `Tempo scansione`, `Planner usati`, righe `fallito`.

### Step 9 — Valutazione
Confronto con dp7 secondo i criteri della sezione 5. Se CIRC fallisce
spesso per `CircTrajectoryConversionFailure` o limiti di giunto si tratta
di attraversamenti di singolarità: atteso nei cambi di anello, PTP li
copre. Se STOMP fallisce dove OMPL riesce, tarare `num_iterations` e
`num_rollouts` nel YAML (Step 2) e ripetere. Se restano tratti su OMPL,
valutare il livello 3 (riordino 2-opt in joint space accoppiato alla DP).

### Step 10 — Commit
Un commit per la catena di planner (Step 2-6) con il riferimento ai
numeri di dp10 nel messaggio. Aggiornare il commento in testa a
`automata_config.yaml` se cita il numero di run di riferimento.

## 5. Criteri di accettazione

- A1. `ros2 param get /move_group planning_pipelines` include `stomp`.
- A2. dp10: 82/82 waypoint raggiunti, 0 fallimenti.
- A3. dp10: conteggio `ompl` nel riepilogo ≤ 1 (obiettivo 0).
- A4. dp10: tempo di scansione ≤ 143 s.
- A5. Nessun tratto con deviazione visibile in RViz fuori dall'intorno
  della sfera (valutazione visiva durante il run).
- A6. `scan_executor_node` con `scan.launch.py` continua a partire con la
  stessa configurazione.
- A7. Con `planners: [pilz_ptp, ompl]` il comportamento è identico a dp7
  (regressione).

## 6. Rischi e mitigazioni

| Rischio | Effetto | Mitigazione |
|---------|---------|-------------|
| Vincolo "center" con regione non vuota | ValidateSolution scarta ogni arco | Regione vuota, commento esplicito nel codice, verifica su dp10 (conteggio CIRC > 0) |
| Vincolo lasciato attivo dopo CIRC | PTP/STOMP/OMPL falliscono o si comportano in modo anomalo | Rimozione incondizionata dopo ogni tentativo CIRC |
| CIRC termina su un ramo IK diverso | Il tratto successivo parte da uno stato non previsto dalla DP | Controllo dell'ultimo punto contro il target, scarto del piano |
| CIRC tentato da home/recovery | Errori Pilz rumorosi, tempo perso | Flag `on_sphere`, CIRC saltato |
| Campionamento CIRC a 0.1 s (fino a ~4 cm fra due controlli a scaling 0.4) | Un ostacolo sottile fra due campioni non viene visto | Stesso comportamento di PTP oggi; `platform_margin` 0.01 copre il disco da 4 mm |
| `global_frame` diverso dal frame del modello | CIRC ruota attorno a un punto sbagliato senza errore | Commento nel YAML e nel codice; oggi coincidono (`world`) |
| STOMP non installato nell'immagine | Pipeline assente, tutti i tratti STOMP falliscono | Step 1 di verifica; exec_depend solo se necessario |
| Tempo STOMP per tratto (40 iterazioni) | Tratti più lenti dei 5 s di OMPL | Arresto alla prima traiettoria valida; misurare su dp10 |

## 7. Fuori scope

- Colonna "planner" per waypoint nella tabella a schermo: si aggiunge solo
  se serve capire quali waypoint cadono su STOMP/OMPL.
- Livello 3 (riordino 2-opt della sequenza): solo se dp10 lascia tratti su
  OMPL.
- Taratura dei parametri STOMP e dei limiti cartesiani Pilz: solo su
  evidenza di dp10.
- Occlusione dell'emisfero superiore, steli nell'STL, trigger foto, cache
  YAML della sequenza, robot reale: fasi successive già pianificate.

## 8. Esito (2026-09-08)

| run | scena | planners | raggiunti | tempo | planner usati |
|---|---|---|---|---|---|
| dp7 (riferimento) | nessun margine, nessun muro | pilz_ptp, ompl | 82/82 | 143 s | PTP 76, OMPL 6 |
| dp10 | margine 0.01, muri ±0.8 / −0.5 | pilz_circ, pilz_ptp, stomp, ompl | 81/82 | 262 s | CIRC 55, PTP 21, STOMP 1, OMPL 5 |
| dp11 | nessun margine, nessun muro | pilz_ptp, ompl | 82/82 | 140 s | PTP 79, OMPL 4 |

- A1, A6, A7 soddisfatti; A2 e A4 falliti con la catena completa (dp10), A3
  di fatto invariato (OMPL 5 contro 6).
- CIRC: 80 tentativi, 25 falliti — 17 per limite di accelerazione dei giunti
  (13 sul wrist_1, fino a −19.5 rad/s²: ribaltamento del polso lungo l'arco,
  non un problema di scaling), 12 per arco in collisione, 1 per ramo IK
  diverso. Ogni tentativo fallito costa ~1.2 s; gli archi riusciti sono ~1 s
  più lenti della retta PTP. La riduzione di `max_trans_acc`/`max_trans_dec`
  a 1.5 ha eliminato le violazioni su spalla e gomito, non quelle del polso.
- STOMP: 6 chiamate, 5 `TIMED_OUT` con `planning_time` 5 s, 1 successo.
- Decisione: `planners: [pilz_ptp, ompl]` resta il default; la catena e i
  file di configurazione (`stomp_planning.yaml`, limiti Pilz) restano
  disponibili per prove future.
- Emerso durante dp10: il check di occlusione guardava solo le origini dei
  link e lasciava passare l'avambraccio davanti alla camera nell'emisfero
  inferiore. Corretto campionando i segmenti fra i giunti; in dp11 scarta
  114 candidati senza perdere waypoint.
- Waypoint 46–50 (anello basso lato base): con margine 0.01 uno dei cinque
  risulta irraggiungibile per autocollisione EE↔avambraccio; senza margine
  tornano tutti. È il limite geometrico della cella, vedi il piano dei
  centri alternativi nel prossimo passo.

## 9. Posizione della piattaforma (2026-09-08, URSim, r 0.30, full 4x15, margine e muri 0)

Il vincolo: base in origine, sfera di diametro 0.60 m dentro una banda
radiale utile di ~0.33-0.92 m (lato vicino: autocollisione EE-avambraccio
con il polso accanto alla base; lato lontano: portata). Conta la distanza
*orizzontale* dei punti bassi dall'asse della base, non la distanza del
centro. Prova = fase 2 (enumerazione + DP) con centro diverso, poi scan
intero sui candidati buoni.

| centro | note | raggiunti | tempo | candidati | anello basso lato base (wp 42-56) |
|---|---|---|---|---|---|
| (0.133, 0.35, 0.470) dp11 | riferimento | 82/82 | 140 s | 2290 | 124 |
| (0.0, 0.374, 0.470) A | centrato sull'asse, stessa distanza | 82/82 | 181 s | 2440 | 138 |
| (0.133, 0.365, 0.470) B | +1.5 cm y | 82/82 | 132 s | 1977 | 151 |
| **(0.133, 0.375, 0.455) C** | +2.5 cm y, -1.5 cm z | 82/82 | 139 s | **2457** | **202** |

- A: il lato vicino non cambia (stessa distanza dal centro) e la sfera a
  cavallo dell'asse pan raddoppia il costo in giunti (3137 -> 6152).
- B: gioco a somma negativa, l'anello basso guadagna il 22% e tutti gli
  altri anelli perdono il 10-20% (lato lontano wp 15: 38 -> 15).
- C: |c| quasi uguale a dp11 (0.604 contro 0.601 m) ma sfera piu'
  orizzontale e bassa: i punti bassi sono 2.5 cm piu' lontani dall'asse
  della base, quelli lontani piu' vicini all'altezza della spalla. Anello
  basso +63%, anello 2 +44%, lato lontano quasi intatto. Costo: 1.5 cm in
  meno sotto il disco, il triplo di tratti bloccati nella DP (19 s invece
  di 4.5, irrilevante). **Adottata come default.**
- wp 46 e 47 (1-2 candidati) non si muovono con nessuna posizione: leve
  strutturali soltanto (`equator_exclusion_lower_deg` 20 -> 25, raggio
  0.28 se la camera lo permette, forma dell'EE). Con `platform_margin`
  0.01 uno di questi cade a ogni run.
- Non provato: (0.133, 0.385, 0.440), per trovare dove la tavola comincia
  a mangiare l'emisfero inferiore.
