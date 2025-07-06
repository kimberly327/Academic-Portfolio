# RoboticSystems

Official repository for the "Robotic Systems" course - A comprehensive collection of path planning algorithms and robotic simulation tools.

## 🎯 Scopo del Progetto

Questo repository contiene implementazioni di algoritmi fondamentali per sistemi robotici, con particolare focus su:
- **Path Planning**: Algoritmi per la pianificazione di percorsi in ambienti 2D
- **Configuration Space**: Gestione delle dimensioni del robot e inflazione degli ostacoli
- **Visualizzazione**: Strumenti per la rappresentazione grafica degli algoritmi
- **Simulazione**: Integrazione con Godot per simulazioni robotiche

## 📁 Struttura del Progetto

### 🧠 `/lib` - Librerie Core
Contiene le implementazioni principali degli algoritmi robotici:

#### 📐 `/lib/utils` - Utilità Geometriche
- **`shape.py`**: Classi per rappresentare forme geometriche rettangolari
  - `Point`: Punti 2D con operazioni matematiche
  - `LineSegment`: Segmenti di linea
  - `Polygon`: Ostacoli rettangolari con algoritmi di contenimento ottimizzati
  - `create_rectangular_obstacle()`: Factory function per creare ostacoli

#### 🗺️ `/lib/path_planner` - Algoritmi di Path Planning
- **`visibility_graph.py`**: Implementazione del Visibility Graph Path Planner
  - Supporto per robot con dimensioni specifiche
  - Approccio Configuration Space (C-Space)
  - Algoritmo di Dijkstra ottimizzato con tie-breaking
  - Visualizzazione interattiva con matplotlib
  - Confronto tra robot puntiforme e robot con dimensioni

#### 🤖 `/lib/system` - Modelli di Sistemi Robotici
- **`basic.py`**: Sistemi dinamici di base
- **`cart.py`**: Modelli per sistemi cart (carrelli)
- **`controllers.py`**: Controllori per sistemi robotici
- **`polar.py`**: Sistemi in coordinate polari
- **`trajectory.py`**: Generazione e gestione di traiettorie

#### 📊 `/lib/data` - Gestione e Visualizzazione Dati
- **`dataplot.py`**: Utilità per plotting e visualizzazione dati

#### 🌐 `/lib/dds` - Data Distribution Service
- **`dds.py`**: Implementazione DDS per comunicazione distribuita

### 📓 `/cart2D` - Notebook Educativi
- **`path_control_visibility_graph.ipynb`**: Jupyter notebook dimostrativo per:
  - Tutorial interattivo sul Visibility Graph
  - Esempi pratici di path planning
  - Visualizzazioni step-by-step degli algoritmi

### 🎮 `/godot` - Simulazione e Visualizzazione
Progetto Godot per simulazioni robotiche interattive:

#### `/godot/cart2D_no_physics_project`
- **`robot.gd`**: Script principale del robot
- **`Body.gd`**: Controllo del corpo del robot
- **`LeftWheel.gd`/`RightWheel.gd`**: Controllo delle ruote
- **`world.tscn`**: Scena principale del mondo di simulazione
- **`robot.tscn`**: Modello del robot
- **`/Components/DDS`**: Integrazione DDS per comunicazione

## 🚀 Caratteristiche Principali

### ✨ Path Planning Avanzato
- **Visibility Graph**: Algoritmo completo per percorsi ottimali
- **Configuration Space**: Gestione automatica delle dimensioni del robot
- **Visualizzazione Dual**: Mostra sia ostacoli fisici che C-Space inflazionato
- **Performance Ottimizzate**: Algoritmi O(1) per forme rettangolari

### 🎯 Focus Educativo
- **Documentazione Dettagliata**: Spiegazioni matematiche e implementative
- **Jupyter Notebooks**: Tutorial interattivi passo-passo
- **Visualizzazioni**: Grafici chiari per comprendere gli algoritmi
- **Esempi Pratici**: Casi d'uso realistici e comparazioni

### 🔧 Architettura Modulare
- **Separazione delle Responsabilità**: Ogni modulo ha uno scopo specifico
- **Riusabilità**: Componenti indipendenti e combinabili
- **Estensibilità**: Struttura che facilita l'aggiunta di nuovi algoritmi
- **Type Safety**: Type hints complete per debugging efficace

## 🛠️ Utilizzo

### Installazione Dipendenze
```bash
pip install numpy matplotlib jupyter
```

### Esempio Base - Visibility Graph
```python
from lib.path_planner.visibility_graph import VisibilityGraph
from lib.utils.shape import create_rectangular_obstacle

# Crea ostacoli
obstacles = [
    create_rectangular_obstacle(2, 2, 2, 1),
    create_rectangular_obstacle(6, 1, 1, 3)
]

# Crea planner con dimensioni robot
planner = VisibilityGraph(obstacles, robot_width=0.8, robot_length=1.0)

# Pianifica percorso
start = (0, 0)
goal = (8, 8)
path, distance = planner.plan_path(start, goal)

# Visualizza risultato
planner.visualize_graph(start, goal)
```

### Jupyter Notebook
```bash
jupyter notebook cart2D/path_control_visibility_graph.ipynb
```

### Simulazione Godot
1. Apri Godot Engine
2. Importa il progetto da `godot/cart2D_no_physics_project/`
3. Esegui la scena `world.tscn`

## 📚 Documentazione

Ogni modulo include documentazione dettagliata:
- **`/lib/path_planner/DOC.md`**: Visibility Graph implementation
- **`/lib/utils/SHAPE_DOC.md`**: Shape utilities documentation

## 🎓 Applicazioni

Questo repository è ideale per:
- **Corsi di Robotica**: Material didattico completo
- **Ricerca**: Base solida per algoritmi avanzati
- **Prototipazione**: Sviluppo rapido di sistemi di navigazione
- **Simulazione**: Test di algoritmi in ambiente controllato

## 🔮 Sviluppi Futuri

- Supporto per ostacoli non rettangolari
- Algoritmi di path planning aggiuntivi (A*, RRT)
- Integrazione con sensori reali
- Estensione a ambienti 3D

---

*Sviluppato per il corso "Robotic Systems" - Focus su algoritmi di path planning e simulazione robotica*