# 📚 Documentazione Dettagliata: Visibility Graph Path Planner

## 🎯 Panoramica Generale

Il **Visibility Graph Path Planner** è un algoritmo di pianificazione di percorsi per ambienti 2D con ostacoli rettangolari. L'algoritmo trova il percorso più breve (in termini di distanza euclidea) tra un punto di partenza e un punto di arrivo, evitando tutti gli ostacoli.

Il planner supporta robot con dimensioni specifiche utilizzando l'approccio del **Configuration Space (C-Space)**, che espande gli ostacoli delle dimensioni del robot permettendo di trattare il robot come un punto nello spazio inflazionato.

### 🔍 Principio di Funzionamento

L'algoritmo si basa sul concetto di **"visibilità"**: due punti sono visibili se la linea retta che li collega non attraversa nessun ostacolo. Il grafo di visibilità connette tutti i punti visibili tra loro, creando una rete di percorsi possibili.

### 🤖 Configuration Space (C-Space)

Il planner implementa il concetto di Configuration Space:
- **Ostacoli fisici**: Gli ostacoli reali nell'ambiente
- **Ostacoli inflazionati**: Gli ostacoli espansi di un raggio pari alla metà della dimensione maggiore del robot
- **Robot come punto**: Nello spazio inflazionato, il robot può essere trattato come un punto

---

## 📄 Analisi Dettagliata del Codice

### 🎨 Header e Documentazione del Modulo

```python
"""
Visibility Graph Path Planner

This module implements a visibility graph-based path planning algorithm for 2D environments
with rectangular obstacles. The visibility graph connects the start and goal points to all
visible vertices of rectangular obstacles, creating an optimal path in terms of Euclidean distance.

The planner accounts for robot dimensions by inflating obstacles (Configuration Space approach).
Each obstacle is expanded by half of the robot's largest dimension on all sides, allowing
the robot to be treated as a point in the inflated space.

Author: Robotic Systems Library
"""
```

**Spiegazione:**
- **Docstring del modulo**: Descrive lo scopo del file
- Specifica che implementa un algoritmo di visibility graph per path planning 2D con **ostacoli rettangolari**
- Menziona che trova percorsi ottimali in termini di distanza euclidea
- **Novità**: Spiega l'approccio Configuration Space per gestire le dimensioni del robot
- Include informazioni sull'autore per riferimenti futuri

---

### 📦 Sezione Import

```python
import math
import heapq
import sys
import os
from typing import List, Tuple, Optional
import matplotlib.pyplot as plt
import matplotlib.patches as patches
```

**Spiegazione linea per linea:**

1. `import math` - **Operazioni matematiche**: sqrt() per distanze euclidee, operazioni trigonometriche
2. `import heapq` - **Priority queue**: Implementa l'algoritmo di Dijkstra con heap binario per efficienza
3. `import sys` - **Sistema**: Necessario per modificare il path Python e accedere ai moduli locali
4. `import os` - **Filesystem**: Gestisce i percorsi dei file in modo cross-platform
5. `from typing import List, Tuple, Optional` - **Type hints**: Migliora la leggibilità e permette il type checking
6. `import matplotlib.pyplot as plt` - **Visualizzazione grafica**: Crea grafici 2D per mostrare il grafo e il percorso
7. `import matplotlib.patches as patches` - **Forme geometriche**: Disegna poligoni (ostacoli) nel grafico

```python
# Import shape classes from utils
sys.path.append("../../")
from lib.utils.shape import create_rectangular_obstacle, Polygon, Point, LineSegment
```

**Spiegazione:**
8. `sys.path.append("../../")` - **Aggiunge il path**: Permette di importare moduli dalla cartella padre
   - `"../../"` sale di due livelli nella gerarchia per raggiungere la root del progetto
9. `from lib.utils.shape import ...` - **Import classi geometriche**: Importa le classi per forme geometriche rettangolari dal modulo `shape.py`
   - **Nota**: Solo ostacoli rettangolari sono supportati in questa versione

---

## 🏗️ Classe Principale: VisibilityGraph

### 🔧 Inizializzazione

```python
class VisibilityGraph:
    """
    Visibility Graph Path Planner
    
    Creates a graph where nodes are start/goal points and rectangular obstacle vertices,
    and edges connect nodes that are mutually visible (no obstacles between them).
    The planner accounts for robot dimensions by inflating obstacles.
    """
    
    def __init__(self, obstacles: List[Polygon], robot_width: float = 0.0, robot_length: float = 0.0):
        """
        Initialize visibility graph with rectangular obstacles and robot dimensions.
        
        Args:
            obstacles: List of Polygon objects representing rectangular obstacles
            robot_width: Width of the robot
            robot_length: Length of the robot
        """
        self.robot_width = robot_width
        self.robot_length = robot_length
        self.robot_radius = max(robot_width, robot_length) / 2.0  # Half of the largest dimension
        
        # Inflate obstacles to account for robot dimensions
        self.original_obstacles = obstacles
        self.obstacles = self._inflate_obstacles(obstacles)
        self.vertices = self._extract_vertices()
        self.graph = {}
```

**Spiegazione linea per linea:**

1. `class VisibilityGraph:` - **Definizione della classe**: Incapsula tutto l'algoritmo
2. Docstring della classe spiega il concetto: **nodi** = punti importanti, **archi** = connessioni visibili, **gestione dimensioni robot**
3. `def __init__(self, obstacles: List[Polygon], robot_width: float = 0.0, robot_length: float = 0.0):` - **Costruttore**: Riceve lista di ostacoli e dimensioni del robot
4. `self.robot_width = robot_width` - **Memorizza larghezza robot**: Per calcolare il raggio di inflazione
5. `self.robot_length = robot_length` - **Memorizza lunghezza robot**: Per calcolare il raggio di inflazione  
6. `self.robot_radius = max(robot_width, robot_length) / 2.0` - **Calcola raggio robot**: Metà della dimensione maggiore per l'inflazione
7. `self.original_obstacles = obstacles` - **Mantiene ostacoli originali**: Per visualizzazione e riferimento
8. `self.obstacles = self._inflate_obstacles(obstacles)` - **Crea ostacoli inflazionati**: Espande gli ostacoli del raggio del robot
9. `self.vertices = self._extract_vertices()` - **Estrae vertici**: Raccoglie tutti i vertici degli ostacoli inflazionati
10. `self.graph = {}` - **Inizializza grafo vuoto**: Dizionario che mapperà ogni punto ai suoi vicini visibili

### 🔍 Inflazione degli Ostacoli (Configuration Space)

```python
def _inflate_obstacles(self, obstacles: List[Polygon]) -> List[Polygon]:
    """
    Inflate rectangular obstacles by robot radius to account for robot dimensions.
    This creates the Configuration Space (C-Space) representation where the robot
    can be treated as a point.
    
    Args:
        obstacles: List of original rectangular obstacles
        
    Returns:
        List of inflated rectangular obstacles
    """
    if self.robot_radius <= 0:
        return obstacles
    
    inflated_obstacles = []
    
    for obstacle in obstacles:
        # For rectangular obstacles, we need to find the bounding box
        # and expand it by robot_radius on all sides
        
        # Extract x and y coordinates of all vertices
        x_coords = [vertex.x for vertex in obstacle.vertices]
        y_coords = [vertex.y for vertex in obstacle.vertices]
        
        # Find the bounding box of the original obstacle
        min_x = min(x_coords)
        max_x = max(x_coords)
        min_y = min(y_coords)
        max_y = max(y_coords)
        
        # Calculate original dimensions
        original_width = max_x - min_x
        original_height = max_y - min_y
        
        # Expand the bounding box by robot_radius on all sides
        # New bottom-left corner (shifted by robot_radius)
        inflated_min_x = min_x - self.robot_radius
        inflated_min_y = min_y - self.robot_radius
        
        # Calculate the new inflated dimensions (add 2 * robot_radius)
        inflated_width = original_width + 2 * self.robot_radius
        inflated_height = original_height + 2 * self.robot_radius
        
        # Create the inflated rectangular obstacle using bottom-left corner
        inflated_obstacle = create_rectangular_obstacle(
            inflated_min_x, inflated_min_y, inflated_width, inflated_height
        )
        
        inflated_obstacles.append(inflated_obstacle)
    
    return inflated_obstacles
```

**Spiegazione del Configuration Space:**

1. `if self.robot_radius <= 0: return obstacles` - **Robot puntiforme**: Se il robot non ha dimensioni, non serve inflazione
2. `inflated_obstacles = []` - **Lista risultato**: Accumula gli ostacoli inflazionati
3. **Loop per ogni ostacolo**: Processa ogni ostacolo rettangolare individualmente
4. **Estrazione coordinate**: `x_coords` e `y_coords` raccolgono tutti i vertici
5. **Calcolo bounding box**: Trova i limiti min/max dell'ostacolo originale
6. **Calcolo dimensioni originali**: `original_width` e `original_height`
7. **Espansione**: Sottrae `robot_radius` ai minimi (sposta l'angolo) e aggiunge `2 * robot_radius` alle dimensioni
8. **Creazione ostacolo inflazionato**: Usa `create_rectangular_obstacle` con le nuove dimensioni
9. **Accumulo risultato**: Aggiunge l'ostacolo inflazionato alla lista

**Concetto chiave**: L'inflazione trasforma il problema:
- **Prima**: Robot con dimensioni che naviga tra ostacoli
- **Dopo**: Robot puntiforme che naviga tra ostacoli espansi

### 🔍 Estrazione dei Vertici

```python
def _extract_vertices(self) -> List[Point]:
    """Extract all vertices from obstacles."""
    vertices = []
    for obstacle in self.obstacles:
        vertices.extend(obstacle.vertices)
    return vertices
```

**Spiegazione:**
1. `def _extract_vertices(self) -> List[Point]:` - **Metodo privato**: Il prefisso `_` indica uso interno
2. `vertices = []` - **Lista vuota**: Accumula tutti i vertici
3. `for obstacle in self.obstacles:` - **Itera ostacoli inflazionati**: Esamina ogni ostacolo inflazionato uno per volta
4. `vertices.extend(obstacle.vertices)` - **Aggiunge vertici**: `extend()` aggiunge tutti gli elementi in una volta
5. `return vertices` - **Restituisce lista completa**: Tutti i vertici di tutti gli ostacoli inflazionati

### 🔍 Controllo Intersezione Segmenti

```python
def _line_intersects_polygon(self, line: LineSegment, polygon: Polygon) -> bool:
    """
    Check if a line segment intersects with any edge of a rectangular polygon.
    
    Args:
        line: Line segment to check
        polygon: Rectangular polygon to check intersection with
        
    Returns:
        True if line intersects polygon, False otherwise
    """
    for edge in polygon.edges:
        if self._segments_intersect(line, edge):
            return True
    return False
```

**Spiegazione:**
1. **Metodo ausiliario**: Verifica se un segmento attraversa un poligono rettangolare
2. `for edge in polygon.edges:` - **Controlla ogni lato**: Itera tutti i lati del rettangolo
3. `if self._segments_intersect(line, edge):` - **Test intersezione**: Usa l'algoritmo geometrico per due segmenti
4. `return True` - **Intersezione trovata**: Anche una sola intersezione invalida la visibilità
5. `return False` - **Nessuna intersezione**: Il segmento non attraversa il rettangolo

### 🧮 Algoritmo di Intersezione Segmenti

```python
def _segments_intersect(self, seg1: LineSegment, seg2: LineSegment) -> bool:
    """
    Check if two line segments intersect using cross product method.
    
    Args:
        seg1: First line segment
        seg2: Second line segment
        
    Returns:
        True if segments intersect, False otherwise
    """
    def ccw(A: Point, B: Point, C: Point) -> bool:
        return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x)
    
    A, B = seg1.p1, seg1.p2
    C, D = seg2.p1, seg2.p2
    
    # Check if endpoints are the same (touching is allowed)
    if A == C or A == D or B == C or B == D:
        return False
    
    return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)
```

**Spiegazione dettagliata:**

1. **Algoritmo CCW (Counter-ClockWise)**: Usa il prodotto vettoriale per determinare l'orientamento
2. `def ccw(A: Point, B: Point, C: Point) -> bool:` - **Funzione interna**: Determina se i punti sono in senso antiorario
3. `return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x)` - **Prodotto vettoriale**: 
   - Calcola il prodotto vettoriale dei vettori AB e AC
   - Se positivo → senso antiorario, se negativo → senso orario
4. `A, B = seg1.p1, seg1.p2` - **Estremi primo segmento**
5. `C, D = seg2.p1, seg2.p2` - **Estremi secondo segmento**
6. **Controllo punti coincidenti**: Se i segmenti condividono un estremo, non c'è "vera" intersezione
7. **Test finale**: Due segmenti si intersecano se e solo se:
   - I punti C e D sono su lati opposti della retta AB
   - I punti A e B sono su lati opposti della retta CD

### 👁️ Controllo di Visibilità

```python
def _is_visible(self, p1: Point, p2: Point) -> bool:
    """
    Check if two points are mutually visible (no obstacles block the line of sight).
    
    Args:
        p1: First point
        p2: Second point
        
    Returns:
        True if points are visible to each other, False otherwise
    """
    if p1 == p2:
        return False
    
    line = LineSegment(p1, p2)
    
    # Check if line intersects any obstacle
    for obstacle in self.obstacles:
        if self._line_intersects_polygon(line, obstacle):
            return False
    
    # Check if line passes through interior of any obstacle
    midpoint = Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2)
    for obstacle in self.obstacles:
        if obstacle.contains_point(midpoint):
            return False
    
    return True
```

**Spiegazione:**
1. `if p1 == p2:` - **Punti identici**: Un punto non è "visibile" a se stesso
2. `line = LineSegment(p1, p2)` - **Crea segmento**: La linea di vista tra i due punti
3. **Primo controllo**: Verifica se la linea interseca i bordi di qualche ostacolo inflazionato
4. `midpoint = Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2)` - **Punto medio**: Controllo aggiuntivo per sicurezza
5. **Secondo controllo**: Verifica se il punto medio è dentro un ostacolo inflazionato
   - Questo cattura casi dove la linea passa attraverso l'interno senza toccare i bordi
6. `return True` - **Visibilità confermata**: Nessun ostacolo inflazionato blocca la vista

### 🏗️ Costruzione del Grafo

```python
def _build_graph(self, start: Point, goal: Point) -> None:
    """
    Build the visibility graph including start and goal points.
    
    Args:
        start: Starting point
        goal: Goal point
    """
    self.graph = {}
    all_points = [start, goal] + self.vertices
    
    for i, point in enumerate(all_points):
        self.graph[point] = []
        for j, other_point in enumerate(all_points):
            if i != j and self._is_visible(point, other_point):
                distance = point.distance_to(other_point)
                self.graph[point].append((other_point, distance))
```

**Spiegazione:**
1. `self.graph = {}` - **Reset del grafo**: Ricrea il grafo da zero
2. `all_points = [start, goal] + self.vertices` - **Tutti i nodi**: 
   - Start e goal sono sempre inclusi
   - Aggiunge tutti i vertici degli ostacoli
3. **Double loop**: Controlla ogni coppia di punti
4. `for i, point in enumerate(all_points):` - **Punto sorgente**: Ogni punto come origine
5. `self.graph[point] = []` - **Lista vuota**: Inizializza lista dei vicini
6. `for j, other_point in enumerate(all_points):` - **Punto destinazione**: Ogni altro punto
7. `if i != j` - **Evita auto-connessioni**: Un punto non si collega a se stesso
8. `and self._is_visible(point, other_point):` - **Test visibilità**: Solo se i punti sono visibili
9. `distance = point.distance_to(other_point)` - **Calcola peso**: Distanza euclidea come peso dell'arco
10. `self.graph[point].append((other_point, distance))` - **Aggiunge arco**: Tupla (destinazione, peso)

### 🗺️ Algoritmo di Dijkstra

```python
def _dijkstra(self, start: Point, goal: Point) -> Tuple[Optional[List[Point]], float]:
    """
    Find shortest path using Dijkstra's algorithm.
    
    Args:
        start: Starting point
        goal: Goal point
        
    Returns:
        Tuple of (path as list of points, total distance)
        Returns (None, float('inf')) if no path exists
    """
    distances = {point: float('inf') for point in self.graph}
    distances[start] = 0
    previous = {}
    unvisited = [(0, 0, start)]  # Added counter as tie-breaker
    visited = set()
    counter = 1  # Counter to ensure unique ordering
    
    while unvisited:
        current_dist, _, current = heapq.heappop(unvisited)  # Updated to handle 3-tuple
        
        if current in visited:
            continue
        
        visited.add(current)
        
        if current == goal:
            # Reconstruct path
            path = []
            while current is not None:
                path.append(current)
                current = previous.get(current)
            path.reverse()
            return path, distances[goal]
        
        for neighbor, edge_weight in self.graph[current]:
            if neighbor in visited:
                continue
            
            distance = current_dist + edge_weight
            
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous[neighbor] = current
                heapq.heappush(unvisited, (distance, counter, neighbor))  # Added counter as tie-breaker
                counter += 1
    
    return None, float('inf')
```

**Spiegazione dettagliata dell'algoritmo di Dijkstra:**

#### Inizializzazione:
1. `distances = {point: float('inf') for point in self.graph}` - **Distanze infinite**: Tutti i punti inizialmente irraggiungibili
2. `distances[start] = 0` - **Partenza a distanza zero**: Il punto di partenza ha distanza 0 da se stesso
3. `previous = {}` - **Traccia percorso**: Memorizza il predecessore di ogni punto per ricostruire il percorso
4. `unvisited = [(0, 0, start)]` - **Priority queue**: Heap con (distanza, counter, punto), inizia con il punto di partenza
5. `visited = set()` - **Punti visitati**: Evita di rivisitare punti già elaborati
6. `counter = 1` - **Contatore tie-breaker**: Assicura ordinamento unico quando le distanze sono uguali

#### Loop principale:
7. `while unvisited:` - **Continua fino a completamento**: Fino a che ci sono punti da visitare
8. `current_dist, _, current = heapq.heappop(unvisited)` - **Punto più vicino**: Estrae il punto con distanza minima (ignora il counter)
9. `if current in visited: continue` - **Evita duplicati**: Skip se già visitato (possibile con heap)
10. `visited.add(current)` - **Marca come visitato**: Aggiunge al set dei visitati

#### Controllo obiettivo:
11. `if current == goal:` - **Obiettivo raggiunto**: Se abbiamo raggiunto la destinazione
12. **Ricostruzione del percorso**:
    - `path = []` - Lista vuota per il percorso
    - `while current is not None:` - Risale all'indietro fino al punto di partenza
    - `path.append(current)` - Aggiunge punto corrente
    - `current = previous.get(current)` - Va al predecessore
    - `path.reverse()` - Inverte per avere ordine start→goal
13. `return path, distances[goal]` - **Restituisce risultato**: Percorso e distanza totale

#### Aggiornamento vicini:
14. `for neighbor, edge_weight in self.graph[current]:` - **Esamina vicini**: Tutti i punti collegati
15. `if neighbor in visited: continue` - **Skip visitati**: Non rielabora punti già ottimizzati
16. `distance = current_dist + edge_weight` - **Nuova distanza**: Distanza attuale + peso arco
17. `if distance < distances[neighbor]:` - **Miglioramento trovato**: Se la nuova distanza è migliore
18. `distances[neighbor] = distance` - **Aggiorna distanza**: Memorizza la distanza migliore
19. `previous[neighbor] = current` - **Aggiorna predecessore**: Per ricostruire il percorso
20. `heapq.heappush(unvisited, (distance, counter, neighbor))` - **Aggiunge alla coda**: Con la nuova distanza e counter per tie-breaking
21. `counter += 1` - **Incrementa counter**: Assicura ordinamento deterministico

#### Fallback:
22. `return None, float('inf')` - **Nessun percorso**: Se non è possibile raggiungere il goal

**Miglioramenti rispetto alla versione base:**
- **Tie-breaking deterministico**: Il counter assicura che punti con distanze uguali siano processati in ordine consistente
- **Gestione heap migliorata**: Evita errori di confronto quando i punti hanno distanze identiche

### 🎯 Metodo Principale di Pianificazione

```python
def plan_path(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Tuple[Optional[List[Tuple[float, float]]], float]:
    """
    Plan a path from start to goal using visibility graph.
    
    Args:
        start: Starting position as (x, y) tuple
        goal: Goal position as (x, y) tuple
        
    Returns:
        Tuple of (path as list of (x, y) tuples, total distance)
        Returns (None, float('inf')) if no path exists
    """
    start_point = Point(start[0], start[1])
    goal_point = Point(goal[0], goal[1])
    
    # Check if start or goal is inside an obstacle
    for obstacle in self.obstacles:
        if obstacle.contains_point(start_point):
            print(f"Warning: Start point {start} is inside an obstacle")
            return None, float('inf')
        if obstacle.contains_point(goal_point):
            print(f"Warning: Goal point {goal} is inside an obstacle")
            return None, float('inf')
    
    # Build visibility graph
    self._build_graph(start_point, goal_point)
    
    # Find shortest path
    path_points, distance = self._dijkstra(start_point, goal_point)
    
    if path_points is None:
        return None, float('inf')
    
    # Convert back to tuple format
    path = [(point.x, point.y) for point in path_points]
    return path, distance
```

**Spiegazione:**
1. **Conversione input**: Da tuple a oggetti Point per compatibilità interna
2. **Validazione input**: Verifica che start e goal non siano dentro ostacoli inflazionati
   - Se il robot è puntiforme, controlla gli ostacoli originali
   - Se il robot ha dimensioni, controlla gli ostacoli inflazionati
3. **Costruzione grafo**: Crea il grafo di visibilità specifico per questo problema
4. **Ricerca percorso**: Applica Dijkstra sul grafo costruito
5. **Conversione output**: Da oggetti Point a tuple per l'interfaccia utente

### 📊 Visualizzazione

```python
def visualize_graph(self, start: Tuple[float, float], goal: Tuple[float, float]):
    """
    Visualize the visibility graph with rectangular obstacles using matplotlib.
    Shows both original obstacles and inflated obstacles for robot dimensions.
    
    Args:
        start: Starting position as (x, y) tuple
        goal: Goal position as (x, y) tuple
    """
    
    fig, ax = plt.subplots(1, 1, figsize=(10, 8))
    
    # Draw original obstacles (actual physical obstacles)
    for i, obstacle in enumerate(self.original_obstacles):
        vertices = [(v.x, v.y) for v in obstacle.vertices]
        # Add label only to the first obstacle
        label = 'Physical Obstacles' if i == 0 else None
        polygon = patches.Polygon(vertices, closed=True, facecolor='red', alpha=0.7, 
                                edgecolor='darkred', linewidth=2, label=label)
        ax.add_patch(polygon)
    
    # Draw inflated obstacles (configuration space obstacles) if robot has dimensions
    if self.robot_radius > 0:
        for i, obstacle in enumerate(self.obstacles):
            vertices = [(v.x, v.y) for v in obstacle.vertices]
            # Add label only to the first inflated obstacle
            label = 'Inflated Obstacles (C-Space)' if i == 0 else None
            polygon = patches.Polygon(vertices, closed=True, facecolor='orange', alpha=0.3, 
                                    edgecolor='darkorange', linewidth=1, linestyle='--', 
                                    label=label)
            ax.add_patch(polygon)
    
    # Build graph for visualization
    start_point = Point(start[0], start[1])
    goal_point = Point(goal[0], goal[1])
    self._build_graph(start_point, goal_point)
    
    # Draw visibility edges
    for point, neighbors in self.graph.items():
        for neighbor, _ in neighbors:
            ax.plot([point.x, neighbor.x], [point.y, neighbor.y], 'b-', alpha=0.3, linewidth=0.5)
    
    # Draw vertices
    for vertex in self.vertices:
        ax.plot(vertex.x, vertex.y, 'ko', markersize=4)
    
    # Draw start and goal
    ax.plot(start[0], start[1], 'go', markersize=8, label='Start')
    ax.plot(goal[0], goal[1], 'ro', markersize=8, label='Goal')
    
    # Find and draw path
    path, distance = self.plan_path(start, goal)
    if path:
        path_x = [p[0] for p in path]
        path_y = [p[1] for p in path]
        ax.plot(path_x, path_y, 'g-', linewidth=3, label=f'Path (dist: {distance:.2f})')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    title = 'Visibility Graph Path Planning'
    if self.robot_radius > 0:
        title += f' - Robot: {self.robot_width:.1f}x{self.robot_length:.1f} (radius: {self.robot_radius:.1f})'
    else:
        title += ' - Point Robot'
    ax.set_title(title)
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    plt.show()
```

**Spiegazione della visualizzazione:**
1. `fig, ax = plt.subplots(1, 1, figsize=(10, 8))` - **Crea figura**: Plot 10x8 pollici
2. **Disegna ostacoli fisici**: Poligoni rossi semi-trasparenti per gli ostacoli reali
3. **Disegna ostacoli inflazionati**: Poligoni arancioni tratteggiati per il C-Space (solo se robot ha dimensioni)
4. **Ricostruisce grafo**: Necessario per la visualizzazione
5. **Disegna archi di visibilità**: Linee blu sottili e trasparenti tra punti visibili
6. **Disegna vertici**: Punti neri piccoli sui vertici degli ostacoli inflazionati
7. **Evidenzia start/goal**: Start verde, goal rosso
8. **Traccia percorso**: Linea verde spessa con distanza nell'etichetta
9. **Titolo dinamico**: Include le dimensioni del robot se presenti
10. **Styling**: Etichette assi, titolo, legenda, griglia, aspect ratio uguale

**Novità rispetto alla versione precedente:**
- **Doppia visualizzazione**: Mostra sia ostacoli fisici che inflazionati
- **Distinzione visiva**: Colori e stili diversi per ostacoli fisici vs C-Space
- **Informazioni robot**: Dimensioni nel titolo
- **Legenda migliorata**: Etichette descrittive per ogni elemento

---

## 🚀 Esempio di Utilizzo

```python
if __name__ == "__main__":
    print("Starting visibility graph example...")
    
    # Create rectangular obstacles
    obstacles = [
        create_rectangular_obstacle(2, 2, 2, 1),
        create_rectangular_obstacle(6, 1, 1, 3)
    ]
    
    print(f"Created {len(obstacles)} rectangular obstacles")
    
    # Define robot dimensions
    robot_width = 0.8   # Robot width
    robot_length = 1.0  # Robot length
    print(f"Robot dimensions: {robot_width} x {robot_length}")
    
    # Create visibility graph planner with robot dimensions
    planner = VisibilityGraph(obstacles, robot_width, robot_length)
    print(f"Created visibility graph planner with robot radius: {planner.robot_radius:.2f}")
    
    # Plan path
    start = (0, 0)
    goal = (8, 8)
    print(f"Planning path from {start} to {goal}")
    
    path, distance = planner.plan_path(start, goal)
    
    if path:
        print(f"Path found with distance: {distance:.2f}")
        print("Path waypoints:")
        for i, point in enumerate(path):
            print(f"  {i}: ({point[0]:.2f}, {point[1]:.2f})")
    else:
        print("No path found!")
    
    # Compare with point robot
    print("\n" + "="*50)
    print("Comparison with point robot:")
    point_planner = VisibilityGraph(obstacles, 0, 0)
    point_path, point_distance = point_planner.plan_path(start, goal)
    
    if point_path:
        print(f"Point robot path distance: {point_distance:.2f}")
        print(f"Difference due to robot size: {distance - point_distance:.2f}")
    
    # Visualize the graph and path
    print("\nGenerating visualization...")
    planner.visualize_graph(start, goal)
    
    print("Example completed!")
```

**Spiegazione dell'esempio:**
1. **Creazione ostacoli**: Due rettangoli come ambiente di test (solo rettangoli supportati)
2. **Definizione dimensioni robot**: Larghezza e lunghezza specifiche
3. **Inizializzazione planner**: Crea l'oggetto VisibilityGraph con dimensioni robot
4. **Definizione problema**: Start (0,0), goal (8,8)
5. **Pianificazione**: Chiama il metodo principale
6. **Output risultati**: Stampa percorso e waypoints
7. **Confronto con robot puntiforme**: Mostra l'effetto delle dimensioni del robot
8. **Visualizzazione**: Mostra il grafico con ostacoli fisici e inflazionati

**Novità rispetto alla versione precedente:**
- **Dimensioni robot**: Specifica larghezza e lunghezza
- **Confronto**: Compara percorsi di robot con dimensioni vs robot puntiforme
- **Solo rettangoli**: Rimuove riferimenti a ostacoli triangolari
- **Output migliorato**: Mostra raggio di inflazione calcolato

---

## 🔍 Complessità e Performance

### ⏱️ Complessità Temporale
- **Costruzione grafo**: O(n³) dove n è il numero di vertici
  - n² coppie di punti da testare
  - Ogni test di visibilità richiede O(n) controlli di intersezione
- **Dijkstra**: O((V + E) log V) dove V = vertici, E = archi
- **Totale**: O(n³) dominante

### 💾 Complessità Spaziale
- **Grafo**: O(n²) nel caso peggiore (grafo completo)
- **Dijkstra**: O(n) per strutture ausiliarie

### 🎯 Proprietà dell'Algoritmo
- **Ottimalità**: Trova sempre il percorso più breve in termini di distanza euclidea
- **Completezza**: Se esiste un percorso, lo trova
- **Determinismo**: Stesso input produce sempre stesso output

---

## ⚠️ Limitazioni e Considerazioni

### 🚫 Limitazioni
1. **Scalabilità**: Complessità cubica limita l'uso su ambienti grandi
2. **Geometrie supportate**: Solo ostacoli rettangolari (no triangoli o forme complesse)
3. **Dinamicità**: Non gestisce ostacoli mobili
4. **Approssimazione robot**: Il robot è approssimato con un cerchio (raggio = max(larghezza, lunghezza)/2)
5. **Sicurezza**: I percorsi "sfiorano" gli ostacoli inflazionati (potenzialmente non sicuro per robot reali)

### 🔧 Possibili Miglioramenti
1. **Reduced Visibility Graph**: Considera solo vertici "essenziali"
2. **Margin di sicurezza aggiuntivo**: Aggiunge margine extra oltre l'inflazione del robot
3. **A* search**: Usa euristica per ridurre spazio di ricerca
4. **Incremental updates**: Aggiorna grafo invece di ricostruirlo
5. **Supporto forme complesse**: Gestione di poligoni non rettangolari
6. **Modelli robot avanzati**: Considerazione di geometrie robot non circolari
7. **Path smoothing**: Post-processing per rendere i percorsi più naturali

---

## 🎓 Conclusioni

Questa implementazione del Visibility Graph fornisce:
- ✅ **Algoritmo completo e funzionante**
- ✅ **Supporto per robot con dimensioni specifiche**
- ✅ **Configuration Space (C-Space) implementation**
- ✅ **Codice ben documentato e leggibile**
- ✅ **Visualizzazione interattiva con doppia rappresentazione**
- ✅ **Gestione errori robusta**
- ✅ **Interfaccia semplice da usare**
- ✅ **Confronto tra robot puntiforme e con dimensioni**

È ideale per:
- 🎯 **Prototipazione rapida**
- 📚 **Scopi educativi** (dimostra concetti di C-Space)
- 🧪 **Testing di algoritmi**
- 🔬 **Ricerca accademica**
- 🤖 **Applicazioni robotiche con ostacoli rettangolari**

**Punti di forza principali:**
- **Approccio Configuration Space**: Gestisce correttamente le dimensioni del robot
- **Visualizzazione educativa**: Mostra chiaramente la differenza tra spazio fisico e C-Space
- **Implementazione robusta**: Dijkstra con tie-breaking per risultati deterministici
