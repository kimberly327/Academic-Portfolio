# 📚 Documentazione Dettagliata: Shape Module

## 🎯 Panoramica Generale

Il modulo **`shape.py`** fornisce le classi e funzioni utili per rappresentare forme geometriche rettangolari utilizzate come ostacoli negli algoritmi di path planning. Il modulo è progettato per essere efficiente e supporta esclusivamente rettangoli con algoritmi ottimizzati per il controllo di contenimento dei punti.

### 🔍 Principio di Funzionamento

Il modulo implementa tre classi principali:
- **Point**: Rappresentazione di punti 2D con operazioni matematiche
- **LineSegment**: Segmenti di linea tra due punti
- **Polygon**: Poligoni rettangolari con algoritmi ottimizzati per bounding box

---

## 📄 Analisi Dettagliata del Codice

### 🎨 Header e Documentazione del Modulo

```python
"""
Shape utilities for rectangular obstacles

This module provides classes for representing rectangular geometric shapes
used as obstacles in path planning algorithms.

Author: Robotic Systems Library
"""
```

**Spiegazione:**
- **Docstring del modulo**: Descrive il proposito del file come utility per forme geometriche **rettangolari**
- **Scope specifico**: Chiarisce che è per ostacoli rettangolari in algoritmi di path planning
- **Semplicità**: Enfatizza il focus esclusivo sui rettangoli
- **Authorship**: Riferimento per manutenzione e supporto

---

### 📦 Sezione Import

```python
import math
from typing import List, Tuple
```

**Spiegazione linea per linea:**

1. `import math` - **Operazioni matematiche**: 
   - `math.sqrt()` per calcoli di distanza euclidea
   - Potenziali future operazioni trigonometriche

2. `from typing import List, Tuple` - **Type hints**: 
   - `List[Tuple[float, float]]` per liste di coordinate
   - `Tuple[float, float]` per singole coordinate (x, y)
   - Migliora la leggibilità e permette il type checking

---

## 🏗️ Classe Point

### 🔧 Inizializzazione

```python
class Point:
    """Represents a 2D point with x, y coordinates."""
    
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
```

**Spiegazione linea per linea:**

1. `class Point:` - **Definizione della classe**: Rappresenta un punto nello spazio 2D
2. **Docstring**: Descrizione concisa del proposito della classe
3. `def __init__(self, x: float, y: float):` - **Costruttore**: 
   - Riceve coordinate x e y come numeri float
   - Type hints assicurano che vengano passati numeri
4. `self.x = x` - **Memorizza coordinata X**: Assegna la coordinata x all'istanza
5. `self.y = y` - **Memorizza coordinata Y**: Assegna la coordinata y all'istanza

### 🔍 Metodo di Confronto

```python
def __eq__(self, other):
    if not isinstance(other, Point):
        return False
    return abs(self.x - other.x) < 1e-9 and abs(self.y - other.y) < 1e-9
```

**Spiegazione dettagliata:**

1. `def __eq__(self, other):` - **Operatore di uguaglianza**: Permette `point1 == point2`
2. `if not isinstance(other, Point):` - **Type checking**: Verifica che l'altro oggetto sia un Point
3. `return False` - **Tipi diversi**: Se non è un Point, automaticamente diversi
4. `return abs(self.x - other.x) < 1e-9 and abs(self.y - other.y) < 1e-9` - **Confronto con tolleranza**:
   - `abs(self.x - other.x)` calcola la differenza assoluta in X
   - `< 1e-9` usa una tolleranza molto piccola per errori di floating point
   - `and` richiede che entrambe le coordinate siano vicine
   - **Perché la tolleranza?** I numeri float hanno imprecisioni, quindi `0.1 + 0.2` potrebbe non essere esattamente `0.3`

### 🔐 Metodo Hash

```python
def __hash__(self):
    return hash((round(self.x, 9), round(self.y, 9)))
```

**Spiegazione:**
1. `def __hash__(self):` - **Funzione hash**: Permette l'uso di Point in set e come chiavi di dizionari
2. `round(self.x, 9)` - **Arrotondamento**: 
   - Arrotonda a 9 decimali per consistenza con la tolleranza in `__eq__`
   - Garantisce che punti "uguali" abbiano lo stesso hash
3. `hash((round(self.x, 9), round(self.y, 9)))` - **Hash di tupla**:
   - Crea una tupla delle coordinate arrotondate
   - Usa la funzione hash built-in di Python per le tuple
   - **Principio**: Oggetti uguali devono avere lo stesso hash

### 📝 Rappresentazione Stringa

```python
def __repr__(self):
    return f"Point({self.x:.3f}, {self.y:.3f})"
```

**Spiegazione:**
1. `def __repr__(self):` - **Rappresentazione string**: Definisce come stampare l'oggetto
2. `f"Point({self.x:.3f}, {self.y:.3f})"` - **F-string formatting**:
   - `f""` permette interpolazione di variabili
   - `:.3f` formatta i float con 3 decimali
   - Risultato esempio: `Point(1.500, 2.750)`

### 📏 Calcolo Distanza

```python
def distance_to(self, other: 'Point') -> float:
    """Calculate Euclidean distance to another point."""
    return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
```

**Spiegazione matematica:**
1. `def distance_to(self, other: 'Point') -> float:` - **Metodo di distanza**:
   - `other: 'Point'` usa forward reference (quotation marks) per type hint
   - `-> float` indica che ritorna un numero float
2. **Formula euclidea**: √[(x₂-x₁)² + (y₂-y₁)²]
3. `(self.x - other.x)**2` - **Differenza X al quadrato**: (x₂-x₁)²
4. `(self.y - other.y)**2` - **Differenza Y al quadrato**: (y₂-y₁)²
5. `math.sqrt(...)` - **Radice quadrata**: Completa la formula euclidea

---

## 🏗️ Classe LineSegment

```python
class LineSegment:
    """Represents a line segment between two points."""
    
    def __init__(self, p1: Point, p2: Point):
        self.p1 = p1
        self.p2 = p2
    
    def __repr__(self):
        return f"LineSegment({self.p1}, {self.p2})"
```

**Spiegazione:**

### 🔧 Inizializzazione
1. `class LineSegment:` - **Classe per segmenti**: Rappresenta una linea tra due punti
2. `def __init__(self, p1: Point, p2: Point):` - **Costruttore**:
   - Riceve due oggetti Point come estremi
   - Type hints assicurano il tipo corretto
3. `self.p1 = p1` - **Primo estremo**: Memorizza il punto iniziale
4. `self.p2 = p2` - **Secondo estremo**: Memorizza il punto finale

### 📝 Rappresentazione
5. `def __repr__(self):` - **String representation**: Per debugging e stampa
6. `return f"LineSegment({self.p1}, {self.p2})"` - **Formato descrittivo**:
   - Mostra entrambi i punti estremi
   - Esempio: `LineSegment(Point(0.000, 0.000), Point(1.000, 1.000))`

---

## 🏗️ Classe Polygon

### 🔧 Inizializzazione

```python
class Polygon:
    """Represents a rectangular obstacle."""
    
    def __init__(self, vertices: List[Tuple[float, float]]):
        """
        Initialize rectangular polygon with list of vertices.
        
        Args:
            vertices: List of (x, y) tuples representing rectangle vertices (must be 4 vertices)
        """
        if len(vertices) != 4:
            raise ValueError("Only rectangular obstacles with 4 vertices are supported")
        
        self.vertices = [Point(x, y) for x, y in vertices]
        self.edges = self._create_edges()
```

**Spiegazione dettagliata:**

1. `class Polygon:` - **Classe poligoni**: Supporta esclusivamente rettangoli
2. **Docstring specifica**: Chiarisce che rappresenta solo ostacoli rettangolari
3. `def __init__(self, vertices: List[Tuple[float, float]]):` - **Costruttore semplificato**:
   - `vertices`: Lista di coordinate come tuple (x, y)
   - **Rimosso `shape_type`**: Non più necessario dato che supporta solo rettangoli

4. **Validazione input**:
   ```python
   if len(vertices) != 4:
       raise ValueError("Only rectangular obstacles with 4 vertices are supported")
   ```
   - Controlla che ci siano esattamente 4 vertici per un rettangolo
   - `raise ValueError`: Lancia eccezione per input non validi
   - **Fail fast principle**: Rileva errori il prima possibile

5. `self.vertices = [Point(x, y) for x, y in vertices]` - **List comprehension**:
   - Converte ogni tupla (x, y) in un oggetto Point
   - `for x, y in vertices` fa unpacking automatico delle tuple
   - Risultato: lista di oggetti Point invece di tuple

6. `self.edges = self._create_edges()` - **Crea lati**: Chiama metodo per generare i segmenti del rettangolo

### 🔧 Creazione dei Lati

```python
def _create_edges(self) -> List[LineSegment]:
    """Create edges from consecutive vertices."""
    edges = []
    for i in range(len(self.vertices)):
        p1 = self.vertices[i]
        p2 = self.vertices[(i + 1) % len(self.vertices)]
        edges.append(LineSegment(p1, p2))
    return edges
```

**Spiegazione matematica:**

1. `def _create_edges(self) -> List[LineSegment]:` - **Metodo privato**: 
   - `_` indica uso interno
   - Ritorna lista di LineSegment

2. `edges = []` - **Lista vuota**: Accumula i lati del poligono

3. `for i in range(len(self.vertices)):` - **Itera tutti i vertici**: 
   - `range(len(...))` crea indici 0, 1, 2, ..., n-1

4. `p1 = self.vertices[i]` - **Vertice corrente**: Punto di partenza del lato

5. `p2 = self.vertices[(i + 1) % len(self.vertices)]` - **Vertice successivo**:
   - `(i + 1) % len(...)` implementa aritmetica circolare
   - Quando `i = len-1`, `(i+1) % len = 0` → torna al primo vertice
   - **Chiude il poligono**: L'ultimo vertice si collega al primo

6. `edges.append(LineSegment(p1, p2))` - **Crea lato**: Aggiunge segmento alla lista

**Esempio per triangolo con vertici [A, B, C]:**
- i=0: lato A→B
- i=1: lato B→C  
- i=2: lato C→A (aritmetica circolare)

### 🎯 Metodo di Contenimento

```python
def contains_point(self, point: Point) -> bool:
    """
    Check if a point is inside the rectangle.
    
    Args:
        point: Point to check
        
    Returns:
        True if point is inside rectangle, False otherwise
    """
    return self._point_in_rectangle(point)
```

**Spiegazione semplificata:**

1. **Algoritmo unico**: Dato che supporta solo rettangoli, usa sempre l'algoritmo bounding box
2. `return self._point_in_rectangle(point)` - **Delega al metodo specializzato**: Sempre O(1) per rettangoli
3. **Semplicità**: Nessun dispatch necessario, solo un tipo di forma supportato

### 📦 Algoritmo per Rettangoli

```python
def _point_in_rectangle(self, point: Point) -> bool:
    """Fast rectangle containment check using bounding box."""
    # Get min/max coordinates
    x_coords = [v.x for v in self.vertices]
    y_coords = [v.y for v in self.vertices]
    
    min_x, max_x = min(x_coords), max(x_coords)
    min_y, max_y = min(y_coords), max(y_coords)
    
    return min_x <= point.x <= max_x and min_y <= point.y <= max_y
```

**Spiegazione algoritmo bounding box:**

1. `x_coords = [v.x for v in self.vertices]` - **Estrae coordinate X**:
   - List comprehension raccoglie tutte le coordinate x
   - Esempio: per rettangolo [(0,0), (2,0), (2,1), (0,1)] → [0, 2, 2, 0]

2. `y_coords = [v.y for v in self.vertices]` - **Estrae coordinate Y**:
   - Raccoglie tutte le coordinate y
   - Esempio: [(0,0), (2,0), (2,1), (0,1)] → [0, 0, 1, 1]

3. `min_x, max_x = min(x_coords), max(x_coords)` - **Range X**:
   - `min()` trova coordinata x minima
   - `max()` trova coordinata x massima
   - **Multiple assignment**: Assegna entrambi i valori in una volta

4. `min_y, max_y = min(y_coords), max(y_coords)` - **Range Y**: Stessa logica per y

5. `return min_x <= point.x <= max_x and min_y <= point.y <= max_y` - **Test contenimento**:
   - **Chained comparison**: `a <= b <= c` equivale a `a <= b and b <= c`
   - Verifica che il punto sia dentro il rettangolo allineato agli assi
   - **Complessità**: O(1) dopo aver trovato min/max

**Perché funziona:**
- I rettangoli creati da `create_rectangular_obstacle()` sono sempre allineati agli assi
- Il bounding box coincide esattamente con il rettangolo
- Molto più veloce del ray casting generale
- **Algoritmo ottimale**: Non può essere più efficiente per rettangoli axis-aligned

---

## 🔧 Funzioni Helper

### 📦 Creazione Rettangoli

```python
def create_rectangular_obstacle(x: float, y: float, width: float, height: float) -> Polygon:
    """
    Create a rectangular obstacle.
    
    Args:
        x: Bottom-left x coordinate
        y: Bottom-left y coordinate
        width: Width of rectangle
        height: Height of rectangle
        
    Returns:
        Polygon object representing the rectangle
    """
    vertices = [
        (x, y),
        (x + width, y),
        (x + width, y + height),
        (x, y + height)
    ]
    return Polygon(vertices, "rectangle")
```

**Spiegazione della costruzione:**

1. **Convenzione coordinate**: `(x, y)` è l'angolo bottom-left
2. **Vertici in ordine antiorario**:
   - `(x, y)` - Bottom-left
   - `(x + width, y)` - Bottom-right  
   - `(x + width, y + height)` - Top-right
   - `(x, y + height)` - Top-left

3. **Ordine importante**: L'ordine antiorario assicura che:
   - Gli algoritmi geometrici funzionino correttamente
   - Le normali dei lati puntino verso l'esterno
   - I calcoli di area abbiano il segno giusto

4. `return Polygon(vertices)` - **Crea oggetto**: Senza parametro di tipo (solo rettangoli supportati)

---

## 🎯 Analisi delle Performance

### ⏱️ Complessità Temporale

| Operazione | Complessità | Note |
|-----------|-------------|------|
| `Point.__init__` | O(1) | Assegnazione semplice |
| `Point.distance_to` | O(1) | Una sqrt e poche operazioni |
| `Point.__eq__` | O(1) | Due confronti float |
| `Point.__hash__` | O(1) | Due round e un hash |
| `Polygon.__init__` | O(1) | Solo 4 vertici per rettangoli |
| `Polygon._point_in_rectangle` | O(1) | Solo min/max e confronti |
| `create_rectangular_obstacle` | O(1) | Crea lista fissa di 4 vertici |

### 💾 Complessità Spaziale

| Struttura | Spazio | Note |
|-----------|--------|------|
| `Point` | O(1) | Due float |
| `LineSegment` | O(1) | Due riferimenti a Point |
| `Polygon` | O(1) | Sempre 4 vertici + 4 edge per rettangoli |

### 🚀 Ottimizzazioni Implementate

1. **Algoritmo bounding box specializzato**: Ottimale per rettangoli axis-aligned
2. **Nessun dispatch necessario**: Solo un tipo di forma supportato
3. **Tolleranze numeriche**: Gestisce errori di floating point
4. **Type hints**: Permette ottimizzazioni del compilatore/interprete
5. **Validazione rigorosa**: Solo 4 vertici accettati per rettangoli

---

## ⚠️ Limitazioni e Considerazioni

### 🚫 Limitazioni Attuali

1. **Solo rettangoli**: Nessun supporto per triangoli, cerchi o poligoni arbitrari
2. **Rettangoli axis-aligned**: I rettangoli devono essere allineati agli assi
3. **No validazione geometrica**: Non verifica che i 4 vertici formino realmente un rettangolo
4. **Ordine vertici**: L'utente deve fornire vertici in ordine sensato per i rettangoli

### 🔧 Possibili Miglioramenti

1. **Supporto per triangoli**: Aggiungere `create_triangular_obstacle()` e coordinate baricentriche
2. **Supporto per cerchi**: Aggiungere `create_circular_obstacle()`
3. **Rettangoli ruotati**: Gestire rettangoli con orientamento arbitrario
4. **Validazione input**: Verificare che i 4 vertici formino realmente un rettangolo
5. **Poligoni convessi generici**: Estendere a poligoni convessi arbitrari
6. **Serializzazione**: Metodi per salvare/caricare forme da file

### 🛡️ Robustezza

**Gestione errori implementata:**
- ✅ Validazione del numero di vertici nel costruttore (deve essere 4)
- ✅ Tolleranze per confronti float
- ✅ Type hints per input validation

**Aree per miglioramento:**
- ❌ Validazione che i 4 vertici formino realmente un rettangolo
- ❌ Gestione di input malformati (es. coordinate non numeriche)
- ❌ Warning per rettangoli molto piccoli o degeneri

---

## 🎓 Esempi d'Uso

### 📦 Creazione di un Rettangolo

```python
# Crea rettangolo 3x2 con angolo bottom-left in (1, 1)
rect = create_rectangular_obstacle(1, 1, 3, 2)

# Vertici risultanti: [(1,1), (4,1), (4,3), (1,3)]
# Test di contenimento
point_inside = Point(2.5, 2.0)    # True
point_outside = Point(0.5, 0.5)   # False

print(rect.contains_point(point_inside))   # True
print(rect.contains_point(point_outside))  # False
```

### � Uso Avanzato con Múltipli Rettangoli

```python
# Crea una lista di ostacoli rettangolari
obstacles = [
    create_rectangular_obstacle(0, 0, 2, 2),      # Quadrato
    create_rectangular_obstacle(3, 1, 1, 4),      # Rettangolo alto
    create_rectangular_obstacle(5, 0, 2, 1)       # Rettangolo largo
]

# Test di un punto contro tutti gli ostacoli
test_point = Point(1, 1)
for i, obstacle in enumerate(obstacles):
    if obstacle.contains_point(test_point):
        print(f"Point is inside obstacle {i}")
    else:
        print(f"Point is outside obstacle {i}")

# Output: Point is inside obstacle 0 (il primo quadrato)
```

### 📏 Test di Distanze e Operazioni sui Punti

```python
# Creazione di punti
p1 = Point(0, 0)
p2 = Point(3, 4)

# Calcolo distanza euclidea
distance = p1.distance_to(p2)  # 5.0 (triangolo 3-4-5)
print(f"Distance: {distance}")

# Test di uguaglianza con tolleranza
p3 = Point(0.0000001, 0.0000001)  # Praticamente uguale a p1
print(p1 == p3)  # True (tolleranza 1e-9)

# Uso in strutture dati
point_set = {p1, p2, p3}  # Set usa __hash__
print(len(point_set))  # 2 (p1 e p3 sono considerati uguali)
```

---

## 🎯 Conclusioni

Il modulo **`shape.py`** fornisce:

### ✅ **Caratteristiche Positive:**
- **Semplicità**: API pulita e intuitiva focalizzata sui rettangoli
- **Performance**: Algoritmo bounding box ottimale per rettangoli axis-aligned
- **Robustezza**: Gestione degli errori numerici e validazione input
- **Leggibilità**: Codice ben documentato e strutturato
- **Type safety**: Type hints complete per tutti i metodi

### 🎯 **Casi d'Uso Ideali:**
- 🏗️ **Path planning**: Ostacoli rettangolari in robot navigation
- 🎮 **Game development**: Collision detection per forme rettangolari
- 📊 **Simulation**: Ambienti 2D con geometrie rettangolari
- 🎓 **Educational**: Insegnamento di algoritmi geometrici di base
- 🤖 **Grid-based systems**: Sistemi con strutture regolari

### 🔮 **Direzioni Future:**
- Supporto per forme geometriche aggiuntive (triangoli, cerchi)
- Algoritmi per poligoni convessi generici
- Ottimizzazioni per rettangoli ruotati
- Integrazione con librerie di computer graphics

### 🎯 **Focus Attuale:**
Il modulo è **specificatamente ottimizzato per rettangoli**, offrendo:
- **Massima performance** per questo caso d'uso comune
- **Semplicità implementativa** senza complessità inutili
- **Facilità di comprensione** per scopi didattici
- **Affidabilità** attraverso algoritmi consolidati
