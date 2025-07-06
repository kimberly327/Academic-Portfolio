import math
from lib.system.trajectory import StraightLine2DMotion, VirtualRobot


class AbstractMovement:
    """
    Classe astratta per i movimenti del drone.
    Definisce l'interfaccia comune per tutti i tipi di movimento.
    """
    
    def __init__(self, _robot):
        self.robot = _robot
        
    def start(self):
        """Inizializza il movimento. Da implementare nelle classi derivate."""
        pass
        
    def evaluate(self, delta_t):
        """Valuta il movimento per il time step corrente. Da implementare nelle classi derivate."""
        pass
    
    def movement_done(self):
        """Verifica se il movimento è completato. Da implementare nelle classi derivate."""
        return False

        

class XYMovement(AbstractMovement):
    """
    Movimento orizzontale del drone nel piano XY.
    Sposta il drone verso le coordinate target usando una traiettoria rettilinea.
    """
    
    def __init__(self, _robot, _x_target, _y_target):
        super().__init__(_robot)
        self.x_target = _x_target
        self.y_target = _y_target
        self.virtual_drone = StraightLine2DMotion(2.0, 4.0, 4.0)
        
    def start(self):
        """Inizializza la traiettoria rettilinea dalla posizione corrente al target."""
        self.virtual_drone.start_motion( (self.robot.x, self.robot.y),
                                         (self.x_target, self.y_target))
        
    def evaluate(self, delta_t):
        """Calcola la posizione target corrente lungo la traiettoria."""
        (x_t, y_t) = self.virtual_drone.evaluate(delta_t)
        self.robot.x_target = x_t
        self.robot.y_target = y_t
        
    def movement_done(self):
        """Il movimento è completato quando la distanza dal target è < 1cm."""
        dx = self.x_target - self.robot.x
        dy = self.y_target - self.robot.y
        dist = math.sqrt(dx*dx + dy*dy)
        return dist < 0.01
        
