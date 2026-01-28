from typing import overload, final
from commands2 import Subsystem
from ntcore import NetworkTableInstance
from wpimath.geometry import * # Pose3d, Rotation3d, Translation3d, Transform3d

class Fuel:
    _position:Pose3d
    _velocity:Translation3d

    def __init__(self, position:Pose3d, velocity:Translation3d):
        self._position = position
        self._velocity = velocity

    def periodic(self):
        # START: Updating Position
        ## Create Delta Velociies
        dx = self._velocity.X() * 0.02
        dy = self._velocity.Y() * 0.02
        dz = self._velocity.Z() * 0.02 

        ## Adjust Delta Z for Floor
        dz = dz if abs( dz ) < self._position.Z() else max( -self._position.Z(), dz )

        ## Update Position
        veloc = Twist3d( dx, dy, dz, 0, 0, 0 )
        self._position = self._position.exp( veloc )

        # START: Updating Velocity
        ## Determine Air or Ground Resistance
        inAir = self._position.Z() > 0.0
        resistance = 0.005 if inAir else 0.025

        ## Update X Velocities (Air/Ground Resistance on Fuel)
        vx_res = resistance * ( 1 if self._velocity.X() > 0 else -1 ) # Produces 1 or -1
        vx = self._velocity.X() - vx_res
        vx = vx if abs( vx ) > 0.01 else 0.0  # Converge to 0.0

        ## Update Y Velocities (Air/Ground Resistance on Fuel)
        vy_res = resistance * ( 1 if self._velocity.Y() > 0 else -1 )  # Produces 1 or -1
        vy = self._velocity.Y() - vy_res
        vy = vy if abs( vy ) > 0.01 else 0.0  # Converge to 0.0

        ## Update Z Velocities (Gravity + Air Resistance on Fuel)
        vz = self._velocity.Z() * ( 1.0 if self._position.Z() != 0.0 else -0.8 ) # Bounce off ground
        vz_res = resistance * ( 1 if self._velocity.Z() >= 0.0 else -1 ) # Air / Ground Resistance
        vg = - 9.8 * 0.02 - vz_res # gravity
        vz = vz + vg
        vz = vz if not (self._position.Z() == 0.0 and vz < -vg) else 0.0 # Convrege to 0.0
        self._velocity = Translation3d( vx, vy, vz )
        
    def updatePosition(self, position:Pose3d):
        self._position = position

    def getPosition(self):
        return self._position
    
    def getVelocity(self):
        return self._velocity

class FuelVisualizer(Subsystem):

    _nt = NetworkTableInstance.getDefault().getTable("111Fuel")
    _log = _nt.getStructArrayTopic( "Fuel", Pose3d ).publish()
    _logV = _nt.getStructArrayTopic( "Velocity", Translation3d ).publish()
    _fuel: list[Fuel] = []

    def __init__(self):
        pass

    def periodic(self):
        poses: list[Pose3d] = []
        velocs: list[Translation3d] = []

        for _ in self._fuel:
            _.periodic()
            poses.append( _.getPosition() )
            velocs.append( _.getVelocity() )
        
        self._log.set( poses )
        self._logV.set( velocs )

        #print( f"Count: {len(self._fuel)}")

    def addFuel( self, fuel: Fuel ):
        self._fuel.append( fuel )

    def newFuel(self, pose: Pose3d, velocity: Translation3d):
        f = Fuel( pose, velocity )
        self.addFuel( f )

    def randomFuel(self) -> None:
        from random import randint

        x: float = randint( 0, 16500) / 1000
        y: float = randint( 0, 8170 ) / 1000
        z: float = randint( 0, 3000 ) / 1000
        vx: float = randint( -2000, 2000 ) / 1000
        vy: float = randint( -2000, 2000 ) / 1000
        vz: float = randint( -2000, 2000 ) / 1000
        pose = Pose3d(x, y, z, Rotation3d() )
        veloc = Translation3d( vx, vy , vz )
        self.newFuel( pose, veloc )
