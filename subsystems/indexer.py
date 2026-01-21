import wpilib
import wpilib.drive 

class Indexer (SubsystemBase):
    def __init__(self):
        super().__init__()
        #Put motors and sensors here
        self.indexer_motor = Spark(motor_port)
        self.top_sensor = DigitalInput(top_sensor_port)
        self.bottom_sensor = DigitalInput(bottom_sensor_port)
        #Limit Switch goes here but i dont know how to write it 

    def run_indexer(self, speed: float):
        #Add logic here to prevent movement if specific conditions are met (e.g., if full)
        self.indexer_motor.set(speed)

    def stop_indexer(self):
        self.indexer_motor.set(0)

    def is_loaded(self):
        return self.limit_switch_loaded.get()
    
    def run_forward(self, speed: float):
        #Runs the indexer motor forward
        self.indexer_motor.set(speed) 

    def run_backwards(self, speed: float):
        #Runs the indexer motor backwards
        self.indexer_motor.set(speed) 