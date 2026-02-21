"""
TelemetryService - Centralized logging service that publishes to NetworkTables
on a separate thread using a Notifier to avoid loop overruns.

Subsystems register data collection callbacks that are called every 100ms
instead of every 20ms robot loop cycle.
"""

from typing import Callable, Dict
from wpilib import Notifier
from ntcore import NetworkTableInstance
import wpiutil.log


class TelemetryService:
    """
    Singleton service that manages telemetry publishing on a separate thread.
    Reduces main loop overhead by batching NetworkTables operations.
    """
    
    _instance = None
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(TelemetryService, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance
    
    def __init__(self, update_period: float = 0.1):
        """
        Initialize the telemetry service.
        
        :param update_period: How often to publish data in seconds (default: 0.1 = 100ms)
        """
        if self._initialized:
            return
            
        self._initialized = True
        self.update_period = update_period
        self._callbacks: Dict[str, Callable[[], None]] = {}
        self._enabled = True
        
        # Create notifier that runs on separate thread
        self._notifier = Notifier(self._publish_all)
        
    def start(self):
        """Start the telemetry publishing notifier."""
        if self._enabled:
            self._notifier.startPeriodic(self.update_period)
            print(f"[TelemetryService] Started with {self.update_period*1000}ms period")
    
    def stop(self):
        """Stop the telemetry publishing notifier."""
        self._notifier.stop()
        print("[TelemetryService] Stopped")
    
    def register_subsystem(self, name: str, callback: Callable[[], None]):
        """
        Register a subsystem's telemetry callback.
        
        :param name: Unique name for the subsystem
        :param callback: Function to call that publishes telemetry data
        """
        self._callbacks[name] = callback
        print(f"[TelemetryService] Registered subsystem: {name}")
    
    def unregister_subsystem(self, name: str):
        """
        Unregister a subsystem's telemetry callback.
        
        :param name: Name of the subsystem to remove
        """
        if name in self._callbacks:
            del self._callbacks[name]
            print(f"[TelemetryService] Unregistered subsystem: {name}")
    
    def _publish_all(self):
        """Called by notifier - executes all registered callbacks."""
        if not self._enabled:
            return
            
        for name, callback in self._callbacks.items():
            try:
                callback()
            except Exception as e:
                print(f"[TelemetryService] Error in {name} telemetry: {e}")
    
    def set_enabled(self, enabled: bool):
        """Enable or disable telemetry publishing."""
        self._enabled = enabled
        if enabled:
            print("[TelemetryService] Enabled")
        else:
            print("[TelemetryService] Disabled")


# Global singleton instance
_telemetry_service = TelemetryService()


def get_telemetry_service() -> TelemetryService:
    """Get the singleton TelemetryService instance."""
    return _telemetry_service
