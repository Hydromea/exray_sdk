int64 id           # Unique identifier for the transducer.
float64 velocity   # Velocity measured by the transducer in meters per second (m/s).
float64 distance   # Distance measured from the transducer to the target or surface in meters (m).
float64 rssi       # Received Signal Strength Indicator (RSSI) in decibels (dB), indicating signal strength.
float64 nsd        # Noise-to-Signal Ratio (NSD) in decibels (dB), representing signal quality.
bool valid         # True if the beam is locked on and providing reliable data, False otherwise.