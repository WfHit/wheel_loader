# Load Analysis Module

Creates a load analysis module that runs on the main board (X7+) to:
1. Subscribe to hbridge_status from both front and rear boards via distributed uORB
2. Analyze motor load data from all H-bridge instances
3. Generate load_lamp_command messages
4. Send commands to rear board via distributed uORB

## Message Flow

```
Front Board H-Bridge ─┐
                     ├─→ uORB Proxy ─→ Main Board (X7+)
Rear Board H-Bridge ─┘                      │
                                           ▼
                                    Load Analysis
                                           │
                                           ▼
                              load_lamp_command
                                           │
                                           ▼
                               uORB Proxy ─→ Rear Board
                                           │
                                           ▼
                                 LoadLampController
```

## Implementation

The load analysis module should:

1. **Subscribe to hbridge_status**:
   - Use SubscriptionMultiArray to receive status from all instances
   - Process data from both front and rear board H-bridges

2. **Load Calculation**:
   - Calculate average load across all active H-bridge channels
   - Apply smoothing/filtering to prevent rapid lamp changes
   - Map load values to lamp command levels

3. **Command Generation**:
   - Publish load_lamp_command with appropriate load level
   - Include blink interval and load value for reference

4. **Board-specific Deployment**:
   - Run only on main board (X7+)
   - Route commands to rear board via distributed uORB

This architecture provides:
- **Centralized Load Analysis**: All motor load data processed on main board
- **Distributed Control**: Physical lamp control on rear board where hardware exists
- **Clean Separation**: Load analysis and lamp control are decoupled
- **Network Efficiency**: Only commands (not raw data) sent to rear board
