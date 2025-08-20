#!/usr/bin/env python3
"""
SmolVLA Dual-Mode Controller Usage Example

This example demonstrates how to interact with the dual-mode wheel loader controller
for both manual and autonomous operation modes.
"""

import time
import struct

# This would use actual uORB Python bindings in a real PX4 environment
# For now, this serves as documentation and example structure

class SmolVLAOutputExample:
    """Example SmolVLA output publisher"""
    
    def __init__(self):
        self.sequence_id = 0
        
    def publish_load_sequence(self):
        """Publish a complete auto load sequence"""
        print("Starting auto load sequence...")
        
        # Step 0: Approach material
        self.publish_smol_vla_output(
            bucket_pos=(1.5, 0.0, -0.2),  # Approach position
            bucket_orient=(0.0, -0.3, 0.0),  # Bucket tilted for digging
            vehicle_pos=(2.0, 0.0),
            operation_mode=2,  # OPERATION_LOAD
            sequence_step=0,
            confidence=0.9
        )
        time.sleep(2)
        
        # Step 1: Engage with material
        self.publish_smol_vla_output(
            bucket_pos=(1.2, 0.0, -0.3),  # Lower position
            bucket_orient=(0.0, 0.1, 0.0),  # Start curling bucket
            vehicle_pos=(1.8, 0.0),
            operation_mode=2,  # OPERATION_LOAD
            sequence_step=1,
            confidence=0.85
        )
        time.sleep(3)
        
        # Step 2: Complete load
        self.publish_smol_vla_output(
            bucket_pos=(1.0, 0.0, 0.5),  # Lift position
            bucket_orient=(0.0, 0.4, 0.0),  # Bucket curled to retain material
            vehicle_pos=(1.5, 0.0),
            operation_mode=2,  # OPERATION_LOAD
            sequence_step=2,
            confidence=0.9,
            sequence_complete=True
        )
        
        print("Auto load sequence complete!")
        
    def publish_dump_sequence(self):
        """Publish a complete auto dump sequence"""
        print("Starting auto dump sequence...")
        
        # Step 0: Approach dump location
        self.publish_smol_vla_output(
            bucket_pos=(1.0, 0.0, 0.5),  # Maintain loaded position
            bucket_orient=(0.0, 0.4, 0.0),  # Keep material contained
            vehicle_pos=(5.0, 0.0),  # Move to dump area
            operation_mode=4,  # OPERATION_DUMP
            sequence_step=0,
            confidence=0.9
        )
        time.sleep(2)
        
        # Step 1: Position for dump
        self.publish_smol_vla_output(
            bucket_pos=(1.0, 0.0, 1.2),  # Raise for dumping
            bucket_orient=(0.0, 0.4, 0.0),  # Still contained
            vehicle_pos=(5.5, 0.0),
            operation_mode=4,  # OPERATION_DUMP
            sequence_step=1,
            confidence=0.85
        )
        time.sleep(1)
        
        # Step 2: Execute dump
        self.publish_smol_vla_output(
            bucket_pos=(1.0, 0.0, 1.2),  # Maintain height
            bucket_orient=(0.0, -0.2, 0.0),  # Tip bucket to dump
            vehicle_pos=(5.5, 0.0),
            operation_mode=4,  # OPERATION_DUMP
            sequence_step=2,
            confidence=0.9
        )
        time.sleep(2)
        
        # Step 3: Complete dump
        self.publish_smol_vla_output(
            bucket_pos=(1.0, 0.0, 0.8),  # Lower slightly
            bucket_orient=(0.0, -0.3, 0.0),  # Full tip to ensure complete dump
            vehicle_pos=(5.5, 0.0),
            operation_mode=4,  # OPERATION_DUMP
            sequence_step=3,
            confidence=0.9,
            sequence_complete=True
        )
        
        print("Auto dump sequence complete!")
        
    def publish_smol_vla_output(self, bucket_pos, bucket_orient, vehicle_pos, 
                                operation_mode, sequence_step, confidence, 
                                sequence_complete=False):
        """Publish SmolVLA output message"""
        timestamp = int(time.time() * 1e6)  # microseconds
        
        msg_data = {
            'timestamp': timestamp,
            'bucket_position_x': bucket_pos[0],
            'bucket_position_y': bucket_pos[1], 
            'bucket_position_z': bucket_pos[2],
            'bucket_orientation_roll': bucket_orient[0],
            'bucket_orientation_pitch': bucket_orient[1],
            'bucket_orientation_yaw': bucket_orient[2],
            'vehicle_position_x': vehicle_pos[0],
            'vehicle_position_y': vehicle_pos[1],
            'vehicle_heading': 0.0,
            'operation_mode': operation_mode,
            'confidence_score': confidence,
            'valid_output': True,
            'emergency_stop': False,
            'sequence_id': self.sequence_id,
            'sequence_step': sequence_step,
            'sequence_complete': sequence_complete
        }
        
        print(f"SmolVLA Output: mode={operation_mode}, step={sequence_step}, "
              f"bucket=({bucket_pos[0]:.1f},{bucket_pos[1]:.1f},{bucket_pos[2]:.1f}), "
              f"conf={confidence:.2f}")
        
        # In real implementation: orb.publish('smol_vla_output', msg_data)

class OperationModeController:
    """Example operation mode controller"""
    
    def request_mode_switch(self, target_mode, emergency=False):
        """Request mode switch"""
        timestamp = int(time.time() * 1e6)
        
        mode_cmd = {
            'timestamp': timestamp,
            'operation_mode': target_mode,  # 0=MANUAL, 1=AUTO
            'mode_switch_request': True,
            'manual_override_active': False,
            'emergency_mode_switch': emergency,
            'mode_switch_approved': False,
            'system_ready_for_auto': True,
            'current_active_mode': 0,  # Will be updated by controller
            'previous_mode': 0,
            'mode_change_time': timestamp
        }
        
        mode_name = "AUTO" if target_mode == 1 else "MANUAL"
        print(f"Requesting mode switch to {mode_name} {'(EMERGENCY)' if emergency else ''}")
        
        # In real implementation: orb.publish('operation_mode_command', mode_cmd)
        
    def activate_manual_override(self):
        """Activate manual override"""
        timestamp = int(time.time() * 1e6)
        
        mode_cmd = {
            'timestamp': timestamp,
            'operation_mode': 0,  # MANUAL
            'mode_switch_request': False,
            'manual_override_active': True,
            'emergency_mode_switch': False,
            'mode_switch_approved': True,
            'system_ready_for_auto': False,
            'current_active_mode': 0,
            'previous_mode': 1,  # Was AUTO
            'mode_change_time': timestamp
        }
        
        print("Manual override ACTIVATED!")
        
        # In real implementation: orb.publish('operation_mode_command', mode_cmd)

def demonstrate_dual_mode_operation():
    """Demonstrate complete dual-mode operation"""
    print("SmolVLA Dual-Mode Controller Demonstration")
    print("=" * 50)
    
    smol_vla = SmolVLAOutputExample()
    mode_controller = OperationModeController()
    
    # 1. Start in manual mode
    print("\n1. Starting in MANUAL mode...")
    time.sleep(1)
    
    # 2. Switch to auto mode
    print("\n2. Switching to AUTO mode...")
    mode_controller.request_mode_switch(target_mode=1)  # AUTO
    time.sleep(1)
    
    # 3. Execute auto load sequence
    print("\n3. Executing auto load sequence...")
    smol_vla.publish_load_sequence()
    time.sleep(1)
    
    # 4. Transport material (simplified)
    print("\n4. Transporting material...")
    smol_vla.publish_smol_vla_output(
        bucket_pos=(1.0, 0.0, 0.5),
        bucket_orient=(0.0, 0.4, 0.0),
        vehicle_pos=(3.0, 0.0),
        operation_mode=3,  # OPERATION_TRANSPORT
        sequence_step=0,
        confidence=0.9
    )
    time.sleep(2)
    
    # 5. Execute auto dump sequence
    print("\n5. Executing auto dump sequence...")
    smol_vla.publish_dump_sequence()
    time.sleep(1)
    
    # 6. Demonstrate manual override
    print("\n6. Demonstrating manual override...")
    mode_controller.activate_manual_override()
    time.sleep(1)
    
    # 7. Emergency mode switch
    print("\n7. Demonstrating emergency mode switch...")
    mode_controller.request_mode_switch(target_mode=0, emergency=True)  # Emergency to MANUAL
    time.sleep(1)
    
    print("\nDemonstration complete!")
    print("\nKey Features Demonstrated:")
    print("- Mode switching between manual and auto")
    print("- Auto load sequence with step progression")
    print("- Auto dump sequence with step progression")
    print("- Manual override capability")
    print("- Emergency mode switching")
    print("- SmolVLA confidence-based validation")

if __name__ == '__main__':
    demonstrate_dual_mode_operation()