# Motion Loader for Mini Pupper
# Create library of behavior motions
# Can load pre-generated joint positions from a file

import os

class MotionLoader:
    def __init__(self, joint_names):
        self.joint_names = joint_names

        self.default_joint_poses = [
            -0.080311, 1.078067, -1.983427, # FL
            0.080311, 1.078067, -1.983427, # FR
            -0.080311, 1.078067, -1.983427, # BL
            0.080311, 1.078067, -1.983427  # BR
        ]

        self.retarget_motion_folder = os.path.join(os.path.dirname(__file__), "..", "..", "retargeted_motions")
        
        self.states = ["sit", "stand", "lay"]
        self.states_transitions = {}
        for state in self.states:
            self.states_transitions[state] = {}
            for next_state in self.states:
                if next_state != state:
                    self.states_transitions[state][next_state] = None  # Will be loaded on demand
        
        self.pose_events = ["look_up",
                           "look_down",
                           "look_left",
                           "look_right",
                           "look_middle",
                           "shift_left",
                           "shift_right",
                           "move_up",
                           "move_down",
                           "pause",]
        
        self.vel_events = ["move_forward", 
                           "move_backward",
                           "move_left",
                           "move_right",
                           "turn_left",
                           "turn_right",
                           "stop"]

        self.yes= ['look_up',
              'look_middle',
              'look_up',
              'look_middle']
        
        self.no= ['look_right',
                   'look_left',
                   'look_right',
                   'look_middle']

        self.shake= ['shift_left',
                     'shift_right',
                     'shift_left',
                     'shift_right',
                     'look_middle']
                     
        self.bow= ['look_down',
                   'pause',
                   'pause',
                   'look_middle']
        
    def _joints_are_equal(self, joint1List, joint2List, tolerance=1e-5):
        """Check if two joints are equal within a tolerance."""
        if len(joint1List) != len(joint2List):
            return False
        for j1, j2 in zip(joint1List, joint2List):
            if abs(j1 - j2) > tolerance:
                return False
        return True

    def recover_current_state(self, current_joint_positions):
        """Recover the current state from the last known joint positions."""
        # compare the current joint positions with stand, sit, lay poses
        # and return the closest state
        # Closest state is determined by the minimum sum of squared differences
        min_diff = float('inf')
        closest_state = "stand"  # Default fallback
        
        # Check if matches default stand pose exactly
        if self._joints_are_equal(current_joint_positions, self.default_joint_poses):
            return "stand"
        
        # Build state reference poses
        state_poses = []
        
        for state in self.states:
            # Load the transition state for the current state, compare only the last frame
            # If the state is "stand", use the default joint positions
            if state == "stand":
                state_pose = self.get_default_joint_poses()
            else:
                try:
                    # Load the transition state from the file
                    state_transition = self.load_transition_state("stand", state)
                    if state_transition:
                        state_pose = state_transition  # Get the last frame of the transition
                    else:
                        state_pose = self.get_default_joint_poses()  # Fallback to default pose
                except Exception as e:
                    print(f"Error loading state transition from 'stand' to '{state}': {e}")
                    state_pose = None
            try:
                diff = sum((j1 - j2) ** 2 for j1, j2 in zip(current_joint_positions, state_pose))
                if diff < min_diff:
                    min_diff = diff
                    closest_state = state
            except Exception as e:
                print(f"Error comparing joint positions for state '{state}': {e}")
                continue

        return closest_state
    
    def get_closest_transition_frame_idx(self, current_joint_positions, current_state, target_state):
        """Get the index of the closest transition frame for the current state to target state."""
        # Load the transition state
        transition_state = self.load_transition_state(current_state, target_state)
        if not transition_state:
            print(f"No transition state found from {current_state} to {target_state}")
            return 0  # Return 0 instead of -1 for safer indexing
        
        # Find the closest frame
        min_diff = float('inf')
        closest_idx = 0  # Default to first frame
        
        for idx, frame in enumerate(transition_state):
            if len(frame) != len(current_joint_positions):
                print(f"Frame {idx} has wrong length: {len(frame)} vs {len(current_joint_positions)}")
                continue
                
            diff = sum((j1 - j2) ** 2 for j1, j2 in zip(current_joint_positions, frame))
            if diff < min_diff:
                min_diff = diff
                closest_idx = idx
        
        return closest_idx

        
    def get_default_joint_poses(self):
        """ Returns the default joint positions for the Mini Pupper. """
        return self.default_joint_poses.copy()

    def load_joint_positions_from_file(self, file_path=None):
        '''Load joint positions from a txt file comma delimited format.
        file_path: str, path to the file containing joint positions.
        Returns a list of joint positions, each as a list of floats.
        '''
        try:
            print(f"Loading joint positions from: {file_path}")
            with open(file_path, 'r') as file:
                lines = file.readlines()
                all_joint_positions = []
                # Parse header to get joint order in file
                header = lines[0].strip().split(', ')
                # print(f"Header joints: {header}")
                # Build index mapping from header to self.joint_names
                index_map = [header.index(joint) for joint in self.joint_names]
                for idx, line in enumerate(lines):
                    if idx == 0:
                        continue  # skip header
                    line = line.strip()
                    positions = [float(pos) for pos in line.split(',')]
                    # Reorder positions according to self.joint_names
                    reordered = [positions[i] for i in index_map]
                    all_joint_positions.append(reordered)
                return all_joint_positions
        except FileNotFoundError:
            print(f"File not found: {file_path}")
            return []
        
    def load_transition_state(self, from_state, to_state):
        """Load a transition state from a file.
        Args:
            from_state (str): The starting state.
            to_state (str): The target state.
        Returns:
            list: List of joint positions for transition from `from_state` to `to_state`.
        """
        # Check if already loaded
        if (from_state in self.states_transitions and 
            to_state in self.states_transitions[from_state] and
            self.states_transitions[from_state][to_state] is not None):
            return self.states_transitions[from_state][to_state]
            
        # Try to load from file
        for file_name in os.listdir(self.retarget_motion_folder):
            if file_name.startswith(f"{from_state}_{to_state}"):
                try:
                    file_path = os.path.join(self.retarget_motion_folder, file_name)
                    transition_data = self.load_joint_positions_from_file(file_path)
                    # Cache the result
                    if from_state not in self.states_transitions:
                        self.states_transitions[from_state] = {}
                    self.states_transitions[from_state][to_state] = transition_data
                    return transition_data
                except Exception as e:
                    print(f"Error loading motion file {file_path}: {e}")
        
        print(f"No transition file found for {from_state} -> {to_state}")
        return []
