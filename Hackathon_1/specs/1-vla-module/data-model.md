# Data Model: Vision-Language-Action (VLA) Module

## Core Entities

### VoiceCommand
**Description**: Natural language input from user that specifies desired robot behavior

**Fields**:
- `id` (string): Unique identifier for the command
- `text` (string): Transcribed speech text
- `timestamp` (datetime): When the command was received
- `confidence` (float): Confidence score of speech recognition (0.0-1.0)
- `raw_audio` (bytes): Optional raw audio data for processing
- `status` (enum): ['pending', 'processing', 'completed', 'failed']

**Validation Rules**:
- `text` must be non-empty
- `confidence` must be between 0.0 and 1.0
- `status` must be one of the allowed values

### TaskPlan
**Description**: Structured sequence of sub-goals and actions generated from natural language commands

**Fields**:
- `id` (string): Unique identifier for the plan
- `command_id` (string): Reference to the original voice command
- `tasks` (array): List of Task objects in execution order
- `status` (enum): ['pending', 'executing', 'completed', 'failed', 'cancelled']
- `created_at` (datetime): When the plan was generated
- `updated_at` (datetime): When the plan was last updated
- `estimated_duration` (int): Estimated execution time in seconds

**Validation Rules**:
- `tasks` array must not be empty
- `status` must be one of the allowed values
- `estimated_duration` must be positive

### Task
**Description**: Individual unit of work within a task plan

**Fields**:
- `id` (string): Unique identifier for the task
- `type` (enum): ['navigation', 'manipulation', 'perception', 'other']
- `description` (string): Human-readable description of the task
- `parameters` (object): Task-specific parameters (e.g., target location, object name)
- `dependencies` (array): List of task IDs that must complete before this task
- `timeout` (int): Maximum time to complete the task in seconds

**Validation Rules**:
- `type` must be one of the allowed values
- `description` must be non-empty
- `timeout` must be positive

### PerceptionData
**Description**: Visual and sensory information from the environment processed by computer vision

**Fields**:
- `id` (string): Unique identifier for the perception data
- `timestamp` (datetime): When the data was captured
- `sensor_type` (enum): ['camera', 'lidar', 'depth', 'other']
- `data` (object): Sensor-specific data structure
- `objects` (array): List of detected objects
- `environment_state` (object): Current state of the environment

**Validation Rules**:
- `sensor_type` must be one of the allowed values
- `timestamp` must be in the past or present

### DetectedObject
**Description**: Object detected by the vision system

**Fields**:
- `id` (string): Unique identifier for the detected object
- `name` (string): Object classification label
- `confidence` (float): Confidence score of detection (0.0-1.0)
- `position` (object): 3D coordinates {x, y, z}
- `bounding_box` (object): 2D coordinates {x, y, width, height}
- `properties` (object): Additional object properties

**Validation Rules**:
- `confidence` must be between 0.0 and 1.0
- `name` must be non-empty

### ActionSequence
**Description**: Ordered list of ROS 2 commands that execute the planned behavior

**Fields**:
- `id` (string): Unique identifier for the action sequence
- `plan_id` (string): Reference to the parent task plan
- `actions` (array): List of Action objects in execution order
- `status` (enum): ['pending', 'executing', 'completed', 'failed', 'interrupted']
- `current_action_index` (int): Index of currently executing action
- `execution_log` (array): Log of executed actions with timestamps

**Validation Rules**:
- `actions` array must not be empty
- `status` must be one of the allowed values
- `current_action_index` must be within the range of the actions array

### Action
**Description**: Individual ROS 2 action to be executed

**Fields**:
- `id` (string): Unique identifier for the action
- `action_type` (enum): ['navigation', 'manipulation', 'service_call', 'topic_publish']
- `parameters` (object): Action-specific parameters
- `timeout` (int): Maximum time to complete the action in seconds
- `success_criteria` (object): Conditions that define successful completion

**Validation Rules**:
- `action_type` must be one of the allowed values
- `timeout` must be positive

### FeedbackLoop
**Description**: Process where perception results are fed back to update the plan or confirm completion

**Fields**:
- `id` (string): Unique identifier for the feedback loop
- `plan_id` (string): Reference to the task plan being monitored
- `perception_data_id` (string): Reference to the perception data used for feedback
- `feedback_type` (enum): ['validation', 'correction', 'replanning', 'status_update']
- `result` (object): Feedback result data
- `timestamp` (datetime): When feedback was processed

**Validation Rules**:
- `feedback_type` must be one of the allowed values
- `timestamp` must be in the past or present

## Relationships

```
VoiceCommand (1) -> (1) TaskPlan
TaskPlan (1) -> (*) Task
Task (1) -> (*) Action
TaskPlan (1) -> (*) ActionSequence
PerceptionData (1) -> (*) DetectedObject
ActionSequence (1) -> (*) Action
TaskPlan (1) -> (*) FeedbackLoop
PerceptionData (1) -> (*) FeedbackLoop
```

## State Transitions

### TaskPlan State Transitions
- `pending` -> `executing` (when execution starts)
- `executing` -> `completed` (when all tasks complete successfully)
- `executing` -> `failed` (when a task fails and no recovery possible)
- `executing` -> `cancelled` (when manually cancelled)

### Task State Transitions
- `pending` -> `executing` (when task execution starts)
- `executing` -> `completed` (when task completes successfully)
- `executing` -> `failed` (when task fails)

### Action State Transitions
- `pending` -> `executing` (when action execution starts)
- `executing` -> `completed` (when action completes successfully)
- `executing` -> `failed` (when action fails)