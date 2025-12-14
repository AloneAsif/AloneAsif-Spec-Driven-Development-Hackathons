"""
Task Generator service for the VLA system.

This module provides specialized functionality for generating specific types of tasks
based on different command patterns and contexts. It works alongside the LLM planner
to create more targeted and efficient task plans.
"""

import json
import asyncio
from typing import Dict, Any, List, Optional
from pydantic import BaseModel
import openai
from ..models.task import Task
from ..models.task_plan import TaskPlan
from ..utils import VLALogger, VLAException, get_logger
from ..config import get_config


class TaskGenerationRequest(BaseModel):
    """
    Request model for task generation operations.
    """
    command: str
    task_type: str
    context: Optional[Dict[str, Any]] = None
    parameters: Optional[Dict[str, Any]] = None


class TaskGenerationResponse(BaseModel):
    """
    Response model for task generation operations.
    """
    tasks: List[Task]
    success: bool
    reasoning: str
    confidence: float


class TaskGenerator:
    """
    Service for generating specific types of tasks based on command patterns.
    """
    def __init__(self, api_key: Optional[str] = None, model: str = "gpt-4-turbo"):
        """
        Initialize the task generator.

        Args:
            api_key: OpenAI API key (if not provided, will use environment variable)
            model: LLM model to use for task generation (default: gpt-4-turbo)
        """
        self.logger = get_logger("TaskGenerator")
        self.config = get_config()

        # Use provided API key or get from config
        if api_key:
            openai.api_key = api_key
        elif self.config.settings.openai_api_key:
            openai.api_key = self.config.settings.openai_api_key
        else:
            raise VLAException(
                "OpenAI API key is required for task generator",
                "MISSING_API_KEY"
            )

        self.model = model or self.config.settings.llm_model
        self.temperature = self.config.settings.llm_temperature

    async def generate_navigation_task(
        self,
        command: str,
        context: Optional[Dict[str, Any]] = None
    ) -> Task:
        """
        Generate a navigation task from a command.

        Args:
            command: Natural language command for navigation
            context: Additional context for navigation

        Returns:
            Navigation Task object
        """
        try:
            # Create a prompt specifically for navigation tasks
            prompt = self._create_navigation_prompt(command, context or {})

            # Call the LLM to generate navigation-specific parameters
            response = await openai.ChatCompletion.acreate(
                model=self.model,
                messages=[
                    {"role": "system", "content": self._get_navigation_system_prompt()},
                    {"role": "user", "content": prompt}
                ],
                temperature=self.temperature,
                functions=[
                    {
                        "name": "generate_navigation_task",
                        "description": "Generate parameters for a navigation task",
                        "parameters": {
                            "type": "object",
                            "properties": {
                                "description": {"type": "string"},
                                "target_location": {"type": "string"},
                                "parameters": {"type": "object"},
                                "timeout": {"type": "integer"},
                                "reasoning": {"type": "string"}
                            },
                            "required": ["description", "target_location", "parameters", "timeout", "reasoning"]
                        }
                    }
                ],
                function_call={"name": "generate_navigation_task"}
            )

            # Parse the function arguments
            function_args = json.loads(response.choices[0].message.function_call.arguments)

            # Create navigation task
            navigation_task = Task(
                type="navigation",
                description=function_args["description"],
                parameters=function_args["parameters"],
                timeout=function_args["timeout"]
            )

            self.logger.info(
                f"Navigation task generated successfully",
                extra_data={
                    "command": command,
                    "target_location": function_args.get("target_location"),
                    "description": function_args["description"]
                }
            )

            return navigation_task

        except Exception as e:
            self.logger.error(
                f"Error generating navigation task: {str(e)}",
                extra_data={"error_type": type(e).__name__, "command": command}
            )
            raise VLAException(
                f"Navigation task generation failed: {str(e)}",
                "NAVIGATION_TASK_GENERATION_ERROR",
                {"error_type": type(e).__name__}
            )

    async def generate_manipulation_task(
        self,
        command: str,
        context: Optional[Dict[str, Any]] = None
    ) -> Task:
        """
        Generate a manipulation task from a command.

        Args:
            command: Natural language command for manipulation
            context: Additional context for manipulation

        Returns:
            Manipulation Task object
        """
        try:
            # Create a prompt specifically for manipulation tasks
            prompt = self._create_manipulation_prompt(command, context or {})

            # Call the LLM to generate manipulation-specific parameters
            response = await openai.ChatCompletion.acreate(
                model=self.model,
                messages=[
                    {"role": "system", "content": self._get_manipulation_system_prompt()},
                    {"role": "user", "content": prompt}
                ],
                temperature=self.temperature,
                functions=[
                    {
                        "name": "generate_manipulation_task",
                        "description": "Generate parameters for a manipulation task",
                        "parameters": {
                            "type": "object",
                            "properties": {
                                "description": {"type": "string"},
                                "object_name": {"type": "string"},
                                "action": {"type": "string"},
                                "parameters": {"type": "object"},
                                "timeout": {"type": "integer"},
                                "reasoning": {"type": "string"}
                            },
                            "required": ["description", "object_name", "action", "parameters", "timeout", "reasoning"]
                        }
                    }
                ],
                function_call={"name": "generate_manipulation_task"}
            )

            # Parse the function arguments
            function_args = json.loads(response.choices[0].message.function_call.arguments)

            # Create manipulation task
            manipulation_task = Task(
                type="manipulation",
                description=function_args["description"],
                parameters=function_args["parameters"],
                timeout=function_args["timeout"]
            )

            self.logger.info(
                f"Manipulation task generated successfully",
                extra_data={
                    "command": command,
                    "object_name": function_args.get("object_name"),
                    "action": function_args.get("action"),
                    "description": function_args["description"]
                }
            )

            return manipulation_task

        except Exception as e:
            self.logger.error(
                f"Error generating manipulation task: {str(e)}",
                extra_data={"error_type": type(e).__name__, "command": command}
            )
            raise VLAException(
                f"Manipulation task generation failed: {str(e)}",
                "MANIPULATION_TASK_GENERATION_ERROR",
                {"error_type": type(e).__name__}
            )

    async def generate_perception_task(
        self,
        command: str,
        context: Optional[Dict[str, Any]] = None
    ) -> Task:
        """
        Generate a perception task from a command.

        Args:
            command: Natural language command for perception
            context: Additional context for perception

        Returns:
            Perception Task object
        """
        try:
            # Create a prompt specifically for perception tasks
            prompt = self._create_perception_prompt(command, context or {})

            # Call the LLM to generate perception-specific parameters
            response = await openai.ChatCompletion.acreate(
                model=self.model,
                messages=[
                    {"role": "system", "content": self._get_perception_system_prompt()},
                    {"role": "user", "content": prompt}
                ],
                temperature=self.temperature,
                functions=[
                    {
                        "name": "generate_perception_task",
                        "description": "Generate parameters for a perception task",
                        "parameters": {
                            "type": "object",
                            "properties": {
                                "description": {"type": "string"},
                                "target_object": {"type": "string"},
                                "action": {"type": "string"},
                                "parameters": {"type": "object"},
                                "timeout": {"type": "integer"},
                                "reasoning": {"type": "string"}
                            },
                            "required": ["description", "target_object", "action", "parameters", "timeout", "reasoning"]
                        }
                    }
                ],
                function_call={"name": "generate_perception_task"}
            )

            # Parse the function arguments
            function_args = json.loads(response.choices[0].message.function_call.arguments)

            # Create perception task
            perception_task = Task(
                type="perception",
                description=function_args["description"],
                parameters=function_args["parameters"],
                timeout=function_args["timeout"]
            )

            self.logger.info(
                f"Perception task generated successfully",
                extra_data={
                    "command": command,
                    "target_object": function_args.get("target_object"),
                    "action": function_args.get("action"),
                    "description": function_args["description"]
                }
            )

            return perception_task

        except Exception as e:
            self.logger.error(
                f"Error generating perception task: {str(e)}",
                extra_data={"error_type": type(e).__name__, "command": command}
            )
            raise VLAException(
                f"Perception task generation failed: {str(e)}",
                "PERCEPTION_TASK_GENERATION_ERROR",
                {"error_type": type(e).__name__}
            )

    def _create_navigation_prompt(self, command: str, context: Dict[str, Any]) -> str:
        """
        Create a prompt for navigation task generation.

        Args:
            command: Navigation command
            context: Additional context

        Returns:
            Navigation-specific prompt string
        """
        return f"""
        You are generating a navigation task for a humanoid robot based on the following command:

        Command: {command}

        Context: {json.dumps(context)}

        Generate a navigation task with:
        1. A clear description of the navigation goal
        2. Target location parameters (coordinates, named location, etc.)
        3. Any specific navigation constraints or requirements
        4. Appropriate timeout for the navigation task
        5. Reasoning for your approach

        The robot operates in simulation with Nav2 for navigation.
        """

    def _create_manipulation_prompt(self, command: str, context: Dict[str, Any]) -> str:
        """
        Create a prompt for manipulation task generation.

        Args:
            command: Manipulation command
            context: Additional context

        Returns:
            Manipulation-specific prompt string
        """
        return f"""
        You are generating a manipulation task for a humanoid robot based on the following command:

        Command: {command}

        Context: {json.dumps(context)}

        Generate a manipulation task with:
        1. A clear description of the manipulation goal
        2. Target object to manipulate
        3. Specific action to perform (grasp, place, move, etc.)
        4. Manipulation parameters (grasp type, placement location, etc.)
        5. Appropriate timeout for the manipulation task
        6. Reasoning for your approach

        The robot has manipulation capabilities for grasping and placing objects.
        """

    def _create_perception_prompt(self, command: str, context: Dict[str, Any]) -> str:
        """
        Create a prompt for perception task generation.

        Args:
            command: Perception command
            context: Additional context

        Returns:
            Perception-specific prompt string
        """
        return f"""
        You are generating a perception task for a humanoid robot based on the following command:

        Command: {command}

        Context: {json.dumps(context)}

        Generate a perception task with:
        1. A clear description of the perception goal
        2. Target object or feature to detect
        3. Specific perception action (detect, classify, locate, etc.)
        4. Perception parameters (detection threshold, search area, etc.)
        5. Appropriate timeout for the perception task
        6. Reasoning for your approach

        The robot has computer vision capabilities for object detection and scene understanding.
        """

    def _get_navigation_system_prompt(self) -> str:
        """
        Get the system prompt for navigation task generation.

        Returns:
            Navigation system prompt string
        """
        return """
        You are an expert in generating navigation tasks for humanoid robots. Create specific, achievable navigation tasks that can be executed with ROS 2 navigation stack. Consider the robot's capabilities, environmental constraints, and safety requirements.
        """

    def _get_manipulation_system_prompt(self) -> str:
        """
        Get the system prompt for manipulation task generation.

        Returns:
            Manipulation system prompt string
        """
        return """
        You are an expert in generating manipulation tasks for humanoid robots. Create specific, achievable manipulation tasks that can be executed with ROS 2 manipulation controllers. Consider the robot's physical capabilities, object properties, and environmental constraints.
        """

    def _get_perception_system_prompt(self) -> str:
        """
        Get the system prompt for perception task generation.

        Returns:
            Perception system prompt string
        """
        return """
        You are an expert in generating perception tasks for humanoid robots. Create specific, achievable perception tasks that can be executed with computer vision systems. Consider the robot's sensing capabilities, environmental conditions, and detection requirements.
        """

    async def generate_tasks_from_command(
        self,
        command: str,
        context: Optional[Dict[str, Any]] = None
    ) -> List[Task]:
        """
        Generate multiple tasks from a single command by determining the appropriate task types.

        Args:
            command: Natural language command that may require multiple tasks
            context: Additional context for task generation

        Returns:
            List of Task objects
        """
        try:
            # Determine the types of tasks needed for the command
            task_types = await self._determine_task_types(command, context or {})

            tasks = []
            for task_type in task_types:
                if task_type == "navigation":
                    task = await self.generate_navigation_task(command, context)
                elif task_type == "manipulation":
                    task = await self.generate_manipulation_task(command, context)
                elif task_type == "perception":
                    task = await self.generate_perception_task(command, context)
                else:
                    # For 'other' or unrecognized types, create a generic task
                    task = await self._create_generic_task(command, task_type, context)

                tasks.append(task)

            self.logger.info(
                f"Multiple tasks generated successfully",
                extra_data={
                    "command": command,
                    "task_count": len(tasks),
                    "task_types": [t.type for t in tasks]
                }
            )

            return tasks

        except Exception as e:
            self.logger.error(
                f"Error generating tasks from command: {str(e)}",
                extra_data={"error_type": type(e).__name__, "command": command}
            )
            raise VLAException(
                f"Task generation from command failed: {str(e)}",
                "TASK_GENERATION_ERROR",
                {"error_type": type(e).__name__}
            )

    async def _determine_task_types(
        self,
        command: str,
        context: Dict[str, Any]
    ) -> List[str]:
        """
        Determine the types of tasks needed for a command.

        Args:
            command: Natural language command
            context: Additional context

        Returns:
            List of task types needed
        """
        try:
            prompt = f"""
            Analyze the following command and determine what types of tasks are needed to execute it:

            Command: {command}

            Context: {json.dumps(context)}

            Identify which of the following task types are needed:
            - navigation: for moving to locations
            - manipulation: for interacting with objects
            - perception: for sensing and detecting objects
            - other: for other types of tasks

            Return a list of the task types needed in the order they should be executed.
            """

            response = await openai.ChatCompletion.acreate(
                model=self.model,
                messages=[
                    {"role": "system", "content": "You are an expert in analyzing commands for robotic task execution. Identify the types of tasks needed to execute a command."},
                    {"role": "user", "content": prompt}
                ],
                temperature=self.temperature,
                functions=[
                    {
                        "name": "identify_task_types",
                        "description": "Identify the types of tasks needed for command execution",
                        "parameters": {
                            "type": "object",
                            "properties": {
                                "task_types": {
                                    "type": "array",
                                    "items": {"type": "string", "enum": ["navigation", "manipulation", "perception", "other"]}
                                },
                                "reasoning": {"type": "string"}
                            },
                            "required": ["task_types", "reasoning"]
                        }
                    }
                ],
                function_call={"name": "identify_task_types"}
            )

            function_args = json.loads(response.choices[0].message.function_call.arguments)
            return function_args["task_types"]

        except Exception as e:
            self.logger.error(
                f"Error determining task types: {str(e)}",
                extra_data={"error_type": type(e).__name__, "command": command}
            )
            # Default to a single 'other' task if analysis fails
            return ["other"]

    async def _create_generic_task(
        self,
        command: str,
        task_type: str,
        context: Optional[Dict[str, Any]] = None
    ) -> Task:
        """
        Create a generic task when the specific type is not recognized.

        Args:
            command: Natural language command
            task_type: Type of task to create
            context: Additional context

        Returns:
            Generic Task object
        """
        return Task(
            type=task_type,
            description=f"Execute command: {command}",
            parameters={"command": command, "context": context or {}},
            timeout=30
        )

    async def generate_sequential_plan(
        self,
        commands: List[str],
        context: Optional[Dict[str, Any]] = None
    ) -> TaskPlan:
        """
        Generate a sequential task plan from multiple commands.

        Args:
            commands: List of natural language commands
            context: Additional context for the plan

        Returns:
            TaskPlan with tasks from all commands
        """
        try:
            all_tasks = []
            for i, command in enumerate(commands):
                tasks = await self.generate_tasks_from_command(command, context)

                # Add dependencies so tasks execute in sequence
                for j, task in enumerate(tasks):
                    if i > 0 or j > 0:  # If not the very first task
                        # Make this task dependent on the previous one
                        if all_tasks:
                            task.dependencies = [all_tasks[-1].id]

                all_tasks.extend(tasks)

            # Create a task plan
            task_plan = TaskPlan(
                command_id="sequential_plan",
                tasks=all_tasks,
                estimated_duration=len(all_tasks) * 30  # Estimate 30 seconds per task
            )

            self.logger.info(
                f"Sequential plan generated successfully",
                extra_data={
                    "command_count": len(commands),
                    "task_count": len(all_tasks),
                    "plan_id": task_plan.id
                }
            )

            return task_plan

        except Exception as e:
            self.logger.error(
                f"Error generating sequential plan: {str(e)}",
                extra_data={"error_type": type(e).__name__}
            )
            raise VLAException(
                f"Sequential plan generation failed: {str(e)}",
                "SEQUENTIAL_PLAN_GENERATION_ERROR",
                {"error_type": type(e).__name__}
            )


# For testing purposes
if __name__ == "__main__":
    import asyncio

    async def test_task_generator():
        # This is just for testing - would need actual OpenAI credentials
        try:
            generator = TaskGenerator()
            print("Task generator initialized successfully")
        except Exception as e:
            print(f"Error initializing generator: {e}")

    # asyncio.run(test_task_generator())