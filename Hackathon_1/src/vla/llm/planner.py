"""
LLM Planner service for the VLA system.

This module provides task planning capabilities using Large Language Models.
It converts natural language commands into structured task plans for execution.
"""

import json
import asyncio
from typing import Dict, Any, List, Optional
from pydantic import BaseModel
import openai
from ..models.task_plan import TaskPlan
from ..models.task import Task
from ..utils import VLALogger, VLAException, get_logger
from ..config import get_config


class PlanningResponse(BaseModel):
    """
    Response model for planning operations.
    """
    task_plan: TaskPlan
    success: bool
    reasoning: str
    estimated_duration: int


class LLMPlanner:
    """
    Service for using LLMs to generate task plans from natural language commands.
    """
    def __init__(self, api_key: Optional[str] = None, model: str = "gpt-4-turbo"):
        """
        Initialize the LLM planner.

        Args:
            api_key: OpenAI API key (if not provided, will use environment variable)
            model: LLM model to use for planning (default: gpt-4-turbo)
        """
        self.logger = get_logger("LLMPlanner")
        self.config = get_config()

        # Use provided API key or get from config
        if api_key:
            openai.api_key = api_key
        elif self.config.settings.openai_api_key:
            openai.api_key = self.config.settings.openai_api_key
        else:
            raise VLAException(
                "OpenAI API key is required for LLM planner",
                "MISSING_API_KEY"
            )

        self.model = model or self.config.settings.llm_model
        self.temperature = self.config.settings.llm_temperature

    async def generate_task_plan(
        self,
        command: str,
        context: Optional[Dict[str, Any]] = None,
        max_retries: int = 3
    ) -> PlanningResponse:
        """
        Generate a structured task plan from a natural language command.

        Args:
            command: Natural language command to plan
            context: Additional context for planning (e.g., environment state)
            max_retries: Maximum number of retries for API calls

        Returns:
            PlanningResponse containing the task plan and metadata
        """
        retry_count = 0
        last_error = None

        while retry_count < max_retries:
            try:
                # Create a structured prompt for the LLM
                prompt = self._create_planning_prompt(command, context or {})

                # Call the LLM with function calling to ensure structured output
                response = await openai.ChatCompletion.acreate(
                    model=self.model,
                    messages=[
                        {"role": "system", "content": self._get_system_prompt()},
                        {"role": "user", "content": prompt}
                    ],
                    temperature=self.temperature,
                    functions=[
                        {
                            "name": "generate_task_plan",
                            "description": "Generate a structured task plan for the VLA system",
                            "parameters": {
                                "type": "object",
                                "properties": {
                                    "tasks": {
                                        "type": "array",
                                        "items": {
                                            "type": "object",
                                            "properties": {
                                                "id": {"type": "string"},
                                                "type": {
                                                    "type": "string",
                                                    "enum": ["navigation", "manipulation", "perception", "other"]
                                                },
                                                "description": {"type": "string"},
                                                "parameters": {"type": "object"},
                                                "dependencies": {
                                                    "type": "array",
                                                    "items": {"type": "string"}
                                                },
                                                "timeout": {"type": "integer"}
                                            },
                                            "required": ["id", "type", "description", "parameters", "timeout"]
                                        }
                                    },
                                    "estimated_duration": {"type": "integer"},
                                    "reasoning": {"type": "string"}
                                },
                                "required": ["tasks", "estimated_duration", "reasoning"]
                            }
                        }
                    ],
                    function_call={"name": "generate_task_plan"}
                )

                # Parse the function arguments
                function_args = json.loads(response.choices[0].message.function_call.arguments)

                # Create tasks from the response
                tasks = []
                for task_data in function_args["tasks"]:
                    task = Task(
                        id=task_data["id"],
                        type=task_data["type"],
                        description=task_data["description"],
                        parameters=task_data["parameters"],
                        dependencies=task_data.get("dependencies", []),
                        timeout=task_data["timeout"]
                    )
                    tasks.append(task)

                # Create task plan
                task_plan = TaskPlan(
                    command_id="unknown",  # Would be set when associated with a voice command
                    tasks=tasks,
                    estimated_duration=function_args["estimated_duration"]
                )

                # Create and return response
                planning_response = PlanningResponse(
                    task_plan=task_plan,
                    success=True,
                    reasoning=function_args["reasoning"],
                    estimated_duration=function_args["estimated_duration"]
                )

                self.logger.info(
                    f"Task plan generated successfully",
                    extra_data={
                        "command": command,
                        "task_count": len(tasks),
                        "estimated_duration": function_args["estimated_duration"]
                    }
                )

                return planning_response

            except json.JSONDecodeError as e:
                self.logger.error(
                    f"Error parsing LLM response: {str(e)}",
                    extra_data={"response": str(response) if 'response' in locals() else "unknown"}
                )
                last_error = VLAException(
                    f"Failed to parse LLM response: {str(e)}",
                    "LLM_RESPONSE_PARSING_ERROR"
                )
            except Exception as e:
                self.logger.error(
                    f"Error generating task plan (attempt {retry_count + 1}): {str(e)}",
                    extra_data={"error_type": type(e).__name__, "command": command}
                )
                last_error = VLAException(
                    f"LLM planning failed: {str(e)}",
                    "LLM_PLANNING_ERROR",
                    {"error_type": type(e).__name__, "attempt": retry_count + 1}
                )

            retry_count += 1
            if retry_count < max_retries:
                # Wait before retrying (exponential backoff)
                await asyncio.sleep(2 ** retry_count)

        # If all retries failed, raise the last error
        raise last_error

    def _create_planning_prompt(self, command: str, context: Dict[str, Any]) -> str:
        """
        Create a structured prompt for task planning.

        Args:
            command: Natural language command
            context: Additional context for planning

        Returns:
            Structured prompt string
        """
        prompt = f"""
        You are an expert robotic task planner. Convert the following natural language command into a structured task plan for a humanoid robot.

        Command: {command}

        Current environment context: {json.dumps(context)}

        Generate a plan that:
        1. Decomposes the command into executable sub-tasks
        2. Specifies task types (navigation, manipulation, perception, other)
        3. Includes necessary parameters for each task
        4. Identifies task dependencies
        5. Estimates execution time
        6. Provides reasoning for your plan

        The robot has the following capabilities:
        - Navigation to specific locations
        - Object manipulation (grasping, placing)
        - Object detection and identification
        - Environmental perception
        - Basic interaction with the environment

        The robot operates in simulation and should consider:
        - Safety constraints
        - Physical limitations
        - Environmental obstacles

        Respond with a structured task plan using the provided function format.
        """

        return prompt

    def _get_system_prompt(self) -> str:
        """
        Get the system prompt for the LLM.

        Returns:
            System prompt string
        """
        return """
        You are an expert robotic task planner. Generate structured task plans that can be executed by a ROS 2-based humanoid robot.
        Ensure tasks are specific, executable, and account for robot capabilities and limitations.
        Consider safety, efficiency, and environmental constraints when planning.
        """

    async def validate_task_plan(self, task_plan: TaskPlan) -> bool:
        """
        Validate a task plan for correctness and executability.

        Args:
            task_plan: Task plan to validate

        Returns:
            True if plan is valid, False otherwise
        """
        try:
            # Check if plan has tasks
            if not task_plan.tasks:
                self.logger.warning(
                    "Task plan validation failed: no tasks",
                    extra_data={"plan_id": task_plan.id}
                )
                return False

            # Check if all tasks have required fields
            for task in task_plan.tasks:
                if not task.description or not task.type:
                    self.logger.warning(
                        "Task plan validation failed: missing required fields",
                        extra_data={
                            "plan_id": task_plan.id,
                            "task_id": task.id,
                            "missing_fields": ["description", "type"]
                        }
                    )
                    return False

                # Validate task type
                if task.type not in ["navigation", "manipulation", "perception", "other"]:
                    self.logger.warning(
                        "Task plan validation failed: invalid task type",
                        extra_data={
                            "plan_id": task_plan.id,
                            "task_id": task.id,
                            "invalid_type": task.type
                        }
                    )
                    return False

            # Check for circular dependencies
            if self._has_circular_dependencies(task_plan):
                self.logger.warning(
                    "Task plan validation failed: circular dependencies detected",
                    extra_data={"plan_id": task_plan.id}
                )
                return False

            self.logger.info(
                "Task plan validation passed",
                extra_data={"plan_id": task_plan.id, "task_count": len(task_plan.tasks)}
            )

            return True

        except Exception as e:
            self.logger.error(
                f"Error validating task plan: {str(e)}",
                extra_data={"error_type": type(e).__name__, "plan_id": task_plan.id}
            )
            return False

    def _has_circular_dependencies(self, task_plan: TaskPlan) -> bool:
        """
        Check if the task plan has circular dependencies.

        Args:
            task_plan: Task plan to check

        Returns:
            True if circular dependencies exist, False otherwise
        """
        # Build dependency graph
        graph = {}
        for task in task_plan.tasks:
            graph[task.id] = set(task.dependencies)

        # Check for cycles using DFS
        visiting = set()
        visited = set()

        def has_cycle(task_id):
            if task_id in visited:
                return False
            if task_id in visiting:
                return True

            visiting.add(task_id)
            for dep_id in graph.get(task_id, []):
                if has_cycle(dep_id):
                    return True
            visiting.remove(task_id)
            visited.add(task_id)
            return False

        for task in task_plan.tasks:
            if has_cycle(task.id):
                return True

        return False

    async def refine_plan(self, original_plan: TaskPlan, feedback: str) -> TaskPlan:
        """
        Refine a task plan based on feedback.

        Args:
            original_plan: Original task plan to refine
            feedback: Feedback to incorporate into the plan

        Returns:
            Refined task plan
        """
        try:
            # Create a prompt to refine the plan based on feedback
            prompt = f"""
            You are refining a robotic task plan based on feedback.

            Original plan: {original_plan.json()}

            Feedback: {feedback}

            Generate an improved plan that addresses the feedback while maintaining the original intent.
            Respond with a structured task plan using the same function format as before.
            """

            # Call the LLM to refine the plan
            response = await openai.ChatCompletion.acreate(
                model=self.model,
                messages=[
                    {"role": "system", "content": self._get_system_prompt()},
                    {"role": "user", "content": prompt}
                ],
                temperature=self.temperature,
                functions=[
                    {
                        "name": "generate_task_plan",
                        "description": "Generate a refined task plan for the VLA system",
                        "parameters": {
                            "type": "object",
                            "properties": {
                                "tasks": {
                                    "type": "array",
                                    "items": {
                                        "type": "object",
                                        "properties": {
                                            "id": {"type": "string"},
                                            "type": {
                                                "type": "string",
                                                "enum": ["navigation", "manipulation", "perception", "other"]
                                            },
                                            "description": {"type": "string"},
                                            "parameters": {"type": "object"},
                                            "dependencies": {
                                                "type": "array",
                                                "items": {"type": "string"}
                                            },
                                            "timeout": {"type": "integer"}
                                        },
                                        "required": ["id", "type", "description", "parameters", "timeout"]
                                    }
                                },
                                "estimated_duration": {"type": "integer"},
                                "reasoning": {"type": "string"}
                            },
                            "required": ["tasks", "estimated_duration", "reasoning"]
                        }
                    }
                ],
                function_call={"name": "generate_task_plan"}
            )

            # Parse the response and create new plan
            function_args = json.loads(response.choices[0].message.function_call.arguments)

            # Create new tasks
            tasks = []
            for task_data in function_args["tasks"]:
                task = Task(
                    id=task_data["id"],
                    type=task_data["type"],
                    description=task_data["description"],
                    parameters=task_data["parameters"],
                    dependencies=task_data.get("dependencies", []),
                    timeout=task_data["timeout"]
                )
                tasks.append(task)

            # Create refined task plan
            refined_plan = TaskPlan(
                command_id=original_plan.command_id,
                tasks=tasks,
                estimated_duration=function_args["estimated_duration"]
            )

            self.logger.info(
                "Task plan refined successfully",
                extra_data={
                    "original_plan_id": original_plan.id,
                    "refined_plan_id": refined_plan.id,
                    "task_count": len(tasks)
                }
            )

            return refined_plan

        except Exception as e:
            self.logger.error(
                f"Error refining task plan: {str(e)}",
                extra_data={"error_type": type(e).__name__}
            )
            raise VLAException(
                f"Task plan refinement failed: {str(e)}",
                "TASK_PLAN_REFINEMENT_ERROR",
                {"error_type": type(e).__name__}
            )


# For testing purposes
if __name__ == "__main__":
    import asyncio

    async def test_llm_planner():
        # This is just for testing - would need actual OpenAI credentials
        try:
            planner = LLMPlanner()
            print("LLM planner initialized successfully")
        except Exception as e:
            print(f"Error initializing planner: {e}")

    # asyncio.run(test_llm_planner())