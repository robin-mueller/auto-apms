# Copyright 2025 Robin Müller
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import ament_index_python
import yaml
import rclpy
import signal
import threading

from argcomplete.completers import ChoicesCompleter
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, CancelGoal, GoalStatus
from rclpy.logging import LoggingSeverity
from rcl_interfaces.msg import LoggerLevel
from rcl_interfaces.srv import SetLoggerLevels, GetParameters
from std_srvs.srv import Trigger
from auto_apms_interfaces.action import StartTreeExecutor
from ros2cli.node import HIDDEN_NODE_PREFIX
from ros2cli.node.strategy import DirectNode, NodeStrategy
from ros2param.api import call_get_parameters, call_set_parameters, get_value
from ros2run.api import run_executable, get_executable_path
from auto_apms_behavior_tree_core.resources import (
    BehaviorResource,
    NodeManifest,
    get_all_behavior_resources,
    RESOURCE_IDENTITY_CATEGORY_SEPARATOR,
)


GENERIC_NODE_NAME = HIDDEN_NODE_PREFIX + "auto_apms_ros2cli"


class PrefixFilteredChoicesCompleter(ChoicesCompleter):
    """A completer that filters choices based on prefix matching."""

    def __init__(self, choices, *args, **kwargs):
        super().__init__(choices, *args, **kwargs)

    def __call__(self, **kwargs):
        options = super(PrefixFilteredChoicesCompleter, self).__call__(**kwargs)
        prefix = kwargs.get("prefix", "")
        return [option for option in options if option.startswith(prefix)]


class BehaviorChoicesCompleter(ChoicesCompleter):
    """
    A completer that filters AutoAPMS behavior resources.
    This completer filters by prefix but it also works if the behavior category is not specified.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(get_all_behavior_resources().keys(), *args, **kwargs)

    def __call__(self, **kwargs):
        options = super(BehaviorChoicesCompleter, self).__call__(**kwargs)
        prefix = kwargs.get("prefix", "")
        return [
            option
            for option in options
            if option.startswith(prefix) or option.split(RESOURCE_IDENTITY_CATEGORY_SEPARATOR, 2)[-1].startswith(prefix)
        ]


def find_start_tree_executor_actions() -> list[tuple[str, str]]:
    """
    Returns:
        list: list of tuples (action_name, node_name) for all available StartTreeExecutor actions.
    """
    action_type = "auto_apms_interfaces/action/StartTreeExecutor"
    actions = []
    with NodeStrategy(None) as node:
        action_list = node.get_action_names_and_types()
        for action_name, types in action_list:
            if action_type in types:
                # action_name is the full action path, e.g. '/my_executor/start_tree_executor'
                # node_name is not directly available, but we can extract the executor name
                # from the action_name if it matches the pattern
                executor_name = action_name.strip("/").split("/")[0]
                actions.append((action_name, executor_name))
    return actions


def call_clear_blackboard_for_executor(node: Node, node_name: str):
    """
    Clears the behavior tree blackboard of an executor.
    """
    client = node.create_client(Trigger, f"{node_name}/clear_blackboard")
    if not client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(f"clear_blackboard service for node '{node_name}' not available after 5 seconds")
    request = Trigger.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if future.done():
        response: Trigger.Response = future.result()
        if not response.success:
            node.get_logger().error(f"Failed to clear blackboard for node '{node_name}': {response.message}")
    else:
        raise RuntimeError(f"clear_blackboard service call for node '{node_name}' did not complete in time")


def call_set_logger_level(node: Node, node_name: str, logging: LoggingSeverity):
    """
    Set the logger level for the executor node.
    """
    client = node.create_client(SetLoggerLevels, f"{node_name}/set_logger_levels")
    if not client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(f"set_logger_levels service for node '{node_name}' not available after 5 seconds")
    request = SetLoggerLevels.Request()
    request.levels = [LoggerLevel(name=node_name, level=logging)]
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if future.done():
        response: SetLoggerLevels.Response = future.result()
        for res in response.results:
            if res.successful:
                node.get_logger().info(f"Logger level set to {logging.name} for node '{node_name}'")
            else:
                node.get_logger().error(f"Failed to set logger level for node '{node_name}': {res.reason}")
    else:
        raise RuntimeError(f"set_logger_levels service call for node '{node_name}' did not complete in time")


def call_set_parameters_for_executor(node: Node, node_name: str, static_params, blackboard_params):
    """
    Set static and blackboard parameters on the executor node using ros2param.api.
    """
    parameters = []
    # Static params
    for k, v in (static_params or {}).items():
        parameters.append(Parameter(name=k, value=v))
    # Blackboard params (with bb. prefix)
    for k, v in (blackboard_params or {}).items():
        parameters.append(Parameter(name=f"bb.{k}", value=v))

    try:
        result = call_set_parameters(
            node=node,
            node_name=node_name,
            parameters=[p.to_parameter_msg() for p in parameters],
        )
        # Evaluate and print result for each parameter
        for idx, res in enumerate(result.results):
            pname = parameters[idx].name if idx < len(parameters) else "<unknown>"
            if not res.successful:
                msg = f"Set parameter '{pname}' failed"
                if res.reason:
                    msg += f": {res.reason}"
                node.get_logger().warning(msg)
    except Exception as e:
        raise RuntimeError(f"Exception while setting parameters on node '{node_name}': {e}") from e


def call_start_tree_action(
    node: Node,
    action_name: str,
    build_request: str,
    build_handler: str = None,
    root_tree: str = None,
    node_manifest: NodeManifest = None,
    clear_blackboard: bool = False,
    timeout_sec=5.0,
):
    """
    Send the tree XML to the specified executor node using the StartTreeExecutor action.
    """
    action_client = ActionClient(node, StartTreeExecutor, action_name)
    last_running_action_ts = -1.0
    last_running_action_name = ""

    def feedback_callback(feedback_msg):
        nonlocal last_running_action_ts, last_running_action_name
        feedback: StartTreeExecutor.Feedback = feedback_msg.feedback
        if last_running_action_ts < 0:
            node.get_logger().info(f"Root Behavior Tree is '{feedback.running_tree_identity}'")
            last_running_action_ts = 0.0
        if feedback.running_action_timestamp > last_running_action_ts:
            if feedback.running_action_name != last_running_action_name:
                last_running_action_name = feedback.running_action_name
                node.get_logger().info(f"» Current action: {feedback.running_action_name}")
            last_running_action_ts = feedback.running_action_timestamp

    if not action_client.wait_for_server(timeout_sec=5.0):
        raise RuntimeError(
            f"Action server '{action_name}' not available after 5 seconds. " "Make sure the executor node is running!"
        )

    goal_msg = StartTreeExecutor.Goal(check_fields=True)
    goal_msg.build_request = build_request
    if build_handler:
        goal_msg.build_handler = build_handler
    if root_tree:
        goal_msg.root_tree = root_tree
    if node_manifest is None:
        goal_msg.node_manifest = node_manifest.dump()
    goal_msg.clear_blackboard = clear_blackboard
    goal_msg.attach = True

    send_goal_future = action_client.send_goal_async(goal_msg, feedback_callback=feedback_callback)
    rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=timeout_sec)
    if not send_goal_future.done():
        node.get_logger().error("Timed out waiting for goal response from executor")
        return
    goal_handle: ClientGoalHandle = send_goal_future.result()
    if not goal_handle or not goal_handle.accepted:
        node.get_logger().error("Goal was rejected by executor!")
        return

    get_result_future = goal_handle.get_result_async()
    cancel_requested = threading.Event()

    def signal_handler(sig, frame):
        cancel_requested.set()

    # Register signal handler for Ctrl+C
    original_handler = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, signal_handler)

    try:
        while not get_result_future.done():
            rclpy.spin_once(node, timeout_sec=0.1)
            if cancel_requested.is_set():
                node.get_logger().warning("Requesting cancellation...")
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=timeout_sec)
                if cancel_future.done():
                    cancel_response: CancelGoal.Response = cancel_future.result()
                    if cancel_response is None:
                        node.get_logger().error(
                            "Exception while canceling goal: {!r}".format(cancel_future.exception())
                        )
                    if len(cancel_response.goals_canceling) == 0:
                        node.get_logger().error("Failed to cancel goal")
                else:
                    node.get_logger().error("Timed out waiting for cancel response from executor")
                    return
                break
    finally:
        # Restore original signal handler
        signal.signal(signal.SIGINT, original_handler)

    rclpy.spin_until_future_complete(node, get_result_future, timeout_sec=timeout_sec)
    if not get_result_future.done():
        node.get_logger().error("Timed out waiting for result response from executor")
        return

    result: StartTreeExecutor.Result = get_result_future.result().result
    status: GoalStatus = get_result_future.result().status

    if status == GoalStatus.STATUS_SUCCEEDED:
        node.get_logger().info(f"Behavior Tree SUCCEEDED!")
    elif status == GoalStatus.STATUS_CANCELED:
        node.get_logger().info("Behavior Tree CANCELED!")
    else:
        node.get_logger().info(f"Behavior Tree FAILED!")
    if result.message:
        node.get_logger().info(result.message)


def parse_key_value_args(argv):
    """
    Parse arbitrary key:=value arguments from argv.

    Returns:
        dict: dict of key-value pairs with values converted to builtin Python types.
    """
    parsed_args = {}
    for argument in argv:
        count = argument.count(":=")
        if count == 0 or argument.startswith(":=") or (count == 1 and argument.endswith(":=")):
            raise RuntimeError(f"Malformed key-value argument '{argument}', expected format '<name>:=<value>'")
        key, value = argument.split(":=", maxsplit=1)
        # last one wins is intentional
        try:
            parsed_args[key] = yaml.safe_load(value)
        except Exception:
            parsed_args[key] = value
    return parsed_args


def sync_run_behavior_with_executor(
    executor_name: str,
    behavior: BehaviorResource,
    static_params: dict = None,
    blackboard_params: dict = None,
    keep_blackboard: bool = False,
    logging_level: LoggingSeverity = None,
):
    """
    Execute a behavior on a remote executor node.

    Args:
        executor_name: Name of the behavior tree executor
        behavior: A valid behavior resource
        static_params: Static parameters to set on the executor
        blackboard_params: Blackboard parameters to set on the executor
        keep_blackboard: Do not clear the blackboard before execution
        logging_level: Logger level to set on the executor
    """
    if not isinstance(behavior, BehaviorResource):
        raise TypeError(f"Expected BehaviorResource, got {type(behavior).__name__}")

    # Find the full action name for the given executor
    try:
        executor_actions = find_start_tree_executor_actions()
        action_name = None
        for full_action, executor in executor_actions:
            if executor == executor_name:
                action_name = full_action
                break
        if not action_name:
            raise RuntimeError(f"No StartTreeExecutor action found for executor '{executor_name}'")
    except Exception as e:
        raise RuntimeError(f"Could not discover behavior executors: {e}") from e

    with DirectNode(None, node_name=GENERIC_NODE_NAME) as node:
        node.get_logger().info(f"Targeting behavior executor '{executor_name}' (Action '{action_name}')")

        # Set logger level if requested
        if logging_level is not None:
            call_set_logger_level(node, executor_name, logging_level)
        # Clear blackboard if not instructed otherwise
        if not keep_blackboard:
            call_clear_blackboard_for_executor(node, executor_name)
        # Set parameters before sending the goal
        if static_params or blackboard_params:
            call_set_parameters_for_executor(node, executor_name, static_params, blackboard_params)
        if static_params and "tick_rate" in static_params:
            tick_rate = static_params["tick_rate"]
        else:
            response: GetParameters.Response = call_get_parameters(
                node=node, node_name=executor_name, parameter_names=["tick_rate"]
            )
            tick_rate = get_value(parameter_value=response.values[0])
        call_start_tree_action(
            node,
            action_name,
            build_request=behavior.build_request,
            build_handler=behavior.default_build_handler,
            # TODO: Rename root_tree to entrypoint in interfaces (and docs) and add it to behavior resource marker files
            node_manifest=behavior.node_manifest,
            clear_blackboard=False,
            timeout_sec=max(tick_rate * 2.5, 5.0),
        )


def sync_run_behavior_locally(
    build_request: str,
    static_params: dict = None,
    blackboard_params: dict = None,
    logging_level: LoggingSeverity = None,
):
    """
    Execute a behavior locally using the run_behavior executable.

    Args:
        build_request: Behavior build request passed to the executor
        static_params: Static parameters to set on the executor
        blackboard_params: Blackboard parameters to set on the executor
        logging_level: Logger level to set on the executor
    """
    # Check if required package is available first
    try:
        ament_index_python.get_package_share_directory("auto_apms_behavior_tree")
    except Exception as e:
        raise RuntimeError(
            "Package 'auto_apms_behavior_tree' not found. Make sure it is installed and the workspace has been sourced"
        ) from e

    run_tree_argv = [build_request]

    # Add ros args
    run_tree_argv.append("--ros-args")
    if logging_level:
        run_tree_argv.append("--log-level")
        run_tree_argv.append(f"run_behavior:={logging_level.name}")

    if static_params or blackboard_params:
        for k, v in (static_params or {}).items():
            run_tree_argv.append("-p")
            run_tree_argv.append(f"{k}:={v}")
        for k, v in (blackboard_params or {}).items():
            run_tree_argv.append("-p")
            run_tree_argv.append(f"bb.{k}:={v}")

    run_executable(
        path=get_executable_path(
            package_name="auto_apms_behavior_tree",
            executable_name="run_behavior",
        ),
        argv=run_tree_argv,
    )
