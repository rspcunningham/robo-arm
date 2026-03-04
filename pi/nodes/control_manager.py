"""Coordinate high-level control modes for policy, record, replay, and idle."""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from nodes._util import publish_or_ignore_shutdown, run_node

STATUS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class ControlManagerNode(Node):
    def __init__(self):
        super().__init__("control_manager")

        self.status_pub = self.create_publisher(String, "/control/status", STATUS_QOS)
        self.policy_enable_cli = self.create_client(SetBool, "/policy_enable")
        self.torque_enable_cli = self.create_client(SetBool, "/torque_enable")

        self.create_service(SetBool, "/control/set_policy_active", self._srv_set_policy_active)
        self.create_service(SetBool, "/control/set_manual_torque", self._srv_set_manual_torque)
        self.create_service(Trigger, "/control/enter_record_mode", self._srv_enter_record_mode)
        self.create_service(Trigger, "/control/exit_record_mode", self._srv_exit_record_mode)
        self.create_service(Trigger, "/control/enter_replay_mode", self._srv_enter_replay_mode)
        self.create_service(Trigger, "/control/exit_replay_mode", self._srv_exit_replay_mode)
        self.create_service(Trigger, "/control/return_idle", self._srv_return_idle)

        self._mode = "idle"
        self._manual_torque_enabled = True
        self._policy_request_in_flight = False
        self._torque_request_in_flight = False
        self._policy_last_applied: bool | None = None
        self._torque_last_applied: bool | None = None
        self._policy_service_ready = False
        self._torque_service_ready = False
        self._last_error: str | None = None
        self._last_status_json: str | None = None

        self.create_timer(0.25, self._reconcile_outputs)
        self.create_timer(1.0, self._publish_status)
        self._publish_status()

        self.get_logger().info("Control manager ready")

    def _desired_policy_enabled(self) -> bool:
        return self._mode == "policy"

    def _desired_torque_enabled(self) -> bool:
        if self._mode == "record":
            return False
        if self._mode in {"policy", "replay"}:
            return True
        return self._manual_torque_enabled

    def _outputs_converged(self) -> bool:
        return (
            self._policy_last_applied == self._desired_policy_enabled()
            and self._torque_last_applied == self._desired_torque_enabled()
            and not self._policy_request_in_flight
            and not self._torque_request_in_flight
        )

    def _status_snapshot(self) -> dict:
        desired_policy = self._desired_policy_enabled()
        desired_torque = self._desired_torque_enabled()
        return {
            "mode": self._mode,
            "manual_torque_enabled": self._manual_torque_enabled,
            "policy_enabled": desired_policy,
            "torque_enabled": desired_torque,
            "policy_request_in_flight": self._policy_request_in_flight,
            "torque_request_in_flight": self._torque_request_in_flight,
            "policy_service_ready": self._policy_service_ready,
            "torque_service_ready": self._torque_service_ready,
            "outputs_converged": self._outputs_converged(),
            "last_error": self._last_error,
        }

    def _publish_status(self):
        payload = json.dumps(self._status_snapshot())
        if payload == self._last_status_json:
            return
        self._last_status_json = payload
        msg = String()
        msg.data = payload
        publish_or_ignore_shutdown(self.status_pub, msg)

    def _set_mode(self, mode: str):
        if self._mode == mode:
            return
        self._mode = mode
        self._publish_status()

    def _service_busy_message(self) -> str:
        return f"Control is currently owned by {self._mode} mode"

    def _call_set_bool(self, client, desired: bool, label: str):
        req = SetBool.Request()
        req.data = desired
        try:
            future = client.call_async(req)
        except Exception as exc:
            setattr(self, f"_{label}_request_in_flight", False)
            self._last_error = f"Failed to update {label}: {exc}"
            self.get_logger().warning(self._last_error)
            self._publish_status()
            return

        def _done(done_future):
            request_in_flight_attr = f"_{label}_request_in_flight"
            last_applied_attr = f"_{label}_last_applied"
            try:
                result = done_future.result()
            except Exception as exc:
                setattr(self, request_in_flight_attr, False)
                self._last_error = f"Failed to update {label}: {exc}"
                self.get_logger().warning(self._last_error)
                self._publish_status()
                return

            setattr(self, request_in_flight_attr, False)
            if result is not None and result.success:
                setattr(self, last_applied_attr, desired)
                self._last_error = None
            else:
                message = "request rejected"
                if result is not None and result.message:
                    message = result.message
                self._last_error = f"Failed to update {label}: {message}"
                self.get_logger().warning(self._last_error)
            self._publish_status()

        future.add_done_callback(_done)

    def _reconcile_outputs(self):
        desired_policy = self._desired_policy_enabled()
        desired_torque = self._desired_torque_enabled()

        self._policy_service_ready = self.policy_enable_cli.wait_for_service(timeout_sec=0.0)
        self._torque_service_ready = self.torque_enable_cli.wait_for_service(timeout_sec=0.0)

        if (
            self._policy_service_ready
            and not self._policy_request_in_flight
            and self._policy_last_applied != desired_policy
        ):
            self._policy_request_in_flight = True
            self._call_set_bool(self.policy_enable_cli, desired_policy, "policy")

        if (
            self._torque_service_ready
            and not self._torque_request_in_flight
            and self._torque_last_applied != desired_torque
        ):
            self._torque_request_in_flight = True
            self._call_set_bool(self.torque_enable_cli, desired_torque, "torque")

        self._publish_status()

    def _srv_set_policy_active(self, req, resp):
        if req.data:
            if self._mode not in {"idle", "policy"}:
                resp.success = False
                resp.message = self._service_busy_message()
                return resp
            self._set_mode("policy")
            resp.success = True
            resp.message = "Policy mode enabled"
            return resp

        if self._mode == "policy":
            self._set_mode("idle")
            resp.success = True
            resp.message = "Returned to idle mode"
            return resp

        if self._mode == "idle":
            resp.success = True
            resp.message = "Already idle"
            return resp

        resp.success = False
        resp.message = self._service_busy_message()
        return resp

    def _srv_set_manual_torque(self, req, resp):
        if self._mode != "idle":
            resp.success = False
            resp.message = "Manual torque can only be changed in idle mode"
            return resp

        self._manual_torque_enabled = bool(req.data)
        self._publish_status()
        state = "enabled" if req.data else "disabled"
        resp.success = True
        resp.message = f"Manual torque {state}"
        return resp

    def _srv_enter_record_mode(self, req, resp):
        if self._mode not in {"idle", "record"}:
            resp.success = False
            resp.message = self._service_busy_message()
            return resp
        self._set_mode("record")
        resp.success = True
        resp.message = "Record mode enabled"
        return resp

    def _srv_exit_record_mode(self, req, resp):
        if self._mode != "record":
            resp.success = False
            resp.message = "Record mode is not active"
            return resp
        self._set_mode("idle")
        resp.success = True
        resp.message = "Returned to idle mode"
        return resp

    def _srv_enter_replay_mode(self, req, resp):
        if self._mode not in {"idle", "replay"}:
            resp.success = False
            resp.message = self._service_busy_message()
            return resp
        self._set_mode("replay")
        resp.success = True
        resp.message = "Replay mode enabled"
        return resp

    def _srv_exit_replay_mode(self, req, resp):
        if self._mode != "replay":
            resp.success = False
            resp.message = "Replay mode is not active"
            return resp
        self._set_mode("idle")
        resp.success = True
        resp.message = "Returned to idle mode"
        return resp

    def _srv_return_idle(self, req, resp):
        self._set_mode("idle")
        resp.success = True
        resp.message = "Returned to idle mode"
        return resp


def main():
    rclpy.init()
    run_node(ControlManagerNode())
