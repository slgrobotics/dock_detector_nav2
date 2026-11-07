"""Explicit no-goal DockingServer lifecycle smoke test in an isolated ROS domain."""
from pathlib import Path
import signal
import subprocess
import tempfile
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import yaml
from dock_detector.nav2_config import merge_nav2


def test_real_docking_server_loads_generated_named_plugin():
    workspace=Path(__file__).resolve().parents[3]
    base=yaml.safe_load((workspace/'src/articubot_one/robots/seggy/config/nav2_params.yaml').read_text())
    catalogue=yaml.safe_load((workspace/'src/dock_detector/config/seggy_docks.yaml').read_text())
    parameters=merge_nav2(base,catalogue)
    rclpy.init();peer=Node('docking_server_smoke')
    with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml') as config, tempfile.TemporaryFile(mode='w+') as log:
        yaml.safe_dump(parameters,config);config.flush()
        process=subprocess.Popen(['ros2','run','opennav_docking','opennav_docking',
            '--ros-args','--params-file',config.name],stdout=log,stderr=subprocess.STDOUT)
        try:
            client=peer.create_client(ChangeState,'docking_server/change_state')
            assert client.wait_for_service(timeout_sec=10.)
            for transition in (Transition.TRANSITION_CONFIGURE,Transition.TRANSITION_ACTIVATE,
                               Transition.TRANSITION_DEACTIVATE,Transition.TRANSITION_CLEANUP):
                request=ChangeState.Request();request.transition.id=transition
                future=client.call_async(request)
                rclpy.spin_until_future_complete(peer,future,timeout_sec=10.)
                assert future.done() and future.result().success, f'transition {transition} failed'
        finally:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=5.)
            except subprocess.TimeoutExpired:process.kill();process.wait()
            log.seek(0);print(log.read());peer.destroy_node();rclpy.shutdown()
