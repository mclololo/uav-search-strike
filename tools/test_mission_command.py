"""Transport/shutdown tests without importing ROS, CUDA or model weights."""
import ast
import pathlib
import re
import types
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
SCRIPT = next(ROOT.glob("*model*/mission_once.py"))
TREE = ast.parse(SCRIPT.read_text(encoding="utf-8"))


class Interrupted(Exception):
    pass


class FakeRos:
    ROSInterruptException = Interrupted

    def __init__(self):
        self.shutdown = False
        self.errors = []

    def is_shutdown(self):
        return self.shutdown

    def logerr(self, text):
        self.errors.append(text)

    def logwarn_throttle(self, *_):
        pass

    def loginfo(self, *_):
        pass


class FakePublisher:
    def __init__(self, connections=1):
        self.connections = connections
        self.sent = []

    def get_num_connections(self):
        return self.connections

    def publish(self, msg):
        self.sent.append(msg.data)


class MissionCommandTest(unittest.TestCase):
    def setUp(self):
        self.ros = FakeRos()
        self.now = 0.0
        self.on_sleep = lambda: None

        def sleep(seconds):
            self.now += seconds
            self.on_sleep()

        self.env = dict(rospy=self.ros, re=re, String=types.SimpleNamespace,
                        time=types.SimpleNamespace(monotonic=lambda: self.now, sleep=sleep),
                        gc=types.SimpleNamespace(collect=lambda: None),
                        torch=types.SimpleNamespace(cuda=types.SimpleNamespace(empty_cache=lambda: None)))
        nodes = [n for n in TREE.body if isinstance(n, ast.FunctionDef) and
                 n.name in ("ensure_ros_running", "publish_mission_command")]
        cls = next(n for n in TREE.body if isinstance(n, ast.ClassDef) and n.name == "MissionGenerator")
        nodes.append(next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "generate_commands"))
        exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SCRIPT), "exec"), self.env)

    def test_connected_publishes_exactly_once(self):
        pub = FakePublisher()
        self.assertTrue(self.env["publish_mission_command"](pub, "takeoff"))
        self.assertEqual(pub.sent, ["takeoff"])

    def test_shutdown_is_not_misreported_as_missing_subscriber(self):
        self.ros.shutdown = True
        pub = FakePublisher(0)
        with self.assertRaises(Interrupted):
            self.env["publish_mission_command"](pub, "takeoff")
        self.assertEqual(pub.sent, [])
        self.assertEqual(self.ros.errors, [])

    def test_shutdown_during_connection_wait(self):
        self.on_sleep = lambda: setattr(self.ros, "shutdown", True)
        pub = FakePublisher(0)
        with self.assertRaises(Interrupted):
            self.env["publish_mission_command"](pub, "takeoff")
        self.assertEqual(pub.sent, [])
        self.assertEqual(self.ros.errors, [])

    def test_connection_appears(self):
        pub = FakePublisher(0)
        self.on_sleep = lambda: setattr(pub, "connections", 1)
        self.assertTrue(self.env["publish_mission_command"](pub, "land"))
        self.assertEqual(pub.sent, ["land"])

    def test_real_missing_subscriber_times_out_without_ros_clock(self):
        pub = FakePublisher(0)
        self.assertFalse(self.env["publish_mission_command"](pub, "takeoff", 0.3))
        self.assertEqual(pub.sent, [])
        self.assertEqual(len(self.ros.errors), 1)

    def test_interrupt_during_stream_discards_commands(self):
        def stream():
            yield "```command\n"
            self.ros.shutdown = True
            yield "takeoff\n```"

        generator = types.SimpleNamespace(
            knowledge_prompt="prompt", use_image_context=False, tokenizer=None,
            model=types.SimpleNamespace(chat=lambda **_: stream()))
        with self.assertRaises(Interrupted):
            self.env["generate_commands"](generator, "task")

    def test_command_publisher_is_not_latched(self):
        publishers = [n for n in ast.walk(TREE) if isinstance(n, ast.Call)
                      and isinstance(n.func, ast.Attribute) and n.func.attr == "Publisher"]
        self.assertEqual(len(publishers), 1)
        latch = next(k.value for k in publishers[0].keywords if k.arg == "latch")
        self.assertIs(latch.value, False)


if __name__ == "__main__":
    unittest.main()
