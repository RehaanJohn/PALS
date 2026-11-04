import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("swarm", ROOT / "scripts/swarm.py")
swarm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(swarm)


class FleetTests(unittest.TestCase):
    def setUp(self):
        self.fleet = swarm.load_fleet(ROOT / "config/fleet.json")

    def load(self, fleet):
        with tempfile.TemporaryDirectory() as temporary:
            config = Path(temporary) / "fleet.json"
            config.write_text(json.dumps(fleet))
            return swarm.load_fleet(config)

    def test_duplicate_identity_rejected(self):
        for field in ("id", "instance"):
            fleet = copy.deepcopy(self.fleet)
            fleet["drones"][1][field] = fleet["drones"][0][field]
            with self.assertRaises(ValueError):
                self.load(fleet)

    def test_overlapping_routes_rejected(self):
        self.fleet["drones"][1]["spawn"][0] = -13
        with self.assertRaises(ValueError):
            self.load(self.fleet)

    def test_invalid_pose_rejected(self):
        for pose in ([0, 0], [float("nan"), 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 1], [0, 0, -1, 0, 0, 0]):
            self.fleet["drones"][0]["spawn"] = pose
            with self.assertRaises(ValueError):
                self.load(self.fleet)

    def test_preparation_isolates_transport_without_changing_source(self):
        source = ROOT / "px4/Tools/simulation/gz/models/x500/model.sdf"
        original = source.read_bytes()
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            manifest = swarm.prepare(self.fleet, output)
            topics, urls, entities = set(), set(), set()
            for drone in manifest["drones"]:
                tree = ET.parse(output / "models" / drone["model"] / "model.sdf")
                sensor_topics = [t.text for t in tree.findall(".//sensor/topic")]
                self.assertEqual(len(sensor_topics), 2)
                self.assertFalse(topics.intersection(sensor_topics))
                self.assertTrue(all(t.startswith(drone["namespace"] + "/sensors/") for t in sensor_topics))
                topics.update(sensor_topics)
                urls.add(drone["fcu_url"])
                entities.add(drone["entity"])
            self.assertEqual(len(urls), 3)
            self.assertEqual(len(entities), 3)
            self.assertEqual(manifest["drones"][0]["fcu_url"], "udp://:14540@127.0.0.1:14580")
        self.assertEqual(original, source.read_bytes())


if __name__ == "__main__":
    unittest.main()
