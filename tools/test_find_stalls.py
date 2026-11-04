import unittest

from tools.find_stalls import ZeroRunTracker


class ZeroRunTrackerTest(unittest.TestCase):
    def test_movement_nonzero_and_disappearance_reset_and_mark_transients(self):
        tracker = ZeroRunTracker()
        at_origin = {"id": 7, "speed": 0.0, "position": {"x": 1, "y": 2}}
        tracker.update(0.0, [at_origin])
        tracker.update(0.25, [at_origin])
        tracker.update(0.5, [{**at_origin, "position": {"x": 1.01, "y": 2}}])
        tracker.update(0.75, [at_origin])
        tracker.update(1.0, [{**at_origin, "speed": 0.1}])
        tracker.update(1.25, [at_origin])
        tracker.update(1.5, [])
        runs = tracker.finish(0.5)

        self.assertEqual([(run["begin"], run["end"]) for run in runs], [
            (0.0, 0.25), (0.5, 0.5), (0.75, 0.75), (1.25, 1.25),
        ])
        self.assertTrue(all(run["transient"] for run in runs))
        tracker.update(2.0, [at_origin])
        tracker.update(2.5, [at_origin])
        self.assertFalse(tracker.finish(0.5)[-1]["transient"])


    def test_free_space_must_persist_in_each_sample(self):
        tracker = ZeroRunTracker()
        driver = {"id": 1, "speed": 0.0, "position": {"x": 0, "y": 0}, "steering_available": True}
        tracker.update(0, [driver])
        tracker.update(1, [driver])
        tracker.update(2, [{**driver, "steering_available": False}])
        tracker.update(3, [driver])
        tracker.update(5, [driver])
        run = tracker.finish(2)[0]
        self.assertEqual(run["duration"], 5)
        self.assertEqual(run["max_clear_seconds"], 2)
        self.assertEqual(run["clear_since"], 3)


if __name__ == "__main__":
    unittest.main()
