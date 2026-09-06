import unittest
from analyze_stopped_stage_timing import analyze


class StoppedTimingTests(unittest.TestCase):
    def test_compute_not_completion_interval(self):
        s = "[stop-stage-begin],1,0,2,0\n[stop-stage],1,100,1,50000,1000000,3000000,7,1,1\n[stop-stage],1,200,1,50000,101000000,103000000,7,1,1\n[stop-stage-end],0,2\n"
        r = analyze(s)["stages"]["trajectory_refresh"]
        self.assertEqual(r["computation"]["max_ms"], 2)
        self.assertEqual(r["completion_intervals"]["max_ms"], 100)
        self.assertEqual(r["completion_rate_hz"], 10)

    def test_missing_or_truncated_rejected(self):
        for s in ["", "[stop-stage-begin],1,0,1,0\n", "[stop-stage-begin],1,0,1,0\n[stop-stage-end],0,1\n"]:
            with self.assertRaises(ValueError):
                analyze(s)

    def test_excludes_post_recording_shutdown(self):
        s = "[stop-stage-begin],1,0,2,0\n[stop-stage],1,100,1,50000,1,2,7,1,1\n[stop-stage],1,200,1,50000,3,4,7,1,1\n[stop-stage-end],0,2\n"
        r = analyze(s, 100, 100)
        self.assertEqual(r["recorded_count"], 2)
        self.assertEqual(r["stages"]["trajectory_refresh"]["computation"]["count"], 1)

    def test_drops_and_failed_output_visible(self):
        s = "[stop-stage-begin],1,0,1,8\n[stop-stage],1,100,1,50000,1,2,7,1,0\n[stop-stage-end],0,1\n"
        r = analyze(s)
        self.assertEqual(r["dropped_records"], 8)
        self.assertEqual(r["stages"]["trajectory_refresh"]["output_not_queued_count"], 1)
        self.assertIsNone(r["stages"]["trajectory_refresh"]["completion_rate_hz"])


if __name__ == "__main__":
    unittest.main()
