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

    def test_parallel_spans_and_pure_compute_are_separate_from_waits(self):
        s = """[stop-stage-begin],2,0,4,0
[stop-stage],4,250000,1,250000,1000000,2000000,7,1,1
[stop-stage],1,250000,2,50000,2100000,4000000,7,1,1
[stop-stage],5,250000,1,250000,12000000,13000000,7,1,1
[stop-stage],2,250000,1,250000,3000000,10000000,7,1,1
[stop-stage-end],0,4
"""
        r = analyze(s)
        self.assertEqual(r["stages"]["combination_kernel"]["computation"]["max_ms"], 7)
        self.assertNotIn("combination_selection", r["stages"])
        total = r["selection_total"]
        self.assertEqual(total["computation"]["max_ms"], 9)
        self.assertEqual(total["dispatch_to_apply"]["max_ms"], 12)
        self.assertEqual(total["request_wait"]["max_ms"], 1)
        self.assertEqual(total["result_wait"]["max_ms"], 2)
        self.assertEqual(total["incomplete_job_count"], 0)

    def test_same_worker_overlap_still_rejected(self):
        s = """[stop-stage-begin],2,0,2,0
[stop-stage],1,100,1,50000,1000000,3000000,7,1,1
[stop-stage],3,100,1,50000,2000000,4000000,7,1,1
[stop-stage-end],0,2
"""
        with self.assertRaises(ValueError):
            analyze(s)

    def test_incomplete_selection_is_not_reported_as_fast(self):
        s = """[stop-stage-begin],2,0,1,0
[stop-stage],4,100,1,250000,1000000,2000000,7,1,1
[stop-stage-end],0,1
"""
        r = analyze(s)["selection_total"]
        self.assertEqual(r["incomplete_job_count"], 1)
        self.assertEqual(r["computation"]["count"], 0)


if __name__ == "__main__":
    unittest.main()
