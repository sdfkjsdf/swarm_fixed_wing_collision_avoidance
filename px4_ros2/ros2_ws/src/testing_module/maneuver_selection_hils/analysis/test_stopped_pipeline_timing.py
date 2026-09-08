import unittest
from analyze_stopped_pipeline_timing import analyze


SAMPLE = '''[stop-stage-begin],1,0,2,0
[stop-stage],1,1000,0,50000,2000000,3000000,7,1,1
[stop-stage],1,51000,0,50000,52000000,53000000,7,1,1
[stop-stage-end],0,2
[stop-pipeline-begin],1,0,2,0,2,0
[stop-pipeline],1000,1000000,1800000,3100000,200000,300000,2,1,1
[stop-pipeline],51000,51000000,51800000,53100000,200000,300000,2,1,1
[stop-belief],1000,900,500000,1500000,0,600000,1100000,1400000,1
[stop-belief],51000,50900,50500000,51500000,0,50600000,51100000,51400000,1
[stop-pipeline-end],0,2,2
'''


class PipelineTimingTest(unittest.TestCase):
    def test_decomposition_units_and_missing_middleware(self):
        r = analyze(SAMPLE)
        self.assertEqual(r['pipeline_dropped'], 0)
        self.assertEqual(r['callback_interval']['max_ms'], 50)
        self.assertEqual(r['enqueue_to_dispatch']['max_ms'], .5)
        self.assertEqual(r['middleware_received_available'], 0)
        self.assertEqual(r['middleware_to_callback_wall']['count'], 0)
        gap = r['largest_completion_gaps']['trajectory_refresh'][0]
        self.assertAlmostEqual(gap['gap_ms'], 50)
        self.assertAlmostEqual(gap['worker_busy_overlap_ms'], 2.1)
        self.assertAlmostEqual(gap['other_time_ms'], 47.9)

    def test_truncated_dump_is_rejected(self):
        with self.assertRaises(ValueError):
            analyze(SAMPLE.replace('[stop-pipeline-end],0,2,2', ''))

    def test_invalid_clock_order_is_rejected(self):
        with self.assertRaises(ValueError):
            analyze(SAMPLE.replace('600000,1100000', '1600000,1100000'))

    def test_source_window(self):
        r = analyze(SAMPLE, 50000, 60000)
        self.assertEqual(r['accepted_beliefs'], 1)
        self.assertEqual(r['largest_completion_gaps']['trajectory_refresh'], [])


if __name__ == '__main__':
    unittest.main()
