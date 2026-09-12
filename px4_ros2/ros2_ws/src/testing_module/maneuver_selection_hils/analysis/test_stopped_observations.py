import unittest
from read_stopped_observations import parse


class StoppedObservations(unittest.TestCase):
    header = '[stop-observation-begin],1,0,worker,/x,type,1,3\n'
    record = '[stop-observation],100,0001\n'
    footer = '[stop-observation-end],0,worker,1\n'

    def test_complete_and_explicit_overflow(self):
        block, = parse(self.header + self.record + self.footer)
        self.assertEqual(block['dropped'], 3)
        self.assertEqual(block['records'], [(100, b'\x00\x01')])

    def test_missing_footer_is_not_empty_success(self):
        with self.assertRaises(ValueError):
            parse(self.header + self.record)

    def test_count_and_identity(self):
        for tail in [self.footer.replace(',1\n', ',2\n'),
                     self.footer.replace(',0,', ',1,')]:
            with self.assertRaises(ValueError):
                parse(self.header + self.record + tail)

    def test_no_live_dump_is_legacy_compatible(self):
        self.assertEqual(parse('[INFO] old bag log\n'), [])

    def test_nested_duplicate_and_outside_record(self):
        valid = self.header + self.record + self.footer
        for text in [self.record, self.header + self.header, valid + valid]:
            with self.assertRaises(ValueError):
                parse(text)


if __name__ == '__main__':
    unittest.main()
