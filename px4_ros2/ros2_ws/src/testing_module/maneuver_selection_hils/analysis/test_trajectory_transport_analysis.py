import unittest
from types import SimpleNamespace
from typing import NamedTuple
from measure_trajectory_transport import (
    analyze, attach_kernel_boundary, direction_summary, matched_delays, parse,
    readable_report, stats)


def stream(vehicle, peer, role, rows):
    return dict(vehicle=vehicle,peer=peer,role=role,count=len(rows),dropped=0,rows=rows)


def encoded_stream(vehicle, peer, role, rows):
    return (f'[stop-transport-begin],1,{vehicle},{peer},{role},{len(rows)},0\n'+
        ''.join('[stop-transport],'+','.join(map(str,r))+'\n' for r in rows)+
        f'[stop-transport-end],{vehicle},{peer},{role},{len(rows)}\n')


class Dump(NamedTuple):
    name: str
    text: str

    def read_text(self):
        return self.text


def fixture(empty_link=None, bad_rx_identity=False, duplicate_unmatched=False):
    """Five-node dump with a known 10 ms Pi clock lead and 5 ms delivery."""
    files=[]
    source_us=10_000_000
    base=10_000_000_000
    for vehicle in range(5):
        offset=10_000_000 if vehicle==0 else 0
        text=encoded_stream(vehicle,-1,'tx',[(source_us,2,3,2**63+17,base+offset,0,0)])
        for peer in range(5):
            if peer==vehicle:continue
            tx_ns=base+(10_000_000 if peer==0 else 0)
            rx_ns=base+offset+5_000_000
            rows=[] if (peer,vehicle)==empty_link else [
                (source_us,2,3,2**63+17,rx_ns,tx_ns,rx_ns-1_000_000)]
            if duplicate_unmatched and (peer,vehicle)==(1,0):
                unknown=(source_us,2,3,99,rx_ns,tx_ns,rx_ns-1_000_000)
                rows.extend([unknown,unknown])
            if bad_rx_identity and (peer,vehicle)==(1,0):
                text+=encoded_stream(0,9,'rx',rows)
            else:text+=encoded_stream(vehicle,peer,'rx',rows)
        files.append(Dump(f'guidance_{vehicle}.log',text))
    directory=SimpleNamespace(name='test_run',glob=lambda _:files)
    before=dict(bounds_consistent=True,lower_ns=9_000_000,upper_ns=11_000_000,pc_mid_time_ns=0)
    after=dict(before,pc_mid_time_ns=220_000_000_000)
    summary=dict(actual_evaluation_start_ns=base,common_duration_s=200)
    return directory,before,after,summary,0


class TransportAnalysisTest(unittest.TestCase):
    def test_clock_sign_and_identity_not_reference_time(self):
        # Pi clock leads PC by 10 ms. Actual delay in either direction is 5 ms.
        tx=[stream(0,-1,'tx',[(1,2,3,4,1_010_000_000,0,0)]),
            stream(1,-1,'tx',[(9,2,3,4,1_000_000_000,0,0)])]
        rx=[stream(1,0,'rx',[(1,2,3,4,1_005_000_000,1_010_000_000,1_004_000_000)]),
            stream(0,1,'rx',[(9,2,3,4,1_015_000_000,1_000_000_000,1_014_000_000)])]
        rows,counts=matched_delays(tx,rx,0,10_000_000)
        self.assertEqual(rows[(0,1)][0]['corrected_ms'],5)
        self.assertEqual(rows[(1,0)][0]['corrected_ms'],5)
        self.assertEqual(rows[(0,1)][0]['raw_ms'],-5)
        self.assertEqual(rows[(0,1)][0]['dds_to_callback_ms'],1)
        self.assertEqual(counts[(0,1)]['unmatched'],0)

    def test_missing_duplicates_and_negative_are_visible(self):
        row=(1,2,3,4,100_000_000,0,0)
        tx=[stream(0,-1,'tx',[row])]
        rx=[stream(1,0,'rx',[(1,2,3,4,99_000_000,0,0),
                             (1,2,3,5,110_000_000,0,0)])]
        values,counts=matched_delays(tx,rx,0,0)
        self.assertEqual(counts[(0,1)]['unmatched'],1)
        self.assertEqual(stats([v['corrected_ms'] for v in values[(0,1)]])['negative_count'],1)
        with self.assertRaises(ValueError): matched_delays(tx+tx,rx,0,0)
        tx[0]['dropped']=1
        with self.assertRaises(ValueError): matched_delays(tx,rx,0,0)

    def test_complete_stopped_dump_contract(self):
        text='[stop-transport-begin],1,0,-1,tx,1,0\n[stop-transport],1,2,3,4,1000,0,0\n[stop-transport-end],0,-1,tx,1\n'
        blocks=parse(text)
        self.assertEqual(blocks[0]['rows'][0],(1,2,3,4,1000,0,0))
        with self.assertRaises(ValueError): parse(text.split('[stop-transport-end]')[0])
        with self.assertRaises(ValueError): parse(text+text)
        with self.assertRaises(ValueError): parse(text.replace('tx,1,0','tx,2,0'))

    def test_seconds_bounds_full_identity_and_existing_values(self):
        r=analyze(*fixture())
        for d in r['directions'].values():
            self.assertEqual(d['corrected_send_to_callback']['max_ms'],5)
            self.assertEqual(d['application_delivery_seconds']['max_s'],.005)
            self.assertEqual(d['worst_message']['clock_envelope_seconds'],[.004,.006])
            self.assertEqual(d['worst_message']['input_revision'],2**63+17)
            self.assertTrue(d['coverage']['complete_unique_delivery'])
            self.assertEqual(sum(d['worst_message']['same_message_segments_ms'].values()),5)
        self.assertFalse(r['measurement_contract']['radio_only_delay_measured'])
        self.assertIn('0.005000초',readable_report(r))
        self.assertIn('순수 Wi-Fi 지연이 아닙니다',readable_report(r))

    def test_missing_link_is_not_omitted(self):
        r=analyze(*fixture(empty_link=(1,0)))
        self.assertEqual(len(r['links']),20)
        self.assertEqual(r['links']['1->0']['sent_not_matched'],1)
        self.assertEqual(r['links']['1->0']['corrected_send_to_callback']['count'],0)
        self.assertFalse(r['directions']['PC_to_Pi']['coverage']['complete_unique_delivery'])

    def test_unmatched_duplicates_do_not_change_sent_coverage(self):
        r=analyze(*fixture(duplicate_unmatched=True))
        link=r['links']['1->0']
        self.assertEqual(link['unmatched'],2)
        self.assertEqual(link['duplicated_rx'],1)
        self.assertEqual(link['sent_not_matched'],0)
        self.assertFalse(r['directions']['PC_to_Pi']['coverage']['complete_unique_delivery'])

    def test_duplicate_or_wrong_stream_identity_rejected(self):
        with self.assertRaises(ValueError):analyze(*fixture(bad_rx_identity=True))

    def test_wrong_clock_bounds_rejected_even_with_true_flag(self):
        args=list(fixture());args[1]['lower_ns']=12_000_000
        with self.assertRaises(ValueError):analyze(*args)

    def test_negative_samples_remain_visible(self):
        row=dict(corrected_ms=-3,source=1,destination=0,source_us=1,epoch=2,
            candidate=3,input_revision=4,send_to_dds_receive_ms=-4,dds_to_callback_ms=1)
        coverage=dict(received=1,matched=1,sent=1,unmatched=0,duplicated_rx=0,sent_not_matched=0)
        d=direction_summary([row],[coverage],1)
        self.assertEqual(d['negative_beyond_clock_envelope_count'],1)
        self.assertEqual(d['application_delivery_seconds']['max_s'],-.003)

    def test_empty_direction_has_no_fake_zero_delay(self):
        c=dict(received=0,matched=0,sent=1,unmatched=0,duplicated_rx=0,sent_not_matched=1)
        d=direction_summary([],[c],1)
        self.assertIsNone(d['worst_message'])
        self.assertEqual(d['application_delivery_seconds'],{})
        self.assertFalse(d['coverage']['complete_unique_delivery'])

    def kernel_fixture(self):
        r=analyze(*fixture());w=r['directions']['PC_to_Pi']['worst_message']
        k=dict(run='test_run',matched=4,worst_total=[dict(source=w['source'],source_us=w['source_us'],
            epoch=w['epoch'],candidate=w['candidate'],revision=w['input_revision'],
            tx_ns=w['tx_wall_ns'],callback_ns=w['rx_wall_ns'],dds_ns=w['dds_receive_wall_ns'],
            kernel_first_ns=w['dds_receive_wall_ns']-500_000,
            kernel_last_ns=w['dds_receive_wall_ns']-500_000,copies_before_dds=1)])
        return r,k

    def test_kernel_joins_same_message_without_double_clock_correction(self):
        r,k=self.kernel_fixture();attach_kernel_boundary(r,k)
        d=r['directions']['PC_to_Pi']['kernel_boundary']
        self.assertEqual(d['same_message_segments_ms'],dict(publish_to_first_pi_kernel=3.5,
            first_pi_kernel_to_dds_timestamp=.5,dds_timestamp_to_callback=1))
        self.assertFalse(d['local_segments_need_cross_host_clock_correction'])
        self.assertTrue(d['accepted_copy_identified'])

    def test_wrong_run_revision_timestamp_or_order_rejected(self):
        for key,value in [('revision',12),('callback_ns',0),('kernel_first_ns',2**63)]:
            r,k=self.kernel_fixture();k['worst_total'][0][key]=value
            with self.assertRaises(ValueError):attach_kernel_boundary(r,k)
        r,k=self.kernel_fixture();k['run']='other_run'
        with self.assertRaises(ValueError):attach_kernel_boundary(r,k)

    def test_multiple_copies_are_not_claimed_to_be_exact_acceptance(self):
        r,k=self.kernel_fixture();k['worst_total'][0]['copies_before_dds']=2
        attach_kernel_boundary(r,k)
        self.assertFalse(r['directions']['PC_to_Pi']['kernel_boundary']['accepted_copy_identified'])
        self.assertIn('실제 채택된 패킷은 불명확',readable_report(r))

    def test_maximum_is_one_message_not_sum_of_independent_maxima(self):
        r=analyze(*fixture());a=r['directions']['PC_to_Pi']['worst_message']
        b=dict(a,corrected_ms=8,send_to_dds_receive_ms=2,dds_to_callback_ms=6)
        a=dict(a,corrected_ms=9,send_to_dds_receive_ms=8,dds_to_callback_ms=1)
        d=direction_summary([a,b],[],1)
        self.assertEqual(sum(d['worst_message']['same_message_segments_ms'].values()),9)
        self.assertNotEqual(sum(d['worst_message']['same_message_segments_ms'].values()),14)


if __name__=='__main__': unittest.main()
