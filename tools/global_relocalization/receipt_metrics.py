"""ROS-free passive receipt counters with explicit status-boundary segmentation."""
class ReceiptMetrics:
    def __init__(self):
        self.run_id = None
        self.session = self.empty()
        self.run = self.empty()
        self.last_wall = None

    @staticmethod
    def empty():
        return {'lidar_count': 0, 'last_stamp_ns': None, 'last_receipt_wall': None,
                'max_stamp_gap_sec': 0., 'max_callback_gap_sec': 0.}

    def receive(self, stamp, wall):
        for counters in (self.session, self.run):
            if counters['last_stamp_ns'] is not None:
                counters['max_stamp_gap_sec'] = max(counters['max_stamp_gap_sec'],
                                                     (stamp-counters['last_stamp_ns'])*1e-9)
                counters['max_callback_gap_sec'] = max(counters['max_callback_gap_sec'],
                                                        wall-counters['last_receipt_wall'])
            counters.update(last_stamp_ns=stamp, last_receipt_wall=wall)
            counters['lidar_count'] += 1
        self.last_wall = wall

    def status(self, run_id):
        if run_id != self.run_id:
            self.run_id = run_id
            self.run = self.empty()
        return {'source_receipt': dict(self.run), 'session_source_receipt': dict(self.session),
                'counter_segment_semantics': 'reset at first observed status of each run_id; callback receipts thereafter'}
