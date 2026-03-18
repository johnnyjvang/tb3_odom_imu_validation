import csv

import rclpy
from rclpy.node import Node

from tb3_odom_imu_validation.result_utils import RESULTS_FILE

TEST_ORDER = [
    'forward_straightness',
    'backward_straightness',
    'rotation_consistency_ccw',
    'rotation_consistency_cw',
    'out_and_back_heading',
]


class SummaryReport(Node):
    def __init__(self):
        super().__init__('summary_report')
        self.print_summary()

    def print_summary(self):
        results = {}

        if RESULTS_FILE.exists():
            with open(RESULTS_FILE, 'r', newline='') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    results[row['test']] = row

        rows = []
        for test_name in TEST_ORDER:
            if test_name in results:
                row = results[test_name]
                rows.append([
                    row.get('test', test_name),
                    row.get('status', 'UNKNOWN'),
                    row.get('measurement', ''),
                    row.get('notes', '')
                ])
            else:
                rows.append([test_name, 'MISSING', '', 'no result found'])

        headers = ['Test', 'Status', 'Measurement', 'Notes']

        note_wrap_width = 60
        widths = [
            len(headers[0]),
            len(headers[1]),
            len(headers[2]),
            max(len(headers[3]), note_wrap_width),
        ]

        for row in rows:
            widths[0] = max(widths[0], len(str(row[0])))
            widths[1] = max(widths[1], len(str(row[1])))
            widths[2] = max(widths[2], len(str(row[2])))

        def format_row(row):
            return '| ' + ' | '.join(
                str(cell).ljust(widths[i]) for i, cell in enumerate(row)
            ) + ' |'

        def wrap_text(text, width):
            words = str(text).split()
            lines = []
            current = ''

            for word in words:
                if len(current) + len(word) + (1 if current else 0) <= width:
                    current += (' ' if current else '') + word
                else:
                    lines.append(current)
                    current = word

            if current:
                lines.append(current)

            return lines if lines else ['']

        border = '+-' + '-+-'.join('-' * w for w in widths) + '-+'

        print()
        print(border)
        print(format_row(headers))
        print(border)

        for row in rows:
            wrapped_notes = wrap_text(row[3], note_wrap_width)
            max_lines = max(1, len(wrapped_notes))

            for i in range(max_lines):
                new_row = [
                    row[0] if i == 0 else '',
                    row[1] if i == 0 else '',
                    row[2] if i == 0 else '',
                    wrapped_notes[i] if i < len(wrapped_notes) else ''
                ]
                print(format_row(new_row))

        print(border)
        print()


def main(args=None):
    rclpy.init(args=args)
    node = SummaryReport()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()