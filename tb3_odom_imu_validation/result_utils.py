import csv
from pathlib import Path

RESULTS_DIR = Path('/tmp/tb3_odom_imu_validation')
RESULTS_FILE = RESULTS_DIR / 'results.csv'


def reset_results_file():
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    with open(RESULTS_FILE, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['test', 'status', 'measurement', 'notes'])


def append_result(test_name, status, measurement, notes=''):
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    with open(RESULTS_FILE, 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([test_name, status, measurement, notes])