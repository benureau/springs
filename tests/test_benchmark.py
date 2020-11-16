"""Verify that performance does not drop suddenly"""

import os
import sys

benchmark_path = os.path.join(os.path.dirname(__file__), '../example/benchmark')
sys.path.insert(0, benchmark_path)

import benchmark


def test_benchmark_starfish():
    perf_cpp   , _ = benchmark.benchmark('cpp', duration=1.0)
    assert perf_cpp > 60000.0  # step/s
    perf_cython, _ = benchmark.benchmark('cython', duration=1.0)
    assert perf_cython > 6000.0  # step/s


if __name__ == '__main__':
    test_benchmark_starfish()
