import pstats, cProfile

import benchmark


if __name__ == '__main__':
    cProfile.runctx("benchmark.benchmark(n=10000)", globals(), locals(), ".profile.prof")

    s = pstats.Stats(".profile.prof")
    s.strip_dirs().sort_stats("time").print_stats()
