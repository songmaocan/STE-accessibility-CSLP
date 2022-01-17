"""
Algorithm: Solve the accessibility-oriented charging station location problem by Lagrangian relaxation
Copyright: Maocan Song, 1097133316@qq.com
Date: 2021-10-10
"""
import time
from Method import Solve

def main():
    start_time = time.time()
    mod=Solve()
    mod.g_solving_the_charging_location_problem_by_LR()
    end_time = time.time()
    spend_time = end_time - start_time
    mod.output_results(spend_time)
    print("CPU running time {} min".format(round(spend_time / 60), 3))

if __name__ == '__main__':
    main()