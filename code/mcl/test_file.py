#!/usr/bin/env python
from particle_filter import particle_filter

initial_position = [1, 1, 1]

def main():
    PF = particle_filter(initial_position)
    # for val in PF.return_chi()
    #     print val
    print PF.return_chi()


if __name__ == '__main__':
    main()
