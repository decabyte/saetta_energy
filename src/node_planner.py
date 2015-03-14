#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

#import numpy as np
#np.set_printoptions(precision=3, suppress=True)

import sys
import os
import traceback
import subprocess
import jinja2 as jin

# import rospy
# import roslib
# roslib.load_manifest('saetta_energy')

# config
BIN_PLANNER = 'optic-clp'
DIR_DATA = '../data'
DIR_BIN = '../bin'

def main():

    # template files
    spec_domain = os.path.join(DIR_DATA, 'template_domain.pddl')
    spec_problem = os.path.join(DIR_DATA, 'template_problem.pddl')
    res_domain = os.path.join(DIR_DATA, 'current_domain.pddl')
    res_problem = os.path.join(DIR_DATA, 'current_problem.pddl')

    # planning variables
    time_gen = 'now'

    # domain variables
    dict_domain = {
        'time': time_gen
    }

    # problem variables
    dict_problem = {
        'time': time_gen,
        'problem_name': 'test',
        'domain_name': 'mission',
        'objects': [
          ['auv', 'vehicle'],
          ['wp0', 'waypoint'],
          ['wp1', 'waypoint'],
          ['wp2', 'waypoint'],
          ['wp3', 'waypoint'],
          ['wp4', 'waypoint'],
        ],
        'init_literals': [
            ['at', ['auv', 'wp0']],
            ['connected', ['wp0', 'wp1']],
            ['connected', ['wp1', 'wp2']],
            ['connected', ['wp2', 'wp3']],
            ['connected', ['wp3', 'wp4']],
        ],
        'init_fluents': [
            [['energy-usage', ['auv']], 0.0],
            [['energy-required', ['wp0', 'wp1']], 5.0],
            [['energy-required', ['wp1', 'wp2']], 10.0],
            [['energy-required', ['wp2', 'wp3']], 15.0],
            [['energy-required', ['wp3', 'wp4']], 20.0],
            [['distance', ['wp0', 'wp1']], 10.0],
            [['distance', ['wp1', 'wp2']], 15.0],
            [['distance', ['wp2', 'wp3']], 20.0],
            [['distance', ['wp3', 'wp4']], 25.0],
        ],
        'goals': [
            ['at', ['auv', 'wp4']]
        ]
    }
    # TODO: add more data to the problem dicts

    # generate the domain and problem files
    with open(spec_domain, 'rt') as f:
        s_domain = f.read()

    with open(spec_problem, 'rt') as f:
        s_problem = f.read()

    tmpl_domain = jin.Template(s_domain, trim_blocks=True, lstrip_blocks=True)
    tmpl_problem = jin.Template(s_problem, trim_blocks=True, lstrip_blocks=True)

    with open(res_domain, 'wt') as f:
        f.write(tmpl_domain.render(**dict_domain))

    with open(res_problem, 'wt') as f:
        f.write(tmpl_problem.render(**dict_problem))

    # call the planner
    bin_planner = os.path.join(DIR_BIN, BIN_PLANNER)
    out_plan = os.path.join(DIR_DATA, 'plan.txt')

    try:
        result = subprocess.check_output([bin_planner, res_domain, res_problem, out_plan])
    except subprocess.CalledProcessError:
        print(traceback.format_exc())
        sys.exit(1)

    # display the results
    print(result)

    # parsing results
    try:
        idx = result.index(';;;; Solution Found')
    except ValueError:
        raise Exception('error')

    plan = result[idx:].splitlines()
    cost = 0.0

    for line in plan:
        if line.startswith('; Cost:'):
            cost = float(line.split(':')[1])
        elif line.startswith(';'):
            continue
        else:
            pass

    #print(plan)
    print('Cost: %.3f' % cost)

if __name__ == '__main__':
    main()
