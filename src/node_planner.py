#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

#import numpy as np
#np.set_printoptions(precision=3, suppress=True)

import os
import traceback
import subprocess
import jinja2 as jin

# import rospy
# import roslib
# roslib.load_manifest('saetta_energy')

# config
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
          ['auv', 'vehicle']
        ],
        'init_literals': [
            ['at', ['auv', 'wp0']]
        ],
        'init_fluents': [
            [['energy-required', ['wp0', 'wp1']], 10.0]
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

    tmpl_domain = jin.Template(s_domain)
    tmpl_problem = jin.Template(s_problem)

    with open(res_domain, 'wt') as f:
        f.write(tmpl_domain.render(**dict_domain))

    with open(res_problem, 'wt') as f:
        f.write(tmpl_problem.render(**dict_problem))

    # call the planner
    bin_planner = os.path.join(DIR_BIN, 'optic-clp')
    out_plan = os.path.join(DIR_DATA, 'plan.txt')

    result = subprocess.check_output([bin_planner, res_domain, res_problem, out_plan])

    print(result)


if __name__ == '__main__':
    main()
