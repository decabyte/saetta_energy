#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import random
import string
import re

# Secure ID generation:
#   see http://stackoverflow.com/questions/2257441/random-string-generation-with-upper-case-letters-and-digits-in-python
def id_generator(size=6, chars=string.ascii_uppercase + string.digits):
    return ''.join(random.SystemRandom().choice(chars) for _ in range(size))

# String cleaning
#   see http://stackoverflow.com/questions/1007481/how-do-i-replace-whitespaces-with-underscore-and-vice-versa
def urlify(s):
    # remove all non-word characters (everything except numbers and letters)
    s = re.sub(r"[^\w\s]", '', s)

    # replace all runs of whitespace with a single dash
    s = re.sub(r"\s+", '-', s)

    return s
