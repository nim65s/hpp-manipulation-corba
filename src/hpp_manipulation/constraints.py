#!/usr/bin/env python
#
# Copyright (c) 2014 CNRS
# Author: Joseph Mirabel
#

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.


class Constraints:
    """
    Container of numerical constraints

    Numerical constraints are stored as
    \\li grasp,
    \\li pregrasp, or
    \\li numerical constraint,
    """

    def __init__(self, grasps=[], pregrasps=[], numConstraints=[], lockedJoints=[]):
        if isinstance(grasps, str):
            raise TypeError("argument grasps should be a list of strings")
        if isinstance(pregrasps, str):
            raise TypeError("argument pregrasps should be a list of strings")
        if isinstance(numConstraints, str):
            raise TypeError("argument numConstraints should be a list of strings")
        if lockedJoints != []:
            from warnings import warn

            warn(
                "argument lockedJoints in constructor of class "
                + "hpp_manipulation.constraints.Constraints "
                + "is deprecated. Locked joints are handled as numerical "
                + "constraints."
            )
            numConstraints.extend(lockedJoints)
        self._grasps = set(grasps)
        self._pregrasps = set(pregrasps)
        self._numConstraints = set(numConstraints)

    def __add__(self, other):
        res = Constraints(
            grasps=self._grasps | other._grasps,
            pregrasps=self._pregrasps | other._pregrasps,
            numConstraints=self._numConstraints | other._numConstraints,
        )
        return res

    def __sub__(self, other):
        res = Constraints(
            grasps=self._grasps - other._grasps,
            pregrasps=self._pregrasps - other._pregrasps,
            numConstraints=self._numConstraints - other._numConstraints,
        )
        return res

    def __iadd__(self, other):
        self._grasps |= other._grasps
        self._pregrasps |= other._pregrasps
        self._numConstraints |= other._numConstraints
        return self

    def __isub__(self, other):
        self._grasps -= other._grasps
        self._pregrasps -= other._pregrasps
        self._numConstraints -= other._numConstraints
        return self

    def empty(self):
        for s in [self._grasps, self._pregrasps, self._numConstraints]:
            if len(s) > 0:
                return False
        return True

    @property
    def grasps(self):
        return list(self._grasps)

    @property
    def pregrasps(self):
        return list(self._pregrasps)

    @property
    def numConstraints(self):
        return list(self._numConstraints)

    def __str__(self):
        res = "constraints\n"
        res += "  grasps: "
        for c in self._grasps:
            res += c + ", "
        res += "\n  pregrasps: "
        for c in self._pregrasps:
            res += c + ", "
        res += "\n  numConstraints: "
        for c in self._numConstraints:
            res += c + ", "
        return res
