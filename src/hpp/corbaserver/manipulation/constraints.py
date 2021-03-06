#!/usr/bin/env python
#
# Copyright (c) 2014 CNRS
# Author: Joseph Mirabel
#
# This file is part of hpp-manipulation-corba.
# hpp-manipulation-corba is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-manipulation-corba is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-manipulation-corba.  If not, see
# <http://www.gnu.org/licenses/>.


## Container of numerical constraints
#
#  Numerical constraints are stored as
#  \li grasp,
#  \li pregrasp, or
#  \li numerical constraint,
class Constraints (object):
    def __init__ (self, grasps = [], pregrasps = [], numConstraints = [],
                  lockedJoints = []):
        if type (grasps) is str:
            raise TypeError ("argument grasps should be a list of strings")
        if type (pregrasps) is str:
            raise TypeError ("argument pregrasps should be a list of strings")
        if type (numConstraints) is str:
            raise TypeError \
                ("argument numConstraints should be a list of strings")
        if lockedJoints != []:
            from warnings import warn
            warn ("argument lockedJoints in constructor of class " +
                  "hpp.corbaserver.manipulation.constraints.Constraints " +
                  "is deprecated. Locked joints are handled as numerical " +
                  "constraints.")
            numConstraints.extend (lockedJoints)
        self._grasps = set (grasps)
        self._pregrasps = set (pregrasps)
        self._numConstraints = set (numConstraints)

    def __add__ (self, other):
        res = Constraints (grasps = self._grasps | other._grasps,
                           pregrasps = self._pregrasps |  other._pregrasps,
                           numConstraints =
                           self._numConstraints | other._numConstraints)
        return res

    def __sub__ (self, other):
        res = Constraints (grasps = self._grasps - other._grasps,
                           pregrasps = self._pregrasps - other._pregrasps,
                           numConstraints = self._numConstraints - \
                           other._numConstraints)
        return res

    def __iadd__ (self, other):
        self._grasps |= other._grasps
        self._pregrasps |= other._pregrasps
        self._numConstraints |= other._numConstraints
        return self

    def __isub__ (self, other):
        self._grasps -= other._grasps
        self._pregrasps -= other._pregrasps
        self._numConstraints -= other._numConstraints
        return self

    def empty (self):
        for s in [ self._grasps, self._pregrasps, self._numConstraints ]:
            if len(s) > 0: return False
        return True

    @property
    def grasps (self):
        return list (self._grasps)

    @property
    def pregrasps (self):
        return list (self._pregrasps)

    @property
    def numConstraints (self):
        return list (self._numConstraints)

    def __str__ (self):
        res = "constraints\n"
        res += "  grasps: "
        for c in self._grasps:
            res += c + ', '
        res += "\n  pregrasps: "
        for c in self._pregrasps:
            res += c + ', '
        res += "\n  numConstraints: "
        for c in self._numConstraints:
            res += c + ', '
        return res
