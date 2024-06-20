#!/usr/bin/env python
#
# Copyright (c) 2014 CNRS
# Author: Florent Lamiraux
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


from hpp.corbaserver.problem_solver import ProblemSolver as Parent


def newProblem(client=None, name=None):
    from hpp.corbaserver.problem_solver import newProblem

    if client is None:
        from hpp.manipulation import Client

        client = Client()
    newProblem(client=client, name=name)


class ProblemSolver(Parent):
    """
    Definition of a manipulation planning problem

    This class wraps the Corba client to the server implemented by
    libhpp-manipulation-corba.so

    Some method implemented by the server can be considered as private. The
    goal of this class is to hide them and to expose those that can be
    considered as public.
    """

    def __init__(self, robot):
        super().__init__(robot, hppcorbaClient=robot.client.basic)

    def selectProblem(self, name):
        """
        Select a problem by its name.
        If no problem with this name exists, a new
        hpp::manipulation::ProblemSolver is created and selected.
        \\param name the problem name.
        \\return true if a new problem was created.
        """
        return self.client.manipulation.problem.selectProblem(name)

    def getAvailable(self, type):
        """
        Return a list of available elements of type type
        \\param type enter "type" to know what types I know of.
                    This is case insensitive.
        """
        if type.lower() == "type":
            res = self.client.basic.problem.getAvailable(
                type
            ) + self.client.manipulation.problem.getAvailable(type)
            return res
        try:
            return self.client.basic.problem.getAvailable(type)
        except Exception:
            return self.client.manipulation.problem.getAvailable(type)

    def getSelected(self, type):
        """
        Return a list of selected elements of type type
        \\param type enter "type" to know what types I know of.
                    This is case insensitive.
        \\note For most of the types, the list will contain only one element.
        """
        try:
            return self.client.basic.problem.getSelected(type)
        except Exception:
            return self.client.manipulation.problem.getSelected(type)

    # # \\name Contact surfaces
    #
    #  In placement states, objects are in contact with other objects or with
    #  the environment through contact surfaces.
    #  \\{

    def getEnvironmentContactNames(self):
        """
        \\copydoc hpp::corbaserver::manipulation::Problem::getEnvironmentContactNames
        """
        return self.client.manipulation.problem.getEnvironmentContactNames()

    def getRobotContactNames(self):
        """
        \\copydoc hpp::corbaserver::manipulation::Problem::getRobotContactNames
        """
        return self.client.manipulation.problem.getRobotContactNames()

    def getEnvironmentContact(self, name):
        """
        \\copydoc hpp::corbaserver::manipulation::Problem::getEnvironmentContact
        """
        return self.client.manipulation.problem.getEnvironmentContact(name)

    def getRobotContact(self, name):
        """
        \\copydoc hpp::corbaserver::manipulation::Problem::getRobotContact
        """
        return self.client.manipulation.problem.getRobotContact(name)

    # # \\}

    # # \\name Constraints
    #  \\{

    def createPlacementConstraints(
        self, placementName, shapeName, envContactName, width=0.05
    ):
        """
        Create placement and pre-placement constraints

        \\param width set to None to skip creation of pre-placement constraint
        \\return names of the placement and preplacement constraints

        See hpp::corbaserver::manipulation::Problem::createPlacementConstraint
        and hpp::corbaserver::manipulation::Problem::createPrePlacementConstraint
        """
        name = placementName
        self.client.manipulation.problem.createPlacementConstraint(
            name, shapeName, envContactName
        )
        if width is not None:
            prename = "pre_" + name
            self.client.manipulation.problem.createPrePlacementConstraint(
                prename, shapeName, envContactName, width
            )
            return name, prename
        return name

    def createQPStabilityConstraint(self, *args):
        """
        Create QP Static stability constraint

         \\copydoc hpp::corbaserver::manipulation::Problem::createQPStabilityConstraint
        """
        self.client.manipulation.problem.createQPStabilityConstraint(*args)

    def registerConstraints(self, *args):
        """
        \\copydoc hpp::corbaserver::manipulation::Problem::registerConstraints
        """
        self.client.manipulation.problem.registerConstraints(*args)

    def balanceConstraints(self):
        """
        Return balance constraints created by method
        ProblemSolver.createStaticStabilityConstraints
        """
        return self.balanceConstraints_

    def getConstantRightHandSide(self, constraintName):
        """
        Get whether right hand side of a numerical constraint is constant
         \\param constraintName Name of the numerical constraint,
         \\return whether right hand side is constant
        """
        return self.client.basic.problem.getConstantRightHandSide(constraintName)

    def lockFreeFlyerJoint(
        self, freeflyerBname, lockJointBname, values=(0, 0, 0, 0, 0, 0, 1)
    ):
        """
        Lock degree of freedom of a FreeFlyer joint
        \\param freeflyerBname base name of the joint
               (It will be completed by '_xyz' and '_SO3'),
        \\param lockJointBname base name of the LockedJoint constraints
               (It will be completed by '_xyz' and '_SO3'),
        \\param values config of the locked joints (7 float)
        """
        lockedJoints = list()
        self.createLockedJoint(lockJointBname, freeflyerBname, values)
        lockedJoints.append(lockJointBname)
        return lockedJoints

    def lockPlanarJoint(self, jointName, lockJointName, values=(0, 0, 1, 0)):
        """
        Lock degree of freedom of a planar joint
        \\param jointName name of the joint
               (It will be completed by '_xy' and '_rz'),
        \\param lockJointName name of the LockedJoint constraint
        \\param values config of the locked joints (4 float)
        """
        lockedJoints = list()
        self.createLockedJoint(lockJointName, jointName, values)
        lockedJoints.append(lockJointName)
        return lockedJoints

    # # \\}

    # # \\name Solve problem and get paths
    #  \\{

    def setTargetState(self, stateId):
        """
        Set the problem target to stateId
        The planner will look for a path from the init configuration to a configuration
        in state stateId
        """
        self.client.manipulation.problem.setTargetState(stateId)

    # # \\}
