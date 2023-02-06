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

from __future__ import print_function
from subprocess import Popen
from .constraints import Constraints


class ConstraintGraph(object):
    """
    Definition of a constraint graph.

    This class wraps the Corba client to the server implemented by
    libhpp-manipulation-corba.so

    Some method implemented by the server can be considered as private. The
    goal of this class is to hide them and to expose those that can be
    considered as public.
    """

    cmdDot = {
        "pdf": ["dot", "-Gsize=7.5,10", "-Tpdf"],
        "svg": ["dot", "-Gsize=7.5,10", "-Tsvg"],
    }
    cmdViewer = {"pdf": ["evince"], "svg": ["firefox"]}

    def __init__(self, robot, graphName, makeGraph=True):
        self.robot = robot
        self.client = robot.client.manipulation
        self.clientBasic = robot.client.basic
        self.graph = robot.client.manipulation.graph
        self.name = graphName
        self.grasps = dict()
        self.pregrasps = dict()
        # A dictionnary mapping the node names to their ID
        self.nodes = dict()
        # A dictionnary mapping the edge names to their ID
        self.edges = dict()
        if makeGraph:
            self.graphId = self.graph.createGraph(graphName)
        else:
            # fetch graph
            try:
                g = self.graph.getGraph()
                self.graphId = g[0].id
                for n in g[1].nodes:
                    if n.name in self.nodes:
                        print("Erasing node", n.name, "id", self.nodes[n.name])
                    self.nodes[n.name] = n.id
                for e in g[1].edges:
                    if e.name in self.edges:
                        print("Erasing edge", e.name, "id", self.edges[e.name])
                    self.edges[e.name] = e.id
            except Exception:
                pass

        self.textToTex = dict()

    # \\name Building the constraint graph
    # \\{

    def createNode(self, node, waypoint=False, priority=None):
        """
        Create one or several node
        \\param node name (resp. list of names) of the node(s) to be created.
        \\param waypoint set to True when creating waypoint nodes.
        \\param priority integer (resp. list of) used to order the states.
                If two states have the same priority,
                then the order is the order of creation.
        \\note The order is important.
               The first should be the most restrictive one as a configuration
        will be in the first node for which the constraint are satisfied.
        """
        if type(node) is str:
            node = [node]
        if priority is None:
            priority = [
                0,
            ] * len(node)
        elif isinstance(priority, int):
            priority = [priority]
        for n, p in zip(node, priority):
            self.nodes[n] = self.graph.createNode(self.graphId, self._(n), waypoint, p)

    def createEdge(self, nodeFrom, nodeTo, name, weight=1, isInNode=None):
        """
        Create an edge
        \\param nodeFrom, nodeTo the extremities of the edge,
        \\param name name of the edge,
        \\param weight see note,
        \\param isInNode name of the node in which paths of the edge are included.
               if None, it consists of the node coming the latest in the list of
               nodes.
        \\note The weights define the probability of selecting an edge among all the
        outgoing edges of a node. The probability of an edge is
        \\f$ \\frac{w_i}{\\sum_j{w_j}} \\f$,
        where each \\f$ w_j \\f$ corresponds to an outgoing edge from a given node.
        To have an edge that cannot be selected by the M-RRT algorithm but is still
        acceptable, set its weight to zero.
        """
        if type(isInNode) != str:
            if isInNode is not None:
                from warnings import warn

                warn("argument isInNode should be of type string")
            else:
                if self.nodes[nodeFrom] > self.nodes[nodeTo]:
                    isInNode = nodeFrom
                else:
                    isInNode = nodeTo
        self.edges[name] = self.graph.createEdge(
            self.nodes[nodeFrom],
            self.nodes[nodeTo],
            self._(name),
            weight,
            self.nodes[isInNode],
        )
        return self.edges[name]

    def setContainingNode(self, edge, node):
        """
        Set in which node an edge is.
        \\param edge the edge,
        \\param node the node.
        Paths satisfying the edge constraints satisfy the node constraints.
        """
        return self.graph.setContainingNode(self.edges[edge], self.nodes[node])

    def getContainingNode(self, edge):
        """
        Get in which node an edge is.
        \\param edge the edge,
        Paths satisfying the edge constraints satisfy the node constraints.
        """
        return self.graph.getContainingNode(self.edges[edge])

    def setShort(self, edge, isShort):
        """
        Set that an edge is short
        \\param edge name of the edge
        \\param True or False

        When an edge is tagged as short, extension along this edge is
        done differently in RRT-like algorithms. Instead of projecting
        a random configuration in the destination node, the
        configuration to extend itself is projected in the destination
        node. This makes the rate of success higher.
        """
        return self.client.graph.setShort(self.edges[edge], isShort)

    def isShort(self, edge):
        return self.client.graph.isShort(self.edges[edge])

    def createWaypointEdge(
        self,
        nodeFrom,
        nodeTo,
        name,
        nb=1,
        weight=1,
        isInNode=None,
        automaticBuilder=True,
    ):
        """
        Create a WaypointEdge.
        \\param nodeFrom, nodeTo, name, weight, isInNode see createEdge note,
        \\param nb number of waypoints,
        \\note See documentation of class hpp::manipulation::graph::WaypointEdge
                for more information.

        \\warning Waypoint are now specified by hand to allow finer handling
        of edge types between waypoints. This function has been updated to be
        backward compatible but except for the return value.
        For a finer control of what you are doing, set automaticBuilder to
        False.
        """

        if type(isInNode) != str:
            if isInNode is not None:
                from warnings import warn

                warn("argument isInNode should be of type string")
            else:
                if self.nodes[nodeFrom] > self.nodes[nodeTo]:
                    isInNode = nodeFrom
                else:
                    isInNode = nodeTo

        if automaticBuilder:
            n = name + "_e" + str(nb)
        else:
            n = name
        wid = self.edges[n] = self.graph.createWaypointEdge(
            self.nodes[nodeFrom],
            self.nodes[nodeTo],
            self._(name),
            nb,
            weight,
            self.nodes[isInNode],
        )

        if not automaticBuilder:
            return

        waypoints = list()
        previous = nodeFrom
        for i in range(nb):
            waypoints.append((name + "_e" + str(i), name + "_n" + str(i)))
            n = waypoints[-1][1]
            e = waypoints[-1][0]
            newN = self.nodes[n] = self.graph.createNode(self.graphId, self._(n), True)
            newE = self.edges[e] = self.createEdge(previous, n, self._(e), -1, isInNode)
            self.graph.setWaypoint(wid, i, newE, newN)
            previous = n

    def createLevelSetEdge(self, nodeFrom, nodeTo, name, weight=1, isInNode=None):
        """
        Create a LevelSetEdge.
        \\param nodeFrom, nodeTo, name, weight, isInNode see createEdge note.
        \\note See documentation of class hpp::manipulation::graph::LevelSetEdge
                for more information.
        """
        if isInNode is None:
            if self.nodes[nodeFrom] > self.nodes[nodeTo]:
                isInNode = nodeFrom
            else:
                isInNode = nodeTo
        self.edges[name] = self.graph.createLevelSetEdge(
            self.nodes[nodeFrom],
            self.nodes[nodeTo],
            self._(name),
            weight,
            self.nodes[isInNode],
        )

    def createGrasp(self, name, gripper, handle):
        """
        Create grasp constraints between robot gripper and object handle

        Creates two contraints between a handle and a gripper.
        \\li The first constraint named "${name}" is defined by
        the type of handle. For instance, an axial handle defines
        a five degree of freedom constraint with free rotation
        around the x-axis.
        \\li the second constraint named "${name}/complement" is
        the complement to the full transformation constraint. For the axial
        handle, it corresponds to the rotation around x.

        \\param name prefix of the constraint names for storing in
               ProblemSolver map,
        \\param gripper name of the gripper used when it has been created
        \\param handle name of the handle in the form "object/handle"
        where object is the name of the object owning the handle and handle
        is the name of the handle in this object.

        \\sa method hpp::corbaserver::manipulation::Problem::createGrasp.
        """
        self.client.problem.createGrasp(self._(name), gripper, handle)
        self.grasps[name] = (self._(name),)

    def createPreGrasp(self, name, gripper, handle):
        """
        Create pre-grasp constraints between robot gripper and object handle

        Creates two contraints between a handle and a gripper.
        \\li The first constraint named "${name}" is the same as the grasp
        defined in createGrasp, except that the translation along x is not
        constrained. For instance, an axial handle defines
        a four degree of freedom constraint with free rotation and translation
        around/along the x-axis,
        \\li the second constraint named "${name}/double_ineq" is a double
        inequality on the relative x-position of the handle and of the gripper.
        the bounds of the inequality are for now [-.001 c, 2.001 c].

        \\param name prefix of the constraint names for storing in
               ProblemSolver map,
        \\param gripper name of the gripper used when it has been created
        \\param handle name of the handle in the form "object/handle"
        where object is the name of the object owning the handle and handle
        is the name of the handle in this object,

        \\sa hpp::corbaserver::manipulation::Problem::createPreGrasp
        """
        self.client.problem.createPreGrasp(self._(name), gripper, handle)
        self.pregrasps[name] = (self._(name),)

    def setProblemConstraints(self, name, target):
        """
        Set the problem constraints to the specified constraint.

        \\param idComp ID of a node or a configuration
        \\param target: ignored for states. For edges:
               \\li true: uses the edge target constraint
               \\li false: uses the edge path constraint
        """
        if name in self.nodes:
            id = self.nodes[name]
        elif name in self.edges:
            id = self.edges[name]
        else:
            raise RuntimeError("No node or edge with name {0}".format(name))
        return self.client.problem.setConstraints(id, target)

    # Add the constraints to an edge, a node or the whole graph
    #
    # This method adds the constraints to an element of the graph and handles
    # the special cases of grasp and pregrasp constraints.
    #
    # \\param graph set to true if you are defining constraints for every nodes,
    # \\param node edge name of a component of the graph,
    #
    # \\param constraints set of constraints containing grasps, pregrasps,
    #                    numerical constraints and locked joints.
    #                    It must be of type hpp.corbaserver.manipulation.Constraints.
    # \\note Exaclty one of the parameter graph, node and edge must be set.
    def addConstraints(self, graph=False, node=None, edge=None, constraints=None):
        """
        Add the constraints to an edge, a node or the whole graph

          This method adds the constraints to an element of the graph and
          handles the special cases of grasp and pregrasp constraints.

          input
            graph: set to true if you are defining constraints for every nodes,
            node, edge: name of a component of the graph,
            constraints: set of constraints containing grasps, pregrasps,
                         numerical constraints and locked joints.
                         It must be of type hpp.corbaserver.manipulation.Constraints.
          note: Exaclty one of the parameter graph, node and edge must be set.
        """
        if not isinstance(constraints, Constraints):
            raise TypeError("argument constraints should be of type Constraints")
        return self._addConstraints(
            graph=graph,
            node=node,
            edge=edge,
            grasps=constraints.grasps,
            pregrasps=constraints.pregrasps,
            numConstraints=constraints.numConstraints,
        )

    def _addConstraints(
        self,
        graph=False,
        node=None,
        edge=None,
        grasps=None,
        pregrasps=None,
        numConstraints=[],
    ):
        if not type(graph) is bool:
            raise TypeError(
                "ConstraintGraph.addConstraints: "
                + "graph argument should be a boolean, got "
                + repr(graph)
            )
        nc = numConstraints[:]
        if grasps is not None:
            for g in grasps:
                for pair in self.grasps[g]:
                    if edge is not None:
                        nc.append(pair.constraint + "/complement")
                    else:
                        nc.append(pair.constraint)
        if pregrasps is not None:
            for g in pregrasps:
                for pair in self.pregrasps[g]:
                    nc.append(pair.constraint)

        if node is not None:
            self.graph.addNumericalConstraints(self.nodes[node], nc)
            self.graph.addNumericalConstraintsForPath(self.nodes[node], nc)
        elif edge is not None:
            self.graph.addNumericalConstraints(self.edges[edge], nc)
        elif graph:
            self.graph.addNumericalConstraints(self.graphId, nc)

    def removeCollisionPairFromEdge(self, edge, joint1, joint2):
        """
        Remove collision pairs from an edge

        \\param edge name of the edge,
        \\param joint1, joint2, names of the joints defining the pair.
        """
        return self.graph.removeCollisionPairFromEdge(self.edges[edge], joint1, joint2)

    def addLevelSetFoliation(
        self,
        edge,
        condGrasps=None,
        condPregrasps=None,
        condNC=[],
        condLJ=[],
        paramGrasps=None,
        paramPregrasps=None,
        paramNC=[],
        paramLJ=[],
    ):
        """
        Add the numerical constraints to a LevelSetEdge that create the foliation.
        \\param edge name of a LevelSetEdge of the graph.
        \\param condGrasps, condPregrasps name, or list of names, of grasp or pregrasp
                that define the foliated manifold
        \\param condNC, condLJ numerical constraints and locked joints that define the
                foliated manifold
        \\param paramGrasps, paramPregrasps name, or list of names, of grasp or
                pregrasp that parametrize the foliation
        \\param paramNC, paramLJ numerical constraints and locked joints
                that parametrize the foliation
        """

        if condLJ != []:
            raise RuntimeError(
                "Locked joints are now handled as numerical"
                " constraints. Please merge elements in list"
                " condLJ with condNC"
            )
        if paramLJ != []:
            raise RuntimeError(
                "Locked joints are now handled as numerical"
                " constraints. Please merge elements in list"
                " paramLJ with paramNC"
            )
        cond_nc = condNC[:]
        if condGrasps is not None:
            for g in condGrasps:
                for pair in self.grasps[g]:
                    cond_nc.append(pair.constraint)
        if condPregrasps is not None:
            for g in condPregrasps:
                for pair in self.pregrasps[g]:
                    cond_nc.append(pair.constraint)

        param_nc = paramNC[:]
        if paramGrasps is not None:
            for g in paramGrasps:
                for pair in self.grasps[g]:
                    param_nc.append(pair.constraint)
        if paramPregrasps is not None:
            for g in paramPregrasps:
                for pair in self.pregrasps[g]:
                    param_nc.extend(pair.constraint)

        self.graph.addLevelSetFoliation(self.edges[edge], cond_nc, param_nc)

    def getWeight(self, edge):
        """
        Get weight of an edge
        """
        return self.client.graph.getWeight(self.edges[edge])

    def setWeight(self, edge, weight):
        """
        Set weight of an edge
        """
        if self.client.graph.getWeight(self.edges[edge]) == -1:
            raise RuntimeError(
                'You cannot set weight for "'
                + edge
                + '". Perhaps it is a waypoint edge ?'
            )
        return self.client.graph.setWeight(self.edges[edge], weight)

    # # \\}

    def addTextToTeXTranslation(self, text, tex):
        """
        Add entry to the local dictionnary
        \\param text plain text
        \\param tex its latex translation
        \\sa ConstraintGraph.setTextToTeXTranslation
        """
        self.textToTex[text] = tex

    def setTextToTeXTranslation(self, textToTex):
        """
        Set the local dictionnary
        \\param textToTex a dictionnary of (plain text, TeX replacment)
        If the name of a node or an edges is a key of the dictionnary,
        it is replaced by the corresponding value.
        """
        if type(textToTex) is not dict:
            raise TypeError("Argument textToTex must be a dictionnary.")
        self.textToTex = textToTex

    # #
    # \\name Working with the constraint graph
    # \\{

    def display(
        self,
        dotOut="/tmp/constraintgraph.dot",
        pdfOut="/tmp/constraintgraph",
        format="pdf",
        open=True,
    ):
        """
        Display the current graph.
        The graph is printed in DOT format. Command dot must be
        available.
        \\param dotOut full path of the generated DOT file.
        \\param pdfOut fill path of the generated PDF document.
        \\param openPDF set to False if you just want to generate the PDF.
        \\note DOT and PDF files will be overwritten and are not automatically
        deleted so you can keep them.
        """

        self.graph.display(dotOut)
        if format not in self.cmdDot:
            raise TypeError(
                "This format is not supported. See member cmdDot for supported format."
            )
        dotCmd = self.cmdDot[format]
        dotCmd.append("-o" + pdfOut + "." + format)
        dotCmd.append(dotOut)
        dot = Popen(dotCmd)
        dot.wait()
        if open and format in self.cmdViewer:
            viewCmd = self.cmdViewer[format]
            viewCmd.append(pdfOut + "." + format)
            Popen(viewCmd)

    def getNodesConnectedByEdge(self, edge):
        """
        Get nodes connected by an edge

        \\param edge name of the edge
        \\param from name of the node the edge starts from,
        \\param to name of the node the edge finishes in.
        """
        return self.client.graph.getNodesConnectedByEdge(self.edges[edge])

    def applyNodeConstraints(self, node, input):
        """
        Apply constaints to a configuration

        \\param node name of the node the constraints of which to apply
        \\param input input configuration,
        \\retval output output configuration,
        \\retval error norm of the residual error.
        """
        return self.client.graph.applyNodeConstraints(self.nodes[node], input)

    def applyEdgeLeafConstraints(self, edge, qfrom, input):
        """
        Apply edge constaints to a configuration

        \\param edge name of the edge
        \\param qfrom configuration defining the right hand side of the edge
               constraint,
        \\param input input configuration,
        \\retval output output configuration,
        \\retval error norm of the residual error.

        If success, the output configuration is reachable from qfrom along
        the transition.
        """
        return self.client.graph.applyEdgeLeafConstraints(
            self.edges[edge], qfrom, input
        )

    def generateTargetConfig(self, edge, qfrom, input):
        """
        Generate configuration in destination state on a given leaf

        \\param edge name of the edge
        \\param qfrom configuration defining the right hand side of the edge
               constraint,
        \\param input input configuration,
        \\retval output output configuration,
        \\retval error norm of the residual error.

        Compute a configuration in the destination node of the edge,
        reachable from qFrom.
        """
        return self.client.graph.generateTargetConfig(self.edges[edge], qfrom, input)

    def buildAndProjectPath(self, edge, qb, qe):
        """
        Build a path from qb to qe using the Edge::build.
        \\param edge name of the edge to use.
        \\param qb configuration at the beginning of the path
        \\param qe configuration at the end of the path
        \\retval return true if the path is built and fully projected.
        \\retval indexNotProj -1 is the path could not be built. The index
                             of the built path (before projection) in the
                             in the ProblemSolver path vector.
        \\retval indexProj -1 is the path could not be fully projected. The
                          index of the built path (before projection) in the
                          in the ProblemSolver path vector.
        No path validation is made. The paths can be retrieved using
        corbaserver::Problem::configAtParam
        """
        return self.client.problem.buildAndProjectPath(self.edges[edge], qb, qe)

    def getConfigErrorForNode(self, nodeId, config):
        """
        Get error of a config with respect to a node constraint

        \\param node name of the node.
        \\param config Configuration,
        \\retval error the error of the node constraint for the
               configuration
        \\return whether the configuration belongs to the node.
        Call method core::ConstraintSet::isSatisfied for the node
        constraints.
        """
        return self.client.graph.getConfigErrorForNode(self.nodes[nodeId], config)

    def getNode(self, config):
        """
         Get the node corresponding to the state of the configuration.
        \\param dofArray the configuration.
        \\return the name of the node
        """
        nodeId = self.client.graph.getNode(config)
        for n, id in self.nodes.items():
            if id == nodeId:
                return n
        raise RuntimeError("No node with id {0}".format(nodeId))

    def getConfigErrorForEdge(self, edgeId, config):
        """
        Get error of a config with respect to a edge constraint

        \\param edge name of the edge.
        \\param config Configuration,
        \\retval error the error of the edge constraint for the
               configuration
        \\return whether the configuration belongs to the edge.
        Call methods core::ConfigProjector::rightHandSideFromConfig with
        the input configuration and then core::ConstraintSet::isSatisfied
        on the edge constraints.
        """
        return self.client.graph.getConfigErrorForEdge(self.edges[edgeId], config)

    def getConfigErrorForEdgeLeaf(self, edgeId, leafConfig, config):
        """
        Get error of a config with respect to an edge foliation leaf

        \\param edgeId id of the edge.
        \\param leafConfig Configuration that determines the foliation leaf,
        \\param config Configuration the error of which is computed
        \\retval error the error
        \\return whether config can be the end point of a path of the edge
                starting at leafConfig
        Call methods core::ConfigProjector::rightHandSideFromConfig with
        leafConfig and then core::ConstraintSet::isSatisfied with config.
        on the edge constraints.
        """
        return self.client.graph.getConfigErrorForEdgeLeaf(
            self.edges[edgeId], leafConfig, config
        )

    def getConfigErrorForEdgeTarget(self, edgeId, leafConfig, config):
        """
        Get error of a config with respect to the target of an edge foliation leaf

        \\param edgeId id of the edge.
        \\param leafConfig Configuration that determines the foliation leaf,
        \\param config Configuration the error of which is computed
        \\retval error the error
        \\return whether config can be the end point of a path of the edge
                starting at leafConfig
        Call methods core::ConfigProjector::rightHandSideFromConfig with
        leafConfig and then core::ConstraintSet::isSatisfied with config.
        on the edge constraints.
        """
        return self.client.graph.getConfigErrorForEdgeTarget(
            self.edges[edgeId], leafConfig, config
        )

    def displayNodeConstraints(self, node):
        """
        Print set of constraints relative to a node in a string

        \\param config Configuration,
        \\param nodeId id of the node.
        \\return string displaying constraints
        """
        return self.graph.displayNodeConstraints(self.nodes[node])

    def displayEdgeConstraints(self, edge):
        """
        Print set of constraints relative to an edge in a string

        \\param config Configuration,
        \\param edgeId id of the edge.
        \\return string displaying path constraints of the edge
        """
        return self.graph.displayEdgeConstraints(self.edges[edge])

    def displayEdgeTargetConstraints(self, edge):
        """
        Print set of constraints relative to an edge in a string

        \\param config Configuration,
        \\param edgeId id of the edge.
        \\return string displaying constraints of the edge and of the target
                node
        """
        return self.graph.displayEdgeTargetConstraints(self.edges[edge])

    # \\}

    # \\name Automatic building
    # \\{
    @staticmethod
    def buildGenericGraph(
        robot,
        name,
        grippers,
        objects,
        handlesPerObjects,
        shapesPerObjects,
        envNames,
        rules=[],
    ):
        """
        # Build a graph
        \\return a Initialized ConstraintGraph object
        \\sa hpp::corbaserver::manipulation::Graph::autoBuild for complete
            documentation.
        """
        robot.client.manipulation.graph.autoBuild(
            name,
            grippers,
            objects,
            handlesPerObjects,
            shapesPerObjects,
            envNames,
            rules,
        )
        graph = ConstraintGraph(robot, name, makeGraph=False)
        graph.initialize()
        return graph

    def initialize(self):
        self.graph.initialize()

    # \\}

    def setSecurityMarginForEdge(self, edge, joint1, joint2, margin):
        """
        Set collision security margin for a pair of joints along an edge

        \\param edge name of the edge,
        \\param joint1, joint2 names of the joints or "universe" for the
               environment,
        \\param margin security margin.
        """
        # names = self.robot.getJointNames()
        self.graph.setSecurityMarginForEdge(self.edges[edge], joint1, joint2, margin)

    def getSecurityMarginMatrixForEdge(self, edge):
        return self.graph.getSecurityMarginMatrixForEdge(self.edges[edge])

    def _(self, text):
        """get the textToTex translation"""
        return self.textToTex.get(text, text)
