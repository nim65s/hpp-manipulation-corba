// Copyright (c) 2012 CNRS
// Author: Florent Lamiraux
//
// This file is part of hpp-manipulation-corba.
// hpp-manipulation-corba is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation-corba is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation-corba.  If not, see
// <http://www.gnu.org/licenses/>.

#include <hpp/corbaserver/server.hh>
#include <hpp/manipulation/package-config.hh>

#if HPP_MANIPULATION_HAS_WHOLEBODY_STEP
# include <hpp/corbaserver/wholebody-step/server.hh>
#endif
#ifdef HPP_MANIPULATION_HAS_RBPRM  
# include <hpp/corbaserver/rbprm/server.hh>
#endif // HPP_MANIPULATION_HAS_RBPRM  

#include <hpp/corbaserver/manipulation/server.hh>
#include <hpp/manipulation/problem-solver.hh>

typedef hpp::corbaServer::Server CorbaServer;
typedef hpp::manipulation::Server ManipulationServer;

typedef hpp::core::ProblemSolver CoreProblemSolver;
typedef hpp::core::ProblemSolverPtr_t CoreProblemSolverPtr_t;
typedef hpp::manipulation::ProblemSolver ProblemSolver;
typedef hpp::manipulation::ProblemSolverPtr_t ProblemSolverPtr_t;

int main (int argc, const char* argv [])
{
  ProblemSolverPtr_t problemSolver = new ProblemSolver();

  CorbaServer corbaServer (problemSolver, argc, argv, true);

  ManipulationServer manipServer (argc, argv, true);
  manipServer.setProblemSolverMap (corbaServer.problemSolverMap());

  corbaServer.startCorbaServer ();

#if HPP_MANIPULATION_HAS_WHOLEBODY_STEP  
  hpp::wholebodyStep::Server wbsServer (argc, argv, true);
  wbsServer.setProblemSolverMap (corbaServer.problemSolverMap());
  wbsServer.startCorbaServer ("hpp", "corbaserver",
				"wholebodyStep", "problem");
#endif

#ifdef HPP_MANIPULATION_HAS_RBPRM  
  // hpp-rbprm-corba does not use the problem solver map so we have to create
  // manually one problem solver for it.
  // Python script MUST call selectProblem("rbprm") before any request
  // related to the RBPRM problem they wish to solve.
  CoreProblemSolverPtr_t problemSolverRbprm = CoreProblemSolver::create();
  corbaServer.problemSolverMap()->map_["rbprm"] = problemSolverRbprm;
  hpp::rbprm::Server rbprmServer (argc, argv, true);
  rbprmServer.setProblemSolver (problemSolverRbprm);
  rbprmServer.startCorbaServer ("hpp", "corbaserver",
				"rbprm");
#endif // HPP_MANIPULATION_HAS_RBPRM  

  manipServer.startCorbaServer ("hpp", "corbaserver",
				"manipulation");
  corbaServer.processRequest(true);
}
