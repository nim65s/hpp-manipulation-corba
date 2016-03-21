// Copyright (C) 2014 CNRS-LAAS
// Author: Florent Lamiraux.
//
// This file is part of the hpp-manipulation-corba.
//
// hpp-manipulation-corba is free software: you can redistribute
// it and/or modify it under the terms of the GNU Lesser General
// Public License as published by the Free Software Foundation, either
// version 3 of the License, or (at your option) any later version.
//
// hpp-manipulation-corba is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with hpp-manipulation-corba.  If not, see
// <http://www.gnu.org/licenses/>.

#include <hpp/util/exception.hh>
#include <hpp/corbaserver/manipulation/server.hh>
#include "graph.impl.hh"
#include "problem.impl.hh"
#include "robot.impl.hh"

namespace hpp {
  namespace manipulation {
    Server::Server (int argc, const char* argv[], bool multiThread,
		    const std::string& poaName) : 
      graphImpl_ (new corba::Server <impl::Graph>
		  (argc, argv, multiThread, poaName)),
      problemImpl_ (new corba::Server <impl::Problem>
		    (argc, argv, multiThread, poaName)),
      robotImpl_ (new corba::Server <impl::Robot>
		  (argc, argv, multiThread, poaName)) {}

    Server::~Server () 
    {
      delete graphImpl_;
      delete problemImpl_;
      delete robotImpl_;
    }
    
    void Server::setProblemSolver (ProblemSolverPtr_t problemSolver)
    {
      graphImpl_->implementation ().setProblemSolver (problemSolver);
      problemImpl_->implementation ().setProblemSolver (problemSolver);
      robotImpl_->implementation ().setProblemSolver (problemSolver);
    }

    /// Start corba server
    void Server::startCorbaServer(const std::string& contextId,
				  const std::string& contextKind,
				  const std::string& objectId)
    {
      if (graphImpl_->startCorbaServer(contextId, contextKind,
				       objectId, "graph") != 0) {
	HPP_THROW_EXCEPTION (hpp::Exception,
			     "Failed to start corba graph server.");
      }
      if (robotImpl_->startCorbaServer(contextId, contextKind,
				       objectId, "robot") != 0) {
	HPP_THROW_EXCEPTION (hpp::Exception,
			     "Failed to start corba robot server.");
      }
      if (problemImpl_->startCorbaServer(contextId, contextKind,
					 objectId, "problem") != 0) {
	HPP_THROW_EXCEPTION (hpp::Exception,
			     "Failed to start corba problem server.");
      }
    }
  } // namespace manipulation
} // namespace hpp
