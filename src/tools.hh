// Copyright (c) 2016 CNRS
// Author: Joseph Mirabel
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#ifndef HPP_MANIPULATION_CORBA_TOOLS_HH
#define HPP_MANIPULATION_CORBA_TOOLS_HH

#include <hpp/corbaserver/conversions.hh>
#include <hpp/manipulation/problem-solver.hh>
#include <pinocchio/spatial/se3.hpp>

namespace hpp {
using corbaServer::c_str;
using corbaServer::floatSeqToVector;
using corbaServer::matrixToIntSeqSeq;
using corbaServer::toIntSeq;
using corbaServer::toNames_t;
using corbaServer::vectorToFloatSeq;
using manipulation::DevicePtr_t;
using manipulation::ProblemSolverPtr_t;
using pinocchio::Transform3s;

inline std::vector<std::string> toStringVector(const Names_t& names) {
  typedef std::vector<std::string> Out_t;
  return corbaServer::toStrings<Out_t>(names);
}

inline std::list<std::string> toStringList(const Names_t& names) {
  typedef std::list<std::string> Out_t;
  return corbaServer::toStrings<Out_t>(names);
}

inline void Transform3sTohppTransform(const Transform3s& transform,
                                      CORBA::Double* config) {
  corbaServer::toHppTransform(transform, config);
}

inline void hppTransformToTransform3s(const CORBA::Double* inConfig,
                                      Transform3s& transform) {
  corbaServer::toTransform3s(inConfig, transform);
}

DevicePtr_t getRobotOrThrow(ProblemSolverPtr_t p);
}  // namespace hpp

#endif  // HPP_MANIPULATION_CORBA_TOOLS_HH
