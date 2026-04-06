/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#ifndef _BASE_TEB_EDGES_H_
#define _BASE_TEB_EDGES_H_

#include "steb_planner/steb_optimal/steb_config.h"

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>
#define UNUSED(x) (void)(x)

namespace steb_planner
{

// BaseTebUnaryEdge: Base edge connecting a single vertex in the STEB optimization problem
template <int D, typename E, typename VertexXi>
class BaseTebUnaryEdge : public g2o::BaseUnaryEdge<D, E, VertexXi>
{
public:
  using typename g2o::BaseUnaryEdge<D, E, VertexXi>::ErrorVector;
  using g2o::BaseUnaryEdge<D, E, VertexXi>::computeError;

  // Compute and return error / cost value.
  ErrorVector & getError()
  {
    computeError();
    return _error;
  }

  // Assign the STEBConfig class for parameters.
  void setSTEBConfig(const STEBConfig & steb_cfg) { steb_cfg_ = &steb_cfg; }

  // Read values from input stream
  virtual bool read(std::istream & is)
  {
    UNUSED(is);
    return true;
  }

  // Write values to an output stream
  virtual bool write(std::ostream & os) const { return os.good(); }

protected:
  using g2o::BaseUnaryEdge<D, E, VertexXi>::_error;
  using g2o::BaseUnaryEdge<D, E, VertexXi>::_vertices;

  const STEBConfig * steb_cfg_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// BaseTebBinaryEdge: Base edge connecting two vertices in the STEB optimization problem
template <int D, typename E, typename VertexXi, typename VertexXj>
class BaseTebBinaryEdge : public g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>
{
public:
  using typename g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::ErrorVector;
  using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::computeError;

  // Compute and return error / cost value.
  ErrorVector & getError()
  {
    computeError();
    return _error;
  }

  // Assign the STEBConfig class for parameters.
  void setSTEBConfig(const STEBConfig & steb_cfg) { steb_cfg_ = &steb_cfg; }

  // Read values from input stream
  virtual bool read(std::istream & is)
  {
    UNUSED(is);
    return true;
  }

  // Write values to an output stream
  virtual bool write(std::ostream & os) const { return os.good(); }

protected:
  using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_error;
  using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_vertices;

  const STEBConfig * steb_cfg_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// BaseTebMultiEdge: Base edge connecting multiple vertices in the STEB optimization problem
template <int D, typename E>
class BaseTebMultiEdge : public g2o::BaseMultiEdge<D, E>
{
public:
  using typename g2o::BaseMultiEdge<D, E>::ErrorVector;
  using g2o::BaseMultiEdge<D, E>::computeError;

  // Overwrites resize() from the parent class
  virtual void resize(size_t size)
  {
    g2o::BaseMultiEdge<D, E>::resize(size);

    for (std::size_t i = 0; i < _vertices.size(); ++i) _vertices[i] = NULL;
  }

  // Compute and return error / cost value.
  ErrorVector & getError()
  {
    computeError();
    return _error;
  }

  // Assign the STEBConfig class for parameters.
  void setSTEBConfig(const STEBConfig & steb_cfg) { steb_cfg_ = &steb_cfg; }

  // Read values from input stream
  virtual bool read(std::istream & is)
  {
    UNUSED(is);
    return true;
  }

  // Write values to an output stream
  virtual bool write(std::ostream & os) const { return os.good(); }

protected:
  using g2o::BaseMultiEdge<D, E>::_error;
  using g2o::BaseMultiEdge<D, E>::_vertices;

  const STEBConfig * steb_cfg_;  //!< Store TebConfig class for parameters

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace steb_planner

#endif
