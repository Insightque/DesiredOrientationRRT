/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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
*********************************************************************/

/* Author: Seho Shin */

#ifndef OMPL_BASE_SPACES_CLOTHOID_STATE_SPACE_
#define OMPL_BASE_SPACES_CLOTHOID_STATE_SPACE_


#include "ompl/base/StateSpace.h"
#include "ompl/base/StateSpaceTypes.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SO2StateSpace.h"
#include "ompl/extensions/clothoid/Clothoid.h"
#include "ompl/base/MotionValidator.h"
#include <boost/math/constants/constants.hpp>

namespace ompl
{
    namespace base
    {

        /** \brief An clothoid state space where distance is measured by the
            length of Clothoid curves.
        */
        class ClothoidStateSpace : public CompoundStateSpace
        {
        public:

            /** \brief A state in SE(2): (x, y, yaw) */
            class StateType : public CompoundStateSpace::StateType
            {
            public:
                StateType() : CompoundStateSpace::StateType()
                {
                }

                /** \brief Get the X component of the state */
                double getX() const
                {
                    return as<RealVectorStateSpace::StateType>(0)->values[0];
                }

                /** \brief Get the Y component of the state */
                double getY() const
                {
                    return as<RealVectorStateSpace::StateType>(0)->values[1];
                }

                /** \brief Get the yaw component of the state. This is
                    the rotation in plane, with respect to the Z
                    axis. */
                double getYaw() const
                {
                    return as<SO2StateSpace::StateType>(1)->value;
                }

                /** \brief Set the X component of the state */
                void setX(double x)
                {
                    as<RealVectorStateSpace::StateType>(0)->values[0] = x;
                }

                /** \brief Set the Y component of the state */
                void setY(double y)
                {
                    as<RealVectorStateSpace::StateType>(0)->values[1] = y;
                }

                /** \brief Set the X and Y components of the state */
                void setXY(double x, double y)
                {
                    setX(x);
                    setY(y);
                }

                /** \brief Set the yaw component of the state. This is
                    the rotation in plane, with respect to the Z
                    axis. */
                void setYaw(double yaw)
                {
                    as<SO2StateSpace::StateType>(1)->value = yaw;
                }

				void setK(double k)
				{
					//k_ = k;
					as<RealVectorStateSpace::StateType>(2)->values[0] = k;
				}

				double getK() const
				{
					//return k_;
					return as<RealVectorStateSpace::StateType>(2)->values[0];
				}
			
            };


            /** \brief Complete description of a Dubins path */
            class ClothoidPath
            {
            public:
                ClothoidPath(Clothoid::ClothoidCurve cc)
                    : reverse_(false)
                {
					cc_ = cc;

                    l_ = cc.s_max;
					k_ = cc.k;
					dk_ = cc.dk;

                }
                double length() const
                {
                	return l_;
				}

				Clothoid::ClothoidCurve cc_;
                
				/** Path segment properties */
				double l_;
                double k_;
                double dk_;
                
				/** Whether the path should be followed "in reverse" */
                bool reverse_;
            };

            ClothoidStateSpace(double turningRadius = 0.17, bool isReversable =
					true)
                : CompoundStateSpace(), rho_(turningRadius),
				isReversable_(isReversable)
            {
				weight_[0]=1.0; // Length
				weight_[1]=20.0; // Curvatue
				weight_[2]=1.5; // Reverse Cost

				setName("CLOTHOID"+getName());
				type_= STATE_SPACE_CLOTHOID;
				addSubspace(StateSpacePtr(new RealVectorStateSpace(2)),1.0);
				addSubspace(StateSpacePtr(new SO2StateSpace()),0.5);
				addSubspace(StateSpacePtr(new RealVectorStateSpace(1)),0.001);
				lock();
            }
            

            /** \copydoc RealVectorStateSpace::setBounds() */
            void setBounds(const RealVectorBounds &bounds)
            {
                as<RealVectorStateSpace>(0)->setBounds(bounds);

				RealVectorBounds bounds_k(1);
				bounds_k.setLow(-rho_);
				bounds_k.setHigh(rho_);
				as<RealVectorStateSpace>(2)->setBounds(bounds_k);
			}

			/** \copydoc RealVectorStateSpace::getBounds() */
            const RealVectorBounds& getBounds() const
            {
                return as<RealVectorStateSpace>(0)->getBounds();
            }

            virtual State* allocState() const;
            virtual void freeState(State *state) const;

            virtual void registerProjections();
			virtual bool isMetricSpace() const
            {
                return false;
            }

            virtual double distance(const State *state1, const State *state2) const;

            virtual void interpolate(const State *from, const State *to, const double t,
                State *state) const;
            virtual void interpolate(const State *from, const State *to, const double t,
                bool &firstTime, ClothoidPath &path, State *state) const;


            virtual void sanityChecks() const
            {
                double zero = std::numeric_limits<double>::epsilon();
                double eps = std::numeric_limits<float>::epsilon();
                int flags = ~(STATESPACE_INTERPOLATION | STATESPACE_TRIANGLE_INEQUALITY | STATESPACE_DISTANCE_BOUND);
                //if (!isSymmetric_)
                flags &= ~STATESPACE_DISTANCE_SYMMETRIC;
                StateSpace::sanityChecks(zero, eps, flags);
            }

            /** \brief Return the shortest Dubins path from SE(2) state state1 to SE(2) state state2 */
            ClothoidPath clothoid(const State *state1, const State *state2) const;

            bool CheckRange(const State *state1, const State *state2) const;
        protected:
            virtual void interpolate(const State *from, const ClothoidPath &path, const double t,
                State *state) const;

            /** \brief Turning radius */
            double rho_;

            /** \brief Whether the distance is "symmetrized"
			 */
            bool isReversable_;
			double weight_[3];

        };

        /** \brief A Dubins motion validator that only uses the state validity checker.
            Motions are checked for validity at a specified resolution.
            This motion validator is almost identical to the DiscreteMotionValidator
            except that it remembers the optimal DubinsPath between different calls to
            interpolate. */
        class ClothoidMotionValidator : public MotionValidator
        {
        public:
            ClothoidMotionValidator(SpaceInformation *si) : MotionValidator(si)
            {
                defaultSettings();
            }
            ClothoidMotionValidator(const SpaceInformationPtr &si) : MotionValidator(si)
            {
                defaultSettings();
            }
            virtual ~ClothoidMotionValidator()
            {
            }
            virtual bool checkMotion(const State *s1, const State *s2) const;
            virtual bool checkMotion(const State *s1, const State *s2, std::pair<State*, double> &lastValid) const;
        private:
            ClothoidStateSpace *stateSpace_;
            void defaultSettings();
        };

    }
}

#endif
