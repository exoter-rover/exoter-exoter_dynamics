#ifndef _EXOTER_REACTION_FORCES_HPP_
#define _EXOTER_REACTION_FORCES_HPP_

#include <vector> /** std vector **/
#include <Eigen/Core>/** Eigen core library **/
#include <Eigen/StdVector> /** For STL container with Eigen types **/

/** Base types **/
#include <base/Eigen.hpp>
#include <base/Pose.hpp>

#include <iostream>

namespace dynamics
{
    static const size_t NUMBER_OF_WHEELS = 6; //rover number of wheels
    static const size_t NUMBER_OF_FEET = 1; //contact points per wheel

    enum wheelIdx
    {
        FRONT_LEFT   = 0,
        FRONT_RIGHT  = 1,
        MIDDLE_LEFT = 2,
        MIDDLE_RIGHT  = 3,
        REAR_LEFT = 4,
        REAR_RIGHT = 5
    };

    class ReactionForces
    {
    public:
       ReactionForces();
    public:

        static void forceAnalysis (const base::Vector3d &centerOfMass,
                const std::vector< Eigen::Matrix<double, 3, 1>, Eigen::aligned_allocator < Eigen::Matrix<double, 3, 1> > > &footPosition,
                const Eigen::Quaterniond &orient,
                const double gravity,
                Eigen::Matrix<double, NUMBER_OF_WHEELS, 1> &forces)
        {
            std::vector< Eigen::Matrix<double, 3, 1> , Eigen::aligned_allocator < Eigen::Matrix<double, 3, 1> > > foot = footPosition; //! foot position w.r.t the Center of Gravity
            Eigen::Matrix<double, NUMBER_OF_WHEELS, NUMBER_OF_WHEELS> A; //! Matrix A of system of equations Ax=b
            Eigen::Matrix<double, NUMBER_OF_WHEELS, 1> b; //! Vector b in Ax=b
            Eigen::Matrix<double, 3, 1> euler;
            Eigen::Quaterniond attitude;

            /** Eliminate heading we are only interested on Pitch and Roll **/
            euler[2] = 0.00;
            euler[1] = base::getEuler(orient)[1];//Pitch
            euler[0] = base::getEuler(orient)[2];//Roll

            /** Attitude **/
            attitude = Eigen::Quaterniond(Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())*
                Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));

            /** Attitude in rotation matrix form **/
            std::cout<<"R:\n"<<attitude.toRotationMatrix()<<"\n";
            Eigen::Vector3d r = attitude.toRotationMatrix().col(2);
            std::cout<<"r vector:\n"<<r<<"\n";

            /** First row is all ones: Constraint 1 [1, 1, 1, 1, 1, 1] **/
            A.row(0).setOnes();

            /** b vector **/
            b << gravity, 0.00, 0.00, 0.00, 0.00, 0.00;

            register int k = 0;
            for (std::vector< Eigen::Matrix<double, 3, 1> , Eigen::aligned_allocator < Eigen::Matrix<double, 3, 1> > > ::iterator it = foot.begin(); it != foot.end(); ++it)
            {
                /** The foot position is w.r.t. the geometrical body center (B_CoM). Make it to the Center of Gravity in case it is different **/
                *it -= centerOfMass;

                std::cout<<"*it:\n"<<*it<<"\n";

                /** Cross product to compute the torque **/
                Eigen::Vector3d torque = (*it).cross(r);

                /** Take the z elements **/
                A(1,k) = torque[0];// Constraint 1: Torque along X in wheels [0..3] is zero [-r0x, -r1x, -r2x, -r3x, -r4x, -r5x] (we will change the two last element to zero afterwards)
                A(2,k) = torque[1];// Constraint 2: Torque along Y in wheels [0,2] is zero
                A(3,k) = torque[1];// Constraint 3: Torque along Y in wheels [1,3] is zero
                A(4,k) = torque[0];// Constraint 4: Torque along X in wheels [4,5] is zero
                A(5,k) = torque[1];// Constraint 5: Torque along Y for all wheels is zero

                k++;
            }
            /** Set to zero the not requires elements **/
            A(1,4) = 0.00; A(1,5) = 0.00;// Constraint 1: [-r0x, -r1x, -r2x, -r3x, 0, 0]
            A(2,1) = 0.00; A(2,3) = 0.00; A(2,4) = 0.00; A(2,5) = 0.00;// Constraint 2:
            A(3,0) = 0.00; A(3,2) = 0.00; A(3,4) = 0.00; A(3,5) = 0.00;// Constraint 3:
            A(4,0) = 0.00; A(4,1) = 0.00; A(4,2) = 0.00; A(4,3) = 0.00;// Constraint 4:

            std::cout<<"A is \n"<<A<<"\n";
            std::cout<<"b is \n"<<b<<"\n";

            /** Solve the system by LU decomposition **/
            forces = A.fullPivLu().solve(b);

            return;
        };


        static void forceAnalysis (const base::Vector3d &centerOfMass,
                const std::vector< Eigen::Matrix<double, 3, 1>, Eigen::aligned_allocator < Eigen::Matrix<double, 3, 1> > > &footPosition,
                const Eigen::Quaterniond &orient,
                const double gravity,
                Eigen::Matrix<double, 4, 1> &forces)
        {
            std::vector< Eigen::Matrix<double, 3, 1> , Eigen::aligned_allocator < Eigen::Matrix<double, 3, 1> > > foot = footPosition; //! foot position w.r.t the Center of Gravity
            Eigen::Matrix<double, 4, 4> A; //! Matrix A of system of equations Ax=b
            Eigen::Matrix<double, 4, 1> b; //! Vector b in Ax=b
            Eigen::Matrix<double, 3, 1> euler;
            Eigen::Quaterniond attitude;

            /** Eliminate heading we are only interested on Pitch and Roll **/
            euler[2] = 0.00;
            euler[1] = base::getEuler(orient)[1];//Pitch
            euler[0] = base::getEuler(orient)[2];//Roll

            /** Attitude **/
            attitude = Eigen::Quaterniond(Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())*
                Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));

            /** Attitude in rotation matrix form **/
            std::cout<<"R:\n"<<attitude.toRotationMatrix()<<"\n";
            Eigen::Vector3d r = attitude.toRotationMatrix().col(2);
            std::cout<<"r vector:\n"<<r<<"\n";

            /** First row is all ones: Constraint 1 [1, 1, 1, 1, 1, 1] **/
            A.row(0).setOnes();

            /** b vector **/
            b << gravity, 0.00, 0.00, 0.00;

            register int k = 0;
            for (std::vector< Eigen::Matrix<double, 3, 1> , Eigen::aligned_allocator < Eigen::Matrix<double, 3, 1> > > ::iterator it = foot.begin(); it != foot.end(); ++it)
            {
                /** The foot position is w.r.t. the geometrical body center (B_CoM). Make it to the Center of Gravity in case it is different **/
                *it -= centerOfMass;

                std::cout<<"*it:\n"<<*it<<"\n";

                /** Cross product to compute the torque **/
                Eigen::Vector3d torque = (*it).cross(r);

                /** Take the z elements **/
                A(1,k) = torque[1];// Constraint 2: Torque along Y is zero
                A(2,k) = torque[0];// Constraint 3: Torque along X in wheels [0,1] is zero
                A(3,k) = torque[0];// Constraint 4: Torque along X in wheels [2,3] is zero

                k++;
            }
            /** Set to zero the not requires elements **/
            A(2,2) = 0.00; A(2,3) = 0.00;// Constraint 3:
            A(3,0) = 0.00; A(3,1) = 0.00; // Constraint 4:

            std::cout<<"A is \n"<<A<<"\n";
            std::cout<<"b is \n"<<b<<"\n";

            /** Solve the system by LU decomposition **/
            forces = A.fullPivLu().solve(b);

            return;
        };

    };
}

#endif
