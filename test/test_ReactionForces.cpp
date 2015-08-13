#include <boost/test/unit_test.hpp>
#include <exoter_dynamics/ReactionForces.hpp>

static const double D2R = M_PI/180.00; /** Convert degree to radian **/
static const double R2D = 180.00/M_PI; /** Convert radian to degree **/

BOOST_AUTO_TEST_CASE(test_for_reaction_forces)
{
    std::vector< Eigen::Matrix<double, 3, 1> , Eigen::aligned_allocator < Eigen::Matrix<double, 3, 1> > > chainPosition; /** Chain position of contact points **/
    Eigen::Matrix<double, ::exoter_dynamics::NUMBER_OF_WHEELS, 1> forces; /** forces to calculate */
    base::Vector3d centerOfMass, euler;
    Eigen::Quaterniond orientation;

    centerOfMass.setZero();
    centerOfMass << 0.0, 0.0, 0.00;
    euler.setZero();
    euler[0] = 0.00 * D2R;

    orientation = Eigen::Quaterniond(Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));

    std::cout<<"ORIENTATION ROLL: "<< euler[0] <<" PITCH: "<< euler[1] <<" YAW: "<< euler[2] <<"\n";

    chainPosition.resize(::exoter_dynamics::NUMBER_OF_WHEELS);

    /** Set the position in a square [-1, 1] **/
    chainPosition[0] << 1.0, 1.0, -1.0;
    chainPosition[1] << 1.0, -1.0, -1.0;
    chainPosition[2] << 0.0, 1.0, -1.0;
    chainPosition[3] << 0.0, -1.0, -1.0;
    chainPosition[4] << -1.0, 1.0, -1.0;
    chainPosition[5] << -1.0, -1.0, -1.0;

    /** Create Reaction forces object **/
    Eigen::Vector3d passive_offset;
    passive_offset << 0.5, 0.0, 0.0;
    ::exoter_dynamics::ReactionForces rf(passive_offset);

    /** Compute reaction forces **/
    rf.forceAnalysis(centerOfMass, chainPosition, orientation, 1.0, forces);
    std::cout<<"FORCES\n"<<forces<<"\n";

}


BOOST_AUTO_TEST_CASE(test_for_reaction_forces_asguard)
{
    std::vector< Eigen::Matrix<double, 3, 1> , Eigen::aligned_allocator < Eigen::Matrix<double, 3, 1> > > chainPosition; /** Chain position of contact points **/
    Eigen::Matrix<double, 4, 1> forces; /** forces to calculate */
    base::Vector3d centerOfMass, euler;
    Eigen::Quaterniond orientation;

    std::cout<<"** ASGUARD **\n";

    centerOfMass.setZero();
    euler.setZero();
    euler[1] = 0.00 * D2R;

    orientation = Eigen::Quaterniond(Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));

    std::cout<<"ORIENTATION ROLL: "<< euler[0] <<" PITCH: "<< euler[1] <<" YAW: "<< euler[2] <<"\n";

    chainPosition.resize(4);

    /** Set the position in a square [-1, 1] **/
    chainPosition[0] << 1.0, 1.0, -1.0;
    chainPosition[1] << 1.0, -1.0, -1.0;
    chainPosition[2] << -1.0, 1.0, -1.0;
    chainPosition[3] << -1.0, -1.0, -1.0;

    /** Compute normal forces **/
    ::exoter_dynamics::ReactionForces::forceAnalysis(centerOfMass, chainPosition, orientation, 9.81, forces);
    std::cout<<"FORCES\n"<<forces<<"\n";

}
