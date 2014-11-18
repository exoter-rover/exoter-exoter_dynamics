#include <boost/test/unit_test.hpp>
#include <dynamics/ReactionForces.hpp>

using namespace dynamics;

BOOST_AUTO_TEST_CASE(test_for_reaction_forces)
{
    std::vector< Eigen::Matrix<double, 3, 1> , Eigen::aligned_allocator < Eigen::Matrix<double, 3, 1> > > chainPosition; /** Chain position of contact points **/
    Eigen::Matrix<double, NUMBER_OF_WHEELS, 1> forces; /** forces to calculate */
    base::Vector3d centerOfMass;
    Eigen::Quaterniond orientation;

    chainPosition.resize(NUMBER_OF_WHEELS);

    ::dynamics::ReactionForces::forceAnalysis(centerOfMass, chainPosition, orientation, 9.81, forces);

}
