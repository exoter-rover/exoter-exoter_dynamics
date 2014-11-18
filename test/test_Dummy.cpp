#include <boost/test/unit_test.hpp>
#include <dynamics/Dummy.hpp>

using namespace dynamics;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    dynamics::DummyClass dummy;
    dummy.welcome();
}
