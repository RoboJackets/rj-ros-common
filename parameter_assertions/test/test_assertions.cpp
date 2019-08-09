#include <gtest/gtest.h>

#include <parameter_assertions/assertions.h>
#include <ros/ros.h>

class TestAssertions : public testing::Test
{
public:
  TestAssertions() : handle_{} {};

protected:
  void SetUp() override
  {
    ros::start();
    handle_.deleteParam(parameter1);
    handle_.deleteParam(parameter2);
    handle_.deleteParam(parameter3);
  }

  void TearDown() override
  {
    if (ros::ok())
    {
      ros::shutdown();
    }
  }

  const std::string parameter1 = "first_param";
  const std::string parameter2 = "second_param";
  const std::string parameter3 = "third_param";
  ros::NodeHandle handle_;
};

//==================
//=  Useful Macros =
//==================
#define EXPECT_VEC_EQ(a, b)                                                                                            \
  do                                                                                                                   \
  {                                                                                                                    \
    ASSERT_EQ(a.size(), b.size());                                                                                     \
    for (size_t i = 0; i < a.size(); i++)                                                                              \
    {                                                                                                                  \
      EXPECT_EQ(a[i], b[i]);                                                                                           \
    }                                                                                                                  \
  } while (false)

#define EXPECT_STR_VEC_EQ(a, b)                                                                                        \
  do                                                                                                                   \
  {                                                                                                                    \
    ASSERT_EQ(a.size(), b.size());                                                                                     \
    for (size_t i = 0; i < a.size(); i++)                                                                              \
    {                                                                                                                  \
      EXPECT_STREQ(a[i].c_str(), b[i].c_str());                                                                        \
    }                                                                                                                  \
  } while (false)

#define EXPECT_FLOAT_VEC_EQ(a, b)                                                                                      \
  do                                                                                                                   \
  {                                                                                                                    \
    ASSERT_EQ(a.size(), b.size());                                                                                     \
    for (size_t i = 0; i < a.size(); i++)                                                                              \
    {                                                                                                                  \
      EXPECT_FLOAT_EQ(a[i], b[i]);                                                                                     \
    }                                                                                                                  \
  } while (false)

#define EXPECT_ROS_ALIVE()                                                                                             \
  do                                                                                                                   \
  {                                                                                                                    \
    EXPECT_TRUE(ros::ok());                                                                                            \
  } while (false)

#define EXPECT_ROS_DEAD()                                                                                              \
  do                                                                                                                   \
  {                                                                                                                    \
    EXPECT_FALSE(ros::ok());                                                                                           \
  } while (false)

//========== Basic Functionality ==========
TEST_F(TestAssertions, getParamGetsParamString)
{
  {
    std::string param;
    std::string set_param = "Vim>Emacs";
    handle_.setParam(parameter1, set_param);
    assertions::getParam(handle_, parameter1, param);
    EXPECT_STREQ(param.c_str(), set_param.c_str());
  }
  {
    std::vector<std::string> param;
    std::vector<std::string> set_param{ "rrt", "prm", "sst", "est" };
    handle_.setParam(parameter2, set_param);
    assertions::getParam(handle_, parameter2, param);
    EXPECT_STR_VEC_EQ(param, set_param);
  }
}

TEST_F(TestAssertions, getParamGetsParamDouble)
{
  {
    double param;
    double set_param = 40.02;
    handle_.setParam(parameter1, set_param);
    assertions::getParam(handle_, parameter1, param);
    EXPECT_FLOAT_EQ(param, set_param);
  }
  {
    std::vector<double> param;
    std::vector<double> set_param{ 1.0, 1.1, 0.9, 0.8 };
    handle_.setParam(parameter2, set_param);
    assertions::getParam(handle_, parameter2, param);
    EXPECT_FLOAT_VEC_EQ(param, set_param);
  }
}

TEST_F(TestAssertions, getParamGetsParamFloat)
{
  {
    float param;
    float set_param = 40.02f;
    handle_.setParam(parameter1, set_param);
    assertions::getParam(handle_, parameter1, param);
    EXPECT_FLOAT_EQ(param, set_param);
  }
  {
    std::vector<float> param;
    std::vector<float> set_param{ 1.0f, 1.1f, 0.9f, 0.8f };
    handle_.setParam(parameter2, set_param);
    assertions::getParam(handle_, parameter2, param);
    EXPECT_FLOAT_VEC_EQ(param, set_param);
  }
}

TEST_F(TestAssertions, getParamGetsParamInt)
{
  {
    int param;
    int set_param = 1;
    handle_.setParam(parameter1, set_param);
    assertions::getParam(handle_, parameter1, param);
    ASSERT_EQ(param, set_param);
  }
  {
    std::vector<int> param;
    std::vector<int> set_param{ 3, 1, 4, 1, 5 };
    handle_.setParam(parameter2, set_param);
    assertions::getParam(handle_, parameter2, param);
    EXPECT_VEC_EQ(param, set_param);
  }
}

TEST_F(TestAssertions, getParamGetsParamBool)
{
  {
    bool param;
    bool set_param = true;
    handle_.setParam(parameter1, set_param);
    assertions::getParam(handle_, parameter1, param);
    EXPECT_EQ(param, set_param);
  }
  {
    std::vector<bool> param;
    std::vector<bool> set_param{ true, false, true, true, false, true };
    handle_.setParam(parameter2, set_param);
    assertions::getParam(handle_, parameter2, param);
    EXPECT_VEC_EQ(param, set_param);
  }
}

TEST_F(TestAssertions, paramGetsParamString)
{
  {
    std::string param;
    std::string set_param = "Vim>Emacs";
    std::string default_param{};
    handle_.setParam(parameter1, set_param);
    assertions::param(handle_, parameter1, param, default_param);
    EXPECT_STREQ(param.c_str(), set_param.c_str());

    std::string result = assertions::param(handle_, parameter1, default_param);
    EXPECT_STREQ(param.c_str(), result.c_str());
  }
  {
    std::vector<std::string> param;
    std::vector<std::string> set_param{ "rrt", "prm", "sst", "est" };
    std::vector<std::string> default_param{};
    handle_.setParam(parameter2, set_param);
    assertions::param(handle_, parameter2, param, default_param);
    EXPECT_STR_VEC_EQ(param, set_param);

    std::vector<std::string> result = assertions::param(handle_, parameter2, default_param);
    EXPECT_STR_VEC_EQ(result, set_param);
  }
}

//========== Guarantees that parameter is set ==========
TEST_F(TestAssertions, getParamEnsuresParamIsSetString)
{
  std::string param;
  assertions::getParam(handle_, parameter1, param);
  EXPECT_ROS_DEAD();
}

TEST_F(TestAssertions, getParamEnsuresParamIsSetDouble)
{
  double param;
  assertions::getParam(handle_, parameter1, param);
  EXPECT_ROS_DEAD();
}

TEST_F(TestAssertions, getParamEnsuresParamIsSetFloat)
{
  float param;
  assertions::getParam(handle_, parameter1, param);
  EXPECT_ROS_DEAD();
}

TEST_F(TestAssertions, getParamEnsuresParamIsSetInt)
{
  int param;
  assertions::getParam(handle_, parameter1, param);
  EXPECT_ROS_DEAD();
}

TEST_F(TestAssertions, getParamEnsuresParamIsSetBool)
{
  bool param;
  assertions::getParam(handle_, parameter1, param);
  EXPECT_ROS_DEAD();
}

//========== param uses default value and doesn't shutdown ==========
TEST_F(TestAssertions, paramUsesDefaultValueString)
{
  {
    std::string param;
    std::string default_param{ "yay testing" };
    assertions::param(handle_, parameter1, param, default_param);
    EXPECT_STREQ(param.c_str(), default_param.c_str());

    std::string result = assertions::param(handle_, parameter1, default_param);
    EXPECT_STREQ(param.c_str(), default_param.c_str());
  }
  {
    std::vector<std::string> param;
    std::vector<std::string> default_param{ "fwafwf", "F", "412431" };
    assertions::param(handle_, parameter2, param, default_param);
    EXPECT_STR_VEC_EQ(param, default_param);

    std::vector<std::string> result = assertions::param(handle_, parameter2, default_param);
    EXPECT_STR_VEC_EQ(result, default_param);
  }
}

TEST_F(TestAssertions, paramUsesDefaultValueDouble)
{
  {
    double param;
    double default_param{ 35.1312 };
    assertions::param(handle_, parameter1, param, default_param);
    EXPECT_FLOAT_EQ(param, default_param);

    double result = assertions::param(handle_, parameter1, default_param);
    EXPECT_FLOAT_EQ(result, default_param);
  }
  {
    std::vector<double> param;
    std::vector<double> default_param{ 9.9, 9.98, 9.932 };
    assertions::param(handle_, parameter2, param, default_param);
    EXPECT_FLOAT_VEC_EQ(param, default_param);

    std::vector<double> result = assertions::param(handle_, parameter2, default_param);
    EXPECT_FLOAT_VEC_EQ(result, default_param);
  }
}

TEST_F(TestAssertions, paramUsesDefaultValueInt)
{
  {
    int param;
    int default_param{ 98481231 };
    assertions::param(handle_, parameter1, param, default_param);
    EXPECT_EQ(param, default_param);

    int result = assertions::param(handle_, parameter2, default_param);
    EXPECT_EQ(result, default_param);
  }
  {
    std::vector<int> param;
    std::vector<int> default_param{ 0, 231, 151, 131, 12, -341 };
    assertions::param(handle_, parameter2, param, default_param);
    EXPECT_VEC_EQ(param, default_param);

    std::vector<int> result = assertions::param(handle_, parameter2, default_param);
    EXPECT_VEC_EQ(result, default_param);
  }
}

// ===========================
// = Test Positive assertion =
// ===========================

TEST_F(TestAssertions, getParamAssertNumberPositivePass)
{
  {
    double param;
    double set_param = 40.02;
    handle_.setParam(parameter1, set_param);
    assertions::getParam(handle_, parameter1, param, { assertions::NumberAssertionType::POSITIVE });
    EXPECT_FLOAT_EQ(param, set_param);
    EXPECT_ROS_ALIVE();
  }
  {
    float param;
    float set_param = 82.1f;
    handle_.setParam(parameter2, set_param);
    assertions::getParam(handle_, parameter2, param, { assertions::NumberAssertionType::POSITIVE });
    EXPECT_FLOAT_EQ(param, set_param);
    EXPECT_ROS_ALIVE();
  }
  {
    int param;
    int set_param = 150;
    handle_.setParam(parameter3, set_param);
    assertions::getParam(handle_, parameter3, param, { assertions::NumberAssertionType::POSITIVE });
    EXPECT_EQ(param, set_param);
    EXPECT_ROS_ALIVE();
  }
}

TEST_F(TestAssertions, getParamAssertNumberPositiveFailDouble)
{
  double param;
  double set_param = -91.312;
  handle_.setParam(parameter1, set_param);
  assertions::getParam(handle_, parameter1, param, { assertions::NumberAssertionType::POSITIVE });
  EXPECT_ROS_DEAD();
}

TEST_F(TestAssertions, getParamAssertNumberPositiveFailFloat)
{
  float param;
  float set_param = -91.312;
  handle_.setParam(parameter1, set_param);
  assertions::getParam(handle_, parameter1, param, { assertions::NumberAssertionType::POSITIVE });
  EXPECT_ROS_DEAD();
}

TEST_F(TestAssertions, getParamAssertNumberPositiveFailInt)
{
  int param;
  int set_param = -32131;
  handle_.setParam(parameter1, set_param);
  assertions::getParam(handle_, parameter1, param, { assertions::NumberAssertionType::POSITIVE });
  EXPECT_ROS_DEAD();
}

// ===============================
// = Test Non-negative assertion =
// ===============================

TEST_F(TestAssertions, getParamAssertNumberNonNegativePass)
{
  {
    double param;
    double set_param = 0.0;
    handle_.setParam(parameter1, set_param);
    assertions::getParam(handle_, parameter1, param, { assertions::NumberAssertionType::NON_NEGATIVE });
    EXPECT_FLOAT_EQ(param, set_param);
    EXPECT_ROS_ALIVE();
  }
  {
    float param;
    float set_param = 0.1f;
    handle_.setParam(parameter2, set_param);
    assertions::getParam(handle_, parameter2, param, { assertions::NumberAssertionType::NON_NEGATIVE });
    EXPECT_FLOAT_EQ(param, set_param);
    EXPECT_ROS_ALIVE();
  }
  {
    int param;
    int set_param = 0;
    handle_.setParam(parameter3, set_param);
    assertions::getParam(handle_, parameter3, param, { assertions::NumberAssertionType::NON_NEGATIVE });
    EXPECT_EQ(param, set_param);
    EXPECT_ROS_ALIVE();
  }
}

TEST_F(TestAssertions, getParamAssertNumberNonNegativeFailDouble)
{
  double param;
  double set_param = -91.312;
  handle_.setParam(parameter1, set_param);
  assertions::getParam(handle_, parameter1, param, { assertions::NumberAssertionType::NON_NEGATIVE });
  EXPECT_ROS_DEAD();
}

TEST_F(TestAssertions, getParamAssertNumberNonNegativeFailFloat)
{
  float param;
  float set_param = -91.312;
  handle_.setParam(parameter1, set_param);
  assertions::getParam(handle_, parameter1, param, { assertions::NumberAssertionType::NON_NEGATIVE });
  EXPECT_ROS_DEAD();
}

TEST_F(TestAssertions, getParamAssertNumberNonNegativeFailInt)
{
  int param;
  int set_param = -32131;
  handle_.setParam(parameter1, set_param);
  assertions::getParam(handle_, parameter1, param, { assertions::NumberAssertionType::NON_NEGATIVE });
  EXPECT_ROS_DEAD();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_assertions");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
