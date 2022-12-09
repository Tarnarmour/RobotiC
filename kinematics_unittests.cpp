#include "gtest/gtest.h"
#include "Eigen/Dense"

#include "toolsforunittests.cpp"
#include "transforms.h"
#include "kinematics.h"

#include <iostream>
#include <vector>
#include <stdexcept>

TEST(SerialArm, CheckDefaultInitializationGivesArmWithSingleRevoluteJoint)
{
    SerialArm arm;
    std::vector<std::vector<double>> dh{{0, 0, 1, 0}};
    std::vector<char> jt{'r'};
    EXPECT_EQ(arm.get_jt(), jt);
    EXPECT_EQ(arm.get_dh(), dh);
    EXPECT_EQ(arm.get_n(), 1);
}

TEST(SerialArm, CheckWhenInitializedWithDhThatjtIsCorrect)
{

    std::vector<std::vector<double>> dh{{0, 0, 1, 0},
                                        {0, 0, 1, 0},
                                        {0, 0, 1, 0}};
    SerialArm arm(dh);
    std::vector<char> jt{'r', 'r', 'r'};
    EXPECT_EQ(arm.get_jt(), jt);
    EXPECT_EQ(arm.get_n(), 3);
}

TEST(SerialArm, CheckThatWhenInitializedWithDHParamsAndJT_NMatchesDHParams)
{
    std::vector<std::vector<double>> dh{{0, 0, 1, 0}, {0, 0, 1, 0}, {0, 0, 1, 0}};
    std::vector<char> jt{'r', 'p', 'r'};
    SerialArm arm{dh, jt};
    EXPECT_EQ(arm.get_n(), dh.size());
    EXPECT_EQ(arm.get_dh(), dh);
    EXPECT_EQ(arm.get_jt(), jt);
}

TEST(SerialArm, CheckIfDHAndjtHaveMismatchedSize_LengthErrorIsThrown)
{

    std::vector<std::vector<double>> dh{{0, 0, 1, 0},
                                        {0, 0, 1, 0},
                                        {0, 0, 1, 0}};
    std::vector<char> jt{'r', 'r'};
    EXPECT_THROW(SerialArm(dh, jt), std::length_error);
}

TEST(SerialArm_fk, CheckIfIncorrectNumberOfJointAngles_ExceptionIsThrown)
{
    std::vector<std::vector<double>> dh{{0, 0, 1, 0}, {0, 0, 1, 0}, {0, 0, 1, 0}};
    std::vector<char> jt{'r', 'r', 'p'};
    SerialArm arm{dh, jt};
    Eigen::VectorXd q{2};
    q << 1.0, 2.0;
    EXPECT_THROW(arm.fk(q), std::length_error);
}

TEST(SerialArm_fk, WhenCalledWithqOnly_CheckCorrectReturnValue)
{
    std::vector<std::vector<double>> dh{{0, 0, 1, 0}, {0, 0, 1, 0}, {0, 0, 1, 0}};
    std::vector<char> jt{'r', 'r', 'p'};
    SerialArm arm{dh, jt};
    Eigen::VectorXd q{3};
    q << 1.0, 2.0, 3.0;
    SE3 T = arm.fk(q);
    Eigen::Matrix4d A;
    A << -0.9899924397468567, -0.14111997, 0.0, -1.4396826, 0.14111999, -0.9899925, 0.0, 1.1237109, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0, 1.0;
    EXPECT_TRUE(compare_eigen_matrices(T.get_A(), A, 1e-4, true));
}

TEST(SerialArm_fk, WhenGivenCallIndex_ExpectCorrectAnswer)
{
    std::vector<std::vector<double>> dh{{0, 0, 1, 0}, {0, 0, 1, 0}, {0, 0, 1, 0}};
    std::vector<char> jt{'r', 'r', 'p'};
    SerialArm arm{dh, jt};
    Eigen::VectorXd q{3};
    q << 1.0, 2.0, 3.0;
    int iend{2};
    SE3 T = arm.fk(q, iend);
    Eigen::Matrix4d A;
    A << -0.9899924397468567, -0.14111997, 0.0, -0.44969016, 0.14111999, -0.9899925, 0.0, 0.9825909, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    EXPECT_TRUE(compare_eigen_matrices(T.get_A(), A, 1e-4, true));
    iend = 3;
    T = arm.fk(q, iend);
    A << -0.9899924397468567, -0.14111997, 0.0, -1.4396826, 0.14111999, -0.9899925, 0.0, 1.1237109, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0, 1.0;
    EXPECT_TRUE(compare_eigen_matrices(T.get_A(), A, 1e-4, true));
}

TEST(SerialArm_fk, WhenGivenStartAndEndIndex_ExpectCorrectValue)
{
    std::vector<std::vector<double>> dh{{0, 0, 1, 0}, {0, 0, 1, 0}, {0, 0, 1, 0}};
    std::vector<char> jt{'r', 'r', 'p'};
    SerialArm arm{dh, jt};
    Eigen::VectorXd q{3};
    q << 1.0, 2.0, 3.0;
    int iend{2};
    int istart{1};
    SE3 T = arm.fk(q, istart, iend);
    Eigen::Matrix4d A;
    A << -0.416146844625473, -0.9092974, 0.0, -0.41614684, 0.9092974, -0.41614684, 0.0, 0.9092974, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    EXPECT_TRUE(compare_eigen_matrices(T.get_A(), A, 1e-4, true));
}


