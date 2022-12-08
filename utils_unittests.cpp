#define _USE_MATH_DEFINES
#include <cmath> // this has to be at the top for some inane reason
#include "gtest/gtest.h"
#include "Eigen/Dense"

#include "toolsforunittests.cpp"
#include "robotic_utils.h"


#include <iostream>


TEST(enforce_orthonormal, WhenCalledOnOrthonormalMatrix_ExpectNoChangeInMatrix)
{
    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d B = ru::enforce_orthonormal(A);
    EXPECT_TRUE(compare_eigen_matrices(A, B));
}

TEST(enforce_orthonormal, WhenCalledOnNonOrthonormalMatrix_ExpectOutputToBeOrthonormal)
{
    Eigen::MatrixXd A = Eigen::Matrix3d::Identity() + Eigen::Matrix3d::Random() * 0.1;
    Eigen::Matrix3d ATA = A.transpose() * A;
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    ASSERT_FALSE(compare_eigen_matrices(ATA, I));
    Eigen::Matrix3d B = ru::enforce_orthonormal(A);
    Eigen::Matrix3d BTB = B.transpose() * B;
    EXPECT_TRUE(compare_eigen_matrices(BTB, I));
}

TEST(to_radians, WhenGiven60Degrees_ExpectPIThirds)
{
    double theta = 60.0;
    double rad = M_PI * 1.0 / 3.0;
    EXPECT_NEAR(rad, ru::to_rads(theta), 1e-10);
}

TEST(to_degrees, WhenGivenPiThirds_Expect60Degrees)
{
    double theta = M_PI / 3.0;
    double degrees = 60.0;
    EXPECT_NEAR(degrees, ru::to_degrees(theta), 1e-10);
}

TEST(skew, WhenGivenVector_ExpectCorrectSkewSymmetricMatrix)
{
    Eigen::Vector3d v{5, 6, 7};
    Eigen::Matrix3d S = ru::skew(v);
    Eigen::Matrix3d A;
    A << 0, -7, 6, 7, 0, -5, -6, 5, 0;
    EXPECT_TRUE(compare_eigen_matrices(A, S));
}
