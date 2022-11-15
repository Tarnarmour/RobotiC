#include "gtest/gtest.h"
#include "Eigen/Dense"

#include "transforms.h"

#include <iostream>

TEST(SE3_DefaultConstructor, WhenDefaultConstructorIsUsed_CheckThatmMatrixIsSetToIdentity)
{
    SE3 T;
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    EXPECT_TRUE(I.isApprox(T.A));
}

TEST(SE3_RConstructor, WhenConstructedWithRotationMatrix_CheckThatCorrectMatrixIsStored)
{
    Eigen::Matrix3d R;
    R << 1 , 2 , 3 , 4 , 5 , 6 , 7 , 8 , 9;
    SE3 T(R);
    Eigen::Matrix3d T_R = T.A({0, 1, 2}, {0, 1, 2});
    EXPECT_TRUE(R.isApprox(T_R));
}

TEST(SE3_pConstructor, WhenConstructedWith3Vector_CheckThatCorrectMatrixIsStored)
{
    Eigen::Vector3d p{1, 2, 3};
    SE3 T(p);
    EXPECT_TRUE(p.isApprox(T.A({0, 1, 2}, {3})));
}

TEST(SE3_RpConstructor, WhenConstructedWithMatrixAnd3Vector_CheckThatCorrectMatrixIsStored)
{
    Eigen::Matrix3d R{{1, 2, 3}, {4 ,5, 6}, {7, 8, 9}};
    Eigen::Vector3d p{1, 2, 3};
    SE3 T(R, p);
    Eigen::Matrix4d A{{1, 2, 3, 1}, {4, 5, 6, 2}, {7, 8, 9, 3}, {0, 0, 0, 1}};
    EXPECT_TRUE(A.isApprox(T.A));
}

TEST(SE3_pRConstructor, WhenConstructedWith3VectorAndMatrix_CheckThatCorrectMatrixIsStored)
{
    Eigen::Matrix3d R{{1, 2, 3}, {4 ,5, 6}, {7, 8, 9}};
    Eigen::Vector3d p{1, 2, 3};
    SE3 T(p, R);
    Eigen::Matrix4d A{{1, 2, 3, 1}, {4, 5, 6, 2}, {7, 8, 9, 3}, {0, 0, 0, 1}};
    EXPECT_TRUE(A.isApprox(T.A));
}

TEST(SE3_AConstructor, WhenConstructedWithAMatrix_ExpectCorrectValueStored)
{
    Eigen::Matrix4d A = Eigen::Matrix4d::Random();
    SE3 T(A);
    EXPECT_TRUE(A.isApprox(T.A));
}

TEST(SE3_get_R, WhenInitialized_CheckThatGetRReturnsCorrectly)
{
    SE3 T;
    Eigen::Matrix3d R;
    R = T.get_R();
    EXPECT_TRUE(R.isApprox(Eigen::Matrix3d::Identity()));
}

TEST(SE3_inv, WhenMemberMethodInvCalled_ExpectAToBeAnInverseOfOriginalMatrix)
{
    Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d R{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}};
    Eigen::Vector3d p{1, 2, 3};
    A({0, 1, 2}, {0, 1, 2}) = R;
    A({0, 1, 2}, {3}) = p;
    SE3 T(A);
    T.inv();
    Eigen::Matrix4d B = A * T.A;
    EXPECT_TRUE(B.isApprox(Eigen::Matrix4d::Identity()));
}

TEST(enforce_orthonormal, WhenCalledOnOrthonormalMatrix_ExpectNoChangeInMatrix)
{
    Eigen::MatrixXd A = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d B = enforce_orthonormal(A);
    EXPECT_TRUE(A.isApprox(B));
}

TEST(enforce_orthonormal, WhenCalledOnNonOrthonormalMatrix_ExpectOutputToBeOrthonormal)
{
    Eigen::MatrixXd A = Eigen::Matrix3d::Identity() + Eigen::Matrix3d::Random() * 0.1;
    ASSERT_FALSE((A.transpose() * A).isApprox(Eigen::Matrix3d::Identity()));
    Eigen::Matrix3d B = enforce_orthonormal(A);
    EXPECT_TRUE((B.transpose() * B).isApprox(Eigen::Matrix3d::Identity()));
}

