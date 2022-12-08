#include "gtest/gtest.h"
#include "Eigen/Dense"

#include "toolsforunittests.cpp"
#include "transforms.h"

#include <iostream>





TEST(SO3_DefaultConstrctor, ExpectDefaultConstructorToGetIdentityMatrix)
{
    SO3 R;
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    EXPECT_TRUE(compare_eigen_matrices(R.get_R(), I));
}

TEST(SO3_RConstructor, ExpectConstructorToSetRToGivenMatrix)
{
    Eigen::Matrix3d A{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    SO3 R{A};
    EXPECT_TRUE(A.isApprox(R.get_R()));
}

TEST(SO3_UnaryMinus, ExpectUnaryMinusOperatorToReturnInverseOfR)
{
    SO3 R{Eigen::Matrix3d::Random()};
    SO3 Q = -R;
    Eigen::Matrix3d U = R.get_R().transpose();
    EXPECT_TRUE(compare_eigen_matrices(Q.get_R(), U));
}

TEST(SO3_inverse, ExpectInvToReturnSO3WithTransposedMatrix)
{
    SO3 R{Eigen::Matrix3d::Random()};
    SO3 Q = R.inv();
    Eigen::Matrix3d U = R.get_R().transpose();
    EXPECT_TRUE(compare_eigen_matrices(Q.get_R(), U));
}

TEST(rotx, ExpectrotxCalledWithThetaToReturnCorrectSO3Object)
{
    double theta = 1.9;
    SO3 R = rotx(theta);
    Eigen::Matrix3d Q;
    Q << 1.0, 0.0, 0.0, 0.0, -0.32328956686350335, -0.9463000876874145, 0.0, 0.9463000876874145, -0.32328956686350335;
    EXPECT_TRUE(compare_eigen_matrices(Q, R.get_R()));
}

TEST(roty, ExpectrotyCalledWithThetaToReturnCorrectSO3Object)
{
    double theta = 1.9;
    SO3 R = roty(theta);
    Eigen::Matrix3d Q;
    Q << -0.32328956686350335, 0.0, 0.9463000876874145, 0.0, 1.0, 0.0, -0.9463000876874145, 0.0, -0.32328956686350335;
    EXPECT_TRUE(compare_eigen_matrices(Q, R.get_R()));
}

TEST(rotz, ExpectrotzCalledWithThetaToReturnCorrectSO3Object)
{
    double theta = 1.9;
    SO3 R = rotz(theta);
    Eigen::Matrix3d Q;
    Q << -0.32328956686350335, -0.9463000876874145, 0.0, 0.9463000876874145, -0.32328956686350335, 0.0, 0.0, 0.0, 1.0;
    EXPECT_TRUE(compare_eigen_matrices(Q, R.get_R()));
}

TEST(from_rpy, WhenGivenArbitraryRPY_ExpectCorrectSO3Object)
{
    double roll{1.1};
    double pitch{-5.6};
    double yaw{7.9};

    SO3 R = from_rpy(roll, pitch, yaw);
    Eigen::Matrix3d Q;
    Q << -0.03567767898496635, -0.7747448193641704, 0.6312666378723216, 0.4272356065314334, -0.5828602696631442, -0.6911900191408475, 0.903436129305201, 0.24503952929392012, 0.3517936744022697;
    EXPECT_TRUE(compare_eigen_matrices(Q, R.get_R()));
}

TEST(from_axis, WhenGivenAxisAngleRepresentation_ExpectCorrectSO3Object)
{
    double theta = 1.5;
    Eigen::Vector3d v{1, 2, 3};
    SO3 R = from_axis(theta, v);
    Eigen::Matrix3d Q;
    Q << 0.13711311583429542, -0.6670234184304649, 0.7323112403422114, 0.9325270750968355, 0.33624085833407347, 0.1316637360783392, -0.3340557553426555, 0.6648472339207727, 0.6681204291670367;
    EXPECT_TRUE(compare_eigen_matrices(Q, R.get_R()));
}

TEST(from_axis, WhenAxisHasNormOfZero_DefaultToIdentityRotation)
{
    double theta = 1.5;
    Eigen::Vector3d v{0.0, 0.0, 0.0};
    SO3 R = from_axis(theta, v);
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    EXPECT_TRUE(compare_eigen_matrices(R.get_R(), I));
}

TEST(from_quat, WhenGivenUnitQuaternion_ReturnCorrectSO3Object)
{
    Eigen::Vector4d q{1, 2, 3, 4};
    SO3 R = from_quat(q);
    Eigen::Matrix3d Q;
    Q << -0.6666666666666667, 0.13333333333333336, 0.7333333333333332, 0.6666666666666665, -0.3333333333333335, 0.6666666666666666, 0.33333333333333326, 0.9333333333333332, 0.13333333333333308;
    EXPECT_TRUE(compare_eigen_matrices(Q, R.get_R()));
}

TEST(from_quat, WhenQuaternionHasNormOfZero_DefaultToIdentityRotation)
{
    Eigen::Vector4d q{0.0, 0.0, 0.0, 0.0};
    SO3 R = from_quat(q);
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    EXPECT_TRUE(compare_eigen_matrices(R.get_R(), I));
}

TEST(SO3_plus_operator, CheckThatCompositionOperatorWorksCorrectly)
{
    SO3 R1 = rotx(2.3);
    SO3 R2 = roty(6.5);
    SO3 R3 = rotz(-0.1);

    SO3 R4 = R1 + R2 + R3;

    Eigen::Matrix3d Q;
    Q << 0.9717087553583611, 0.09749607933144244, 0.21511998808781552, 0.2261312956905364, -0.6469325294134821, -0.7282464826526752, 0.06816694117549947, 0.7562888448877658, -0.6506769177011774;
    EXPECT_TRUE(compare_eigen_matrices(Q, R4.get_R()));
}

TEST(SO3_minus_operator, CheckThatMinusAppliesCompositionOfInverseMatrix)
{
    SO3 R1 = rotx(2.3);
    SO3 R2 = roty(6.5);
    SO3 R3 = rotz(-0.1);

    SO3 R4 = R1 - R2 + R3; // note that this does NOT equal R1 - (R2 + R3)

    Eigen::Matrix3d Q;
    Q << 0.9717087553583611, 0.09749607933144244, -0.21511998808781552, -0.09309807242209718, -0.6789623033831088, -0.7282464826526752, -0.21705953946139928, 0.7276707394829754, -0.6506769177011774;
    EXPECT_TRUE(compare_eigen_matrices(Q, R4.get_R()));
}

TEST(SO3_equals_operator, CheckThatTheSameSO3MatrixEvaluatesAsTrue)
{
    SO3 R1 = rotx(2.3);
    SO3 R2 = rotx(2.3);

    EXPECT_TRUE(R1 == R2);
}

TEST(SO3_equals_operator, CheckThatDifferentSO3EvaluatesAsFalse)
{
    SO3 R1 = rotx(1.0);
    SO3 R2 = rotx(1.5);

    EXPECT_FALSE(R1 == R2);
}

TEST(SO3_get_z, CheckThatZAxisIsReturned)
{
    SO3 R = rotz(1.0);
    Eigen::Vector3d v{0, 0, 1.0};
    EXPECT_TRUE(v.isApprox(R.get_z()));
}

TEST(SO3_get_axis, CheckThatGetAxisReturnsACorrectAxisAngleRep)
{
    SO3 R = rotx(0.6) + roty(0.2) + rotz(-0.4);
    Eigen::Vector3d v;
    double theta{0.7109740259287559};
    v << 0.7735580844064497, 0.0, -0.4595160558860435;
    AxisAngle aa = R.get_axis();
    EXPECT_NEAR(aa.theta, theta, 1e-12);
    EXPECT_TRUE(compare_eigen_matrices(v, aa.v));
}

TEST(SO3_get_axis, CheckThatIdentityMatrixReturnsXAxis0Theta)
{
    SO3 R;
    Eigen::Vector3d v{1, 0, 0};
    double theta{0.0};

    AxisAngle aa = R.get_axis();
    EXPECT_EQ(aa.theta, theta);
    EXPECT_TRUE(compare_eigen_matrices(aa.v, v));
}

TEST(SE3_DefaultConstructor, WhenDefaultConstructorIsUsed_CheckThatmMatrixIsSetToIdentity)
{
    SE3 T;
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    EXPECT_TRUE(I.isApprox(T.get_A()));
}

TEST(SE3_RConstructor, WhenConstructedWithRotationMatrix_CheckThatCorrectMatrixIsStored)
{
    SO3 R = rotx(1.0) + roty(2.0);
    SE3 T(R);
    EXPECT_TRUE(compare_eigen_matrices(R.get_R(), T.get_R().get_R()));
}

TEST(SE3_pConstructor, WhenConstructedWith3Vector_CheckThatCorrectMatrixIsStored)
{
    Eigen::Vector3d p{1, 2, 3};
    SE3 T(p);
    EXPECT_TRUE(compare_eigen_matrices(p, T.get_p()));
}

TEST(SE3_RpConstructor, WhenConstructedWithSO3And3Vector_CheckThatCorrectMatrixIsStored)
{
    SO3 R = from_rpy(1, 2, 3);
    Eigen::Vector3d p{1, 2, 3};
    SE3 T(R, p);
    Eigen::Matrix4d A;
    A << 0.411982245665683, 0.05872664492762098, 0.9092974268256817, 1.0, -0.6812427202564033, -0.642872836134547, 0.35017548837401463, 2.0, 0.6051272472413688, -0.7637183366502791, -0.2248450953661529, 3.0, 0.0, 0.0, 0.0, 1.0;
    EXPECT_TRUE(compare_eigen_matrices(A, T.get_A()));
}

TEST(SE3_AConstructor, WhenConstructedWithAMatrix_ExpectCorrectValueStored)
{
    Eigen::Matrix4d A = Eigen::Matrix4d::Random();
    SE3 T(A);
    EXPECT_TRUE(compare_eigen_matrices(A, T.get_A()));
}

TEST(SE3_inv, WhenMemberMethodInvCalled_ExpectAToBeAnInverseOfOriginalMatrix)
{
    SE3 A(from_rpy(1, 2, 3), Eigen::Vector3d::Random());
    SE3 Ainv = A.inv();
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d AiA = Ainv.get_A() * A.get_A();
    EXPECT_TRUE(compare_eigen_matrices(AiA, I));
}

TEST(transl, CallWithVector_ExpectThatSE3ObjectIsCorrect)
{
    Eigen::Vector3d p{1, 2, 3};
    SE3 T = transl(p);
    Eigen::Matrix4d A;
    A << 1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 2.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0, 1.0;
    EXPECT_TRUE(compare_eigen_matrices(T.get_A(), A));
}

TEST(transl, CallWith3Doubles_ExpectSE3ObjectToBeCorrect)
{
    double x{1.0};
    double y{2.0};
    double z{3.0};
    SE3 T = transl(x, y, z);
    Eigen::Matrix4d A;
    A << 1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 2.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0, 1.0;
    EXPECT_TRUE(compare_eigen_matrices(T.get_A(), A));
}

TEST(trotx, CallWithThetaValue_ExpectCorrectSE3Object)
{
    double theta = 2.0;
    SE3 A = trotx(theta);
    Eigen::Matrix4d B;
    B << 1.0, 0.0, 0.0, 0.0, 0.0, -0.4161468365471424, -0.9092974268256817, 0.0, 0.0, 0.9092974268256817, -0.4161468365471424, 0.0, 0.0, 0.0, 0.0, 1.0;
    EXPECT_TRUE(compare_eigen_matrices(A.get_A(), B));
}

TEST(troty, CallWithThetaValue_ExpectCorrectSE3Object)
{
    double theta = 2.0;
    SE3 A = troty(theta);
    Eigen::Matrix4d B;
    B << -0.4161468365471424, 0.0, 0.9092974268256817, 0.0, 0.0, 1.0, 0.0, 0.0, -0.9092974268256817, 0.0, -0.4161468365471424, 0.0, 0.0, 0.0, 0.0, 1.0;
    EXPECT_TRUE(compare_eigen_matrices(A.get_A(), B));
}

TEST(trotz, CallWithThetaValue_ExpectCorrectSE3Object)
{
    double theta = 2.0;
    SE3 A = trotz(theta);
    Eigen::Matrix4d B;
    B << -0.4161468365471424, -0.9092974268256817, 0.0, 0.0, 0.9092974268256817, -0.4161468365471424, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    EXPECT_TRUE(compare_eigen_matrices(A.get_A(), B));
}

