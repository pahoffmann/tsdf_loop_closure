// Bring in my package's API, which is what I'm testing
#include <loop_closure/loop_closure/lc_line_rejector.h>
#include <loop_closure/path/path.h>
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(LCRejectionTestSuite, test_line_detection)
{
    // <test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
    Path *path = new Path();
    Vector3f start_vec;
    start_vec << 0.0f, 0.0f, 0.0f;
    Matrix4f start_mat = Matrix4f::Identity();
    start_mat.block<3, 1>(0, 3) = start_vec;

    Vector3f end_vec;
    end_vec << 3.0f, 0.0f, 0.0f;
    Matrix4f end_mat = Matrix4f::Identity();
    end_mat.block<3, 1>(0, 3) = end_vec;

    Vector3f one;
    one << 1.2f, 0.4f, 0.0f;
    Matrix4f one_mat = Matrix4f::Identity();
    one_mat.block<3, 1>(0, 3) = one;

    Vector3f two;
    two << 1.8f, -0.23f, 0.0f;
    Matrix4f two_mat = Matrix4f::Identity();
    two_mat.block<3, 1>(0, 3) = two;

    Vector3f three;
    three << 2.5f, 0.45f, 0.0f;
    Matrix4f three_mat = Matrix4f::Identity();
    three_mat.block<3, 1>(0, 3) = three;

    path->add_pose(Pose(start_mat));

    path->add_pose(Pose(one_mat));
    path->add_pose(Pose(two_mat));
    path->add_pose(Pose(three_mat));

    path->add_pose(Pose(end_mat));

    auto line_detect_result_exp_true = LCRejectors::is_line(path, 0, 4, 0.5f);
    auto line_detect_result_exp_false = LCRejectors::is_line(path, 0, 4, 0.4f);

    ASSERT_TRUE(line_detect_result_exp_true);
    ASSERT_FALSE(line_detect_result_exp_false);
}

// Declare another test
TEST(LCRejectionTestSuite, test_line_lc_rejection)
{
    // <test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
    Path *path = new Path();
    Vector3f start_vec;
    start_vec << 0.0f, 0.0f, 0.0f;
    Matrix4f start_mat = Matrix4f::Identity();
    start_mat.block<3, 1>(0, 3) = start_vec;

    Vector3f end_vec;
    end_vec << 3.0f, 0.0f, 0.0f;
    Matrix4f end_mat = Matrix4f::Identity();
    end_mat.block<3, 1>(0, 3) = end_vec;

    Vector3f one;
    one << 1.2f, 0.4f, 0.0f;
    Matrix4f one_mat = Matrix4f::Identity();
    one_mat.block<3, 1>(0, 3) = one;

    Vector3f two;
    two << 1.8f, -0.23f, 0.0f;
    Matrix4f two_mat = Matrix4f::Identity();
    two_mat.block<3, 1>(0, 3) = two;

    Vector3f three;
    three << 2.5f, 0.45f, 0.0f;
    Matrix4f three_mat = Matrix4f::Identity();
    three_mat.block<3, 1>(0, 3) = three;

    path->add_pose(Pose(start_mat));

    path->add_pose(Pose(one_mat));
    path->add_pose(Pose(two_mat));

    std::cout << "Test_mat: " << Pose(three_mat) << std::endl;
    path->add_pose(Pose(three_mat));

    path->add_pose(Pose(end_mat));

    Matrix4f test_transform1 = Matrix4f::Identity();
    Vector3f test_transl1;
    test_transl1 << -2.8f, 0.0f, 0.0f;
    test_transform1.block<3, 1>(0, 3) = test_transl1;

    Matrix4f test_transform2 = Matrix4f::Identity();
    Vector3f test_transl2;
    test_transl2 << -4.0f, 0.5f, 0.0f;
    test_transform2.block<3, 1>(0, 3) = test_transl2;

    ASSERT_FALSE(LCRejectors::reject_line_loop_closure(path, 0, 4, test_transform1));
    ASSERT_TRUE(LCRejectors::reject_line_loop_closure(path, 0, 4, test_transform2));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "lc_rejection_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
