#include <gtest/gtest.h>
#include <tf/tf.h>
#include <math_utils/angles.h>
#include <sys/time.h>

#include "LinearMath/btVector3.h"


void seed_rand()
{
  //Seed random number generator with current microseond count
  timeval temp_time_struct;
  gettimeofday(&temp_time_struct,NULL);
  srand(temp_time_struct.tv_usec);
};

void generate_rand_vectors(double scale, unsigned int runs, std::vector<double>& xvalues, std::vector<double>& yvalues, std::vector<double>&zvalues)
{
  seed_rand();
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    xvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
  }
}


using namespace tf;


TEST(tf, ListOneForward)
{
  unsigned int runs = 400;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i])), 10 + i, "child");
    mTR.setTransform(tranStamped, "my_parent");
  }

  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( unsigned int i = 0; i < runs ; i++ )

  {
    Stamped<btTransform> inpose (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), 10 + i, "child");

    try{
    Stamped<btTransform> outpose;
    outpose.data_.setIdentity(); //to make sure things are getting mutated
    mTR.transformStamped("my_parent",inpose, outpose);
    EXPECT_NEAR(outpose.data_.getOrigin().x(), xvalues[i], epsilon);
    EXPECT_NEAR(outpose.data_.getOrigin().y(), yvalues[i], epsilon);
    EXPECT_NEAR(outpose.data_.getOrigin().z(), zvalues[i], epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
}


TEST(tf, ListOneInverse)
{
  unsigned int runs = 400;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i])), 10 + i, "child");
    mTR.setTransform(tranStamped, "my_parent");
  }

  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( unsigned int i = 0; i < runs ; i++ )

  {
    Stamped<btTransform> inpose (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), 10 + i, "my_parent");

    try{
    Stamped<btTransform> outpose;
    outpose.data_.setIdentity(); //to make sure things are getting mutated
    mTR.transformStamped("child",inpose, outpose);
    EXPECT_NEAR(outpose.data_.getOrigin().x(), -xvalues[i], epsilon);
    EXPECT_NEAR(outpose.data_.getOrigin().y(), -yvalues[i], epsilon);
    EXPECT_NEAR(outpose.data_.getOrigin().z(), -zvalues[i], epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
}



TEST(tf, TransformTransformsCartesian)
{
  unsigned int runs = 400;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i])), 10 + i, "child");
    mTR.setTransform(tranStamped, "my_parent");

    /// \todo remove fix for graphing problem
    Stamped<btTransform> tranStamped2(btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), 10 + i, "my_parent");
    mTR.setTransform(tranStamped2, "my_parent2");
  }

  //std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( unsigned int i = 0; i < runs ; i++ )

  {
    Stamped<btTransform> inpose (btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), 10 + i, "child");

    try{
    Stamped<btTransform> outpose;
    outpose.data_.setIdentity(); //to make sure things are getting mutated
    mTR.transformStamped("my_parent",inpose, outpose);
    EXPECT_NEAR(outpose.data_.getOrigin().x(), xvalues[i], epsilon);
    EXPECT_NEAR(outpose.data_.getOrigin().y(), yvalues[i], epsilon);
    EXPECT_NEAR(outpose.data_.getOrigin().z(), zvalues[i], epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
}

TEST(tf, TransformVector3Cartesian)
{
  unsigned int runs = 400;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    xvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 10.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;

    Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i])), 10 + i, "child");
    mTR.setTransform(tranStamped, "my_parent");

    /// \todo remove fix for graphing problem
    Stamped<btTransform> tranStamped2(btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), 10 + i, "my_parent");
    mTR.setTransform(tranStamped2, "my_parent2");
  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( unsigned int i = 0; i < runs ; i++ )

  {
    Stamped<btVector3> invec (btVector3(0,0,0), 10 + i, "child");

    try{
    Stamped<btVector3> outvec(btVector3(0,0,0), 10 + i, "child");
    //    outpose.data_.setIdentity(); //to make sure things are getting mutated
    mTR.transformStamped("my_parent",invec, outvec);
    EXPECT_NEAR(outvec.data_.x(), xvalues[i], epsilon);
    EXPECT_NEAR(outvec.data_.y(), yvalues[i], epsilon);
    EXPECT_NEAR(outvec.data_.z(), zvalues[i], epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
}

TEST(tf, TransformQuaternionCartesian)
{
  unsigned int runs = 400;
  double epsilon = 1e-6;
  seed_rand();
  
  tf::Transformer mTR(true);
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    xvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    yvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    zvalues[i] = 1.0 * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;


    Stamped<btTransform> tranStamped(btTransform(btQuaternion(0,0,0), btVector3(xvalues[i],yvalues[i],zvalues[i])), 10 + i, "child");
    mTR.setTransform(tranStamped, "my_parent");

    /// \todo remove fix for graphing problem
    Stamped<btTransform> tranStamped2(btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), 10 + i, "my_parent");
    mTR.setTransform(tranStamped2, "my_parent2");
  }

  //  std::cout << mTR.allFramesAsString() << std::endl;
  //  std::cout << mTR.chainAsString("child", 0, "my_parent2", 0, "my_parent2") << std::endl;

  for ( unsigned int i = 0; i < runs ; i++ )

  {
    Stamped<btQuaternion> invec (btQuaternion(xvalues[i],yvalues[i],zvalues[i]), 10 + i, "child");
    //    printf("%f, %f, %f\n", xvalues[i],yvalues[i], zvalues[i]);

    try{
    Stamped<btQuaternion> outvec(btQuaternion(xvalues[i],yvalues[i],zvalues[i]), 10 + i, "child");

    mTR.transformStamped("my_parent",invec, outvec);
    EXPECT_NEAR(outvec.data_.angle(invec.data_) , 0, epsilon);
    }
    catch (tf::TransformException & ex)
    {
      std::cout << "TransformExcepion got through!!!!! " << ex.what() << std::endl;
      bool exception_improperly_thrown = true;
      EXPECT_FALSE(exception_improperly_thrown);
    }
  }
  
}

TEST(data, Vector3Conversions)
{
  
  unsigned int runs = 400;
  double epsilon = 1e-6;
  
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    btVector3 btv = btVector3(xvalues[i], yvalues[i], zvalues[i]);
    btVector3 btv_out = btVector3(0,0,0);
    std_msgs::Vector3 msgv;
    Vector3BtToMsg(btv, msgv);
    Vector3MsgToBt(msgv, btv_out);
    EXPECT_NEAR(btv.x(), btv_out.x(), epsilon);
    EXPECT_NEAR(btv.y(), btv_out.y(), epsilon);
    EXPECT_NEAR(btv.z(), btv_out.z(), epsilon);
  } 
  
}

TEST(data, Vector3StampedConversions)
{
  
  unsigned int runs = 400;
  double epsilon = 1e-6;
  
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    Stamped<btVector3> btv = Stamped<btVector3>(btVector3(xvalues[i], yvalues[i], zvalues[i]), 1000000000ULL, "no frame");
    Stamped<btVector3> btv_out;
    std_msgs::Vector3Stamped msgv;
    Vector3StampedBtToMsg(btv, msgv);
    Vector3StampedMsgToBt(msgv, btv_out);
    EXPECT_NEAR(btv.data_.x(), btv_out.data_.x(), epsilon);
    EXPECT_NEAR(btv.data_.y(), btv_out.data_.y(), epsilon);
    EXPECT_NEAR(btv.data_.z(), btv_out.data_.z(), epsilon);
    EXPECT_STREQ(btv.frame_id_.c_str(), btv_out.frame_id_.c_str());
    EXPECT_EQ(btv.stamp_, btv_out.stamp_);
  } 
}

TEST(data, QuaternionConversions)
{
  
  unsigned int runs = 400;
  double epsilon = 1e-6;
  
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    btQuaternion btv = btQuaternion(xvalues[i], yvalues[i], zvalues[i]);
    btQuaternion btv_out = btQuaternion(0,0,0);
    std_msgs::Quaternion msgv;
    QuaternionBtToMsg(btv, msgv);
    QuaternionMsgToBt(msgv, btv_out);
    EXPECT_NEAR(btv.x(), btv_out.x(), epsilon);
    EXPECT_NEAR(btv.y(), btv_out.y(), epsilon);
    EXPECT_NEAR(btv.z(), btv_out.z(), epsilon);
    EXPECT_NEAR(btv.w(), btv_out.w(), epsilon);
  } 
  
}

TEST(data, QuaternionStampedConversions)
{
  
  unsigned int runs = 400;
  double epsilon = 1e-6;
  
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    Stamped<btQuaternion> btv = Stamped<btQuaternion>(btQuaternion(xvalues[i], yvalues[i], zvalues[i]), 1000000000ULL, "no frame");
    Stamped<btQuaternion> btv_out;
    std_msgs::QuaternionStamped msgv;
    QuaternionStampedBtToMsg(btv, msgv);
    QuaternionStampedMsgToBt(msgv, btv_out);
    EXPECT_NEAR(btv.data_.x(), btv_out.data_.x(), epsilon);
    EXPECT_NEAR(btv.data_.y(), btv_out.data_.y(), epsilon);
    EXPECT_NEAR(btv.data_.z(), btv_out.data_.z(), epsilon);
    EXPECT_NEAR(btv.data_.w(), btv_out.data_.w(), epsilon);
    EXPECT_STREQ(btv.frame_id_.c_str(), btv_out.frame_id_.c_str());
    EXPECT_EQ(btv.stamp_, btv_out.stamp_);
  } 
}

TEST(data, TransformConversions)
{
  
  unsigned int runs = 400;
  double epsilon = 1e-6;
  
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  std::vector<double> xvalues2(runs), yvalues2(runs), zvalues2(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    btTransform btv = btTransform(btQuaternion(xvalues2[i], yvalues2[i], zvalues2[i]), btVector3(xvalues[i], yvalues[i], zvalues[i]));
    btTransform btv_out;
    std_msgs::Transform msgv;
    TransformBtToMsg(btv, msgv);
    TransformMsgToBt(msgv, btv_out);
    EXPECT_NEAR(btv.getOrigin().x(), btv_out.getOrigin().x(), epsilon);
    EXPECT_NEAR(btv.getOrigin().y(), btv_out.getOrigin().y(), epsilon);
    EXPECT_NEAR(btv.getOrigin().z(), btv_out.getOrigin().z(), epsilon);
    EXPECT_NEAR(btv.getRotation().x(), btv_out.getRotation().x(), epsilon);
    EXPECT_NEAR(btv.getRotation().y(), btv_out.getRotation().y(), epsilon);
    EXPECT_NEAR(btv.getRotation().z(), btv_out.getRotation().z(), epsilon);
    EXPECT_NEAR(btv.getRotation().w(), btv_out.getRotation().w(), epsilon);
  } 
  
}

TEST(data, TransformStampedConversions)
{
  
  unsigned int runs = 400;
  double epsilon = 1e-6;
  
  std::vector<double> xvalues(runs), yvalues(runs), zvalues(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  std::vector<double> xvalues2(runs), yvalues2(runs), zvalues2(runs);
  generate_rand_vectors(1.0, runs, xvalues, yvalues,zvalues);
  
  for ( unsigned int i = 0; i < runs ; i++ )
  {
    Stamped<btTransform> btv = Stamped<btTransform>(btTransform(btQuaternion(xvalues2[i], yvalues2[i], zvalues2[i]), btVector3(xvalues[i], yvalues[i], zvalues[i])), 1000000000ULL, "no frame");
    Stamped<btTransform> btv_out;
    std_msgs::TransformStamped msgv;
    TransformStampedBtToMsg(btv, msgv);
    TransformStampedMsgToBt(msgv, btv_out);
    EXPECT_NEAR(btv.data_.getOrigin().x(), btv_out.data_.getOrigin().x(), epsilon);
    EXPECT_NEAR(btv.data_.getOrigin().y(), btv_out.data_.getOrigin().y(), epsilon);
    EXPECT_NEAR(btv.data_.getOrigin().z(), btv_out.data_.getOrigin().z(), epsilon);
    EXPECT_NEAR(btv.data_.getRotation().x(), btv_out.data_.getRotation().x(), epsilon);
    EXPECT_NEAR(btv.data_.getRotation().y(), btv_out.data_.getRotation().y(), epsilon);
    EXPECT_NEAR(btv.data_.getRotation().z(), btv_out.data_.getRotation().z(), epsilon);
    EXPECT_NEAR(btv.data_.getRotation().w(), btv_out.data_.getRotation().w(), epsilon);
    EXPECT_STREQ(btv.frame_id_.c_str(), btv_out.frame_id_.c_str());
    EXPECT_EQ(btv.stamp_, btv_out.stamp_);
  } 
}


#ifdef UNDEFINED
/** @todo Make this actually Assert something */

TEST(libTF, DataTypes)
{
  tf::Transformer mTR(true, 0);
  timeval temp_time_struct;
  gettimeofday(&temp_time_struct,NULL);
  unsigned long long atime = temp_time_struct.tv_sec * 1000000000ULL + (unsigned long long)temp_time_struct.tv_usec * 1000ULL;

  libTF::TFPose2D mappose;
  libTF::TFPose2D odompose;
  libTF::TFPose2D apose;

  odompose.x = 1;
  odompose.y = 0;
  odompose.yaw = math_utils::from_degrees(60);
  odompose.frame = "3";
  odompose.time = atime;

  mappose.x = 30.0;
  mappose.y = 40.0;
  //  mappose.yaw = math_utils::from_degrees(-36.0);
  mappose.yaw = math_utils::from_degrees(90);
  mappose.frame = "1";
  mappose.time = atime;

  apose.x = 0;
  apose.y = 0;
  apose.yaw = math_utils::from_degrees(0);
  apose.frame = "3";
  apose.time = atime;

  libTF::TFPose diffpose;
  diffpose.x = mappose.x-odompose.x;
  diffpose.y = mappose.y-odompose.y;
  diffpose.yaw = mappose.yaw-odompose.yaw;
  //diffpose.x = 10;
  //diffpose.y = 100;
  diffpose.z = 1000;
  //diffpose.yaw = math_utils::from_degrees(0);
  diffpose.pitch = math_utils::from_degrees(0);
  //diffpose.roll = math_utils::from_degrees(90);
  diffpose.roll = math_utils::from_degrees(0);
  //  diffpose.yaw = 0;
  diffpose.frame = "1";
  diffpose.time = atime;

  libTF::TFPose2D diffpose2;
  //diffpose.x = mappose.x-odompose.x;
  //diffpose.y = mappose.y-odompose.y;
  //diffpose.yaw = mappose.yaw-odompose.yaw;
  diffpose2.x = diffpose.x;
  diffpose2.y = diffpose.y;
  diffpose2.yaw = diffpose.yaw;
  diffpose2.frame = "1";
  diffpose2.time = atime;

  mTR.setWithEulers("2",
                    mappose.frame,
                    mappose.x,
                    mappose.y,
                    0.0,
                    mappose.yaw,
                    0.0,
                    0.0,
                    atime);

  mTR.setWithMatrix("3",
  		    "2",
		    Pose3D::matrixFromEuler(odompose.x, odompose.y, 0.0, odompose.yaw, 0.0, 0.0).i(),
		    atime);

  // See the list of transforms to get between the frames
  std::cout<<"Viewing (1,3):"<<std::endl;  
  std::cout << mTR.viewChain("1","3");
  std::cout<<"Viewing (2,3):"<<std::endl;  
  std::cout << mTR.viewChain("2","3");


  libTF::TFPose2D result = mTR.transformPose2D(mappose.frame,odompose);
  libTF::TFPose2D result2 = mTR.transformPose2D(mappose.frame,apose);

  libTF::TFPoint point_in;
  point_in.frame = odompose.frame;
  point_in.x = odompose.x;
  point_in.y = odompose.y;
  point_in.z = 0;
  point_in.time = atime;

  libTF::TFPoint point = mTR.transformPoint(mappose.frame, point_in);

  puts("--------------------------");
  printf("Odom: %.3f %.3f %.3f\n",
         odompose.x, odompose.y, math_utils::to_degrees(odompose.yaw));
  printf("Map : %.3f %.3f %.3f\n",
         mappose.x, mappose.y, math_utils::to_degrees(mappose.yaw));
  printf("Diff : %.3f %.3f %.3f %.3f %.3f %.3f\n",
         diffpose.x, diffpose.y, diffpose.z, math_utils::to_degrees(diffpose.yaw), math_utils::to_degrees(diffpose.pitch), math_utils::to_degrees(diffpose.roll));
  puts("--------------------------");
  printf("2D out Out odompose: %.3f %.3f %.3f\n",
         result.x, result.y, math_utils::to_degrees(result.yaw));
  printf("2D out Out apose : %.3f %.3f %.3f\n",
         result2.x, result2.y, math_utils::to_degrees(result2.yaw));
  printf("Point : %.3f %.3f %.3f\n",
         point.x, point.y,point.z);

  std::cout<<"1,2:"<<std::endl<<mTR.getMatrix("1","2",atime);
  std::cout<<"2,3"<<std::endl<<mTR.getMatrix("2","3",atime);
  std::cout<<"1,3:"<<std::endl<<mTR.getMatrix("1","3",atime);

  NEWMAT::Matrix matPoint(4,1);
  matPoint(1,1) = point_in.x;
  matPoint(2,1) = point_in.y;
  matPoint(3,1) = point_in.z;
  matPoint(4,1) = 1;


  //  std::cout <<"point from, to:"<<std::endl;
  // std::cout<<matPoint<<std::endl;
  //  std::cout<<mTR.getMatrix(1,2,atime)*matPoint;
  //std::cout<<mTR.getMatrix(1,point_in.frame,atime)*matPoint;

  // std::cout <<"vector from, to:"<<std::endl;
  //matPoint(4,1)=0;
  //std::cout<<matPoint<<std::endl;
  //std::cout<<mTR.getMatrix(1,point_in.frame,atime)*matPoint;


  //  libTF::TFPoint point2 = mTR.transformPoint(1,point_in);

  libTF::TFEulerYPR ypr_in;
  ypr_in.frame = odompose.frame;
  ypr_in.yaw = 0;
  ypr_in.pitch = 0;
  ypr_in.roll = 0;
  ypr_in.time = atime;

  libTF::TFEulerYPR ypr = mTR.transformEulerYPR("1", ypr_in);

  printf("Ypr : %.3f %.3f %.3f\n",
         ypr.yaw, ypr.pitch,ypr.roll);


  TFVector2D vec2_in;
  vec2_in.x = 1;
  vec2_in.y = 1;
  vec2_in.frame = "2";
  vec2_in.time = atime;

  TFVector2D vec2 = mTR.transformVector2D("1",vec2_in);
  
  printf("Vector : %.3f %.3f\n",
         vec2.x, vec2.y);
  

  TFVector xaxis;
  xaxis.x = 1;
  xaxis.y = 0;
  xaxis.z = 0;
  xaxis.frame = "9";
  xaxis.time = atime;

  TFVector yaxis;
  yaxis.x = 0;
  yaxis.y = 1;
  yaxis.z = 0;
  yaxis.frame = "9";
  yaxis.time = atime;

  TFVector zaxis;
  zaxis.x = 0;
  zaxis.y = 0;
  zaxis.z = 1;
  zaxis.frame = "9";
  zaxis.time = atime;


  printf("--------------------------TESTING VECTORS\n");
  printf("From Frame 9 into frame 1\n");
  for (int ind = 0; ind < 2; ind++)
    for (int ind1 = 0; ind1 < 2; ind1++)
      for (int ind2 = 0; ind2 < 2; ind2++)
	{	
	  
	  mTR.setWithEulers("9",
			    "1",
			    diffpose.x,
			    diffpose.y,
			    diffpose.z,
			    math_utils::from_degrees(90*ind),
			    math_utils::from_degrees(90*ind1),
			    math_utils::from_degrees(90*ind2),
			    atime++);


	  printf("Ypr : %.3f %.3f %.3f\n",
		 90.0*ind, 90.0*ind1, 90.0*ind2);


  TFVector vec_out = mTR.transformVector("1",xaxis);
  printf("X Axis rotated = ");
  printf("Vector : %.3f %.3f %.3f\n",
         vec_out.x, vec_out.y, vec_out.z);
  
  vec_out = mTR.transformVector("1",yaxis);
  printf("Y Axis rotated = ");
  printf("Vector : %.3f %.3f %.3f\n",
         vec_out.x, vec_out.y, vec_out.z);

  vec_out = mTR.transformVector("1",zaxis);
  printf("Z Axis rotated = ");
  printf("Vector : %.3f %.3f %.3f\n",
         vec_out.x, vec_out.y, vec_out.z);
  
	}
  



  TFPoint xunit;
  xunit.x = 1;
  xunit.y = 0;
  xunit.z = 0;
  xunit.frame = "9";
  xunit.time = atime;

  TFPoint yunit;
  yunit.x = 0;
  yunit.y = 1;
  yunit.z = 0;
  yunit.frame = "9";
  yunit.time = atime;

  TFPoint zunit;
  zunit.x = 0;
  zunit.y = 0;
  zunit.z = 1;
  zunit.frame = "9";
  zunit.time = atime;


  printf("-------------------TESTING POINTS\n");
	 printf("From Frame 9 into frame 1\n");

  printf("Fixed offset:\n %.3f %.3f %.3f \n", 
	 diffpose.x, diffpose.y, diffpose.z);

  for (int ind = 0; ind < 2; ind++)
    for (int ind1 = 0; ind1 < 2; ind1++)
      for (int ind2 = 0; ind2 < 2; ind2++)
	{	
	  
	  mTR.setWithEulers("9",
			    "1",
			    diffpose.x,
			    diffpose.y,
			    diffpose.z,
			    math_utils::from_degrees(90*ind),
			    math_utils::from_degrees(90*ind1),
			    math_utils::from_degrees(90*ind2),
			    atime++);


	  printf("Ypr : %.3f %.3f %.3f\n",
		 90.0*ind, 90.0*ind1, 90.0*ind2);


  TFPoint vec_out = mTR.transformPoint("1",xunit);
  printf("X Unit rotated = ");
  printf("Point : %.3f %.3f %.3f\n",
         vec_out.x, vec_out.y, vec_out.z);
  
  vec_out = mTR.transformPoint("1",yunit);
  printf("Y Unit rotated = ");
  printf("Point : %.3f %.3f %.3f\n",
         vec_out.x, vec_out.y, vec_out.z);

  vec_out = mTR.transformPoint("1",zunit);
  printf("Z Unit rotated = ");
  printf("Point : %.3f %.3f %.3f\n",
         vec_out.x, vec_out.y, vec_out.z);
  
	}
  

}


#endif //0
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
