#include <gtest/gtest.h>
#include <libTF/Pose3D.h>
#include <angles/angles.h>
#include <sys/time.h>
#include <cstdlib>

  //Seed random number generator with current microseond count
void seed_rand(){
  timeval temp_time_struct;
  gettimeofday(&temp_time_struct,NULL);
  srand(temp_time_struct.tv_usec);
};

using namespace libTF;

TEST(Pose3D, DefaultConstructor){
  Pose3D p;
//EXPECT_EQ(0, p.getMagnitude());
  Euler e = p.getEuler();
  EXPECT_EQ(0, e.yaw);
  EXPECT_EQ(0, e.pitch);
  EXPECT_EQ(0, e.roll);
  Quaternion q = p.getQuaternion();
  EXPECT_EQ(0, q.x);
  EXPECT_EQ(0, q.y);
  EXPECT_EQ(0, q.z);
  EXPECT_EQ(1, q.w);
  Position x = p.getPosition();
  EXPECT_EQ(0, x.x);
  EXPECT_EQ(0, x.y);
  EXPECT_EQ(0, x.z);
}
TEST(Vector, DefaultConstructor){
  Vector v;
  EXPECT_EQ(0, v.x);
  EXPECT_EQ(0, v.y);
  EXPECT_EQ(0, v.z);
}


TEST(Vector, CopyConstructor){
  seed_rand();
  Vector a,b,c;
  for (unsigned int i = 0; i < 1000; i++)
  {  
    a.x = rand();
    a.y = rand();
    a.z = rand();
    b.x = rand();
    b.y = rand();
    b.z = rand();

    b = a;
    EXPECT_EQ(a.x, b.x);
    EXPECT_EQ(a.y, b.y);
    EXPECT_EQ(a.z, b.z);
  }
  
}

TEST(Vector, Addition){
  seed_rand();
  Vector a,b,c;
for (unsigned int i = 0; i < 1000; i++)
 {  
   a.x = rand();
   a.y = rand();
   a.z = rand();
   b.x = rand();
   b.y = rand();
   b.z = rand();
   c = a + b;
   EXPECT_EQ(c.x, a.x + b.x);
   EXPECT_EQ(c.y, a.y + b.y);
   EXPECT_EQ(c.z, a.z + b.z);
 }
}

TEST(Vector, PlusEqual){
  seed_rand();
  Vector a,b,c;
for (unsigned int i = 0; i < 1000; i++)
 {  
   a.x = rand();
   a.y = rand();
   a.z = rand();
   b.x = rand();
   b.y = rand();
   b.z = rand();
   c = a + b;
   a += b;
   EXPECT_EQ(c.x, a.x);
   EXPECT_EQ(c.y, a.y);
   EXPECT_EQ(c.z, a.z);
 }
}

TEST(Vector, Subtraction){
  seed_rand();
  Vector a,b,c;
for (unsigned int i = 0; i < 1000; i++)
 {  
   a.x = rand();
   a.y = rand();
   a.z = rand();
   b.x = rand();
   b.y = rand();
   b.z = rand();
   c = a - b;
   EXPECT_EQ(c.x, a.x - b.x);
   EXPECT_EQ(c.y, a.y - b.y);
   EXPECT_EQ(c.z, a.z - b.z);
 }
}


TEST(Vector, MinusEqual){
  seed_rand();
  Vector a,b,c;
for (unsigned int i = 0; i < 1000; i++)
 {  
   a.x = rand();
   a.y = rand();
   a.z = rand();
   b.x = rand();
   b.y = rand();
   b.z = rand();
   c = a - b;
   a -= b;
   EXPECT_EQ(c.x, a.x);
   EXPECT_EQ(c.y, a.y);
   EXPECT_EQ(c.z, a.z);
 }
}

TEST(Vector, TimesEqual){
  seed_rand();
  Vector a,b,c;
  double scalar;
for (unsigned int i = 0; i < 1000; i++)
 {  
   scalar = rand();
   a.x = rand();
   a.y = rand();
   a.z = rand();
   c = a;
   c *= scalar;
   EXPECT_EQ(c.x, a.x * scalar);
   EXPECT_EQ(c.y, a.y * scalar);
   EXPECT_EQ(c.z, a.z * scalar);
 }
}

TEST(Vector, Times){
  seed_rand();
  Vector a,b,c;
  double scalar;
for (unsigned int i = 0; i < 1000; i++)
 {  
   scalar = rand();
   a.x = rand();
   a.y = rand();
   a.z = rand();
   c = a * scalar;
   EXPECT_EQ(c.x, a.x * scalar);
   EXPECT_EQ(c.y, a.y * scalar);
   EXPECT_EQ(c.z, a.z * scalar);
 }
}

TEST(Vector, rot2D){
  seed_rand();
  Vector a,b,c;
  double scalar;
for (unsigned int i = 0; i < 1000; i++)
 {  
   scalar = rand();
   a.x = rand();
   a.y = rand();
   a.z = rand();
   c = a.rot2D(scalar);
   EXPECT_LT(fabs(-c.x + a.x * cos(scalar) - a.y * sin(scalar)),0.000001); //Can't go higher precision w/o failing??
   EXPECT_LT(fabs(-c.y+ a.x * sin(scalar) + a.y * cos(scalar)), 0.000001);
   EXPECT_LT(fabs(-c.z + a.z), 0.0000001);
 }
}

TEST(Position, DefaultConstructor){
  Position po;
  EXPECT_EQ(0, po.x);
  EXPECT_EQ(0, po.y);
  EXPECT_EQ(0, po.z);
}

TEST(Position, CopyConstructor){
  seed_rand();
  Position a,b;
  for (unsigned int i = 0; i < 1000; i++)
  {  
    a.x = rand();
    a.y = rand();
    a.z = rand();
    b.x = rand();
    b.y = rand();
    b.z = rand();

    b = a;
    EXPECT_EQ(a.x, b.x);
    EXPECT_EQ(a.y, b.y);
    EXPECT_EQ(a.z, b.z);
  }
  
}

TEST(Position, Addition){
  seed_rand();
  Position a,b,c;
for (unsigned int i = 0; i < 1000; i++)
 {  
   a.x = rand();
   a.y = rand();
   a.z = rand();
   b.x = rand();
   b.y = rand();
   b.z = rand();
   c = a + b;
   EXPECT_EQ(c.x, a.x + b.x);
   EXPECT_EQ(c.y, a.y + b.y);
   EXPECT_EQ(c.z, a.z + b.z);
 }
}

TEST(Position, PlusEquals){
  seed_rand();
  Position a,b,c;
for (unsigned int i = 0; i < 1000; i++)
 {  
   a.x = rand();
   a.y = rand();
   a.z = rand();
   b.x = rand();
   b.y = rand();
   b.z = rand();
   c = a + b;
   a += b;
   EXPECT_EQ(c.x, a.x);
   EXPECT_EQ(c.y, a.y);
   EXPECT_EQ(c.z, a.z);
 }
}

TEST(Position, Subtraction){
  seed_rand();
  Position a,b,c;
for (unsigned int i = 0; i < 1000; i++)
 {  
   a.x = rand();
   a.y = rand();
   a.z = rand();
   b.x = rand();
   b.y = rand();
   b.z = rand();
   c = a - b;
   EXPECT_EQ(c.x, a.x - b.x);
   EXPECT_EQ(c.y, a.y - b.y);
   EXPECT_EQ(c.z, a.z - b.z);
 }
}

TEST(Position, MinusEquals){
  seed_rand();
  Position a,b,c;
for (unsigned int i = 0; i < 1000; i++)
 {  
   a.x = rand();
   a.y = rand();
   a.z = rand();
   b.x = rand();
   b.y = rand();
   b.z = rand();
   c = a - b;
   a -= b;
   EXPECT_EQ(c.x, a.x);
   EXPECT_EQ(c.y, a.y);
   EXPECT_EQ(c.z, a.z);
 }
}

TEST(Position, TimesEqual){
  seed_rand();
  Position a,b,c;
  double scalar;
  for (unsigned int i = 0; i < 1000; i++)
 {  
   scalar = rand();
   a.x = rand();
   a.y = rand();
   a.z = rand();
   c = a;
   c *= scalar;
   EXPECT_EQ(c.x, a.x * scalar);
   EXPECT_EQ(c.y, a.y * scalar);
   EXPECT_EQ(c.z, a.z * scalar);
 }
}

TEST(Position, Times){
  seed_rand();
  Position a,b,c;
  double scalar;
  for (unsigned int i = 0; i < 1000; i++)
 {  
   scalar = rand();
   a.x = rand();
   a.y = rand();
   a.z = rand();
   c = a * scalar;
   EXPECT_EQ(c.x, a.x * scalar);
   EXPECT_EQ(c.y, a.y * scalar);
   EXPECT_EQ(c.z, a.z * scalar);
 }
}

TEST(Position, rot2D){
  seed_rand();
  Position a,b,c;
  double scalar;
for (unsigned int i = 0; i < 1000; i++)
 {  
   scalar = rand();
   a.x = rand();
   a.y = rand();
   a.z = rand();
   c = a.rot2D(scalar);
   EXPECT_LT(fabs(-c.x + a.x * cos(scalar) - a.y * sin(scalar)),0.000001); //Can't go higher precision w/o failing??
   EXPECT_LT(fabs(-c.y+ a.x * sin(scalar) + a.y * cos(scalar)), 0.000001);
   EXPECT_LT(fabs(-c.z + a.z), 0.0000001);
 }
}

TEST(Quaterion, DefaultConstructor){
  Quaternion quat;
  EXPECT_EQ(0, quat.x);
  EXPECT_EQ(0, quat.y);
  EXPECT_EQ(0, quat.z);
}

TEST(Euler, DefaultConstructor){
  Euler e;
  EXPECT_EQ(0, e.yaw);
  EXPECT_EQ(0, e.pitch);
  EXPECT_EQ(0, e.roll);
}

void testEulerConversion(double yaw, double pitch, double roll){
  Pose3D p;
  p.setFromEuler(0, 0, 0, yaw, pitch, roll);
  Euler e = p.getEuler();
  EXPECT_FLOAT_EQ(e.yaw, yaw);
  EXPECT_FLOAT_EQ(e.pitch, pitch);
  EXPECT_FLOAT_EQ(e.roll, roll);
}

TEST(Pose3D, ToFromEulerConversions){
  Pose3D p;
  testEulerConversion(0, 0, 0);
  testEulerConversion(1, 0, 0);
  testEulerConversion(0, 1, 0);
  testEulerConversion(0, 0, 1);
  testEulerConversion(1, 1, 0);
  testEulerConversion(0, 1, 1);
  testEulerConversion(1, 0, 1);
  testEulerConversion(1, 1, 1);
  testEulerConversion(-1, -1, -1);
  testEulerConversion(0, 0, 3.14);
  testEulerConversion(0, 0, -3.14);
}

TEST(Pose3D, EulerToMatrixToQuaternionToMatrixToEuler)
{
  seed_rand();
  libTF::Pose3D aPose;

  int i_max = 100;

  //Test many different premutations
  for (int i = -i_max; i < i_max; i++)
  {

    double yaw, pitch, roll;
    yaw = 2 * M_PI * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    pitch = 2 * M_PI * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    roll = 2 * M_PI * ((double) rand() - (double)RAND_MAX /2.0) /(double)RAND_MAX;
    
      
    NEWMAT::Matrix m = libTF::Pose3D::matrixFromEuler(0.0,
                                                      0.0,
                                                      0.0,
                                                      yaw,
                                                      pitch,
                                                      roll);
      
    //          std::cout << m <<std::endl;
    aPose.setFromMatrix(m);
    NEWMAT::Matrix n = aPose.asMatrix();
          
    for (unsigned int row = 1; row < 5; row++)
      for (unsigned int col = 1; col < 5; col++)
      {
        EXPECT_LT(fabs(n(row,col) - m(row,col)), 0.00000001);
      }
    libTF::Euler out = libTF::Pose3D::eulerFromMatrix(n);
    libTF::Euler out2 = libTF::Pose3D::eulerFromMatrix(n,2);

    //Test the difference between input and output accounting for 2Pi redundancy.  
    bool difference = ((fabs(angles::modNPiBy2(out.yaw) - angles::modNPiBy2(yaw)) > 0.001 || fabs(angles::modNPiBy2(out.pitch) - angles::modNPiBy2(pitch)) > 0.001 || fabs(angles::modNPiBy2(out.roll) -angles::modNPiBy2(roll)) > 0.0001) &&
                       (fabs(angles::modNPiBy2(out2.yaw) - angles::modNPiBy2(yaw)) > 0.001 || fabs(angles::modNPiBy2(out2.pitch) - angles::modNPiBy2(pitch)) > 0.001 || fabs(angles::modNPiBy2(out2.roll) -angles::modNPiBy2(roll)) > 0.0001));
          
    ASSERT_FALSE(difference);
    if (difference){
      printf("in: %.3f %.3f %.3f\n",
             yaw, pitch, roll);

      std::cout << m;


      printf("out: %.3f %.3f %.3f\n",
             out.yaw, out.pitch, out.roll);
      printf("out2: %.3f %.3f %.3f\n\n",
             out2.yaw, out2.pitch, out2.roll);
	  
      printf("FAILURE!!!!!!!!!!\n\n");
    }
  }

}


TEST(Pose3D, MatrixToPosition){
  
  for (unsigned int i = 0; i < 1000; i++)
  {
    NEWMAT::Matrix m(4,4);
    m << 0 << 0 << 0 << rand()
      << 0 << 0 << 0 << rand()
      << 0 << 0 << 0 << rand()
      << 0 << 0 << 0 << 1;
    
    Position t = Pose3D::positionFromMatrix(m);
    
    EXPECT_EQ(t.x, m(1,4));
    EXPECT_EQ(t.y, m(2,4));
    EXPECT_EQ(t.z, m(3,4));
  }

}



int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
