#include <gtest/gtest.h>
#include "configuration_dictionary/configuration_dictionary.h"

TEST(ConfigurationDictionary, empty)
{
  MutableConfigurationDictionary d;
  bool bool_val = true;
  int int_val = 123;
  double double_val = 1.23;
  std::string string_val = "123";
  ConfigurationDictionary dictionary_val;

  const std::string key = "key"; 
 
  ASSERT_FALSE(d.hasKey(key));

  ASSERT_FALSE(d.getParam(key, bool_val));

  ASSERT_FALSE(d.getParam(key, int_val));

  ASSERT_FALSE(d.getParam(key, double_val));

  ASSERT_FALSE(d.getParam(key, string_val));

  ASSERT_FALSE(d.getParam(key, dictionary_val));  
}

TEST(ConfigurationDictionary, preserve_defaults)
{
  MutableConfigurationDictionary d;
  bool bool_val = false;
  int int_val = 123;
  double double_val = 1.23;
  std::string string_val = "123";
  ConfigurationDictionary dictionary_val;

  const std::string key = "mykey"; 
 
  ASSERT_FALSE(d.hasKey(key));

  ASSERT_FALSE(d.getParam(key, bool_val));
  ASSERT_EQ(bool_val, false);

  ASSERT_FALSE(d.getParam(key, int_val));
  ASSERT_EQ(int_val, 123);

  ASSERT_FALSE(d.getParam(key, double_val));
  ASSERT_EQ(double_val, 1.23);

  ASSERT_FALSE(d.getParam(key, string_val));
  ASSERT_STREQ(string_val.c_str(), "123");

  ASSERT_FALSE(d.getParam(key, dictionary_val));  
  //TODO = test that if not found, default dictionary isn't changed
}

TEST(ConfigurationDictionary, set_and_get)
{
  MutableConfigurationDictionary d;
  bool bool_val = false;
  int int_val = 123;
  double double_val = 1.23;
  std::string string_val = "123";
  std::string key = "mykey";

  //Set bool
  d.setParam(key, true);

  ASSERT_TRUE(d.hasKey(key));

  ASSERT_TRUE(d.getParam(key, bool_val));
  ASSERT_EQ(bool_val, true);
  bool_val = false;

  ASSERT_FALSE(d.getParam(key, int_val));
  ASSERT_EQ(int_val, 123);

  ASSERT_FALSE(d.getParam(key, double_val));
  ASSERT_EQ(double_val, 1.23);

  ASSERT_FALSE(d.getParam(key, string_val));
  ASSERT_STREQ(string_val.c_str(), "123");

  //Set int
  d.setParam(key, 321);

  ASSERT_TRUE(d.hasKey(key));

  ASSERT_FALSE(d.getParam(key, bool_val));
  ASSERT_EQ(bool_val, false);

  ASSERT_TRUE(d.getParam(key, int_val));
  ASSERT_EQ(int_val, 321);
  int_val = 123;
  
  ASSERT_FALSE(d.getParam(key, double_val));
  ASSERT_EQ(double_val, 1.23);

  ASSERT_FALSE(d.getParam(key, string_val));
  ASSERT_STREQ(string_val.c_str(), "123");

  //Set double
  d.setParam(key, 3.21);

  ASSERT_TRUE(d.hasKey(key));

  ASSERT_FALSE(d.getParam(key, bool_val));
  ASSERT_EQ(bool_val, false);

  ASSERT_FALSE(d.getParam(key, int_val));
  ASSERT_EQ(int_val, 123);

  ASSERT_TRUE(d.getParam(key, double_val));
  ASSERT_EQ(double_val, 3.21);
  double_val = 1.23;

  ASSERT_FALSE(d.getParam(key, string_val));
  ASSERT_STREQ(string_val.c_str(), "123");

  //Set string  
  d.setParam(key, "321");

  ASSERT_TRUE(d.hasKey(key));

  ASSERT_FALSE(d.getParam(key, bool_val));
  ASSERT_EQ(bool_val, false);

  ASSERT_FALSE(d.getParam(key, int_val));
  ASSERT_EQ(int_val, 123);

  ASSERT_FALSE(d.getParam(key, double_val));
  ASSERT_EQ(double_val, 1.23);

  ASSERT_TRUE(d.getParam(key, string_val));
  ASSERT_STREQ(string_val.c_str(), "321");
  string_val = "123";

  //Clear param again
  d.deleteParam(key);
  ASSERT_FALSE(d.hasKey(key));

  ASSERT_FALSE(d.getParam(key, bool_val));
  ASSERT_EQ(bool_val, false);

  ASSERT_FALSE(d.getParam(key, int_val));
  ASSERT_EQ(int_val, 123);

  ASSERT_FALSE(d.getParam(key, double_val));
  ASSERT_EQ(double_val, 1.23);

  ASSERT_FALSE(d.getParam(key, string_val));
  ASSERT_STREQ(string_val.c_str(), "123");

}


int main(int argc, char *argv[]){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
