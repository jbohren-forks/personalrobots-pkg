#ifndef __OBJECT_CLASS_H__
#define __OBJECT_CLASS_H__

#include <ext/hash_set>
#include <iostream>
#include <stdexcept>

class DeserializationError : public std::runtime_error {
public: DeserializationError() : std::runtime_error("DeserializationError") { }
};

/**
   @brief Represents a class of object to be recognized
 */
class ObjectClass {
public:
  /**
     @param aid The unique id of the object
     @param aname A name for the object
   */
  ObjectClass(int aid, std::string aname) :
    id(aid),
    name(aname) { };

  /**
     @brief Deserializer
     @param istr Stream from which to deserialize
     @todo Add better error detection
   */
  ObjectClass(std::istream& istr) {
    /*
    char buf[1000];
    istr.getline(buf, sizeof(buf));
    int nvalid = sscanf(buf, "oc %d %s", &id, buf);
    if (nvalid != 2) 
      throw "Deserialization error";
    */
    std::string soc;
    istr >> soc;
    
    if (soc.compare("oc") != 0) 
      throw DeserializationError();
    
    istr >> id;
    istr >> name;
  }

  /**
     @param ostr Stream to which to serialize
   */
  void serialize(std::ostream& ostr) const {
    ostr << "oc " << id << " " << name << std::endl;
  }

  /**
     @brief Prints a nicely-formatted representation
     @param ostr Stream to which to print
   */
  void prettyPrint(std::ostream& ostr) const {
    ostr << "<" << id << "," << name << "> ";
  }

  /**
     @brief Unique object id
   */
  int id;

  /**
     @brief name Object's name as a string
   */
  std::string name;
};

struct ObjectClassHasher {
  size_t operator()(const ObjectClass& obj) const {
    return obj.id;
  }
};

struct ObjectClassComparer {
  bool operator()(const ObjectClass& o1, const ObjectClass& o2) const {
    return o1.id == o2.id;
  }
};

/*
typedef __gnu_cxx::hash_set<ObjecbtClass, 
			    ObjectClassHasher, 
			    ObjectClassComparer> ObjectSet;
*/

typedef __gnu_cxx::hash_set<ObjectClass, 
			    ObjectClassHasher, 
			    ObjectClassComparer> ObjectSetRaw;

/**
   @brief A set of objects to be recognized
 */
class ObjectSet : public ObjectSetRaw {
public:

  ObjectSet() { };

  /**
     @brief Deserializing constructor
     @param istr Stream from which to deserialize
   */
  ObjectSet(std::istream& istr) {
    //    char buf[1000];
    //    istr.getline(buf, sizeof(buf));
    using namespace std;;

    string sbuf;
    istr >> sbuf;

    cout << "OSET " << sbuf << std::endl;

    if (sbuf.compare("objset") != 0) 
      throw DeserializationError();

    int nObjects = 0;

    cout << "OSET " << sbuf.c_str() << endl;

    istr >> nObjects;

    //    int nvalid = sscanf(sbuf.c_str(), "objset %d", &nObjects);

    for (int ii = 0; ii < nObjects; ii++) {
      ObjectClass oclass(istr);
      insert(oclass);
    }

    /// @todo ensure that the user did not misspecify the number of objects
  }

  /**
     @param ovec A vector of ObjectClass'es from which to construct this object
   */
  ObjectSet(const std::vector<ObjectClass>& ovec) {
    for (std::vector<ObjectClass>::const_iterator it = ovec.begin();
	 it != ovec.end(); 
	 it++) {
      insert(*it);
    }
  }

  /**
     @param An object id
     @return The name corresponding to the id
   */
  std::string findName(int id) const {
    ObjectClass oclass(id,"");
    ObjectSet::iterator found = find(oclass);
    //    assert(found != end());

    if (found == end()) {
      char buf[200];
      snprintf(buf, sizeof(buf), "<bad id %d>", id);
      return std::string(buf);
    }

    return (*found).name;
  }

  /**
     @param ostr Stream to which to serialize 
   */
  void serialize(std::ostream& ostr) const {
    ostr << "objset " << this->size() << std::endl;

    for (ObjectSet::iterator it = begin();
	 it != end();
	 it++) {
      (*it).serialize(ostr);
    }
  }

  /**
     @param ostr Stream to which to pretty-print
   */
  void prettyPrint(std::ostream& ostr) const {
    for (ObjectSet::iterator it = begin();
	 it != end();
	 it++) {
      (*it).prettyPrint(ostr);
    }
  }

  /**
     @brief Converts this object to a vector of object classes
     @param ovec Output argument
   */
  void vectorize(std::vector<ObjectClass>& ovec) const {
    for (ObjectSet::iterator it = begin();
	 it != end();
	 it++) {
      ObjectClass oclass = *it;
      ovec.push_back(oclass);
    }
  }
};

#endif
