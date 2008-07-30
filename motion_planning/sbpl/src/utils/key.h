#ifndef __KEY_H_
#define __KEY_H_

#define KEY_SIZE 2

#define INFINITECOST 1000000000

class CKey
{
//data
public: 
	long int key[KEY_SIZE];
	int key_size;

//constructors
public:
	CKey(){
		key_size = KEY_SIZE;
		for(int i = 0; i < key_size; i++)
		{	
			key[i] = 0;
		}
	};
	~CKey(){};

//functions
public:
	void SetKeytoInfinity()
	{
		for(int i = 0; i < key_size; i++)
		{	
			key[i] = INFINITECOST;
		}
	};
	void SetKeytoZero()
	{
		for(int i = 0; i < key_size; i++)
		{	
			key[i] = 0;
		}
	};
	


//operators
public:

	void operator = (CKey RHSKey)
	{
		//iterate through the keys
		//the 0ht is the most important key
		key_size = RHSKey.key_size;
		for(int i = 0; i < key_size; i++)
			key[i] = RHSKey.key[i];
	};

	CKey operator - (const CKey& RHSKey) const
	{
		CKey RetKey;

		if(key_size != RHSKey.key_size)
		{
			printf("ERROR: keys should be of the same size for '-' operator\n");
			exit(1);
		}

		//iterate through the keys
		//the 0ht is the most important key
		RetKey.key_size = key_size;
		for(int i = 0; i < key_size; i++)
			RetKey.key[i] = key[i] - RHSKey.key[i];

		return RetKey;
	};


	bool operator > (CKey& RHSKey)
	{
	  //iterate through the keys
	  //the 0ht is the most important key
	  for(int i = 0; i < key_size; i++)
	    {
	      //compare the current key
	      if(key[i] > RHSKey.key[i])
		return true;
	      else if(key[i] < RHSKey.key[i])
		return false;
	    }
	  
	  //all are equal
	  return false;
	};

	bool operator == (CKey& RHSKey)
	{
	  //iterate through the keys
	  //the 0ht is the most important key
	  for(int i = 0; i < key_size; i++)
	    {
	      //compare the current key
	      if(key[i] != RHSKey.key[i])
		return false;
	    }
	  
	  //all are equal
	  return true;
	};

	bool operator != (CKey& RHSKey)
	{
	  return !(*this == RHSKey);
	};

	bool operator < (CKey& RHSKey)
	{	
	  return (!(*this > RHSKey) && !(*this == RHSKey));
	};

	bool operator >= (CKey& RHSKey)
	{
	  return ((*this > RHSKey) || (*this == RHSKey));
	};

	bool operator <= (CKey& RHSKey)
	{
	  return !(*this > RHSKey);
	};
	
	long int operator [] (int i)
	{
		return key[i];
	};

};



#endif
