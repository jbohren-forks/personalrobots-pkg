/* tree_node.cc
 *
 * The only implementation here is in reading/writing
 *
 */
//#include "stdafx.h"
#include "assert.h"
#include "tree_node.h"
namespace librf {
void tree_node::writeverbose(ostream& o) const {
  // we shouldn't be saving any other kind of node
  assert(status == TERMINAL || status == SPLIT);
  o << int(status);
  switch(status) {
	case TERMINAL:
		o << " leafno:" << leaf_no;
		o << " fgbg_label:" << int(fgbglabel);
		o << " viewpart_label:" << int(viewpartlabel);
		//#ifdef _DEBUG
		o << " fgbg_dist:";
		for (int i = 0; i < fgbgdist.size(); i++)
			o << " " << fgbgdist.at(i);
		o << " view_dist:";
		for (int i = 0; i < viewdist.size(); i++)
			o << " " << viewdist.at(i);
		//#endif
		o << " gain:" << gain;
		o << " patch_size:" << int(vecpatchid.size());
		for (int i = 0; i < vecpatchid.size(); i++)
		{
			o << " " << vecpatchid.at(i);
		}
		o << endl;
		//o << " " << leaf_no;
		//o << " " << int(fgbglabel);
		//o << " " << int(viewpartlabel);
		///*o << " fgbg_dist:";
		//for (int i = 0; i < fgbgdist.size(); i++)
		//	o << " " << fgbgdist[i];
		//o << " view_dist:";
		//for (int i = 0; i < viewdist.size(); i++)
		//	o << " " << viewdist[i];*/
		//o << " " << int(patchid.size());
		//for (int i = 0; i < patchid.size(); i++)
		//{
		//	o << " " << patchid[i];
		//}
		//o << endl;
		break;
	case SPLIT:
		o << " " << left << " " << right << " " <<  attr << " " << split_point << endl;
		break;
  }
}

void tree_node::write( ostream& o ) const
{
	// we shouldn't be saving any other kind of node
	assert(status == TERMINAL || status == SPLIT);
	o << int(status);
	switch(status) {
	case TERMINAL:
		o << " " << leaf_no;
		o << " " << int(fgbglabel);
		o << " " << int(viewpartlabel);
		o << " " << gain;
		o << " " << int(vecpatchid.size());
		for (int i = 0; i < vecpatchid.size(); i++)
		{
			o << " " << vecpatchid.at(i);
		}
		o << endl;
		break;
	case SPLIT:
		o << " " << left << " " << right << " " <<  attr << " " << split_point << endl;
		break;
	}
}

bool tree_node::read(istream& i) {
  int status_int;
  i >> status_int;
  int tmp;
  status = NodeStatusType(status_int);
  assert(status != EMPTY);
  switch(status) {
    case TERMINAL:
      int temp;
      i >> temp;
	  leaf_no = temp;
	  i >> temp;
      fgbglabel = uchar(temp);
	  i >> temp;
	  viewpartlabel = int(temp);
	  i >> gain;
	  i >> temp;
	  vecpatchid.resize(temp);
	  for (int j = 0; j < vecpatchid.size(); j++)
	  {
		  i >> tmp;
		  vecpatchid.at(j) = tmp;
	  }
    return true;
    case SPLIT:
      i >> left >> right >> attr >> split_point;
    return false;
  }
}



} // namespace
