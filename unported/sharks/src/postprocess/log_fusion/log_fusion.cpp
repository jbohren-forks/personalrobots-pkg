#include "vacuum/vacuparse.h"

int main(int argc, char **argv)
{
  VacuParse vp("string.dump");
  uint32_t atom_len, publish_count, secs, nsecs;
  uint8_t *atom_ptr;
  vp.get_next_atom(&atom_len, &atom_ptr, &publish_count, &secs, &nsecs);
  vp.get_next_atom(&atom_len, &atom_ptr, &publish_count, &secs, &nsecs);
  vp.get_next_atom(&atom_len, &atom_ptr, &publish_count, &secs, &nsecs);
  vp.get_next_atom(&atom_len, &atom_ptr, &publish_count, &secs, &nsecs);
  return 0;
}
