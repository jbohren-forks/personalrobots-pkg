#ifndef KBHIT_H
#define KBHIT_H

// this code copied from linux-sys.org, where it was copied from "Beginning
// Linux Programming" a book by Wrox Press

void init_keyboard(void);
void close_keyboard(void);
int _kbhit(void);
char _getch(void);

#endif

