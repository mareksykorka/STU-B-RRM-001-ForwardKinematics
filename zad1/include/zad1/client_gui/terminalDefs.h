#ifndef CATKIN_PRVE_ZADANIE_TERMINALDEFS_H
#define CATKIN_PRVE_ZADANIE_TERMINALDEFS_H

//Term positions_
#define TERM_POS_MENU_S 1
#define TERM_POS_TABL_S 2
#define TERM_POS_CTRL_S 3
#define TERM_POS_CTRL   4
#define TERM_POS_INPT_S 8
#define TERM_POS_INPT   9
#define TERM_POS_STAT_S 10
#define TERM_POS_STAT   11
#define TERM_POS_STAT_E 14

// Graphics special functions defines
#define cls() printf("\033[H\033[J")
#define clrline() printf("\x1b[2K")
#define set_cursor(x,y) printf("\033[%d;%dH", (y), (x))

#endif //CATKIN_PRVE_ZADANIE_TERMINALDEFS_H